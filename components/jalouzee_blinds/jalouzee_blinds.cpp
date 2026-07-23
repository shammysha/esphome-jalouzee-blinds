#include <cmath>
#include "jalouzee_blinds.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jalouzee_blinds {

static const char *const TAG = "jalouzee_blinds";

static const char *const MSG_ENTER_CALIBRATION = "Перейти в режим калибровки";
static const char *const MSG_WAIT_CLOSED =
    "Переведите ламели в крайнее нижнее положение и еще раз нажмите кнопку";
static const char *const MSG_WAIT_OPEN =
    "Переведите ламели в крайнее верхнее положение и еще раз нажмите кнопку";
static const char *const MSG_DONE = "Калибровка завершена";

static const float FAULT_ANGLE_EPSILON = 0.5f;    // % — минимальное изменение угла, чтобы не считать "завис"
static const float STEP_TARGET_EPSILON = 0.5f;    // % — попадание в целевую позицию
// В простое пишем редко — ручного управления в проекте не предполагается, риск
// расхождения позиции копится только между визитами, а не посреди хода.
// Защита от обрыва питания ПОСРЕДИ хода реализована отдельно и точно — через
// store_.movement_in_progress (см. start_move_to_percent_/setup()), а не через
// учащённую запись позиции по таймеру.
static const uint32_t FLASH_SAVE_MIN_INTERVAL_MS = 300000;  // 5 мин
// publish_state() во время движения не чаще этого интервала — без throttling
// вызывался бы на каждой итерации loop() (сотни-тысячи раз в секунду), забивая
// API-соединение и мешая обработке входящих команд (см. обсуждение лагов).
static const uint32_t POSITION_PUBLISH_INTERVAL_MS = 1000;

// =====================================================================
// setup / dump_config
// =====================================================================
void JalouzeeBlinds::setup() {
  this->motor_.setup();

  this->angle_cal_.set_sensors(&this->hall_adc_, &this->mpu_);
  this->angle_cal_.set_store(&this->store_);

  // --- preferences (flash) ---
  {
    char object_id_buf[OBJECT_ID_MAX_LEN];
    size_t object_id_len = this->write_object_id_to(object_id_buf, sizeof(object_id_buf));
    this->pref_ = global_preferences->make_preference<JalouzeeBlindsStore>(
        fnv1_hash("jalouzee_blinds_" + std::string(object_id_buf, object_id_len)));
  }
  this->load_from_flash_();

  // если пользователь не переопределял через сеттер codegen — берём значение из YAML config
  if (this->store_.angle_source_mode == 0 && this->configured_angle_source_mode_ != ANGLE_SOURCE_AUTO) {
    this->store_.angle_source_mode = this->configured_angle_source_mode_;
  }

  this->current_percent_ = this->store_.last_angle_percent;

  // Восстанавливаем счётчик импульсов Холла из последней сохранённой позиции —
  // иначе после ребута он стартует с 0, теряя привязку к калибровочным точкам
  // hall_closed/hall_open. Обязательно ДО hall_adc_.setup() (там прикрепляются
  // прерывания).
  if (this->store_.hall_calibrated) {
    float seed = this->angle_cal_.percent_to_raw(ACTIVE_SOURCE_HALL, this->store_.last_angle_percent);
    if (!std::isnan(seed)) {
      this->hall_adc_.seed_hall_pulse_count(static_cast<int32_t>(lroundf(seed)));
    }
  }
  this->hall_adc_.setup();

  // --- п.2: движение было прервано потерей питания (movement_in_progress не
  // сброшен штатным завершением — см. store.h) --- восстановленная выше
  // позиция Hall в этом случае недостоверна: неизвестно, сколько реально
  // прошло с последнего сохранения. ADC не затрагиваем — это абсолютный
  // датчик (текущее напряжение = текущее положение прямо сейчас).
  bool movement_interrupted = this->store_.movement_in_progress;
  if (movement_interrupted) {
    this->store_.movement_in_progress = false;
    this->save_to_flash_();
  }
  if (movement_interrupted && this->hall_adc_.has_hall()) {
    bool mpu_ok = this->mpu_.has_mpu() && this->store_.mpu_calibrated;
    if (mpu_ok) {
      ESP_LOGW(TAG, "Обнаружено движение, прерванное потерей питания. Позиция Hall недостоверна — "
                     "временно (на текущую сессию) используем MPU6050 как источник угла.");
      // ничего дополнительно менять не нужно — resolve_active_source() сама
      // отдаст приоритет MPU и не станет использовать Hall, пока он не будет
      // переподтверждён калибровкой. Реализовано через operation_blocked_.
    } else {
      ESP_LOGW(TAG, "Обнаружено движение, прерванное потерей питания, а резервный MPU6050 "
                     "недоступен/не откалиброван. Управление жалюзи заблокировано до калибровки.");
      this->operation_blocked_ = true;
    }
  }

  {
    const char *initial_mode = "auto";
    if (this->store_.angle_source_mode == ANGLE_SOURCE_MPU6050) initial_mode = "angle";
    else if (this->store_.angle_source_mode == ANGLE_SOURCE_ENCODER) initial_mode = "encoder";
    this->sub_entities_.setup(this, this->get_name(), this->hall_adc_.has_hall(), this->hall_adc_.has_adc(),
                               this->mpu_.has_mpu(), initial_mode, this->fault_timeout_s_);
  }
  this->publish_calibration_diagnostics_();
  this->set_calibration_message_(MSG_ENTER_CALIBRATION);

  this->position = this->current_percent_ / 100.0f;
  this->publish_state();
}

void JalouzeeBlinds::dump_config() {
  ESP_LOGCONFIG(TAG, "Jalouzee Blinds:");
  ESP_LOGCONFIG(TAG, "  Мотор: DC GA12-N20, IN1/IN2 заданы");
  if (this->hall_adc_.has_hall()) {
    ESP_LOGCONFIG(TAG, "  Датчик Холла энкодера: 7 PPR x передаточное число редуктора, A/B заданы");
  }
  if (this->hall_adc_.has_adc()) {
    ESP_LOGCONFIG(TAG, "  Резистор на оси мотора: ADC пин задан (штатный ADC-компонент ESPHome)");
  }
  if (this->mpu_.has_mpu()) {
    ESP_LOGCONFIG(TAG, "  MPU6050: используется внешний sensor");
  }
  ESP_LOGCONFIG(TAG, "  Режим определения угла (сохранён): %u", this->store_.angle_source_mode);
  ESP_LOGCONFIG(TAG, "  Таймаут аварии: %lu с", this->fault_timeout_s_);
}

cover::CoverTraits JalouzeeBlinds::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(this->operation_blocked_);
  traits.set_supports_position(true);
  traits.set_supports_tilt(false);
  traits.set_supports_stop(true);
  return traits;
}

// =====================================================================
// loop
// =====================================================================
void JalouzeeBlinds::loop() {
  const uint32_t now = millis();

  // истечение временного сообщения калибровки ("Калибровка завершена")
  if (this->cal_message_is_temporary_ && now > this->cal_message_expire_ms_) {
    this->cal_message_is_temporary_ = false;
    this->set_calibration_message_(this->cal_state_ == CAL_IDLE ? MSG_ENTER_CALIBRATION
                                                                  : this->sub_entities_.calibration_message_state());
  }

  // пересчёт текущего угла (если есть хоть один рабочий калиброванный источник)
  ActiveAngleSource src = this->angle_cal_.resolve_active_source(this->operation_blocked_);
  if (src != ACTIVE_SOURCE_NONE) {
    float raw = this->angle_cal_.read_raw(src);
    float pct = this->angle_cal_.raw_to_percent(src, raw);
    if (!std::isnan(pct)) {
      if (fabsf(pct - this->last_seen_percent_for_fault_) > FAULT_ANGLE_EPSILON || std::isnan(this->last_seen_percent_for_fault_)) {
        this->last_angle_change_ms_ = now;
        this->last_seen_percent_for_fault_ = pct;
      }
      this->current_percent_ = pct;
    }
  }

  if (this->jog_mode_) {
    // ручной джог во время калибровки — без цели, без проверки аварии
  } else if (this->motor_.direction() != MOTOR_STOP) {
    this->check_fault_();
    if (!this->fault_active_) {
      this->handle_movement_();
    }
  }

  // периодическое сохранение текущего угла (не чаще раза в FLASH_SAVE_MIN_INTERVAL_MS)
  if (this->motor_.direction() == MOTOR_STOP && (now - this->last_flash_save_ms_) > FLASH_SAVE_MIN_INTERVAL_MS) {
    if (fabsf(this->current_percent_ - this->store_.last_angle_percent) > 0.5f) {
      this->store_.last_angle_percent = this->current_percent_;
      this->save_to_flash_();
      this->last_flash_save_ms_ = now;
    }
  }
}

// =====================================================================
// Калибровка (п.3)
// =====================================================================
void JalouzeeBlinds::on_calibration_button_pressed() {
  switch (this->cal_state_) {
    case CAL_IDLE:
      this->enter_calibration_();
      break;
    case CAL_WAIT_CLOSED:
      this->capture_calibration_point_(true);
      this->cal_state_ = CAL_WAIT_OPEN;
      this->set_calibration_message_(MSG_WAIT_OPEN);
      break;
    case CAL_WAIT_OPEN:
      this->capture_calibration_point_(false);
      this->finish_calibration_();
      break;
  }
}

void JalouzeeBlinds::enter_calibration_() {
  ESP_LOGI(TAG, "Начало калибровки");
  this->motor_.stop();
  this->target_percent_ = NAN;
  this->clear_movement_in_progress_();
  this->jog_mode_ = true;
  this->cal_state_ = CAL_WAIT_CLOSED;
  this->set_calibration_message_(MSG_WAIT_CLOSED);
  // 50% держит обе стрелки (вверх/вниз) активными в HA на время калибровки —
  // реальная позиция ещё не откалибрована, репортим её обратно в finish/cancel.
  this->position = 0.5f;
  this->publish_state();
}

void JalouzeeBlinds::capture_calibration_point_(bool is_closed_point) {
  if (is_closed_point) {
    if (this->hall_adc_.has_hall()) this->temp_hall_closed_ = this->hall_adc_.read_hall_raw();
    if (this->hall_adc_.has_adc()) this->temp_adc_closed_ = this->hall_adc_.read_adc_raw();
    if (this->mpu_.has_mpu()) this->temp_mpu_closed_ = this->mpu_.read_raw();
  }
  // "открытая" точка обрабатывается сразу в finish_calibration_()
}

void JalouzeeBlinds::finish_calibration_() {
  // пытаемся принять калибровку для ВСЕХ доступных датчиков (п.3), но только
  // если реально зафиксировано движение — иначе источник остаётся некалиброванным
  // (см. Controller::try_finish_calibration).
  if (this->hall_adc_.has_hall()) {
    this->angle_cal_.try_finish_calibration(ACTIVE_SOURCE_HALL, this->temp_hall_closed_,
                                             this->hall_adc_.read_hall_raw());
  }
  if (this->hall_adc_.has_adc()) {
    this->angle_cal_.try_finish_calibration(ACTIVE_SOURCE_ADC, this->temp_adc_closed_, this->hall_adc_.read_adc_raw());
  }
  if (this->mpu_.has_mpu()) {
    this->angle_cal_.try_finish_calibration(ACTIVE_SOURCE_MPU6050, this->temp_mpu_closed_, this->mpu_.read_raw());
  }

  this->operation_blocked_ = false;
  this->cal_state_ = CAL_IDLE;
  this->jog_mode_ = false;
  this->motor_.stop();

  // точка "открыто" только что зафиксирована — текущее физическое положение ей и является
  this->current_percent_ = 100.0f;
  this->store_.last_angle_percent = this->current_percent_;
  this->position = this->current_percent_ / 100.0f;
  this->publish_state();

  this->save_to_flash_();
  this->publish_calibration_diagnostics_();
  this->set_calibration_message_(MSG_DONE, /*temporary=*/true);

  ESP_LOGI(TAG, "Калибровка завершена и сохранена");
}

void JalouzeeBlinds::on_cancel_calibration_button_pressed() {
  if (this->cal_state_ == CAL_IDLE) return;  // недоступно вне калибровки
  this->cancel_calibration_();
}

void JalouzeeBlinds::cancel_calibration_() {
  ESP_LOGI(TAG, "Калибровка отменена пользователем");
  this->cal_state_ = CAL_IDLE;
  this->jog_mode_ = false;
  this->motor_.stop();
  this->set_calibration_message_(MSG_ENTER_CALIBRATION);
  // возвращаем реальную (последнюю известную) позицию вместо принудительных 50%
  this->position = this->current_percent_ / 100.0f;
  this->publish_state();
}

void JalouzeeBlinds::set_calibration_message_(const std::string &msg, bool temporary) {
  this->sub_entities_.set_calibration_message(msg);
  this->cal_message_is_temporary_ = temporary;
  if (temporary) {
    this->cal_message_expire_ms_ = millis() + 5000;
  }
}

void JalouzeeBlinds::publish_calibration_diagnostics_() {
  this->sub_entities_.set_calibrated(this->angle_cal_.is_any_calibrated());
  if (this->hall_adc_.has_hall()) {
    this->sub_entities_.set_hall_calibrated(this->angle_cal_.is_calibrated(ACTIVE_SOURCE_HALL));
  }
  if (this->hall_adc_.has_adc()) {
    this->sub_entities_.set_adc_calibrated(this->angle_cal_.is_calibrated(ACTIVE_SOURCE_ADC));
  }
  if (this->mpu_.has_mpu()) {
    this->sub_entities_.set_angle_calibrated(this->angle_cal_.is_calibrated(ACTIVE_SOURCE_MPU6050));
  }
}

// =====================================================================
// Авария (п.6)
// =====================================================================
void JalouzeeBlinds::check_fault_() {
  uint32_t now = millis();
  if ((now - this->last_angle_change_ms_) > (this->fault_timeout_s_ * 1000UL)) {
    this->trigger_fault_();
  }
}

void JalouzeeBlinds::trigger_fault_() {
  if (this->fault_active_) return;
  ESP_LOGE(TAG, "АВАРИЯ: угол наклона не меняется дольше %lu с при активном движении мотора", this->fault_timeout_s_);
  this->fault_active_ = true;
  this->motor_.stop();
  this->target_percent_ = NAN;
  this->clear_movement_in_progress_();
  this->sub_entities_.set_fault(true);
}

void JalouzeeBlinds::on_fault_reset_button_pressed() {
  if (!this->fault_active_) return;
  this->clear_fault_();
}

void JalouzeeBlinds::clear_fault_() {
  ESP_LOGI(TAG, "Статус аварии сброшен пользователем");
  this->fault_active_ = false;
  this->last_angle_change_ms_ = millis();
  this->last_seen_percent_for_fault_ = NAN;
  this->sub_entities_.set_fault(false);
}

// =====================================================================
// Select / Number обработчики
// =====================================================================
void JalouzeeBlinds::on_angle_source_select_changed(const std::string &value) {
  uint8_t mode = ANGLE_SOURCE_AUTO;
  if (value == "angle") mode = ANGLE_SOURCE_MPU6050;
  else if (value == "encoder") mode = ANGLE_SOURCE_ENCODER;

  this->angle_cal_.set_mode(mode);
  this->save_to_flash_();
  this->sub_entities_.set_angle_source_state(value);
  ESP_LOGI(TAG, "Режим определения угла изменён пользователем: %s", value.c_str());
}

void JalouzeeBlinds::on_fault_timeout_changed(float seconds) {
  if (seconds < 1) seconds = 1;
  this->fault_timeout_s_ = static_cast<uint32_t>(seconds);
  this->sub_entities_.set_fault_timeout_state(this->fault_timeout_s_);
  ESP_LOGI(TAG, "Таймаут аварии изменён: %lu с", this->fault_timeout_s_);
}

// =====================================================================
// Управление жалюзи (cover::Cover::control) — п.7
// =====================================================================
void JalouzeeBlinds::control(const cover::CoverCall &call) {
  if (this->cal_state_ != CAL_IDLE) {
    // В режиме калибровки: open/close работают как ручной джог "вверх/вниз",
    // stop — останавливает мотор. Кнопка калибровки фиксирует точки.
    if (call.get_stop()) {
      this->motor_.stop();
      return;
    }
    if (call.get_position().has_value()) {
      float pos = *call.get_position();
      if (pos >= 0.5f) this->motor_.open();
      else this->motor_.close();
      return;
    }
    return;
  }

  // Блокируем управление, если нет ни одного откалиброванного и доступного сейчас
  // источника угла — это покрывает и полностью не откалиброванное устройство
  // (после первой прошивки), и обнаруженное прерванное движение (см. setup()).
  if (this->angle_cal_.resolve_active_source(this->operation_blocked_) == ACTIVE_SOURCE_NONE) {
    ESP_LOGW(TAG, "Управление жалюзи заблокировано: нет откалиброванного источника угла. "
                   "Выполните калибровку.");
    return;
  }
  if (this->fault_active_) {
    ESP_LOGW(TAG, "Управление жалюзи заблокировано: активна авария. Сбросьте её кнопкой сброса аварии.");
    return;
  }

  if (call.get_stop()) {
    this->motor_.stop();
    this->target_percent_ = NAN;
    this->clear_movement_in_progress_();
    return;
  }

  if (call.get_position().has_value()) {
    float pos = *call.get_position();  // 0.0..1.0
    float pct = pos * 100.0f;

    // Явные значения 0 / 1 (обычные open()/close() без указания слайдера) —
    // обрабатываем через пошаговую логику "закрыто -> 50% -> открыто" (п.7).
    if (pct <= 0.5f) {
      this->handle_open_close_request_(false);
    } else if (pct >= 99.5f) {
      this->handle_open_close_request_(true);
    } else {
      // произвольное значение (например, из слайдера в HA) — двигаемся напрямую туда
      this->current_step_index_ = (pct < 25) ? 0 : (pct < 75 ? 1 : 2);
      this->start_move_to_percent_(pct);
    }
  }
}

void JalouzeeBlinds::handle_open_close_request_(bool opening) {
  static const float STEPS[3] = {0.0f, 50.0f, 100.0f};
  int8_t next = this->current_step_index_;
  if (opening) {
    next = (next < 2) ? next + 1 : 2;
  } else {
    next = (next > 0) ? next - 1 : 0;
  }
  this->current_step_index_ = next;
  this->start_move_to_percent_(STEPS[next]);
}

void JalouzeeBlinds::start_move_to_percent_(float target_percent) {
  this->target_percent_ = target_percent;
  this->last_angle_change_ms_ = millis();
  this->last_seen_percent_for_fault_ = NAN;

  if (target_percent > this->current_percent_ + STEP_TARGET_EPSILON) {
    this->motor_.open();
  } else if (target_percent < this->current_percent_ - STEP_TARGET_EPSILON) {
    this->motor_.close();
  } else {
    this->motor_.stop();
    return;  // уже на месте — реального движения не было, писать flash не нужно
  }

  // Реально начали двигаться — фиксируем во flash, чтобы после ребута точно
  // знать, было ли движение прервано потерей питания (см. setup()).
  if (!this->store_.movement_in_progress) {
    this->store_.movement_in_progress = true;
    this->save_to_flash_();
  }
}

void JalouzeeBlinds::handle_movement_() {
  if (std::isnan(this->target_percent_)) return;

  bool reached = false;
  if (this->motor_.direction() == MOTOR_OPENING &&
      this->current_percent_ >= this->target_percent_ - STEP_TARGET_EPSILON) {
    reached = true;
  } else if (this->motor_.direction() == MOTOR_CLOSING &&
             this->current_percent_ <= this->target_percent_ + STEP_TARGET_EPSILON) {
    reached = true;
  }

  if (reached) {
    this->motor_.stop();
    this->target_percent_ = NAN;
    this->store_.last_angle_percent = this->current_percent_;
    this->store_.movement_in_progress = false;
    this->save_to_flash_();
    this->last_flash_save_ms_ = millis();
    this->position = this->current_percent_ / 100.0f;
    this->publish_state();

    // Реально дошли до края хода — оппортунистическая автокалибровка любых
    // доступных, но пока не откалиброванных источников (см. Controller).
    bool auto_calibrated;
    if (this->current_percent_ <= STEP_TARGET_EPSILON) {
      auto_calibrated = this->angle_cal_.try_auto_calibrate_at_endpoint(true);
    } else if (this->current_percent_ >= 100.0f - STEP_TARGET_EPSILON) {
      auto_calibrated = this->angle_cal_.try_auto_calibrate_at_endpoint(false);
    } else {
      auto_calibrated = false;
    }
    if (auto_calibrated) {
      this->save_to_flash_();
      this->publish_calibration_diagnostics_();
    }
  } else {
    // Throttled — без этого publish_state() уходил бы на каждой итерации
    // loop() во время движения, забивая API-соединение (см. POSITION_PUBLISH_INTERVAL_MS).
    uint32_t now = millis();
    if (now - this->last_position_publish_ms_ >= POSITION_PUBLISH_INTERVAL_MS) {
      this->last_position_publish_ms_ = now;
      this->position = this->current_percent_ / 100.0f;
      this->publish_state();
    }
  }
}

// =====================================================================
// Flash
// =====================================================================
void JalouzeeBlinds::save_to_flash_() { this->pref_.save(&this->store_); }

void JalouzeeBlinds::load_from_flash_() {
  if (!this->pref_.load(&this->store_)) {
    this->store_ = JalouzeeBlindsStore{};
    this->store_.angle_source_mode = this->configured_angle_source_mode_;
  }
  // Для НЕоткалиброванных источников closed/open должны быть NAN (а не 0.0 из
  // zero-init/старых данных), иначе auto-калибровка ошибочно решит, что одна
  // из точек уже поймана.
  this->angle_cal_.normalize_uncalibrated();
}

void JalouzeeBlinds::clear_movement_in_progress_() {
  if (this->store_.movement_in_progress) {
    this->store_.movement_in_progress = false;
    this->save_to_flash_();
  }
}

}  // namespace jalouzee_blinds
}  // namespace esphome
