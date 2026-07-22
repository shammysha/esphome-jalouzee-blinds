#include <cmath>
#include "jalouzee_blinds.h"
#include "esphome/core/application.h"
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
static const uint32_t FLASH_SAVE_MIN_INTERVAL_MS = 30000;  // не пишем в flash чаще, бережём ресурс

// =====================================================================
// button::Button / select::Select / number::Number обвязки
// =====================================================================
void CalibrationButton::press_action() { this->parent_->on_calibration_button_pressed(); }
void CancelCalibrationButton::press_action() { this->parent_->on_cancel_calibration_button_pressed(); }
void FaultResetButton::press_action() { this->parent_->on_fault_reset_button_pressed(); }
void AngleSourceSelect::control(const std::string &value) {
  this->parent_->on_angle_source_select_changed(value);
}
void FaultTimeoutNumber::control(float value) { this->parent_->on_fault_timeout_changed(value); }

// =====================================================================
// setup / dump_config
// =====================================================================
void JalouzeeBlinds::set_hall_encoder_pins(InternalGPIOPin *a, InternalGPIOPin *b) {
  this->encoder_a_pin_ = a;
  this->encoder_b_pin_ = b;
  this->has_hall_ = true;
}

void JalouzeeBlinds::setup() {
  // --- мотор ---
  this->in1_pin_->setup();
  this->in2_pin_->setup();
  this->in1_pin_->digital_write(false);
  this->in2_pin_->digital_write(false);

  // --- энкодер Холла ---
  if (this->has_hall_) {
    this->encoder_a_pin_->setup();
    this->encoder_b_pin_->setup();
    this->encoder_a_pin_->attach_interrupt(&JalouzeeBlinds::hall_isr_, this, gpio::INTERRUPT_ANY_EDGE);
  }
  // --- ADC (резистор на оси мотора) ---
  // Используем штатный ADC-компонент ESPHome (ESP-IDF adc_oneshot драйвер,
  // включая калибровку по эталонной кривой/линии, если она доступна для
  // конкретного чипа). Объект создаём и настраиваем сами, в App не
  // регистрируем (не нужен периодический update()) — читаем sample() вручную.
  if (this->has_adc_) {
    this->adc_sensor_ = new adc::ADCSensor();  // NOLINT(cppcoreguidelines-owning-memory)
    this->adc_sensor_->set_pin(this->adc_gpio_pin_);
#ifdef USE_ESP32
    this->adc_sensor_->set_attenuation(adc::ADC_ATTEN_DB_12_COMPAT);
#endif
    this->adc_sensor_->setup();
  }

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

  // --- п.2: поведение после потери питания, если выбран Hall/ADC ---
  if (this->store_.angle_source_mode == ANGLE_SOURCE_ENCODER) {
    bool mpu_ok = this->has_mpu_ && this->store_.mpu_calibrated;
    if (mpu_ok) {
      ESP_LOGW(TAG, "После перезагрузки: показания Hall/ADC не абсолютны или недостоверны. "
                     "Временно (на текущую сессию) используем MPU6050 как источник угла.");
      // ничего дополнительно менять не нужно — resolve_active_source_() сама
      // отдаст приоритет MPU, если store_.angle_source_mode запросил ENCODER,
      // но энкодер ещё не переподтверждён калибровкой в этой сессии.
      // Реализовано через operation_blocked_ = false и forced-фолбэк ниже.
    } else {
      ESP_LOGW(TAG, "После перезагрузки: источник Hall/ADC выбран, но резервный MPU6050 "
                     "недоступен/не откалиброван. Управление жалюзи заблокировано до калибровки.");
      this->operation_blocked_ = true;
    }
  }

  this->register_sub_entities_();
  this->update_calibrated_binary_sensor_();
  this->set_calibration_message_(MSG_ENTER_CALIBRATION);

  this->publish_state(this->current_percent_ / 100.0f);
}

void JalouzeeBlinds::register_sub_entities_() {
  const std::string base_name = this->get_name();

  // configure_entity_() only stores a StringRef (no copy) — keep the built name
  // strings alive in entity_name_storage_ for the lifetime of the device. Reserve
  // exactly the number of make_name() calls below so the vector never reallocates
  // (which would invalidate the c_str() pointers already handed to entities).
  this->entity_name_storage_.reserve(8);
  auto make_name = [this](std::string name) -> const char * {
    this->entity_name_storage_.push_back(std::move(name));
    return this->entity_name_storage_.back().c_str();
  };

  this->calibration_button_ = new CalibrationButton();
  this->calibration_button_->set_parent(this);
  App.register_button(this->calibration_button_, make_name(base_name + " Калибровка"), 0, 0);

  this->cancel_calibration_button_ = new CancelCalibrationButton();
  this->cancel_calibration_button_->set_parent(this);
  // Всегда видна в HA — ESPHome не поддерживает динамическое отключение/скрытие
  // кнопки в рантайме. Нажатие вне калибровки безопасно игнорируется в
  // on_cancel_calibration_button_pressed().
  App.register_button(this->cancel_calibration_button_, make_name(base_name + " Отменить калибровку"), 0, 0);

  this->fault_reset_button_ = new FaultResetButton();
  this->fault_reset_button_->set_parent(this);
  App.register_button(this->fault_reset_button_, make_name(base_name + " Сброс аварии"), 0, 0);

  this->angle_source_select_ = new AngleSourceSelect();
  this->angle_source_select_->set_parent(this);
  {
    FixedVector<const char *> options;
    options.init(3);
    options.push_back("auto");
    if (this->has_mpu_)
      options.push_back("mpu6050");
    if (this->has_hall_ || this->has_adc_)
      options.push_back("encoder");
    this->angle_source_select_->traits.set_options(options);
  }
  App.register_select(this->angle_source_select_, make_name(base_name + " Источник угла наклона"), 0, 0);
  {
    const char *cur = "auto";
    if (this->store_.angle_source_mode == ANGLE_SOURCE_MPU6050) cur = "mpu6050";
    else if (this->store_.angle_source_mode == ANGLE_SOURCE_ENCODER) cur = "encoder";
    this->angle_source_select_->publish_state(cur);
  }

  this->fault_timeout_number_ = new FaultTimeoutNumber();
  this->fault_timeout_number_->set_parent(this);
  this->fault_timeout_number_->traits.set_min_value(1);
  this->fault_timeout_number_->traits.set_max_value(300);
  this->fault_timeout_number_->traits.set_step(1);
  App.register_number(this->fault_timeout_number_, make_name(base_name + " Таймаут аварии (сек)"), 0, 0);
  this->fault_timeout_number_->publish_state(this->fault_timeout_s_);

  this->calibration_text_sensor_ = new text_sensor::TextSensor();
  App.register_text_sensor(this->calibration_text_sensor_, make_name(base_name + " Сообщение калибровки"), 0, 0);

  this->calibrated_binary_sensor_ = new binary_sensor::BinarySensor();
  App.register_binary_sensor(this->calibrated_binary_sensor_, make_name(base_name + " Откалибровано"), 0, 0);

  this->fault_binary_sensor_ = new binary_sensor::BinarySensor();
  App.register_binary_sensor(this->fault_binary_sensor_, make_name(base_name + " Авария"), 0, 0);
  this->fault_binary_sensor_->publish_state(false);
}

void JalouzeeBlinds::dump_config() {
  ESP_LOGCONFIG(TAG, "Jalouzee Blinds:");
  ESP_LOGCONFIG(TAG, "  Мотор: DC GA12-N20, IN1/IN2 заданы");
  if (this->has_hall_) {
    ESP_LOGCONFIG(TAG, "  Датчик Холла энкодера: 7 PPR x передаточное число редуктора, A/B заданы");
  }
  if (this->has_adc_) {
    ESP_LOGCONFIG(TAG, "  Резистор на оси мотора: ADC пин задан (ESP-IDF adc_oneshot драйвер)");
  }
  if (this->has_mpu_) {
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
                                                                  : this->calibration_text_sensor_->state);
  }

  // пересчёт текущего угла (если есть хоть один рабочий калиброванный источник)
  ActiveAngleSource src = this->resolve_active_source_();
  if (src != ACTIVE_SOURCE_NONE) {
    float raw = this->read_raw_(src);
    float pct = this->raw_to_percent_(src, raw);
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
  } else if (this->motor_dir_ != MOTOR_STOP) {
    this->check_fault_();
    if (!this->fault_active_) {
      this->handle_movement_();
    }
  }

  // периодическое сохранение текущего угла (не чаще раза в FLASH_SAVE_MIN_INTERVAL_MS)
  if (this->motor_dir_ == MOTOR_STOP && (now - this->last_flash_save_ms_) > FLASH_SAVE_MIN_INTERVAL_MS) {
    if (fabsf(this->current_percent_ - this->store_.last_angle_percent) > 0.5f) {
      this->store_.last_angle_percent = this->current_percent_;
      this->save_to_flash_();
      this->last_flash_save_ms_ = now;
    }
  }
}

// =====================================================================
// Мотор
// =====================================================================
void JalouzeeBlinds::motor_open_() {
  this->motor_dir_ = MOTOR_OPENING;
  this->in1_pin_->digital_write(true);
  this->in2_pin_->digital_write(false);
}
void JalouzeeBlinds::motor_close_() {
  this->motor_dir_ = MOTOR_CLOSING;
  this->in1_pin_->digital_write(false);
  this->in2_pin_->digital_write(true);
}
void JalouzeeBlinds::motor_stop_() {
  this->motor_dir_ = MOTOR_STOP;
  this->in1_pin_->digital_write(false);
  this->in2_pin_->digital_write(false);
}

// =====================================================================
// Источники угла
// =====================================================================
bool JalouzeeBlinds::is_source_calibrated_(ActiveAngleSource src) {
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      return this->store_.mpu_calibrated;
    case ACTIVE_SOURCE_HALL:
      return this->store_.hall_calibrated;
    case ACTIVE_SOURCE_ADC:
      return this->store_.adc_calibrated;
    default:
      return false;
  }
}

bool JalouzeeBlinds::is_source_available_(ActiveAngleSource src) {
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      return this->has_mpu_ && this->mpu_sensor_ != nullptr && this->mpu_sensor_->has_state();
    case ACTIVE_SOURCE_HALL:
      return this->has_hall_;
    case ACTIVE_SOURCE_ADC:
      return this->has_adc_;
    default:
      return false;
  }
}

ActiveAngleSource JalouzeeBlinds::resolve_active_source_() {
  uint8_t mode = this->store_.angle_source_mode;

  auto hall_or_adc_active = [this]() -> ActiveAngleSource {
    if (this->has_hall_ && this->is_source_calibrated_(ACTIVE_SOURCE_HALL)) return ACTIVE_SOURCE_HALL;
    if (this->has_adc_ && this->is_source_calibrated_(ACTIVE_SOURCE_ADC)) return ACTIVE_SOURCE_ADC;
    return ACTIVE_SOURCE_NONE;
  };
  auto mpu_active = [this]() -> ActiveAngleSource {
    if (this->has_mpu_ && this->is_source_available_(ACTIVE_SOURCE_MPU6050) &&
        this->is_source_calibrated_(ACTIVE_SOURCE_MPU6050))
      return ACTIVE_SOURCE_MPU6050;
    return ACTIVE_SOURCE_NONE;
  };

  if (mode == ANGLE_SOURCE_MPU6050) {
    return mpu_active();
  }
  if (mode == ANGLE_SOURCE_ENCODER) {
    // п.2: если после потери питания энкодер ещё не переподтверждён —
    // используем MPU как временный fallback этой сессии, если он доступен.
    if (this->operation_blocked_) return ACTIVE_SOURCE_NONE;
    ActiveAngleSource enc = hall_or_adc_active();
    if (enc != ACTIVE_SOURCE_NONE) return enc;
    return mpu_active();
  }
  // AUTO: приоритет 1) MPU6050  2) Hall/ADC
  ActiveAngleSource m = mpu_active();
  if (m != ACTIVE_SOURCE_NONE) return m;
  return hall_or_adc_active();
}

float JalouzeeBlinds::read_adc_raw_() {
  if (this->adc_sensor_ == nullptr) return NAN;
  // sample() выполняет одиночное измерение через ESP-IDF adc_oneshot API
  // (с калибровкой, если она доступна) и возвращает напряжение в вольтах.
  // Для наших целей единица измерения неважна — калибровка "закрыто/открыто"
  // работает с любой монотонной величиной.
  return this->adc_sensor_->sample();
}

float JalouzeeBlinds::read_raw_(ActiveAngleSource src) {
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      return this->mpu_sensor_->state;
    case ACTIVE_SOURCE_HALL:
      return static_cast<float>(this->hall_pulse_count_);
    case ACTIVE_SOURCE_ADC:
      return this->read_adc_raw_();
    default:
      return NAN;
  }
}

float JalouzeeBlinds::raw_to_percent_(ActiveAngleSource src, float raw) {
  float closed = 0, open = 0;
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      closed = this->store_.mpu_closed;
      open = this->store_.mpu_open;
      break;
    case ACTIVE_SOURCE_HALL:
      closed = this->store_.hall_closed;
      open = this->store_.hall_open;
      break;
    case ACTIVE_SOURCE_ADC:
      closed = this->store_.adc_closed;
      open = this->store_.adc_open;
      break;
    default:
      return NAN;
  }
  if (open == closed) return NAN;
  // формула сама учитывает "зеркальность" установки датчика (п.5):
  // если open < closed, знаменатель отрицательный — направление инвертируется автоматически.
  float pct = (raw - closed) / (open - closed) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

void JalouzeeBlinds::hall_isr_(JalouzeeBlinds *arg) {
  // простое квадратурное декодирование по фазе B на фронте A
  bool b_level = arg->encoder_b_pin_->digital_read();
  if (b_level) {
    arg->hall_pulse_count_++;
  } else {
    arg->hall_pulse_count_--;
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
  this->motor_stop_();
  this->target_percent_ = NAN;
  this->jog_mode_ = true;
  this->cal_state_ = CAL_WAIT_CLOSED;
  this->set_calibration_message_(MSG_WAIT_CLOSED);
  // 50% держит обе стрелки (вверх/вниз) активными в HA на время калибровки —
  // реальная позиция ещё не откалибрована, репортим её обратно в finish/cancel.
  this->publish_state(0.5f);
}

void JalouzeeBlinds::capture_calibration_point_(bool is_closed_point) {
  if (is_closed_point) {
    if (this->has_hall_) this->temp_hall_closed_ = static_cast<float>(this->hall_pulse_count_);
    if (this->has_adc_) this->temp_adc_closed_ = this->read_adc_raw_();
    if (this->has_mpu_ && this->mpu_sensor_ != nullptr) this->temp_mpu_closed_ = this->mpu_sensor_->state;
  }
  // "открытая" точка обрабатывается сразу в finish_calibration_()
}

void JalouzeeBlinds::finish_calibration_() {
  // сохраняем данные калибровки для ВСЕХ доступных датчиков (п.3)
  if (this->has_hall_) {
    this->store_.hall_closed = this->temp_hall_closed_;
    this->store_.hall_open = static_cast<float>(this->hall_pulse_count_);
    this->store_.hall_calibrated = true;
  }
  if (this->has_adc_) {
    this->store_.adc_closed = this->temp_adc_closed_;
    this->store_.adc_open = this->read_adc_raw_();
    this->store_.adc_calibrated = true;
  }
  if (this->has_mpu_ && this->mpu_sensor_ != nullptr) {
    this->store_.mpu_closed = this->temp_mpu_closed_;
    this->store_.mpu_open = this->mpu_sensor_->state;
    this->store_.mpu_calibrated = true;
  }

  this->operation_blocked_ = false;
  this->cal_state_ = CAL_IDLE;
  this->jog_mode_ = false;
  this->motor_stop_();

  // точка "открыто" только что зафиксирована — текущее физическое положение ей и является
  this->current_percent_ = 100.0f;
  this->store_.last_angle_percent = this->current_percent_;
  this->publish_state(this->current_percent_ / 100.0f);

  this->save_to_flash_();
  this->update_calibrated_binary_sensor_();
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
  this->motor_stop_();
  this->set_calibration_message_(MSG_ENTER_CALIBRATION);
  // возвращаем реальную (последнюю известную) позицию вместо принудительных 50%
  this->publish_state(this->current_percent_ / 100.0f);
}

void JalouzeeBlinds::set_calibration_message_(const std::string &msg, bool temporary) {
  this->calibration_text_sensor_->publish_state(msg);
  this->cal_message_is_temporary_ = temporary;
  if (temporary) {
    this->cal_message_expire_ms_ = millis() + 5000;
  }
}

void JalouzeeBlinds::update_calibrated_binary_sensor_() {
  bool calibrated = this->store_.hall_calibrated || this->store_.adc_calibrated || this->store_.mpu_calibrated;
  this->calibrated_binary_sensor_->publish_state(calibrated);
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
  this->motor_stop_();
  this->target_percent_ = NAN;
  this->fault_binary_sensor_->publish_state(true);
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
  this->fault_binary_sensor_->publish_state(false);
}

// =====================================================================
// Select / Number обработчики
// =====================================================================
void JalouzeeBlinds::on_angle_source_select_changed(const std::string &value) {
  uint8_t mode = ANGLE_SOURCE_AUTO;
  if (value == "mpu6050") mode = ANGLE_SOURCE_MPU6050;
  else if (value == "encoder") mode = ANGLE_SOURCE_ENCODER;

  this->store_.angle_source_mode = mode;
  this->save_to_flash_();
  this->angle_source_select_->publish_state(value);
  ESP_LOGI(TAG, "Режим определения угла изменён пользователем: %s", value.c_str());
}

void JalouzeeBlinds::on_fault_timeout_changed(float seconds) {
  if (seconds < 1) seconds = 1;
  this->fault_timeout_s_ = static_cast<uint32_t>(seconds);
  this->fault_timeout_number_->publish_state(this->fault_timeout_s_);
  ESP_LOGI(TAG, "Таймаут аварии изменён: %lu с", this->fault_timeout_s_);
}

// =====================================================================
// Управление жалюзи (cover::Cover::control) — п.7
// =====================================================================
void JalouzeeBlinds::control(const cover::CoverCall &call) {
  // ВРЕМЕННАЯ диагностика — убрать после выяснения причины бага с блокировкой.
  ESP_LOGD(TAG,
           "control() DEBUG: cal_state=%d jog_mode=%d mode=%u hall_cal=%d adc_cal=%d mpu_cal=%d "
           "operation_blocked=%d resolved_src=%d fault_active=%d",
           static_cast<int>(this->cal_state_), this->jog_mode_, this->store_.angle_source_mode,
           this->store_.hall_calibrated, this->store_.adc_calibrated, this->store_.mpu_calibrated,
           this->operation_blocked_, static_cast<int>(this->resolve_active_source_()), this->fault_active_);

  if (this->cal_state_ != CAL_IDLE) {
    // В режиме калибровки: open/close работают как ручной джог "вверх/вниз",
    // stop — останавливает мотор. Кнопка калибровки фиксирует точки.
    if (call.get_stop()) {
      this->motor_stop_();
      return;
    }
    if (call.get_position().has_value()) {
      float pos = *call.get_position();
      if (pos >= 0.5f) this->motor_open_();
      else this->motor_close_();
      return;
    }
    return;
  }

  // Блокируем управление, если нет ни одного откалиброванного и доступного сейчас
  // источника угла — это покрывает и полностью не откалиброванное устройство
  // (после первой прошивки), и уже существующий кейс operation_blocked_ (см. setup()).
  if (this->resolve_active_source_() == ACTIVE_SOURCE_NONE) {
    ESP_LOGW(TAG, "Управление жалюзи заблокировано: нет откалиброванного источника угла. "
                   "Выполните калибровку.");
    return;
  }
  if (this->fault_active_) {
    ESP_LOGW(TAG, "Управление жалюзи заблокировано: активна авария. Сбросьте её кнопкой сброса аварии.");
    return;
  }

  if (call.get_stop()) {
    this->motor_stop_();
    this->target_percent_ = NAN;
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
    this->motor_open_();
  } else if (target_percent < this->current_percent_ - STEP_TARGET_EPSILON) {
    this->motor_close_();
  } else {
    this->motor_stop_();
  }
}

void JalouzeeBlinds::handle_movement_() {
  if (std::isnan(this->target_percent_)) return;

  bool reached = false;
  if (this->motor_dir_ == MOTOR_OPENING && this->current_percent_ >= this->target_percent_ - STEP_TARGET_EPSILON) {
    reached = true;
  } else if (this->motor_dir_ == MOTOR_CLOSING &&
             this->current_percent_ <= this->target_percent_ + STEP_TARGET_EPSILON) {
    reached = true;
  }

  if (reached) {
    this->motor_stop_();
    this->target_percent_ = NAN;
    this->store_.last_angle_percent = this->current_percent_;
    this->save_to_flash_();
    this->last_flash_save_ms_ = millis();
    this->publish_state(this->current_percent_ / 100.0f);
  } else {
    this->publish_state(this->current_percent_ / 100.0f);
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
}

}  // namespace jalouzee_blinds
}  // namespace esphome
