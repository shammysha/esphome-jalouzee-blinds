#include <cmath>
#include "angle_calibration.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jalouzee_blinds {

static const char *const TAG = "jalouzee_blinds";

// Минимальная разница |open - closed| для каждого источника, ниже которой калибровка
// считается невалидной (датчик, скорее всего, не двигался/отключён от ламелей) и
// НЕ помечается как calibrated — иначе шумный, фактически неподвижный источник может
// быть ошибочно приоритизирован в режиме auto, а деление на почти нулевой диапазон
// в raw_to_percent() усилит шум до полного хода жалюзи.
static const float MPU_MIN_CAL_DELTA = 1.0f;    // m/s²
static const float HALL_MIN_CAL_DELTA = 10.0f;  // импульсов
static const float ADC_MIN_CAL_DELTA = 0.1f;    // В

void AngleCalibration::normalize_uncalibrated() {
  if (!this->store_->hall_calibrated) this->store_->hall_closed = this->store_->hall_open = NAN;
  if (!this->store_->adc_calibrated) this->store_->adc_closed = this->store_->adc_open = NAN;
  if (!this->store_->mpu_calibrated) this->store_->mpu_closed = this->store_->mpu_open = NAN;
}

bool AngleCalibration::is_any_calibrated() const {
  return this->store_->hall_calibrated || this->store_->adc_calibrated || this->store_->mpu_calibrated;
}

bool AngleCalibration::is_source_calibrated_(ActiveAngleSource src) const {
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      return this->store_->mpu_calibrated;
    case ACTIVE_SOURCE_HALL:
      return this->store_->hall_calibrated;
    case ACTIVE_SOURCE_ADC:
      return this->store_->adc_calibrated;
    default:
      return false;
  }
}

bool AngleCalibration::is_source_available_(ActiveAngleSource src) const {
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      return this->mpu_->is_available();
    case ACTIVE_SOURCE_HALL:
      return this->hall_adc_->has_hall();
    case ACTIVE_SOURCE_ADC:
      return this->hall_adc_->has_adc();
    default:
      return false;
  }
}

ActiveAngleSource AngleCalibration::resolve_active_source(bool operation_blocked) const {
  uint8_t mode = this->store_->angle_source_mode;

  auto hall_or_adc_active = [this]() -> ActiveAngleSource {
    if (this->hall_adc_->has_hall() && this->is_source_calibrated_(ACTIVE_SOURCE_HALL)) return ACTIVE_SOURCE_HALL;
    if (this->hall_adc_->has_adc() && this->is_source_calibrated_(ACTIVE_SOURCE_ADC)) return ACTIVE_SOURCE_ADC;
    return ACTIVE_SOURCE_NONE;
  };
  auto mpu_active = [this]() -> ActiveAngleSource {
    if (this->mpu_->has_mpu() && this->is_source_available_(ACTIVE_SOURCE_MPU6050) &&
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
    if (operation_blocked) return ACTIVE_SOURCE_NONE;
    ActiveAngleSource enc = hall_or_adc_active();
    if (enc != ACTIVE_SOURCE_NONE) return enc;
    return mpu_active();
  }
  // AUTO: приоритет 1) MPU6050  2) Hall/ADC
  ActiveAngleSource m = mpu_active();
  if (m != ACTIVE_SOURCE_NONE) return m;
  return hall_or_adc_active();
}

float AngleCalibration::read_raw(ActiveAngleSource src) const {
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      return this->mpu_->read_raw();
    case ACTIVE_SOURCE_HALL:
      return this->hall_adc_->read_hall_raw();
    case ACTIVE_SOURCE_ADC:
      return this->hall_adc_->read_adc_raw();
    default:
      return NAN;
  }
}

float AngleCalibration::raw_to_percent(ActiveAngleSource src, float raw) const {
  float closed = 0, open = 0, min_delta = 0;
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      closed = this->store_->mpu_closed;
      open = this->store_->mpu_open;
      min_delta = MPU_MIN_CAL_DELTA;
      break;
    case ACTIVE_SOURCE_HALL:
      closed = this->store_->hall_closed;
      open = this->store_->hall_open;
      min_delta = HALL_MIN_CAL_DELTA;
      break;
    case ACTIVE_SOURCE_ADC:
      closed = this->store_->adc_closed;
      open = this->store_->adc_open;
      min_delta = ADC_MIN_CAL_DELTA;
      break;
    default:
      return NAN;
  }
  // Защита от старых/повреждённых калибровочных данных с почти нулевым диапазоном
  // (см. HALL/ADC/MPU_MIN_CAL_DELTA) — иначе шум усиливается делением на ~0.
  if (fabsf(open - closed) < min_delta) return NAN;
  // формула сама учитывает "зеркальность" установки датчика (п.5):
  // если open < closed, знаменатель отрицательный — направление инвертируется автоматически.
  float pct = (raw - closed) / (open - closed) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

bool AngleCalibration::try_finish_calibration(ActiveAngleSource src, float closed, float open) {
  float min_delta = 0;
  float delta = fabsf(open - closed);
  bool accepted;
  switch (src) {
    case ACTIVE_SOURCE_HALL:
      min_delta = HALL_MIN_CAL_DELTA;
      accepted = delta >= min_delta;
      if (!accepted) ESP_LOGW(TAG, "Калибровка Hall отклонена: движение не обнаружено (разница %.1f имп.)", delta);
      break;
    case ACTIVE_SOURCE_ADC:
      min_delta = ADC_MIN_CAL_DELTA;
      accepted = delta >= min_delta;
      if (!accepted) ESP_LOGW(TAG, "Калибровка ADC отклонена: движение не обнаружено (разница %.3f В)", delta);
      break;
    case ACTIVE_SOURCE_MPU6050:
      min_delta = MPU_MIN_CAL_DELTA;
      accepted = delta >= min_delta;
      if (!accepted)
        ESP_LOGW(TAG,
                 "Калибровка MPU6050 отклонена: движение не обнаружено (разница %.3f м/с² — "
                 "датчик, вероятно, отключён от ламелей)",
                 delta);
      break;
    default:
      return false;
  }

  if (!accepted) {
    closed = open = NAN;
  }

  switch (src) {
    case ACTIVE_SOURCE_HALL:
      this->store_->hall_closed = closed;
      this->store_->hall_open = open;
      this->store_->hall_calibrated = accepted;
      break;
    case ACTIVE_SOURCE_ADC:
      this->store_->adc_closed = closed;
      this->store_->adc_open = open;
      this->store_->adc_calibrated = accepted;
      break;
    case ACTIVE_SOURCE_MPU6050:
      this->store_->mpu_closed = closed;
      this->store_->mpu_open = open;
      this->store_->mpu_calibrated = accepted;
      break;
    default:
      break;
  }
  return accepted;
}

bool AngleCalibration::try_auto_calibrate_at_endpoint(bool is_closed_point) {
  bool any = false;
  if (this->hall_adc_->has_hall() && !this->store_->hall_calibrated) {
    any |= this->auto_calibrate_capture_(ACTIVE_SOURCE_HALL, is_closed_point, this->hall_adc_->read_hall_raw());
  }
  if (this->hall_adc_->has_adc() && !this->store_->adc_calibrated) {
    any |= this->auto_calibrate_capture_(ACTIVE_SOURCE_ADC, is_closed_point, this->hall_adc_->read_adc_raw());
  }
  if (this->mpu_->has_mpu() && !this->store_->mpu_calibrated && this->mpu_->is_available()) {
    any |= this->auto_calibrate_capture_(ACTIVE_SOURCE_MPU6050, is_closed_point, this->mpu_->read_raw());
  }
  return any;
}

bool AngleCalibration::auto_calibrate_capture_(ActiveAngleSource src, bool is_closed_point, float raw) {
  // Работаем через локальные копии, а не указатели на поля store_ — она
  // __attribute__((packed)), и &store_->hall_closed и т.п. дают предупреждение
  // компилятора о невыровненном указателе (-Waddress-of-packed-member).
  float closed = NAN, open = NAN, min_delta = 0;
  const char *name = "";
  switch (src) {
    case ACTIVE_SOURCE_HALL:
      closed = this->store_->hall_closed;
      open = this->store_->hall_open;
      min_delta = HALL_MIN_CAL_DELTA;
      name = "Hall";
      break;
    case ACTIVE_SOURCE_ADC:
      closed = this->store_->adc_closed;
      open = this->store_->adc_open;
      min_delta = ADC_MIN_CAL_DELTA;
      name = "ADC";
      break;
    case ACTIVE_SOURCE_MPU6050:
      closed = this->store_->mpu_closed;
      open = this->store_->mpu_open;
      min_delta = MPU_MIN_CAL_DELTA;
      name = "MPU6050";
      break;
    default:
      return false;
  }

  if (is_closed_point) {
    closed = raw;
  } else {
    open = raw;
  }

  bool now_calibrated = !std::isnan(closed) && !std::isnan(open) && fabsf(open - closed) >= min_delta;

  switch (src) {
    case ACTIVE_SOURCE_HALL:
      this->store_->hall_closed = closed;
      this->store_->hall_open = open;
      if (now_calibrated) this->store_->hall_calibrated = true;
      break;
    case ACTIVE_SOURCE_ADC:
      this->store_->adc_closed = closed;
      this->store_->adc_open = open;
      if (now_calibrated) this->store_->adc_calibrated = true;
      break;
    case ACTIVE_SOURCE_MPU6050:
      this->store_->mpu_closed = closed;
      this->store_->mpu_open = open;
      if (now_calibrated) this->store_->mpu_calibrated = true;
      break;
    default:
      break;
  }

  if (now_calibrated) {
    ESP_LOGI(TAG, "Автокалибровка %s завершена по опорным точкам активного источника", name);
  }
  return now_calibrated;
}

}  // namespace jalouzee_blinds
}  // namespace esphome
