#include <cmath>
#include "controller.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jalouzee_blinds {

static const char *const TAG = "jalouzee_blinds";

// Minimum |open - closed| difference for each source, below which the
// calibration is considered invalid (the sensor most likely didn't move /
// is physically disconnected from the slats) and is NOT marked as
// calibrated — otherwise a noisy, effectively stationary source could get
// wrongly prioritized in auto mode, and dividing by a near-zero range in
// raw_to_percent() would amplify that noise across the blinds' full travel.
static const float MPU_MIN_CAL_DELTA = 1.0f;    // m/s²
static const float HALL_MIN_CAL_DELTA = 10.0f;  // pulses
static const float ADC_MIN_CAL_DELTA = 0.1f;    // V

void Controller::normalize_uncalibrated() {
  if (!this->store_->hall_calibrated) this->store_->hall_closed = this->store_->hall_open = NAN;
  if (!this->store_->adc_calibrated) this->store_->adc_closed = this->store_->adc_open = NAN;
  if (!this->store_->mpu_calibrated) this->store_->mpu_closed = this->store_->mpu_open = NAN;
}

bool Controller::is_any_calibrated() const {
  return this->store_->hall_calibrated || this->store_->adc_calibrated || this->store_->mpu_calibrated;
}

bool Controller::is_source_calibrated_(ActiveAngleSource src) const {
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

bool Controller::is_source_available_(ActiveAngleSource src) const {
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

ActiveAngleSource Controller::resolve_active_source(bool hall_untrusted) const {
  uint8_t mode = this->store_->angle_source_mode;

  // hall_untrusted gates only Hall — ADC is absolute (the current voltage
  // IS the current position right now) and doesn't need this protection.
  // Checked here rather than in a separate branch per mode — otherwise the
  // protection wouldn't work at all in auto mode, since AUTO uses
  // hall_or_adc_active() as a fallback.
  auto hall_or_adc_active = [this, hall_untrusted]() -> ActiveAngleSource {
    if (!hall_untrusted && this->hall_adc_->has_hall() && this->is_source_calibrated_(ACTIVE_SOURCE_HALL))
      return ACTIVE_SOURCE_HALL;
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
    ActiveAngleSource enc = hall_or_adc_active();
    if (enc != ACTIVE_SOURCE_NONE) return enc;
    // See point 2: if Hall is untrusted and there's no ADC left — fall back
    // to MPU for this session, if available.
    return mpu_active();
  }
  // AUTO: priority 1) MPU6050  2) Hall/ADC
  ActiveAngleSource m = mpu_active();
  if (m != ACTIVE_SOURCE_NONE) return m;
  return hall_or_adc_active();
}

float Controller::read_raw(ActiveAngleSource src) const {
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

float Controller::raw_to_percent(ActiveAngleSource src, float raw) const {
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
  // Guards against stale/corrupted calibration data with a near-zero range
  // (see HALL/ADC/MPU_MIN_CAL_DELTA) — otherwise noise gets amplified by
  // dividing by ~0.
  if (fabsf(open - closed) < min_delta) return NAN;
  // The formula naturally accounts for sensor mounting "mirroring" (point
  // 5): if open < closed, the denominator is negative and the direction
  // inverts automatically.
  float pct = (raw - closed) / (open - closed) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

float Controller::percent_to_raw(ActiveAngleSource src, float percent) const {
  float closed = 0, open = 0;
  switch (src) {
    case ACTIVE_SOURCE_MPU6050:
      closed = this->store_->mpu_closed;
      open = this->store_->mpu_open;
      break;
    case ACTIVE_SOURCE_HALL:
      closed = this->store_->hall_closed;
      open = this->store_->hall_open;
      break;
    case ACTIVE_SOURCE_ADC:
      closed = this->store_->adc_closed;
      open = this->store_->adc_open;
      break;
    default:
      return NAN;
  }
  // An uncalibrated source gives closed/open == NAN, which naturally
  // propagates to the result — the caller must check calibrated itself.
  return closed + (percent / 100.0f) * (open - closed);
}

bool Controller::try_finish_calibration(ActiveAngleSource src, float closed, float open) {
  float min_delta = 0;
  float delta = fabsf(open - closed);
  bool accepted;
  switch (src) {
    case ACTIVE_SOURCE_HALL:
      min_delta = HALL_MIN_CAL_DELTA;
      accepted = delta >= min_delta;
      if (!accepted) ESP_LOGW(TAG, "Hall calibration rejected: no movement detected (delta %.1f pulses)", delta);
      break;
    case ACTIVE_SOURCE_ADC:
      min_delta = ADC_MIN_CAL_DELTA;
      accepted = delta >= min_delta;
      if (!accepted) ESP_LOGW(TAG, "ADC calibration rejected: no movement detected (delta %.3f V)", delta);
      break;
    case ACTIVE_SOURCE_MPU6050:
      min_delta = MPU_MIN_CAL_DELTA;
      accepted = delta >= min_delta;
      if (!accepted)
        ESP_LOGW(TAG,
                 "MPU6050 calibration rejected: no movement detected (delta %.3f m/s² — "
                 "the sensor is probably disconnected from the slats)",
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

bool Controller::try_auto_calibrate_at_endpoint(bool is_closed_point) {
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

bool Controller::auto_calibrate_capture_(ActiveAngleSource src, bool is_closed_point, float raw) {
  // Work through local copies rather than pointers into store_'s fields — it
  // is __attribute__((packed)), and &store_->hall_closed etc. trigger a
  // compiler warning about an unaligned pointer (-Waddress-of-packed-member).
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
    ESP_LOGI(TAG, "%s auto-calibration completed from reference points of the active source", name);
  }
  return now_calibrated;
}

}  // namespace jalouzee_blinds
}  // namespace esphome
