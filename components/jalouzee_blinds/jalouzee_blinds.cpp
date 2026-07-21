#include "jalouzee_blinds.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace jalouzee_blinds {

static const char *const TAG = "jalouzee_blinds";
// Fixed hash so calibration survives recompiles as long as the object id
// namespace stays "jalouzee_blinds_calibration".
static const uint32_t CALIBRATION_PREF_HASH = 0xA1C3B7E5UL;

// ------------------------------------------------------------------------
// JalouzeeBlinds hub
// ------------------------------------------------------------------------

void JalouzeeBlinds::setup() {
  this->cw_pin_->setup();
  this->ccw_pin_->setup();
  this->ph_a_pin_->setup();
  this->ph_b_pin_->setup();
  this->cw_pin_->digital_write(false);
  this->ccw_pin_->digital_write(false);
  this->last_ph_a_ = this->ph_a_pin_->digital_read();
  this->last_ph_b_ = this->ph_b_pin_->digital_read();

  this->load_calibration_();
  this->initialize_();
}

void JalouzeeBlinds::load_calibration_() {
  this->pref_ = global_preferences->make_preference<CalibrationData>(CALIBRATION_PREF_HASH);
  CalibrationData data{};
  if (this->pref_.load(&data)) {
    this->pos_low_ = data.pos_low;
    this->pos_high_ = data.pos_high;
    this->step_total_ = data.step_total;
    this->curr_step_ = data.curr_step;
  }
}

void JalouzeeBlinds::save_calibration_() {
  CalibrationData data{this->pos_low_, this->pos_high_, this->step_total_, this->curr_step_};
  this->pref_.save(&data);
}

void JalouzeeBlinds::initialize_() {
  // Original YAML picked the mode at boot based on whether the MPU6050
  // I2C component had failed to initialize. Here it's simpler and more
  // robust: mode is just "did the user wire up an angle_sensor:".
  this->mode_ = (this->angle_sensor_ != nullptr) ? MODE_ANGLE : MODE_ENCODER;

  if (this->mode_ == MODE_ANGLE) {
    this->calibrated_ = (this->pos_low_ != 0.0f) && (this->pos_high_ != 0.0f);
  } else {
    this->calibrated_ = (this->step_total_ != 0.0f) && (this->curr_step_ != 0.0f);
  }

  if (this->calibrated_sensor_ != nullptr)
    this->calibrated_sensor_->publish_state(this->calibrated_);
  if (this->calibrate_step_sensor_ != nullptr)
    this->calibrate_step_sensor_->publish_state(0);

  ESP_LOGI(TAG, "Positioning by %s", this->mode_ == MODE_ANGLE ? "ANGLE SENSOR" : "ENCODER");
  if (this->calibrated_) {
    ESP_LOGI(TAG, "pos_low=%.2f pos_high=%.2f step_total=%.2f curr_step=%.2f", this->pos_low_, this->pos_high_,
              this->step_total_, this->curr_step_);
  } else {
    ESP_LOGW(TAG, "Not calibrated yet - press the Calibrate button");
  }
}

void JalouzeeBlinds::loop() {
  // Poll the encoder inputs every loop and act on rising edges (mirrors the
  // default software-debounced behaviour of ESPHome's gpio binary_sensor
  // that the original YAML relied on).
  bool a = this->ph_a_pin_->digital_read();
  bool b = this->ph_b_pin_->digital_read();
  if (a && !this->last_ph_a_)
    this->handle_encoder_pulse_(true);
  if (b && !this->last_ph_b_)
    this->handle_encoder_pulse_(false);
  this->last_ph_a_ = a;
  this->last_ph_b_ = b;

  uint32_t now = millis();
  if (now - this->last_update_ms_ < 100)
    return;
  this->last_update_ms_ = now;

  this->update_position_();
}

void JalouzeeBlinds::handle_encoder_pulse_(bool is_a_phase) {
  if (this->cw_active_) {
    this->curr_step_ += 1;
    if (this->calibrate_ == CALIBRATION_STEP_2)
      this->step_count_ += 1;
    float limit = this->step_total_ + (is_a_phase ? 0.0f : 1.0f);
    if (this->calibrated_ && this->curr_step_ > limit)
      this->curr_step_ = this->step_total_ + 1;
  } else if (this->ccw_active_) {
    this->curr_step_ -= 1;
    if (this->calibrate_ == CALIBRATION_STEP_2)
      this->step_count_ -= 1;
    if (this->curr_step_ < 1)
      this->curr_step_ = 1;
  }
}

float JalouzeeBlinds::compute_angle_value_(float raw) {
  raw = std::round(raw * 10.0f) / 10.0f;
  bool sens_side = this->pos_low_ > this->pos_high_;  // false = left, true = right

  if (this->calibrate_ != CALIBRATION_NONE) {
    // mid-calibration: don't reinterpret the raw angle yet
    return raw;
  }

  float x;
  if (sens_side) {
    if (raw > this->pos_low_)
      return 0.0f;
    if (raw < this->pos_high_)
      return 1.0f;
    x = 1.0f - std::round((raw - this->pos_high_) / (this->pos_low_ - this->pos_high_) * 100.0f) / 100.0f;
  } else {
    if (raw < this->pos_low_)
      return 0.0f;
    if (raw > this->pos_high_)
      return 1.0f;
    x = std::round((raw - this->pos_low_) / (this->pos_high_ - this->pos_low_) * 100.0f) / 100.0f;
  }
  this->curr_step_ = std::round(x * this->step_total_) + 1;
  return x;
}

float JalouzeeBlinds::compute_rotary_value_() {
  if (!this->calibrated_ || this->step_total_ == 0.0f)
    return NAN;

  float x = (this->curr_step_ - 1.0f) / this->step_total_;
  if (x > 0.9f || (x < 0.5f && x >= 0.1f)) {
    x -= 0.04f;
  } else if (x < 0.1f || (x > 0.5f && x <= 0.9f)) {
    x += 0.04f;
  }
  return std::round(x * 10.0f) / 10.0f;
}

void JalouzeeBlinds::update_position_() {
  float sens;
  if (this->mode_ == MODE_ANGLE) {
    float raw = this->angle_sensor_ != nullptr ? this->angle_sensor_->state : 0.0f;
    sens = this->compute_angle_value_(raw);
  } else {
    sens = this->compute_rotary_value_();
  }

  if (std::isnan(sens)) {
    ESP_LOGW(TAG, "Not calibrated yet - can't compute position");
    return;
  }

  bool moving = this->cw_active_ || this->ccw_active_;

  // Stop the motor once the tilt target has been reached.
  if (moving && this->calibrated_) {
    if ((this->cw_active_ && sens >= this->curr_tilt_) || (this->ccw_active_ && sens <= this->curr_tilt_)) {
      this->stop_motor_();
      if (this->cover_ != nullptr)
        this->cover_->current_operation = cover::COVER_OPERATION_IDLE;
    }
  }

  if (this->calibrated_ && moving)
    this->update_stall_detection_(sens);

  float rounded = std::round(sens * 2.0f) / 2.0f;
  if (this->cover_ != nullptr && (std::isnan(this->last_published_value_) || rounded != this->last_published_value_ ||
                                    !moving)) {
    this->last_published_value_ = rounded;
    this->cover_->position = rounded;
    this->cover_->tilt = rounded;
    this->cover_->publish_state();
  }
}

void JalouzeeBlinds::update_stall_detection_(float sens) {
  if (this->cycle_time_ < 9) {
    this->cycle_time_++;
    return;
  }
  if (this->old_state_ == sens) {
    ESP_LOGE(TAG, "Motor stalled - no movement detected, stopping for safety");
    this->set_has_problem(true);
    this->stop_motor_();
    if (this->cover_ != nullptr) {
      this->cover_->current_operation = cover::COVER_OPERATION_IDLE;
      this->cover_->publish_state();
    }
  }
}

void JalouzeeBlinds::start_motor_(bool clockwise) {
  if (this->has_problem_) {
    this->stop_motor_();
    if (this->cover_ != nullptr) {
      this->cover_->current_operation = cover::COVER_OPERATION_IDLE;
      this->cover_->publish_state();
    }
    return;
  }

  this->old_state_ =
      this->mode_ == MODE_ANGLE ? (this->angle_sensor_ != nullptr ? this->angle_sensor_->state : 0.0f)
                                 : this->compute_rotary_value_();
  this->cycle_time_ = 0;

  if (clockwise) {
    this->ccw_pin_->digital_write(false);
    this->ccw_active_ = false;
    this->cw_pin_->digital_write(true);
    this->cw_active_ = true;
  } else {
    this->cw_pin_->digital_write(false);
    this->cw_active_ = false;
    this->ccw_pin_->digital_write(true);
    this->ccw_active_ = true;
  }
}

void JalouzeeBlinds::stop_motor_() {
  this->cw_pin_->digital_write(false);
  this->ccw_pin_->digital_write(false);
  this->cw_active_ = false;
  this->ccw_active_ = false;
}

void JalouzeeBlinds::request_open() {
  if (this->curr_tilt_ >= 0.5f && this->curr_tilt_ < 1.0f) {
    this->curr_tilt_ = 1.0f;
  } else {
    this->curr_tilt_ = 0.5f;
  }
  if (this->cover_ != nullptr) {
    this->cover_->current_operation = cover::COVER_OPERATION_OPENING;
    this->cover_->publish_state();
  }
  this->start_motor_(true);
}

void JalouzeeBlinds::request_close() {
  if (this->curr_tilt_ > 0.0f && this->curr_tilt_ <= 0.5f) {
    this->curr_tilt_ = 0.0f;
  } else {
    this->curr_tilt_ = 0.5f;
  }
  if (this->cover_ != nullptr) {
    this->cover_->current_operation = cover::COVER_OPERATION_CLOSING;
    this->cover_->publish_state();
  }
  this->start_motor_(false);
}

void JalouzeeBlinds::request_tilt(float tilt) {
  if (this->curr_tilt_ < tilt) {
    this->curr_tilt_ = tilt;
    this->start_motor_(true);
  } else if (this->curr_tilt_ > tilt) {
    this->curr_tilt_ = tilt;
    this->start_motor_(false);
  }
}

void JalouzeeBlinds::request_stop() {
  this->stop_motor_();
  if (this->cover_ != nullptr) {
    this->cover_->current_operation = cover::COVER_OPERATION_IDLE;
    this->cover_->publish_state();
  }
}

void JalouzeeBlinds::press_calibrate() {
  switch (this->calibrate_) {
    case CALIBRATION_NONE: {
      this->calibrated_ = false;
      if (this->calibrated_sensor_ != nullptr)
        this->calibrated_sensor_->publish_state(false);
      this->calibrate_ = CALIBRATION_STEP_1;
      ESP_LOGI(TAG, "Calibrating - step 1/2");
      ESP_LOGI(TAG, "Press DOWN, then STOP, when the tilt is fully closed");
      if (this->calibrate_message_sensor_ != nullptr)
        this->calibrate_message_sensor_->publish_state("Press DOWN, then STOP, when tilt is fully closed");
      break;
    }
    case CALIBRATION_STEP_1: {
      this->pos_low_ = this->angle_sensor_ != nullptr ? this->angle_sensor_->state : 0.0f;
      this->step_count_ = 0;
      this->curr_step_ = 1;
      this->calibrate_ = CALIBRATION_STEP_2;
      ESP_LOGI(TAG, "Calibrating - step 2/2");
      ESP_LOGI(TAG, "Press UP, then STOP, when the tilt is fully closed");
      if (this->calibrate_message_sensor_ != nullptr)
        this->calibrate_message_sensor_->publish_state("Press UP, then STOP, when tilt is fully closed");
      break;
    }
    case CALIBRATION_STEP_2: {
      this->pos_high_ = this->angle_sensor_ != nullptr ? this->angle_sensor_->state : 0.0f;
      this->step_total_ = this->step_count_;
      this->calibrate_ = CALIBRATION_NONE;
      ESP_LOGI(TAG, "Calibration finished: pos_low=%.2f pos_high=%.2f step_total=%.2f", this->pos_low_,
               this->pos_high_, this->step_total_);
      if (this->calibrate_message_sensor_ != nullptr)
        this->calibrate_message_sensor_->publish_state("Calibration finished");
      this->calibrated_ = true;
      if (this->calibrated_sensor_ != nullptr)
        this->calibrated_sensor_->publish_state(true);
      this->save_calibration_();
      break;
    }
  }
  if (this->calibrate_step_sensor_ != nullptr)
    this->calibrate_step_sensor_->publish_state((float) this->calibrate_);
}

void JalouzeeBlinds::set_use_angle_mode(bool enabled) {
  this->mode_ = enabled ? MODE_ANGLE : MODE_ENCODER;
  if (this->use_angle_switch_ != nullptr)
    this->use_angle_switch_->publish_state(enabled);
}

void JalouzeeBlinds::set_has_problem(bool problem) {
  this->has_problem_ = problem;
  if (this->has_problem_switch_ != nullptr)
    this->has_problem_switch_->publish_state(problem);
}

void JalouzeeBlinds::dump_config() {
  ESP_LOGCONFIG(TAG, "Jalouzee Blinds:");
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->mode_ == MODE_ANGLE ? "Angle sensor" : "Encoder");
  ESP_LOGCONFIG(TAG, "  Calibrated: %s", YESNO(this->calibrated_));
  LOG_PIN("  CW Pin: ", this->cw_pin_);
  LOG_PIN("  CCW Pin: ", this->ccw_pin_);
  LOG_PIN("  Phase A Pin: ", this->ph_a_pin_);
  LOG_PIN("  Phase B Pin: ", this->ph_b_pin_);
}

// ------------------------------------------------------------------------
// JalouzeeBlindsOutput (the exposed cover entity)
// ------------------------------------------------------------------------

cover::CoverTraits JalouzeeBlindsOutput::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  traits.set_supports_tilt(true);
  traits.set_supports_stop(true);
  traits.set_supports_toggle(false);
  return traits;
}

void JalouzeeBlindsOutput::control(const cover::CoverCall &call) {
  if (this->parent_ == nullptr)
    return;

  if (call.get_stop()) {
    this->parent_->request_stop();
    return;
  }
  if (call.get_tilt().has_value()) {
    this->parent_->request_tilt(*call.get_tilt());
    return;
  }
  if (call.get_position().has_value()) {
    float pos = *call.get_position();
    if (pos >= 0.5f) {
      this->parent_->request_open();
    } else {
      this->parent_->request_close();
    }
  }
}

// ------------------------------------------------------------------------
// Switches / button
// ------------------------------------------------------------------------

void UseAngleSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_use_angle_mode(state);
}

void HasProblemSwitch::write_state(bool state) {
  if (this->parent_ != nullptr)
    this->parent_->set_has_problem(state);
}

void CalibrateButton::press_action() {
  if (this->parent_ != nullptr)
    this->parent_->press_calibrate();
}

}  // namespace jalouzee_blinds
}  // namespace esphome
