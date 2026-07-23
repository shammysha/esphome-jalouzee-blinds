#include <cmath>
#include "jalouzee_blinds.h"
#include "esphome/core/log.h"

namespace esphome {
namespace jalouzee_blinds {

static const char *const TAG = "jalouzee_blinds";

static const char *const MSG_ENTER_CALIBRATION = "Enter calibration mode";
static const char *const MSG_WAIT_CLOSED =
    "Move the slats to the fully closed position and press the button again";
static const char *const MSG_WAIT_OPEN =
    "Move the slats to the fully open position and press the button again";
static const char *const MSG_DONE = "Calibration complete";

static const float FAULT_ANGLE_EPSILON = 0.5f;    // % — minimum angle change to not count as "stuck"
static const float STEP_TARGET_EPSILON = 0.5f;    // % — reaching the target position
// We write rarely while idle — manual intervention isn't expected in this
// project, so position drift can only accumulate between visits, not mid-
// movement. Protection against a power loss MID-movement is handled
// separately and precisely — via store_.movement_in_progress (see
// start_move_to_percent_/setup()), not via more frequent timer-based saves.
static const uint32_t FLASH_SAVE_MIN_INTERVAL_MS = 300000;  // 5 min
// publish_state() during movement, no more often than this interval —
// without throttling it would fire on every loop() iteration (hundreds to
// thousands of times per second), flooding the API connection and
// interfering with incoming commands (see the lag discussion).
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

  // if the user hasn't overridden it via the codegen setter — take the value from the YAML config
  if (this->store_.angle_source_mode == 0 && this->configured_angle_source_mode_ != ANGLE_SOURCE_AUTO) {
    this->store_.angle_source_mode = this->configured_angle_source_mode_;
  }

  this->current_percent_ = this->store_.last_angle_percent;

  // Restore the Hall pulse counter from the last saved position — otherwise
  // after a reboot it would start at 0, losing its link to the
  // hall_closed/hall_open calibration points. Must happen BEFORE
  // hall_adc_.setup() (that's where the interrupts get attached).
  if (this->store_.hall_calibrated) {
    float seed = this->angle_cal_.percent_to_raw(ACTIVE_SOURCE_HALL, this->store_.last_angle_percent);
    if (!std::isnan(seed)) {
      this->hall_adc_.seed_hall_pulse_count(static_cast<int32_t>(lroundf(seed)));
    }
  }
  this->hall_adc_.setup();

  // --- point 2: movement was interrupted by a power loss
  // (movement_in_progress wasn't cleared by a normal completion — see
  // store.h) --- the Hall position restored above is untrustworthy in this
  // case: we don't know how much actually happened since the last save. ADC
  // is unaffected — it's an absolute sensor (the current voltage IS the
  // current position right now).
  bool movement_interrupted = this->store_.movement_in_progress;
  if (movement_interrupted) {
    this->store_.movement_in_progress = false;
    this->save_to_flash_();
  }
  if (movement_interrupted && this->hall_adc_.has_hall()) {
    // Gate Hall regardless of whether an MPU fallback exists —
    // hall_untrusted_ is not conflated with operation_blocked_ (which only
    // means "no source at all, control is blocked"), otherwise in "encoder"
    // mode (no auto-switch to MPU) resolve_active_source() wouldn't get the
    // distrust signal and would keep trusting Hall.
    this->hall_untrusted_ = true;
    bool mpu_ok = this->mpu_.has_mpu() && this->store_.mpu_calibrated;
    if (mpu_ok) {
      ESP_LOGW(TAG, "Detected a movement interrupted by a power loss. The Hall position is untrustworthy — "
                     "temporarily (for this session) using MPU6050 as the angle source.");
      // nothing else to do — resolve_active_source() will itself prioritize
      // MPU and won't use Hall until it's reconfirmed by calibration.
      // Implemented via hall_untrusted_.
    } else {
      ESP_LOGW(TAG, "Detected a movement interrupted by a power loss, and the backup MPU6050 is "
                     "unavailable/uncalibrated. Blind control is blocked until calibration.");
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
  ESP_LOGCONFIG(TAG, "  Motor: DC motor, IN1/IN2 set");
  if (this->hall_adc_.has_hall()) {
    ESP_LOGCONFIG(TAG, "  Hall encoder: 7 PPR x gear ratio, A/B set");
  }
  if (this->hall_adc_.has_adc()) {
    ESP_LOGCONFIG(TAG, "  Resistor on the motor shaft: ADC pin set (ESPHome's built-in ADC component)");
  }
  if (this->mpu_.has_mpu()) {
    ESP_LOGCONFIG(TAG, "  MPU6050: using an external sensor");
  }
  ESP_LOGCONFIG(TAG, "  Angle source mode (saved): %u", this->store_.angle_source_mode);
  ESP_LOGCONFIG(TAG, "  Fault timeout: %lu s", this->fault_timeout_s_);
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

  // expiry of the temporary calibration message ("Calibration complete")
  if (this->cal_message_is_temporary_ && now > this->cal_message_expire_ms_) {
    this->cal_message_is_temporary_ = false;
    this->set_calibration_message_(this->cal_state_ == CAL_IDLE ? MSG_ENTER_CALIBRATION
                                                                  : this->sub_entities_.calibration_message_state());
  }

  // recompute the current angle (if there's at least one working calibrated source)
  ActiveAngleSource src = this->angle_cal_.resolve_active_source(this->hall_untrusted_);
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
    // manual jog during calibration — no target, no fault check
  } else if (this->motor_.direction() != MOTOR_STOP) {
    this->check_fault_();
    if (!this->fault_active_) {
      this->handle_movement_();
    }
  }

  // periodic save of the current angle (no more often than once per FLASH_SAVE_MIN_INTERVAL_MS)
  if (this->motor_.direction() == MOTOR_STOP && (now - this->last_flash_save_ms_) > FLASH_SAVE_MIN_INTERVAL_MS) {
    if (fabsf(this->current_percent_ - this->store_.last_angle_percent) > 0.5f) {
      this->store_.last_angle_percent = this->current_percent_;
      this->save_to_flash_();
      this->last_flash_save_ms_ = now;
    }
  }
}

// =====================================================================
// Calibration
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
  ESP_LOGI(TAG, "Starting calibration");
  this->motor_.stop();
  this->target_percent_ = NAN;
  this->clear_movement_in_progress_();
  this->jog_mode_ = true;
  this->cal_state_ = CAL_WAIT_CLOSED;
  this->set_calibration_message_(MSG_WAIT_CLOSED);
  // 50% keeps both arrows (up/down) active in HA during calibration — the
  // real position isn't calibrated yet; we report it back in finish/cancel.
  this->position = 0.5f;
  this->publish_state();
}

void JalouzeeBlinds::capture_calibration_point_(bool is_closed_point) {
  if (is_closed_point) {
    if (this->hall_adc_.has_hall()) this->temp_hall_closed_ = this->hall_adc_.read_hall_raw();
    if (this->hall_adc_.has_adc()) this->temp_adc_closed_ = this->hall_adc_.read_adc_raw();
    if (this->mpu_.has_mpu()) this->temp_mpu_closed_ = this->mpu_.read_raw();
  }
  // the "open" point is handled right away in finish_calibration_()
}

void JalouzeeBlinds::finish_calibration_() {
  // try to accept calibration for ALL available sensors, but only if
  // movement was actually detected — otherwise the source stays
  // uncalibrated (see Controller::try_finish_calibration).
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
  this->hall_untrusted_ = false;
  this->cal_state_ = CAL_IDLE;
  this->jog_mode_ = false;
  this->motor_.stop();

  // the "open" point was just captured — the current physical position IS that point
  this->current_percent_ = 100.0f;
  this->store_.last_angle_percent = this->current_percent_;
  this->position = this->current_percent_ / 100.0f;
  this->publish_state();

  this->save_to_flash_();
  this->publish_calibration_diagnostics_();
  this->set_calibration_message_(MSG_DONE, /*temporary=*/true);

  ESP_LOGI(TAG, "Calibration complete and saved");
}

void JalouzeeBlinds::on_cancel_calibration_button_pressed() {
  if (this->cal_state_ == CAL_IDLE) return;  // not available outside calibration
  this->cancel_calibration_();
}

void JalouzeeBlinds::cancel_calibration_() {
  ESP_LOGI(TAG, "Calibration cancelled by the user");
  this->cal_state_ = CAL_IDLE;
  this->jog_mode_ = false;
  this->motor_.stop();
  this->set_calibration_message_(MSG_ENTER_CALIBRATION);
  // restore the real (last known) position instead of the forced 50%
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
// Fault
// =====================================================================
void JalouzeeBlinds::check_fault_() {
  uint32_t now = millis();
  if ((now - this->last_angle_change_ms_) > (this->fault_timeout_s_ * 1000UL)) {
    this->trigger_fault_();
  }
}

void JalouzeeBlinds::trigger_fault_() {
  if (this->fault_active_) return;
  ESP_LOGE(TAG, "FAULT: the tilt angle hasn't changed for over %lu s while the motor is actively moving", this->fault_timeout_s_);
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
  ESP_LOGI(TAG, "Fault status reset by the user");
  this->fault_active_ = false;
  this->last_angle_change_ms_ = millis();
  this->last_seen_percent_for_fault_ = NAN;
  this->sub_entities_.set_fault(false);
}

// =====================================================================
// Select / Number handlers
// =====================================================================
void JalouzeeBlinds::on_angle_source_select_changed(const std::string &value) {
  uint8_t mode = ANGLE_SOURCE_AUTO;
  if (value == "angle") mode = ANGLE_SOURCE_MPU6050;
  else if (value == "encoder") mode = ANGLE_SOURCE_ENCODER;

  this->angle_cal_.set_mode(mode);
  this->save_to_flash_();
  this->sub_entities_.set_angle_source_state(value);
  ESP_LOGI(TAG, "Angle source mode changed by the user: %s", value.c_str());
}

void JalouzeeBlinds::on_fault_timeout_changed(float seconds) {
  if (seconds < 1) seconds = 1;
  this->fault_timeout_s_ = static_cast<uint32_t>(seconds);
  this->sub_entities_.set_fault_timeout_state(this->fault_timeout_s_);
  ESP_LOGI(TAG, "Fault timeout changed: %lu s", this->fault_timeout_s_);
}

// =====================================================================
// Blind control (cover::Cover::control)
// =====================================================================
void JalouzeeBlinds::control(const cover::CoverCall &call) {
  if (this->cal_state_ != CAL_IDLE) {
    // In calibration mode: open/close act as a manual "up/down" jog, stop
    // stops the motor. The calibration button captures the points.
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

  // Block control if there's no calibrated and currently available angle
  // source — this covers both a fully uncalibrated device (after first
  // flashing) and a detected interrupted movement (see setup()).
  if (this->angle_cal_.resolve_active_source(this->hall_untrusted_) == ACTIVE_SOURCE_NONE) {
    ESP_LOGW(TAG, "Blind control is blocked: no calibrated angle source. "
                   "Run calibration.");
    return;
  }
  if (this->fault_active_) {
    ESP_LOGW(TAG, "Blind control is blocked: a fault is active. Clear it with the fault reset button.");
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

    // Explicit 0 / 1 values (plain open()/close() without a slider) — go
    // through the stepped "closed -> 50% -> open" logic.
    if (pct <= 0.5f) {
      this->handle_open_close_request_(false);
    } else if (pct >= 99.5f) {
      this->handle_open_close_request_(true);
    } else {
      // an arbitrary value (e.g. from a slider in HA) — move straight there
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
    return;  // already there — no real movement happened, no need to write to flash
  }

  // We actually started moving — record this to flash so that after a
  // reboot we can reliably tell whether the movement was interrupted by a
  // power loss (see setup()).
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

    // We actually reached the end of travel — opportunistic auto-
    // calibration of any available but not-yet-calibrated sources (see
    // Controller).
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
    // Throttled — without this, publish_state() would fire on every loop()
    // iteration during movement, flooding the API connection (see
    // POSITION_PUBLISH_INTERVAL_MS).
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
  // For UNcalibrated sources, closed/open must be NAN (not 0.0 from zero-
  // init/stale data), otherwise auto-calibration would wrongly conclude
  // that one of the points was already captured.
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
