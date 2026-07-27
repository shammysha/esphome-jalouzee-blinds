#pragma once

#include <string>
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "store.h"
#include "motor_controller.h"
#include "motor_sensor.h"
#include "mpu_sensor.h"
#include "controller.h"
#include "sub_entities.h"
#include "ble_relay.h"

namespace esphome {
namespace jalouzee_blinds {

enum CalibrationState : uint8_t {
  CAL_IDLE = 0,
  CAL_WAIT_CLOSED = 1,  // waiting for the user to set "closed" and press the button again
  CAL_WAIT_OPEN = 2,    // waiting for "open"
};

// ---------------------------------------------------------------------------
// General logic: the Cover entity, calibration state, fault detection,
// stepped movement (closed->50%->open), flash persistence. Owns the motor/
// sensors/calibration/nested entities and wires them together.
// ---------------------------------------------------------------------------
class JalouzeeBlinds : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  cover::CoverTraits get_traits() override;

  // --- setters called from codegen (cover.py) ---
  void set_motor_pins(GPIOPin *in1, GPIOPin *in2) { this->motor_.set_pins(in1, in2); }
  void set_hall_encoder_pins(InternalGPIOPin *a, InternalGPIOPin *b) { this->hall_adc_.set_hall_pins(a, b); }
  void set_adc_pin(InternalGPIOPin *pin) { this->hall_adc_.set_adc_pin(pin); }
  void set_mpu6050_sensor(sensor::Sensor *sens) { this->mpu_.set_sensor(sens); }
  void set_angle_source_mode(uint8_t mode) { this->configured_angle_source_mode_ = mode; }
  void set_fault_timeout(uint32_t seconds) { this->fault_timeout_s_ = seconds; }
#ifdef USE_ESP32
  void set_ble_relay_enabled(bool enabled) { this->ble_relay_enabled_ = enabled; }
  void set_net_key(std::array<uint8_t, 16> key) { this->ble_relay_.set_net_key(key); }
#endif

  // --- calls from nested entities (buttons/select/number), see sub_entities.h ---
  void on_calibration_button_pressed();
  void on_cancel_calibration_button_pressed();
  void on_fault_reset_button_pressed();
  void on_angle_source_select_changed(const std::string &value);
  void on_fault_timeout_changed(float seconds);

#ifdef USE_ESP32
  // Read by ble_relay.cpp's broadcast_own_status_() -- an uncalibrated
  // blind's `position` is meaningless (never actually measured, just
  // whatever it defaulted/was last saved to), so it must not be flooded
  // to the fleet as if it were a real reading. See docs/plans/
  // vivid-noodling-gray.md's position-collection design.
  bool is_calibrated() const { return this->angle_cal_.is_any_calibrated(); }
  // Also read by broadcast_own_status_() -- lets the app tell "this
  // broadcast reflects a blind actually at rest" apart from "this is just
  // the routine periodic tick firing while it happens to still be
  // mid-movement" (the periodic broadcast doesn't know or care whether
  // anything is moving, it fires on a plain timer). See main.dart's
  // _isSettled, which only trusts a broadcast as confirming settlement if
  // this is false. Only whether it's moving at all matters here, not which
  // direction.
  bool is_moving() const { return this->motor_.direction() != MOTOR_STOP; }
  // Also read by broadcast_own_status_() -- lets the app show a warning
  // icon for a blind whose control() is currently blocked by
  // trigger_fault_() (see check_fault_()'s doc comment), and offer to clear
  // it (COVER_CMD_RESET_FAULT -> on_fault_reset_button_pressed()) without
  // the user having no way to even find out why a blind stopped
  // responding, short of the HA-only fault binary sensor.
  bool is_fault_active() const { return this->fault_active_; }
#endif

 protected:
  void control(const cover::CoverCall &call) override;

  // --- calibration ---
  void enter_calibration_();
  void cancel_calibration_();
  void capture_calibration_point_(bool is_closed_point);
  void finish_calibration_();
  void set_calibration_message_(const std::string &msg, bool temporary = false);
  // Publishes "Calibrated" and the per-source diagnostic binary sensors —
  // call on any change to the calibration data.
  void publish_calibration_diagnostics_();

  // --- movement toward the target / 3-position logic ---
  void handle_open_close_request_(bool opening);
  void start_move_to_percent_(float target_percent);
  void handle_movement_();

  // --- fault ---
  void check_fault_();
  void trigger_fault_();
  void clear_fault_();

  // --- flash ---
  void save_to_flash_();
  void load_from_flash_();
  // Clears store_.movement_in_progress (if it was true) and saves — call on
  // ANY normal end of movement (explicit stop, fault, entering calibration),
  // except "target reached" in handle_movement_(), which already
  // unconditionally writes to flash. See store.h.
  void clear_movement_in_progress_();

  // ------------------------- components -------------------------
  MotorController motor_;
  MotorSensor hall_adc_;
  MpuSensor mpu_;
  Controller angle_cal_;
  SubEntities sub_entities_;
#ifdef USE_ESP32
  BleRelay ble_relay_;
  bool ble_relay_enabled_{false};
#endif

  uint8_t configured_angle_source_mode_{ANGLE_SOURCE_AUTO};
  uint32_t fault_timeout_s_{10};

  // calibration
  CalibrationState cal_state_{CAL_IDLE};
  float temp_hall_closed_{0};
  float temp_adc_closed_{0};
  float temp_mpu_closed_{0};
  uint32_t cal_message_expire_ms_{0};
  bool cal_message_is_temporary_{false};

  // fault
  bool fault_active_{false};
  uint32_t last_angle_change_ms_{0};
  float last_seen_percent_for_fault_{NAN};

  // after a power loss with no valid source
  bool operation_blocked_{false};
  // Hall is not considered reliable this session — see
  // resolve_active_source()/setup(). Unlike operation_blocked_ (no source at
  // all — block control), this can be true even when MPU6050 is available as
  // a fallback and control is allowed — otherwise in "encoder" mode (no
  // auto-switch to MPU) resolve_active_source() would keep trusting the
  // untrusted Hall.
  bool hall_untrusted_{false};

  // movement
  bool jog_mode_{false};  // true = manual jog during calibration (no target/no fault check)
  float target_percent_{NAN};
  float current_percent_{0};
  int8_t current_step_index_{0};  // 0=closed, 1=50%, 2=open
  uint32_t last_position_publish_ms_{0};  // throttles publish_state() during movement — see handle_movement_()

  uint32_t last_flash_save_ms_{0};

  JalouzeeBlindsStore store_{};
  ESPPreferenceObject pref_;
};

}  // namespace jalouzee_blinds
}  // namespace esphome
