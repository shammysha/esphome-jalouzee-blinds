#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"

namespace esphome {
namespace tilt_cover {

enum CalibrationStep : uint8_t {
  CALIBRATION_NONE = 0,
  CALIBRATION_STEP_1 = 1,
  CALIBRATION_STEP_2 = 2,
};

enum PositionMode : uint8_t {
  MODE_ENCODER = 0,
  MODE_ANGLE = 1,
};

// Values persisted to flash between reboots (mirrors the `restore_value: yes`
// globals in the original YAML). Written through ESPHome's normal deferred
// preferences flush, so set `preferences: { flash_write_interval: ... }` in
// your main YAML as usual.
struct CalibrationData {
  float pos_low;
  float pos_high;
  float step_total;
  float curr_step;
};

class TiltCoverOutput;
class UseAngleSwitch;
class HasProblemSwitch;
class CalibrateButton;

// The hub. Owns the motor/encoder pins, the (optional) angle sensor
// reference, and all of the control/calibration logic. Child entities
// (cover, binary_sensor, sensor, text_sensor, switches, button) register
// themselves here and the hub pushes state updates out to them.
class TiltCover : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // --- wiring, set from Python codegen ---
  void set_cw_pin(GPIOPin *pin) { cw_pin_ = pin; }
  void set_ccw_pin(GPIOPin *pin) { ccw_pin_ = pin; }
  void set_ph_a_pin(GPIOPin *pin) { ph_a_pin_ = pin; }
  void set_ph_b_pin(GPIOPin *pin) { ph_b_pin_ = pin; }
  void set_angle_sensor(sensor::Sensor *sens) { angle_sensor_ = sens; }

  // --- child entities, set from Python codegen ---
  void set_cover(TiltCoverOutput *c) { cover_ = c; }
  void set_calibrated_binary_sensor(binary_sensor::BinarySensor *s) { calibrated_sensor_ = s; }
  void set_calibrate_step_sensor(sensor::Sensor *s) { calibrate_step_sensor_ = s; }
  void set_calibrate_message_sensor(text_sensor::TextSensor *s) { calibrate_message_sensor_ = s; }
  void set_use_angle_switch(UseAngleSwitch *s) { use_angle_switch_ = s; }
  void set_has_problem_switch(HasProblemSwitch *s) { has_problem_switch_ = s; }

  // --- called by the cover entity ---
  void request_open();
  void request_close();
  void request_tilt(float tilt);
  void request_stop();

  // --- called by the switch/button entities ---
  void press_calibrate();
  void set_use_angle_mode(bool enabled);
  void set_has_problem(bool problem);

  bool is_calibrated() const { return calibrated_; }
  bool has_problem() const { return has_problem_; }
  float current_value() const { return last_published_value_; }
  float current_tilt_target() const { return curr_tilt_; }

 protected:
  void initialize_();
  void handle_encoder_pulse_(bool is_a_phase);
  float compute_angle_value_(float raw);
  float compute_rotary_value_();
  void update_position_();
  void update_stall_detection_(float sens);
  void start_motor_(bool clockwise);
  void stop_motor_();
  void save_calibration_();
  void load_calibration_();

  GPIOPin *cw_pin_{nullptr};
  GPIOPin *ccw_pin_{nullptr};
  GPIOPin *ph_a_pin_{nullptr};
  GPIOPin *ph_b_pin_{nullptr};
  sensor::Sensor *angle_sensor_{nullptr};

  TiltCoverOutput *cover_{nullptr};
  binary_sensor::BinarySensor *calibrated_sensor_{nullptr};
  sensor::Sensor *calibrate_step_sensor_{nullptr};
  text_sensor::TextSensor *calibrate_message_sensor_{nullptr};
  UseAngleSwitch *use_angle_switch_{nullptr};
  HasProblemSwitch *has_problem_switch_{nullptr};

  ESPPreferenceObject pref_;

  // runtime state - mirrors the `globals:` block in the original YAML
  PositionMode mode_{MODE_ENCODER};
  float pos_low_{0.0f};
  float pos_high_{0.0f};
  float step_total_{0.0f};
  float curr_step_{0.0f};
  float step_count_{0.0f};
  float curr_tilt_{0.0f};
  float old_state_{0.0f};
  float last_published_value_{NAN};
  bool calibrated_{false};
  bool has_problem_{false};
  bool cw_active_{false};
  bool ccw_active_{false};
  bool last_ph_a_{false};
  bool last_ph_b_{false};
  CalibrationStep calibrate_{CALIBRATION_NONE};
  uint8_t cycle_time_{0};
  uint32_t last_update_ms_{0};
};

// Thin Cover wrapper - all logic delegates to the TiltCover hub.
class TiltCoverOutput : public cover::Cover, public Component {
 public:
  void set_parent(TiltCover *parent) { parent_ = parent; }
  void setup() override {}
  float get_setup_priority() const override { return setup_priority::DATA; }
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;

 protected:
  TiltCover *parent_{nullptr};
};

class UseAngleSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(TiltCover *parent) { parent_ = parent; }
  void setup() override {}

 protected:
  void write_state(bool state) override;
  TiltCover *parent_{nullptr};
};

class HasProblemSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(TiltCover *parent) { parent_ = parent; }
  void setup() override {}

 protected:
  void write_state(bool state) override;
  TiltCover *parent_{nullptr};
};

class CalibrateButton : public button::Button, public Component {
 public:
  void set_parent(TiltCover *parent) { parent_ = parent; }
  void setup() override {}

 protected:
  void press_action() override;
  TiltCover *parent_{nullptr};
};

}  // namespace tilt_cover
}  // namespace esphome
