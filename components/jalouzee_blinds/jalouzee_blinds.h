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

namespace esphome {
namespace jalouzee_blinds {

enum CalibrationState : uint8_t {
  CAL_IDLE = 0,
  CAL_WAIT_CLOSED = 1,  // ждём, что пользователь выставит "закрыто" и нажмёт кнопку повторно
  CAL_WAIT_OPEN = 2,    // ждём "открыто"
};

// ---------------------------------------------------------------------------
// Общая логика: Cover-сущность, состояние калибровки, детекция аварии,
// пошаговое движение (закрыто->50%->открыто), flash-персистентность.
// Владеет мотором/датчиками/калибровкой/вложенными сущностями и связывает их.
// ---------------------------------------------------------------------------
class JalouzeeBlinds : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  cover::CoverTraits get_traits() override;

  // --- сеттеры, вызываемые из codegen (cover.py) ---
  void set_motor_pins(GPIOPin *in1, GPIOPin *in2) { this->motor_.set_pins(in1, in2); }
  void set_hall_encoder_pins(InternalGPIOPin *a, InternalGPIOPin *b) { this->hall_adc_.set_hall_pins(a, b); }
  void set_adc_pin(InternalGPIOPin *pin) { this->hall_adc_.set_adc_pin(pin); }
  void set_mpu6050_sensor(sensor::Sensor *sens) { this->mpu_.set_sensor(sens); }
  void set_angle_source_mode(uint8_t mode) { this->configured_angle_source_mode_ = mode; }
  void set_fault_timeout(uint32_t seconds) { this->fault_timeout_s_ = seconds; }

  // --- вызовы от вложенных сущностей (кнопки/select/number), см. sub_entities.h ---
  void on_calibration_button_pressed();
  void on_cancel_calibration_button_pressed();
  void on_fault_reset_button_pressed();
  void on_angle_source_select_changed(const std::string &value);
  void on_fault_timeout_changed(float seconds);

 protected:
  void control(const cover::CoverCall &call) override;

  // --- калибровка ---
  void enter_calibration_();
  void cancel_calibration_();
  void capture_calibration_point_(bool is_closed_point);
  void finish_calibration_();
  void set_calibration_message_(const std::string &msg, bool temporary = false);

  // --- движение к цели / логика 3 положений ---
  void handle_open_close_request_(bool opening);
  void start_move_to_percent_(float target_percent);
  void handle_movement_();

  // --- авария ---
  void check_fault_();
  void trigger_fault_();
  void clear_fault_();

  // --- flash ---
  void save_to_flash_();
  void load_from_flash_();

  // ------------------------- компоненты -------------------------
  MotorController motor_;
  MotorSensor hall_adc_;
  MpuSensor mpu_;
  Controller angle_cal_;
  SubEntities sub_entities_;

  uint8_t configured_angle_source_mode_{ANGLE_SOURCE_AUTO};
  uint32_t fault_timeout_s_{10};

  // калибровка
  CalibrationState cal_state_{CAL_IDLE};
  float temp_hall_closed_{0};
  float temp_adc_closed_{0};
  float temp_mpu_closed_{0};
  uint32_t cal_message_expire_ms_{0};
  bool cal_message_is_temporary_{false};

  // авария
  bool fault_active_{false};
  uint32_t last_angle_change_ms_{0};
  float last_seen_percent_for_fault_{NAN};

  // после потери питания без валидного источника
  bool operation_blocked_{false};

  // движение
  bool jog_mode_{false};  // true = ручной jog во время калибровки (без цели/без проверки аварии)
  float target_percent_{NAN};
  float current_percent_{0};
  int8_t current_step_index_{0};  // 0=закрыто, 1=50%, 2=открыто

  uint32_t last_flash_save_ms_{0};

  JalouzeeBlindsStore store_{};
  ESPPreferenceObject pref_;
};

}  // namespace jalouzee_blinds
}  // namespace esphome
