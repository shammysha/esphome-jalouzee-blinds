#pragma once

#include <string>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/adc/adc_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Режим определения угла, который выбрал пользователь (хранится во flash)
enum AngleSourceMode : uint8_t {
  ANGLE_SOURCE_AUTO = 0,     // 1) MPU6050  2) Hall/ADC (по приоритету)
  ANGLE_SOURCE_MPU6050 = 1,  // принудительно MPU6050
  ANGLE_SOURCE_ENCODER = 2,  // принудительно Hall либо ADC (что задано в YAML)
};

// Реально используемый в данный момент источник (после разрешения приоритетов,
// доступности и калибровки)
enum ActiveAngleSource : uint8_t {
  ACTIVE_SOURCE_NONE = 0,
  ACTIVE_SOURCE_MPU6050 = 1,
  ACTIVE_SOURCE_HALL = 2,
  ACTIVE_SOURCE_ADC = 3,
};

enum CalibrationState : uint8_t {
  CAL_IDLE = 0,
  CAL_WAIT_CLOSED = 1,  // ждём, что пользователь выставит "закрыто" и нажмёт кнопку повторно
  CAL_WAIT_OPEN = 2,    // ждём "открыто"
};

enum MotorDirection : uint8_t {
  MOTOR_STOP = 0,
  MOTOR_OPENING = 1,
  MOTOR_CLOSING = 2,
};

class JalouzeeBlinds;

// ---------------------------------------------------------------------------
// Вспомогательные сущности, создаваемые компонентом автоматически
// (в YAML не конфигурируются, см. п. "Внутри компонента создаются автоматически")
// ---------------------------------------------------------------------------

class CalibrationButton : public button::Button {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;
  JalouzeeBlinds *parent_{nullptr};
};

class CancelCalibrationButton : public button::Button {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;
  JalouzeeBlinds *parent_{nullptr};
};

class FaultResetButton : public button::Button {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;
  JalouzeeBlinds *parent_{nullptr};
};

class AngleSourceSelect : public select::Select {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void control(const std::string &value) override;
  JalouzeeBlinds *parent_{nullptr};
};

class FaultTimeoutNumber : public number::Number {
 public:
  void set_parent(JalouzeeBlinds *parent) { this->parent_ = parent; }

 protected:
  void control(float value) override;
  JalouzeeBlinds *parent_{nullptr};
};

// ---------------------------------------------------------------------------
// Данные, сохраняемые во flash (NVS). Статус аварии сюда НЕ входит (п.8).
// ---------------------------------------------------------------------------
struct JalouzeeBlindsStore {
  bool hall_calibrated;
  bool adc_calibrated;
  bool mpu_calibrated;

  float hall_closed;  // "сырое" значение накопленных импульсов при закрытых ламелях
  float hall_open;
  float adc_closed;  // "сырое" значение АЦП при закрытых ламелях
  float adc_open;
  float mpu_closed;  // значение sensor'а MPU6050 при закрытых ламелях
  float mpu_open;

  uint8_t angle_source_mode;  // выбор пользователя: auto/mpu6050/encoder
  float last_angle_percent;   // последний известный угол наклона, 0..100%
} __attribute__((packed));

// ---------------------------------------------------------------------------
class JalouzeeBlinds : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  cover::CoverTraits get_traits() override;

  // --- сеттеры, вызываемые из codegen (cover.py) ---
  void set_motor_pins(GPIOPin *in1, GPIOPin *in2) {
    this->in1_pin_ = in1;
    this->in2_pin_ = in2;
  }
  void set_hall_encoder_pins(InternalGPIOPin *a, InternalGPIOPin *b);
  void set_adc_pin(InternalGPIOPin *pin) {
    this->adc_gpio_pin_ = pin;
    this->has_adc_ = true;
  }
  void set_mpu6050_sensor(sensor::Sensor *sens) {
    this->mpu_sensor_ = sens;
    this->has_mpu_ = true;
  }
  void set_angle_source_mode(uint8_t mode) { this->configured_angle_source_mode_ = mode; }
  void set_fault_timeout(uint32_t seconds) { this->fault_timeout_s_ = seconds; }

  // --- вызовы от вложенных сущностей (кнопки/select/number) ---
  void on_calibration_button_pressed();
  void on_cancel_calibration_button_pressed();
  void on_fault_reset_button_pressed();
  void on_angle_source_select_changed(const std::string &value);
  void on_fault_timeout_changed(float seconds);

 protected:
  void control(const cover::CoverCall &call) override;

  // --- инициализация вложенных сущностей ---
  void register_sub_entities_();

  // --- мотор ---
  void motor_open_();
  void motor_close_();
  void motor_stop_();

  // --- источники угла ---
  ActiveAngleSource resolve_active_source_();
  bool is_source_calibrated_(ActiveAngleSource src);
  bool is_source_available_(ActiveAngleSource src);
  float read_raw_(ActiveAngleSource src);
  // сырое значение -> проценты 0..100 по калибровочным точкам источника
  float raw_to_percent_(ActiveAngleSource src, float raw);
  float read_adc_raw_();

  // --- калибровка ---
  void enter_calibration_();
  void calibration_step_();
  void cancel_calibration_();
  void capture_calibration_point_(bool is_closed_point);
  void finish_calibration_();
  void set_calibration_message_(const std::string &msg, bool temporary = false);

  // --- оппортунистическая автокалибровка отклонённых источников ---
  // Когда жалюзи реально доходят до 0%/100% (по уже доверенному активному
  // источнику) в обычном режиме работы, ловим эту же точку для любого
  // доступного, но ещё не откалиброванного источника (например, MPU, чья
  // ручная калибровка была отклонена из-за отсутствия движения) — так он
  // сможет самостоятельно доехать до calibrated=true, если его позже
  // физически восстановят, без повторного ручного прохода.
  void try_auto_calibrate_at_endpoint_(bool is_closed_point);
  void auto_calibrate_capture_(ActiveAngleSource src, bool is_closed_point, float raw);
  void update_calibrated_binary_sensor_();

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

  // --- энкодер Холла (ISR) ---
  static void hall_isr_(JalouzeeBlinds *arg);

  // ------------------------- поля -------------------------
  GPIOPin *in1_pin_{nullptr};
  GPIOPin *in2_pin_{nullptr};
  InternalGPIOPin *encoder_a_pin_{nullptr};
  InternalGPIOPin *encoder_b_pin_{nullptr};
  // ISR-safe копия encoder_b_pin_ (см. hall_isr_) — обычный InternalGPIOPin::digital_read()
  // это виртуальный вызов и не гарантированно безопасен из прерывания.
  ISRInternalGPIOPin encoder_b_isr_;
  InternalGPIOPin *adc_gpio_pin_{nullptr};
  // Внутренний экземпляр штатного ADC-сенсора ESPHome (ESP-IDF adc_oneshot
  // драйвер). Не регистрируется в App (нет периодического update()) — читаем
  // значение вручную через sample() когда нужно (см. read_adc_raw_()).
  adc::ADCSensor *adc_sensor_{nullptr};
  sensor::Sensor *mpu_sensor_{nullptr};

  bool has_hall_{false};
  bool has_adc_{false};
  bool has_mpu_{false};

  volatile int32_t hall_pulse_count_{0};

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
  MotorDirection motor_dir_{MOTOR_STOP};
  bool jog_mode_{false};  // true = ручной jog во время калибровки (без цели/без проверки аварии)
  float target_percent_{NAN};
  float current_percent_{0};
  int8_t current_step_index_{0};  // 0=закрыто, 1=50%, 2=открыто

  uint32_t last_flash_save_ms_{0};

  JalouzeeBlindsStore store_{};
  ESPPreferenceObject pref_;

  // Backing storage for dynamically-built entity names: configure_entity_() only
  // stores a StringRef (no copy), so these must outlive the entities themselves.
  std::vector<std::string> entity_name_storage_;

  // вложенные сущности (владеет ими данный компонент)
  CalibrationButton *calibration_button_{nullptr};
  CancelCalibrationButton *cancel_calibration_button_{nullptr};
  FaultResetButton *fault_reset_button_{nullptr};
  AngleSourceSelect *angle_source_select_{nullptr};
  FaultTimeoutNumber *fault_timeout_number_{nullptr};
  text_sensor::TextSensor *calibration_text_sensor_{nullptr};
  binary_sensor::BinarySensor *calibrated_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *fault_binary_sensor_{nullptr};

  friend class CalibrationButton;
  friend class CancelCalibrationButton;
  friend class FaultResetButton;
  friend class AngleSourceSelect;
  friend class FaultTimeoutNumber;
};

}  // namespace jalouzee_blinds
}  // namespace esphome
