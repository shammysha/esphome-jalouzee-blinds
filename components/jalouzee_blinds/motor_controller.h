#pragma once

#include "esphome/core/hal.h"

namespace esphome {
namespace jalouzee_blinds {

enum MotorDirection : uint8_t {
  MOTOR_STOP = 0,
  MOTOR_OPENING = 1,
  MOTOR_CLOSING = 2,
};

// Обёртка над DC-мотором на двух пинах (IN1/IN2) без ШИМ/энкодера скорости —
// направление задаётся комбинацией digital_write, останов — оба LOW.
class MotorController {
 public:
  void set_pins(GPIOPin *in1, GPIOPin *in2) {
    this->in1_pin_ = in1;
    this->in2_pin_ = in2;
  }

  void setup() {
    this->in1_pin_->setup();
    this->in2_pin_->setup();
    this->in1_pin_->digital_write(false);
    this->in2_pin_->digital_write(false);
  }

  void open() {
    this->dir_ = MOTOR_OPENING;
    this->in1_pin_->digital_write(true);
    this->in2_pin_->digital_write(false);
  }

  void close() {
    this->dir_ = MOTOR_CLOSING;
    this->in1_pin_->digital_write(false);
    this->in2_pin_->digital_write(true);
  }

  void stop() {
    this->dir_ = MOTOR_STOP;
    this->in1_pin_->digital_write(false);
    this->in2_pin_->digital_write(false);
  }

  MotorDirection direction() const { return this->dir_; }

 protected:
  GPIOPin *in1_pin_{nullptr};
  GPIOPin *in2_pin_{nullptr};
  MotorDirection dir_{MOTOR_STOP};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
