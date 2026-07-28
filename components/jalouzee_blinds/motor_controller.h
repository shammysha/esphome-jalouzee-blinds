#pragma once

#include "esphome/core/hal.h"

namespace esphome {
namespace jalouzee_blinds {

enum MotorDirection : uint8_t {
  MOTOR_STOP = 0,
  MOTOR_OPENING = 1,
  MOTOR_CLOSING = 2,
};

// Wrapper around a DC motor on two pins (IN1/IN2), no PWM/speed encoder —
// direction is set via a digital_write combination, stop is both LOW.
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
    this->write_pins_(!this->inverted_);
  }

  void close() {
    this->dir_ = MOTOR_CLOSING;
    this->write_pins_(this->inverted_);
  }

  void stop() {
    this->dir_ = MOTOR_STOP;
    this->in1_pin_->digital_write(false);
    this->in2_pin_->digital_write(false);
  }

  MotorDirection direction() const { return this->dir_; }

  // Compensates for in1/in2 physically swapped at installation (pressing
  // open/close moves the blind the wrong way) — see JalouzeeBlinds::
  // detect_motor_polarity_(), which is the only writer. open()/close() keep
  // their normal meaning for every caller; only the actual GPIO pattern
  // underneath flips.
  void set_direction_inverted(bool inverted) { this->inverted_ = inverted; }
  bool is_direction_inverted() const { return this->inverted_; }

 protected:
  void write_pins_(bool forward) {
    this->in1_pin_->digital_write(forward);
    this->in2_pin_->digital_write(!forward);
  }

  GPIOPin *in1_pin_{nullptr};
  GPIOPin *in2_pin_{nullptr};
  MotorDirection dir_{MOTOR_STOP};
  bool inverted_{false};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
