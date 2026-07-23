#pragma once

#include "esphome/core/hal.h"
#include "esphome/components/adc/adc_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Sensors physically located on/near the motor shaft: a quadrature Hall
// encoder (7 PPR x gear ratio) and/or a resistor on the shaft, read via
// ESPHome's built-in ADC component. Both are mutually exclusive ways of
// determining the angle via an encoder/resistor on the motor's fast shaft
// (see Controller).
class MotorSensor {
 public:
  void set_hall_pins(InternalGPIOPin *a, InternalGPIOPin *b) {
    this->encoder_a_pin_ = a;
    this->encoder_b_pin_ = b;
    this->has_hall_ = true;
  }
  void set_adc_pin(InternalGPIOPin *pin) {
    this->adc_gpio_pin_ = pin;
    this->has_adc_ = true;
  }

  bool has_hall() const { return this->has_hall_; }
  bool has_adc() const { return this->has_adc_; }

  // Restores the pulse counter from the value saved to flash (see
  // JalouzeeBlinds::setup()) — otherwise after a reboot the counter would
  // start at 0, losing its link to the closed/open calibration points. Must
  // be called BEFORE setup().
  void seed_hall_pulse_count(int32_t value) { this->hall_pulse_count_ = value; }

  void setup();

  float read_hall_raw() const { return static_cast<float>(this->hall_pulse_count_); }
  float read_adc_raw();

 protected:
  static void hall_isr_(MotorSensor *arg);

  InternalGPIOPin *encoder_a_pin_{nullptr};
  InternalGPIOPin *encoder_b_pin_{nullptr};
  // ISR-safe copies of encoder_a_pin_/encoder_b_pin_ (see hall_isr_) — a
  // regular InternalGPIOPin::digital_read() is a virtual call and not
  // guaranteed to be ISR-safe.
  ISRInternalGPIOPin encoder_a_isr_;
  ISRInternalGPIOPin encoder_b_isr_;
  volatile int32_t hall_pulse_count_{0};
  // Current 2-bit state (A<<1|B) for the full (4x) quadrature decode in
  // hall_isr_ — see .cpp.
  volatile uint8_t hall_last_state_{0};

  InternalGPIOPin *adc_gpio_pin_{nullptr};
  // Internal instance of ESPHome's built-in ADC sensor (the driver depends
  // on the platform). Not registered with App (no periodic update()) — we
  // read the value manually via sample() when needed (see read_adc_raw()).
  adc::ADCSensor *adc_sensor_{nullptr};

  bool has_hall_{false};
  bool has_adc_{false};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
