#include <cmath>
#include "motor_sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Transition table for the full (4x) quadrature decode of the Hall sensor
// (A/B). Index — (previous_state << 2) | new_state, where state = (A<<1)|B.
// Value — count direction: 0 for "impossible" transitions (missed edge/
// bounce), ±1 for valid adjacent Gray-code transitions. More reliable than
// guessing direction from the neighboring channel's level at interrupt
// time — see hall_isr_().
static const int8_t HALL_QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,   //
    1, 0, 0, -1,   //
    -1, 0, 0, 1,   //
    0, 1, -1, 0,   //
};

void MotorSensor::setup() {
  if (this->has_hall_) {
    this->encoder_a_pin_->setup();
    this->encoder_b_pin_->setup();
    this->encoder_a_isr_ = this->encoder_a_pin_->to_isr();
    this->encoder_b_isr_ = this->encoder_b_pin_->to_isr();
    // Starting state — before the first real edge, so we don't count a
    // phantom transition on the very first interrupt.
    this->hall_last_state_ =
        (this->encoder_a_pin_->digital_read() ? 2 : 0) | (this->encoder_b_pin_->digital_read() ? 1 : 0);
    // Full (4x) quadrature decoding requires interrupts on BOTH channels, not just A.
    this->encoder_a_pin_->attach_interrupt(&MotorSensor::hall_isr_, this, gpio::INTERRUPT_ANY_EDGE);
    this->encoder_b_pin_->attach_interrupt(&MotorSensor::hall_isr_, this, gpio::INTERRUPT_ANY_EDGE);
  }
  // --- ADC (resistor on the motor shaft) ---
  // Uses ESPHome's built-in ADC component (the specific driver depends on
  // the platform — ESP-IDF adc_oneshot on ESP32, the built-in ADC on
  // ESP8266, etc., including calibration against a reference curve/line
  // where available for the specific chip). We create and configure the
  // object ourselves and don't register it with App (no periodic update()
  // needed) — we read sample() manually.
  if (this->has_adc_) {
    this->adc_sensor_ = new adc::ADCSensor();  // NOLINT(cppcoreguidelines-owning-memory)
    this->adc_sensor_->set_pin(this->adc_gpio_pin_);
#ifdef USE_ESP32
    this->adc_sensor_->set_attenuation(adc::ADC_ATTEN_DB_12_COMPAT);
#endif
    this->adc_sensor_->setup();
  }
}

float MotorSensor::read_adc_raw() {
  if (this->adc_sensor_ == nullptr) return NAN;
  // sample() performs a single measurement via the platform's built-in ADC
  // driver (with calibration where available) and returns the voltage in
  // volts. The unit doesn't matter for our purposes — the closed/open
  // calibration works with any monotonic value.
  return this->adc_sensor_->sample();
}

void MotorSensor::hall_isr_(MotorSensor *arg) {
  // Full (4x) quadrature decode across both channels (see
  // HALL_QUADRATURE_TABLE) — the interrupt fires on any edge of A OR B, we
  // read both levels and get ±1 or 0 from the transition table (for
  // "impossible"/bounce transitions, which weren't reliably filtered when
  // decoding from a single channel).
  uint8_t a = arg->encoder_a_isr_.digital_read() ? 1 : 0;
  uint8_t b = arg->encoder_b_isr_.digital_read() ? 1 : 0;
  uint8_t new_state = (a << 1) | b;
  uint8_t index = (arg->hall_last_state_ << 2) | new_state;
  arg->hall_pulse_count_ += HALL_QUADRATURE_TABLE[index];
  arg->hall_last_state_ = new_state;
}

}  // namespace jalouzee_blinds
}  // namespace esphome
