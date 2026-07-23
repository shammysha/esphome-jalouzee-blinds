#pragma once

#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace jalouzee_blinds {

// Thin wrapper around an external sensor::Sensor (a specific MPU6050
// accelerometer axis, or a ready-made angle from a separate template
// sensor, see cover.py). The component only calibrates the
// "closed..open" range against this sensor's values.
class MpuSensor {
 public:
  void set_sensor(sensor::Sensor *sens) {
    this->sensor_ = sens;
    this->has_mpu_ = true;
  }

  bool has_mpu() const { return this->has_mpu_; }
  bool is_available() const { return this->has_mpu_ && this->sensor_ != nullptr && this->sensor_->has_state(); }
  float read_raw() const { return this->sensor_->state; }

 protected:
  sensor::Sensor *sensor_{nullptr};
  bool has_mpu_{false};
};

}  // namespace jalouzee_blinds
}  // namespace esphome
