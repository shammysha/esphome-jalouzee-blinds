#include "angle_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
  namespace jalouzee_blinds {

    static const char *const TAG = "jalouzee_blinds.angle_sensor";

    void AngleSensor::set_primary(sensor::Sensor *sensor) {

      primary_ = sensor;

      if (primary_ != nullptr) {

        primary_received_ = false;

        primary_->add_on_state_callback([this](float value) {

          primary_value_ = value;

          primary_received_ = true;

          last_angle_ = value;

          last_angle_valid_ = true;

        }
        );

      }

    }

    void AngleSensor::set_secondary(sensor::Sensor *sensor) {

      secondary_ = sensor;

      if (secondary_ != nullptr) {

        secondary_received_ = false;

        primary_->add_on_state_callback([this](float value) {

          primary_value_ = value;

          primary_received_ = true;

          last_angle_ = value;

          last_angle_valid_ = true;

        }
        );

      }

    }

    void AngleSensor::set_source(AngleSource source) {

      source_ = source;

      ESP_LOGD(TAG, "Angle source changed: %d", static_cast<int>(source_));

    }

    void AngleSensor::update_values() {

      /*
       * Sensor callbacks update values immediately.
       *
       * This method is intentionally kept
       * for future filtering.
       */

    }

    bool AngleSensor::primary_available() {

      return primary_ != nullptr && primary_received_;

    }

    bool AngleSensor::secondary_available() {

      return secondary_ != nullptr && secondary_received_;

    }

    bool AngleSensor::available() {

      switch (source_) {

        case AngleSource::PRIMARY:

          return primary_available();

        case AngleSource::SECONDARY:

          return secondary_available();

        case AngleSource::AUTO:

          return primary_available() || secondary_available();

      }

      return false;

    }

    float AngleSensor::get_angle() {

      update_values();

      switch (source_) {

        case AngleSource::PRIMARY:

          if (primary_available()) {
            return primary_value_;
          }

          break;

        case AngleSource::SECONDARY:

          if (secondary_available()) {
            return secondary_value_;
          }

          break;

        case AngleSource::AUTO:

          if (primary_available()) {
            return primary_value_;
          }

          if (secondary_available()) {
            return secondary_value_;
          }

          break;

      }

      if(last_angle_valid_)
      {
          return last_angle_;
      }


      return NAN;

    }

  }  // namespace jalouzee_blinds
}  // namespace esphome
