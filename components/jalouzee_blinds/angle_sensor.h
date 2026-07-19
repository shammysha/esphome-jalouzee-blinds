#pragma once

#include <cstdint>

#include "esphome/components/sensor/sensor.h"

#include "jalouzee_blinds_types.h"

namespace esphome {
  namespace jalouzee_blinds {

    class AngleSensor {

      public:

        AngleSensor() = default;

        void set_primary(sensor::Sensor *sensor);

        void set_secondary(sensor::Sensor *sensor);

        void set_source(AngleSource source);

        float get_angle();

        bool available();

        bool primary_available();

        bool secondary_available();

      protected:

        sensor::Sensor *primary_ { nullptr };

        sensor::Sensor *secondary_ { nullptr };

        AngleSource source_ { AngleSource::AUTO };

        float primary_value_ { 0.0f };

        float secondary_value_ { 0.0f };

        float last_angle_ { 0.0f };

        bool last_angle_valid_ { false };

        bool primary_received_ { false };

        bool secondary_received_ { false };

        void update_values();

    };

  }  // namespace jalouzee_blinds
}  // namespace esphome
