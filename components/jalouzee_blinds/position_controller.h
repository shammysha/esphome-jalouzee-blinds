#pragma once

#include "types.h"

namespace esphome {
  namespace jalouzee_blinds {

    class MotorDriver;
    class Sensors;
    class Calibration;

    class PositionController {

      public:

        PositionController(Sensors *sensors, Calibration *calibration, MotorDriver *motor);

        void update();

        void set_position(float position);

        void stop();

        float position() const;

      protected:

        bool position_available() const;

        float calculate_position() const;

        void move_to_target();

      protected:

        Sensors *sensors_ = nullptr;

        Calibration *calibration_ = nullptr;

        MotorDriver *motor_ = nullptr;

        float target_position_ = 0;

        float current_position_ = 0;

        bool moving_ = false;

    };

  } // namespace jalouzee_blinds
} // namespace esphome
