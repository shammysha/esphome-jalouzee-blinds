#pragma once

#include <cstdint>

namespace esphome {
  namespace jalouzee_blinds {

    enum class CalibrationState : uint8_t {

      IDLE = 0,

      WAIT_CLOSED = 1,

      WAIT_OPEN = 2,

      COMPLETE = 3

    };

    class Calibration {

      public:
        void start();
        void reset();
        void restore(float closed, float open, bool inverted);

        void set_closed_angle(float angle);

        void set_open_angle(float angle);

        bool is_complete() const;

        CalibrationState state() const;

        float closed_angle() const;

        float open_angle() const;

        bool inverted() const;

      protected:

        CalibrationState state_ { CalibrationState::IDLE };

        float closed_angle_ { 0.0f };

        float open_angle_ { 0.0f };

        bool inverted_ { false };

    };

  }  // namespace jalouzee_blinds
}  // namespace esphome
