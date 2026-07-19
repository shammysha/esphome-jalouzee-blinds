#include "calibration.h"

#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
  namespace jalouzee_blinds {

    static const char *const TAG = "jalouzee_blinds.calibration";

    void Calibration::start() {

      state_ = CalibrationState::WAIT_CLOSED;

      closed_angle_ = 0.0f;

      open_angle_ = 0.0f;

      inverted_ = false;

      ESP_LOGI(TAG, "Calibration started. Waiting for CLOSED position");

    }

    void Calibration::reset() {

      state_ = CalibrationState::IDLE;

      closed_angle_ = 0.0f;

      open_angle_ = 0.0f;

      inverted_ = false;

    }

    void Calibration::set_closed_angle(float angle) {

      if (state_ != CalibrationState::WAIT_CLOSED) {
        return;
      }

      closed_angle_ = angle;

      state_ = CalibrationState::WAIT_OPEN;

      ESP_LOGI(TAG, "Closed angle stored: %.2f", closed_angle_);

    }

    void Calibration::set_open_angle(float angle) {

      if (state_ != CalibrationState::WAIT_OPEN) {
        return;
      }

      open_angle_ = angle;

      /*
       * Determine direction.
       *
       * Normal installation:
       *
       * closed < open
       *
       *
       * Mirrored installation:
       *
       * closed > open
       *
       */

      if (open_angle_ < closed_angle_) {

        inverted_ = true;

      } else {

        inverted_ = false;

      }

      state_ = CalibrationState::COMPLETE;

      ESP_LOGI(TAG, "Open angle stored: %.2f", open_angle_);

      ESP_LOGI(TAG, "Direction: %s", inverted_ ? "INVERTED" : "NORMAL");

    }

    bool Calibration::is_complete() const {

      if (state_ != CalibrationState::COMPLETE) {
        return false;
      }

      /*
       * Protection against invalid calibration.
       */

      return fabs(open_angle_ - closed_angle_) > 1.0f;

    }

    CalibrationState Calibration::state() const {

      return state_;

    }

    float Calibration::closed_angle() const {

      return closed_angle_;

    }

    float Calibration::open_angle() const {

      return open_angle_;

    }

    bool Calibration::inverted() const {

      return inverted_;

    }

    void Calibration::restore(float closed, float open, bool inverted) {

      closed_angle_ = closed;

      open_angle_ = open;

      inverted_ = inverted;

      state_ = CalibrationState::COMPLETE;

    }

  }  // namespace jalouzee_blinds
}  // namespace esphome
