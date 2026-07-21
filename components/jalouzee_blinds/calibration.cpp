#include "calibration.h"

#include <cmath>
#include <cstdlib>

namespace esphome {
  namespace jalouzee_blinds {

    Calibration::Calibration() {

    }

    void Calibration::start() {

      reset_runtime();

      pending_ = CalibrationData();

      commit_requested_ = false;

      stage_ = CalibrationStage::WAIT_CLOSED;

    }

    CalibrationResult Calibration::next(const SensorData &sensor) {

      switch (stage_) {

        case CalibrationStage::NONE: {

          start();

          return CalibrationResult::NOT_READY;

        }

        case CalibrationStage::WAIT_CLOSED: {

          runtime_.closed_angle = sensor.angle;

          runtime_.closed_encoder = sensor.encoder;

          stage_ = CalibrationStage::WAIT_OPEN;

          return CalibrationResult::NOT_READY;

        }

        case CalibrationStage::WAIT_OPEN: {

          runtime_.open_angle = sensor.angle;

          runtime_.open_encoder = sensor.encoder;

          CalibrationResult result = build_pending();

          if (result == CalibrationResult::OK) {

            stage_ = CalibrationStage::WAIT_COMMIT;

          } else {

            stage_ = CalibrationStage::NONE;

          }

          return result;

        }

        case CalibrationStage::WAIT_COMMIT: {

          /*
           * Здесь данные готовы.
           *
           * Никакого сохранения.
           */

          return CalibrationResult::NOT_READY;

        }

      }

      return CalibrationResult::NOT_READY;

    }

    bool Calibration::commit() {

      if (stage_ != CalibrationStage::WAIT_COMMIT) {

        return false;

      }

      commit_requested_ = true;

      return true;

    }

    CalibrationResult Calibration::build_pending() {

      CalibrationResult result = validate();

      if (result != CalibrationResult::OK) {

        return result;

      }

      CalibrationData result_data;

      result_data.valid = true;

      result_data.closed_angle = runtime_.closed_angle;

      result_data.open_angle = runtime_.open_angle;

      result_data.closed_encoder = runtime_.closed_encoder;

      result_data.open_encoder = runtime_.open_encoder;

      result_data.encoder_range = abs(result_data.open_encoder - result_data.closed_encoder);

      calculate_signs(result_data);

      pending_ = result_data;

      return CalibrationResult::OK;

    }

    CalibrationResult Calibration::validate() const {

      if (fabs(runtime_.open_angle - runtime_.closed_angle) < 5.0f) {

        return CalibrationResult::INVALID_ANGLE_RANGE;

      }

      if (abs(runtime_.open_encoder - runtime_.closed_encoder) < 10) {

        return CalibrationResult::INVALID_ENCODER_RANGE;

      }

      return CalibrationResult::OK;

    }

    void Calibration::calculate_signs(CalibrationData &data) {

      data.imu_sign = (data.open_angle > data.closed_angle) ? 1 : -1;

      data.encoder_sign = (data.open_encoder > data.closed_encoder) ? 1 : -1;

    }

    void Calibration::cancel() {

      reset_runtime();

      pending_ = CalibrationData();

      commit_requested_ = false;

      stage_ = CalibrationStage::NONE;

    }

    void Calibration::apply_pending() {

      calibration_ = pending_;

      pending_ = CalibrationData();

      commit_requested_ = false;

      stage_ = CalibrationStage::NONE;

    }

    void Calibration::load(const CalibrationData &data) {

      calibration_ = data;

    }

    bool Calibration::active() const {

      return stage_ != CalibrationStage::NONE;

    }

    CalibrationStage Calibration::stage() const {

      return stage_;

    }

    const CalibrationData&
    Calibration::data() const {

      return calibration_;

    }

    const CalibrationData&
    Calibration::pending() const {

      return pending_;

    }

    void Calibration::reset_runtime() {

      runtime_ = CalibrationRuntime();

    }

  } // namespace jalouzee_blinds
} // namespace esphome
