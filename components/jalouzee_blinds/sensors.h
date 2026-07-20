#pragma once

#include "types.h"

namespace esphome {
  namespace jalouzee_blinds {

    class MotorDriver;

    class Sensors {

      public:

        explicit Sensors(MotorDriver *motor);

        void setup();

        void update();

        /*
         * MPU6050
         */

        void set_mpu_angle(float angle);

        void set_imu_sign(int8_t sign);

        SensorData data() const;

        bool position_valid() const;

      protected:

        void update_encoder_state();

        void calculate_position();

      protected:

        MotorDriver *motor_ = nullptr;

        /*
         * MPU
         */

        float mpu_angle_ = 0;

        int8_t imu_sign_ = 1;

        bool mpu_valid_ = false;

        /*
         * Encoder
         */

        int32_t encoder_position_ = 0;

        bool encoder_valid_ = false;

        /*
         * Общая оценка
         */

        float position_ = 0;

    };

  } // namespace jalouzee_blinds
} // namespace esphome
