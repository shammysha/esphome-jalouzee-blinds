#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"

#include "motor_driver.h"
#include "sensors.h"
#include "calibration.h"
#include "storage.h"
#include "position_controller.h"

namespace esphome {
  namespace jalouzee_blinds {

    class JalouzeeBlinds: public cover::Cover, public Component {

      public:

        JalouzeeBlinds();

        /*
         * ESPHome lifecycle
         */

        void setup() override;

        void loop() override;

        float get_setup_priority() const override
        {

          return setup_priority::DATA;

        }

        /*
         * Cover
         */

        void control(const cover::CoverCall &call) override;

        cover::CoverTraits get_traits() const override;

        /*
         * Configuration
         *
         * Motor
         */

        void set_motor_pins(GPIOPin *in1, GPIOPin *in2);

        /*
         * Encoder
         */

        void set_encoder_pins(GPIOPin *a, GPIOPin *b);

        /*
         * MPU
         */

        void set_angle_sensor(sensor::Sensor *sensor);

        /*
         * Buttons
         */

        void set_calibration_button(GPIOPin *pin);

        void set_cancel_button(GPIOPin *pin);

      protected:

        void calibration_pressed();

        void cancel_pressed();

        void check_buttons();

        void publish_position();

        void handle_error();

      protected:

        /*
         * Internal blocks
         */

        MotorDriver motor_;

        Sensors sensors_;

        Calibration calibration_;

        Storage storage_;

        PositionController position_controller_;

        /*
         * MPU source
         */

        sensor::Sensor *angle_sensor_ = nullptr;

        /*
         * Buttons
         */

        GPIOPin *calibration_button_ = nullptr;

        GPIOPin *cancel_button_ = nullptr;

        bool last_calibration_button_ = false;

        bool last_cancel_button_ = false;

        float last_position_ = -1;

        SystemState state_ = SystemState::IDLE;

    };

  } // namespace jalouzee_blinds
} // namespace esphome
