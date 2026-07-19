#pragma once

#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/preferences.h"
#include "esphome/core/gpio.h"

#include "jalouzee_blinds_types.h"

#include "angle_sensor.h"
#include "calibration.h"

namespace esphome {
  namespace jalouzee_blinds {

    struct PersistentData {

        uint32_t magic { DATA_MAGIC };

        float closed_angle { 0.0f };

        float open_angle { 90.0f };

        float current_angle { 0.0f };

        bool inverted { false };

        bool calibrated { false };

        uint8_t angle_source { static_cast<uint8_t>(AngleSource::AUTO) };

        uint8_t position { static_cast<uint8_t>(BlindPosition::UNKNOWN) };

        bool fault { false };

    };

    class JalouzeeBlinds: public Component, public cover::Cover {

      public:

        JalouzeeBlinds();

        void setup() override;

        void loop() override;

        cover::CoverTraits get_traits() override;

        void control(const cover::CoverCall &call) override;

        float get_setup_priority() const override;

        /*
         * Motor
         */

        void set_motor_pins(GPIOPin *open_pin, GPIOPin *close_pin);

        void motor_open();

        void motor_close();

        void motor_stop();

        /*
         * Angle sensors
         */

        void set_primary_sensor(sensor::Sensor *sensor);

        void set_secondary_sensor(sensor::Sensor *sensor);

        void set_angle_source(AngleSource source);

        float get_angle();

        bool angle_available();

        /*
         * Calibration
         */

        void start_calibration();

        void save_closed_position();

        void save_open_position();

        bool is_calibrated();

        /*
         * Fault handling
         */

        void set_fault();

        void clear_fault();

        bool has_fault();

        /*
         * Entities
         */

        void set_angle_output(sensor::Sensor *sensor);

        void set_position_output(sensor::Sensor *sensor);

        void set_fault_output(binary_sensor::BinarySensor *sensor);

        void set_calibrated_output(binary_sensor::BinarySensor *sensor);

        /*
         * Configuration
         */

        void set_stall_timeout(uint32_t timeout);

      protected:

        void move_to(BlindPosition target);

        void update_position();

        void publish_entities();

        void load_state();

        void save_state();

        AngleSource source_ { AngleSource::AUTO };

        AngleSource angle_source() const;

      protected:

        /*
         * Hardware
         */

        GPIOPin *motor_open_pin_ { nullptr };

        GPIOPin *motor_close_pin_ { nullptr };

        /*
         * Sensors
         */

        AngleSensor angle_sensor_;

        /*
         * Calibration
         */

        Calibration calibration_;

        /*
         * Runtime state
         */

        BlindState state_ { BlindState::IDLE };

        BlindPosition position_ { BlindPosition::UNKNOWN };

        float target_angle_ { 0.0f };

        uint32_t movement_start_time_ { 0 };

        uint32_t last_angle_change_time_ { 0 };

        float last_angle_ { 0.0f };

        /*
         * Settings
         */

        uint32_t stall_timeout_ { 10000 };

        /*
         * Persistent storage
         */

        ESPPreferenceObject preference_;

        PersistentData data_;

        /*
         * HA entities
         */

        sensor::Sensor *angle_output_ { nullptr };
        sensor::Sensor *position_output_ { nullptr };

        binary_sensor::BinarySensor *fault_output_ { nullptr };
        binary_sensor::BinarySensor *calibrated_output_ { nullptr };

        void create_entities(bool angle, bool position, bool fault, bool calibrated);

    };

  }  // namespace jalouzee_blinds
}  // namespace esphome
