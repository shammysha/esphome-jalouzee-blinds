#pragma once

#include <cmath>
#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/select/select.h"

#include "motor.h"
#include "angle_sensor.h"
#include "calibration.h"


namespace esphome {
namespace jalouzee_blinds {



static const char *TAG =
    "jalouzee_blinds";



enum class BlindState : uint8_t
{
    CLOSED,
    HALF,
    OPEN,
    MOVING,
    FAULT
};




enum class AngleSource : uint8_t
{
    AUTO = 0,
    MPU6050 = 1,
    HALL = 2
};





struct PersistentData
{

    uint8_t version;


    /*
       flags:

       bit 0 - inverted
       bit 1 - calibrated
       bit 2 - fault

    */

    uint8_t flags;



    float angle_closed;


    float angle_open;


    float current_angle;



    uint8_t angle_source;

};





class JalouzeeBlinds :
    public Component,
    public cover::Cover
{


public:


    JalouzeeBlinds();



    void setup() override;


    void loop() override;



    cover::CoverTraits get_traits()
        override;



    void control(
        const cover::CoverCall &call
    ) override;



    void dump_config()
        override;




    /*
       hardware
    */


    void set_motor(
        DualGPIOMotor *motor
    )
    {
        motor_ = motor;
    }



    void set_mpu_sensor(
        sensor::Sensor *sensor
    );


    void set_hall_sensor(
        sensor::Sensor *sensor
    );




    /*
       HA entities
    */


    void set_angle_output(
        sensor::Sensor *sensor
    )
    {
        angle_output_ = sensor;
    }



    void set_fault_output(
        binary_sensor::BinarySensor *sensor
    )
    {
        fault_output_ = sensor;
    }



    void set_angle_select(
        select::Select *select
    )
    {
        angle_select_ = select;
    }



    /*
       calibration
    */


    void start_calibration();


    void save_closed_position();


    void save_open_position();




    /*
       fault
    */


    void clear_fault();



    /*
       settings
    */


    void set_angle_source(
        AngleSource source
    );



protected:


    void move_to_angle(
        float angle
    );


    float get_angle();



    AngleSensor *get_active_sensor();



    void check_stall();



    void save_preferences();


    void load_preferences();




    /*
       flags
    */


    bool is_inverted();


    void set_inverted(
        bool value
    );



    bool is_calibrated();


    void set_calibrated(
        bool value
    );



    bool has_fault();


    void set_fault(
        bool value
    );




protected:


    DualGPIOMotor *motor_{nullptr};



    ESPHomeAngleSensor *mpu_sensor_{nullptr};


    ESPHomeAngleSensor *hall_sensor_{nullptr};




    sensor::Sensor *angle_output_{nullptr};


    binary_sensor::BinarySensor *fault_output_{nullptr};


    select::Select *angle_select_{nullptr};




    JalouzeeCalibration calibration_;




    ESPPreferenceObject preference_;


    PersistentData data_{};



    BlindState state_ =
        BlindState::CLOSED;



    float target_angle_{0};


    float last_angle_{0};



    uint32_t last_angle_change_{0};



    uint32_t stall_timeout_ =
        10000;



};



}
}
