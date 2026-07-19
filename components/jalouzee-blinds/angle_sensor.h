#pragma once


#include <vector>
#include <cmath>


#include "esphome/components/sensor/sensor.h"



namespace esphome {
namespace jalouzee_blinds {



enum class SensorType : uint8_t
{
    NONE = 0,
    MPU6050 = 1,
    HALL = 2
};





class AngleSensor
{

public:


    virtual bool available() = 0;


    virtual float angle() = 0;


    virtual SensorType type() = 0;



    virtual ~AngleSensor() = default;

};








class ESPHomeAngleSensor :
    public AngleSensor
{


public:


    ESPHomeAngleSensor(
        sensor::Sensor *sensor,
        SensorType type
    );



    bool available() override;



    float angle() override;



    SensorType type() override
    {
        return type_;
    }




protected:


    float filter(
        float value
    );



protected:


    sensor::Sensor *sensor_{nullptr};


    SensorType type_ =
        SensorType::NONE;



    float last_angle_{0};



    bool initialized_{false};



    std::vector<float> samples_;


};




}
}
