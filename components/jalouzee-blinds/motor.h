#pragma once


#include "esphome/core/component.h"
#include "esphome/core/gpio.h"


namespace esphome {
namespace jalouzee_blinds {



enum class MotorDirection : uint8_t
{
    STOP = 0,
    OPEN = 1,
    CLOSE = 2
};





class DualGPIOMotor
{

public:


    DualGPIOMotor();



    void setup();



    void set_open_pin(
        InternalGPIOPin *pin
    )
    {
        open_pin_ = pin;
    }



    void set_close_pin(
        InternalGPIOPin *pin
    )
    {
        close_pin_ = pin;
    }





    void open();


    void close();


    void stop();



    MotorDirection direction()
    {
        return direction_;
    }



protected:


    void write_state(
        bool open,
        bool close
    );



protected:


    InternalGPIOPin *open_pin_{nullptr};


    InternalGPIOPin *close_pin_{nullptr};



    MotorDirection direction_ =
        MotorDirection::STOP;


};



}
}
