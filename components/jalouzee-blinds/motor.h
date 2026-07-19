#pragma once


#include "esphome/core/hal.h"



namespace esphome {
namespace jalouzee_blinds {



class DualGPIOMotor
{


public:


    virtual ~DualGPIOMotor() = default;



    virtual void setup()
    {
    }



    virtual void open() = 0;



    virtual void close() = 0;



    virtual void stop() = 0;



};









class GPIOMotor :
    public DualGPIOMotor
{


public:



    void set_open_pin(
        GPIOPin *pin
    )
    {
        open_pin_ = pin;
    }





    void set_close_pin(
        GPIOPin *pin
    )
    {
        close_pin_ = pin;
    }






    void setup() override;



    void open() override;



    void close() override;



    void stop() override;







protected:


    GPIOPin *open_pin_{nullptr};



    GPIOPin *close_pin_{nullptr};



};





}
}
