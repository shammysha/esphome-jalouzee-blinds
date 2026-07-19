#pragma once


#include <cmath>
#include <cstdint>



namespace esphome {
namespace jalouzee_blinds {



class JalouzeeCalibration
{


public:


    void reset();



    void start();




    bool set_closed(
        float angle
    );



    bool set_open(
        float angle
    );





    bool completed() const
    {
        return completed_;
    }




    float closed_angle() const
    {
        return closed_angle_;
    }




    float open_angle() const
    {
        return open_angle_;
    }





    bool inverted() const
    {
        return inverted_;
    }





protected:



    void calculate_direction();



protected:



    float closed_angle_{0};


    float open_angle_{0};




    bool closed_saved_{false};


    bool open_saved_{false};




    bool completed_{false};



    bool inverted_{false};



};



}
}
