#include "angle_sensor.h"


#include <algorithm>


namespace esphome {
namespace jalouzee_blinds {



ESPHomeAngleSensor::ESPHomeAngleSensor(
    sensor::Sensor *sensor,
    SensorType type
)
{

    sensor_ = sensor;

    type_ = type;

}








bool ESPHomeAngleSensor::available()
{

    if(!sensor_)
        return false;



    return !std::isnan(
        sensor_->state
    );

}








float ESPHomeAngleSensor::angle()
{


    if(!available())
    {

        return last_angle_;

    }




    float value =
        sensor_->state;



    return filter(
        value
    );

}









float ESPHomeAngleSensor::filter(
    float value
)
{


    /*
       Первый запуск

    */


    if(!initialized_)
    {

        last_angle_ =
            value;


        initialized_ =
            true;


        samples_.push_back(
            value
        );


        return value;

    }





    /*
       Защита от выбросов

       Например:
       MPU дал скачок
       10° -> 180°

    */


    if(
       fabs(
          value -
          last_angle_
       )
       >
       45.0f
      )
    {

        return last_angle_;

    }







    samples_.push_back(
        value
    );



    /*
       Окно фильтра

       5 последних измерений

    */


    if(
       samples_.size()
       >
       5
      )
    {

        samples_.erase(
            samples_.begin()
        );

    }






    float sum = 0;



    for(
        float v :
        samples_
    )
    {

        sum += v;

    }






    float result =
        sum /
        samples_.size();





    last_angle_ =
        result;



    return result;

}







}
}
