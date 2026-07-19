#include "calibration.h"


namespace esphome {
namespace jalouzee_blinds {



static constexpr float
MIN_CALIBRATION_RANGE = 5.0f;








void JalouzeeCalibration::reset()
{

    closed_angle_ = 0;


    open_angle_ = 0;



    closed_saved_ = false;


    open_saved_ = false;



    completed_ = false;



    inverted_ = false;


}









void JalouzeeCalibration::start()
{

    reset();


}









bool JalouzeeCalibration::set_closed(
    float angle
)
{


    closed_angle_ =
        angle;



    closed_saved_ =
        true;



    if(open_saved_)
    {

        calculate_direction();

    }



    return completed_;

}









bool JalouzeeCalibration::set_open(
    float angle
)
{


    open_angle_ =
        angle;



    open_saved_ =
        true;




    if(closed_saved_)
    {

        calculate_direction();

    }




    return completed_;

}









void JalouzeeCalibration::calculate_direction()
{




    float delta =
        open_angle_
        -
        closed_angle_;





    /*
       Проверка диапазона

       Например:
       закрыто 80°
       открыто 83°

       это ошибка

    */


    if(
       fabs(delta)
       <
       MIN_CALIBRATION_RANGE
      )
    {


        completed_ =
            false;



        return;

    }








    /*
       Обычная установка:

       закрыто 10°
       открыто 170°

       delta > 0


       Зеркальная:

       закрыто 170°
       открыто 10°

       delta < 0

    */



    inverted_ =
        delta < 0;





    completed_ =
        true;



}






}
}
