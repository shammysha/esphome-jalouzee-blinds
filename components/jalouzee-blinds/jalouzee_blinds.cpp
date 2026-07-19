#include "jalouzee_blinds.h"

#include "esphome/core/log.h"



namespace esphome {
namespace jalouzee_blinds {



JalouzeeBlinds::JalouzeeBlinds()
{

}





void JalouzeeBlinds::setup()
{


    preference_ =
        global_preferences
        ->
        make_preference<PersistentData>(
            0x4A424C42
        );



    load_preferences();



    if(motor_)
        motor_->setup();



    last_angle_ =
        get_angle();



    last_angle_change_ =
        millis();



    ESP_LOGI(
        TAG,
        "Setup complete"
    );

}






void JalouzeeBlinds::loop()
{


    float angle =
        get_angle();



    if(angle_output_)
    {
        angle_output_->publish_state(
            angle
        );
    }



    if(fault_output_)
    {
        fault_output_->publish_state(
            has_fault()
        );
    }



    if(has_fault())
        return;



    check_stall();


}







cover::CoverTraits
JalouzeeBlinds::get_traits()
{

    auto traits =
        cover::CoverTraits();



    traits.set_is_assumed_state(
        true
    );


    return traits;

}






void JalouzeeBlinds::control(
    const cover::CoverCall &call
)
{


    if(has_fault())
    {
        ESP_LOGW(
            TAG,
            "Command ignored - FAULT"
        );

        return;
    }




    if(call.get_stop())
    {

        motor_->stop();

        state_ =
            BlindState::MOVING;

        publish_state();

        return;

    }




    if(call.get_open())
    {


        if(state_ == BlindState::CLOSED)
        {

            move_to_angle(
                data_.angle_closed +
                (
                 data_.angle_open -
                 data_.angle_closed
                ) * 0.5f
            );


            state_ =
                BlindState::HALF;

        }
        else
        if(state_ == BlindState::HALF)
        {

            move_to_angle(
                data_.angle_open
            );


            state_ =
                BlindState::OPEN;

        }


    }






    if(call.get_close())
    {


        if(state_ == BlindState::OPEN)
        {

            move_to_angle(
                data_.angle_closed +
                (
                 data_.angle_open -
                 data_.angle_closed
                ) * 0.5f
            );


            state_ =
                BlindState::HALF;


        }
        else
        if(state_ == BlindState::HALF)
        {

            move_to_angle(
                data_.angle_closed
            );


            state_ =
                BlindState::CLOSED;

        }


    }



    publish_state();

}







void JalouzeeBlinds::move_to_angle(
    float angle
)
{

    target_angle_ =
        angle;



    float current =
        get_angle();



    bool open_direction;



    if(is_inverted())
    {
        open_direction =
            target_angle_ < current;
    }
    else
    {
        open_direction =
            target_angle_ > current;
    }



    if(open_direction)
        motor_->open();

    else
        motor_->close();



    last_angle_change_ =
        millis();



}






float JalouzeeBlinds::get_angle()
{

    auto sensor =
        get_active_sensor();



    if(!sensor)
        return last_angle_;



    last_angle_ =
        sensor->angle();


    return last_angle_;

}





void JalouzeeBlinds::check_stall()
{

    float angle =
        get_angle();



    if(
       fabs(
          angle-last_angle_
       )
       > 0.5f
      )
    {

        last_angle_change_ =
            millis();

        last_angle_ =
            angle;

    }



    if(
       millis()
       -
       last_angle_change_
       >
       stall_timeout_
      )
    {

        motor_->stop();


        set_fault(
            true
        );


        save_preferences();



        ESP_LOGE(
            TAG,
            "Motor stalled"
        );


    }

}

void JalouzeeBlinds::set_mpu_sensor(
    sensor::Sensor *sensor
)
{

    if(sensor)
    {

        mpu_sensor_ =
            new ESPHomeAngleSensor(
                sensor,
                SensorType::MPU6050
            );


        ESP_LOGI(
            TAG,
            "MPU6050 sensor attached"
        );

    }

}






void JalouzeeBlinds::set_hall_sensor(
    sensor::Sensor *sensor
)
{

    if(sensor)
    {

        hall_sensor_ =
            new ESPHomeAngleSensor(
                sensor,
                SensorType::HALL
            );


        ESP_LOGI(
            TAG,
            "Hall sensor attached"
        );

    }

}







AngleSensor *
JalouzeeBlinds::get_active_sensor()
{

    AngleSource source =
        static_cast<AngleSource>(
            data_.angle_source
        );



    /*
       Пользовательский выбор
    */


    if(source == AngleSource::MPU6050)
    {

        return mpu_sensor_;

    }



    if(source == AngleSource::HALL)
    {

        return hall_sensor_;

    }



    /*
       AUTO

       приоритет:
       MPU6050
       затем Hall

    */


    if(
       mpu_sensor_ &&
       mpu_sensor_->available()
      )
    {
        return mpu_sensor_;
    }



    if(
       hall_sensor_ &&
       hall_sensor_->available()
      )
    {
        return hall_sensor_;
    }



    return nullptr;

}







void JalouzeeBlinds::start_calibration()
{

    calibration_.start();



    ESP_LOGI(
        TAG,
        "Calibration started"
    );


}






void JalouzeeBlinds::save_closed_position()
{

    float angle =
        get_angle();



    calibration_.set_closed(
        angle
    );



    ESP_LOGI(
        TAG,
        "Closed position %.2f",
        angle
    );

}








void JalouzeeBlinds::save_open_position()
{

    float angle =
        get_angle();



    if(
       calibration_.set_open(
           angle
       )
      )
    {


        data_.angle_closed =
            calibration_.closed_angle();



        data_.angle_open =
            calibration_.open_angle();



        set_inverted(
            calibration_.inverted()
        );



        set_calibrated(
            true
        );



        data_.current_angle =
            angle;



        save_preferences();



        ESP_LOGI(
            TAG,
            "Calibration saved"
        );

    }
    else
    {

        ESP_LOGE(
            TAG,
            "Calibration error"
        );

    }

}







void JalouzeeBlinds::clear_fault()
{

    set_fault(
        false
    );


    save_preferences();



    state_ =
        BlindState::CLOSED;



    publish_state();



}








void JalouzeeBlinds::set_angle_source(
    AngleSource source
)
{

    data_.angle_source =
        static_cast<uint8_t>(
            source
        );



    save_preferences();


}









void JalouzeeBlinds::load_preferences()
{

    if(
       !preference_.load(
            &data_
        )
      )
    {


        memset(
            &data_,
            0,
            sizeof(data_)
        );



        data_.version =
            1;



        data_.angle_source =
            static_cast<uint8_t>(
                AngleSource::AUTO
            );


    }



    ESP_LOGI(
        TAG,
        "Preferences loaded"
    );

}









void JalouzeeBlinds::save_preferences()
{

    preference_.save(
        &data_
    );

}









bool JalouzeeBlinds::is_inverted()
{

    return
        data_.flags & 0x01;

}







void JalouzeeBlinds::set_inverted(
    bool value
)
{

    if(value)

        data_.flags |= 0x01;

    else

        data_.flags &= ~0x01;

}









bool JalouzeeBlinds::is_calibrated()
{

    return
        data_.flags & 0x02;

}







void JalouzeeBlinds::set_calibrated(
    bool value
)
{

    if(value)

        data_.flags |= 0x02;

    else

        data_.flags &= ~0x02;

}









bool JalouzeeBlinds::has_fault()
{

    return
        data_.flags & 0x04;

}







void JalouzeeBlinds::set_fault(
    bool value
)
{

    if(value)

        data_.flags |= 0x04;

    else

        data_.flags &= ~0x04;

}







void JalouzeeBlinds::dump_config()
{

    ESP_LOGCONFIG(
        TAG,
        "Jalouzee Blinds"
    );


    ESP_LOGCONFIG(
        TAG,
        "Closed angle: %.2f",
        data_.angle_closed
    );


    ESP_LOGCONFIG(
        TAG,
        "Open angle: %.2f",
        data_.angle_open
    );


    ESP_LOGCONFIG(
        TAG,
        "Current angle: %.2f",
        data_.current_angle
    );


    ESP_LOGCONFIG(
        TAG,
        "Inverted: %s",
        is_inverted()
        ?
        "YES"
        :
        "NO"
    );


    ESP_LOGCONFIG(
        TAG,
        "Calibrated: %s",
        is_calibrated()
        ?
        "YES"
        :
        "NO"
    );

}

}
}
