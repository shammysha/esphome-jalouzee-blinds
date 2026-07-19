import esphome.codegen as cg
import esphome.config_validation as cv


from esphome.components import (
    cover,
    sensor,
    binary_sensor,
    button,
    select,
)


from esphome.const import (
    CONF_ID,
)


from esphome import pins



DEPENDENCIES = [
    "sensor",
]



CODEOWNERS = [
    "@local"
]



jalouzee_ns = cg.esphome_ns.namespace(
    "jalouzee_blinds"
)



JalouzeeBlinds = jalouzee_ns.class_(
    "JalouzeeBlinds",
    cg.Component,
    cover.Cover
)



DualGPIOMotor = jalouzee_ns.class_(
    "DualGPIOMotor"
)





CalibrationStartButton = jalouzee_ns.class_(
    "CalibrationStartButton",
    button.Button
)



SaveClosedButton = jalouzee_ns.class_(
    "SaveClosedButton",
    button.Button
)



SaveOpenButton = jalouzee_ns.class_(
    "SaveOpenButton",
    button.Button
)



ClearFaultButton = jalouzee_ns.class_(
    "ClearFaultButton",
    button.Button
)



AngleSourceSelect = jalouzee_ns.class_(
    "AngleSourceSelect",
    select.Select
)





CONF_MOTOR = "motor"


CONF_OPEN_PIN = "open_pin"


CONF_CLOSE_PIN = "close_pin"



CONF_MPU_SENSOR = "mpu_sensor"


CONF_HALL_SENSOR = "hall_sensor"





CONFIG_SCHEMA = (
    cv.Schema(
        {


            cv.GenerateID():
            cv.declare_id(
                JalouzeeBlinds
            ),



            cv.Required(
                CONF_MOTOR
            ):
            cv.Schema(
                {


                    cv.Required(
                        CONF_OPEN_PIN
                    ):
                    pins.gpio_output_pin_schema,



                    cv.Required(
                        CONF_CLOSE_PIN
                    ):
                    pins.gpio_output_pin_schema,


                }
            ),





            cv.Optional(
                CONF_MPU_SENSOR
            ):
            cv.use_id(
                sensor.Sensor
            ),




            cv.Optional(
                CONF_HALL_SENSOR
            ):
            cv.use_id(
                sensor.Sensor
            ),



        }
    )
    .extend(
        cover.cover_schema(
            JalouzeeBlinds,
            device_class="blind"
        )
    )
    .extend(
        cv.COMPONENT_SCHEMA
    )
)






async def to_code(config):



    var = cg.new_Pvariable(
        config[CONF_ID]
    )



    await cg.register_component(
        var,
        config
    )



    await cover.register_cover(
        var,
        config
    )





    #
    # MOTOR
    #


    motor_conf = config[
        CONF_MOTOR
    ]



    motor = cg.new_Pvariable(
        cg.new_id(),
        DualGPIOMotor
    )




    open_pin = await cg.gpio_pin_expression(
        motor_conf[
            CONF_OPEN_PIN
        ]
    )



    close_pin = await cg.gpio_pin_expression(
        motor_conf[
            CONF_CLOSE_PIN
        ]
    )



    cg.add(
        motor.set_open_pin(
            open_pin
        )
    )



    cg.add(
        motor.set_close_pin(
            close_pin
        )
    )



    cg.add(
        var.set_motor(
            motor
        )
    )





    #
    # ANGLE SENSORS
    #



    if CONF_MPU_SENSOR in config:

        cg.add(
            var.set_mpu_sensor(
                config[
                    CONF_MPU_SENSOR
                ]
            )
        )




    if CONF_HALL_SENSOR in config:

        cg.add(
            var.set_hall_sensor(
                config[
                    CONF_HALL_SENSOR
                ]
            )
        )






    #
    # SELECT
    #


    source_select = cg.new_Pvariable(
        cg.new_id(),
        AngleSourceSelect
    )



    await select.register_select(
        source_select,
        {
            "options":
            [
                "AUTO",
                "MPU6050",
                "HALL",
            ]
        }
    )



    cg.add(
        source_select.set_parent(
            var
        )
    )






    #
    # BUTTONS
    #



    buttons = [

        (
            CalibrationStartButton,
            "Start calibration",
            "start_calibration"
        ),


        (
            SaveClosedButton,
            "Save closed",
            "save_closed_position"
        ),


        (
            SaveOpenButton,
            "Save open",
            "save_open_position"
        ),


        (
            ClearFaultButton,
            "Clear fault",
            "clear_fault"
        ),

    ]





    for cls, name, method in buttons:


        b = cg.new_Pvariable(
            cg.new_id(),
            cls
        )



        await button.register_button(
            b,
            {
                "name": name
            }
        )



        cg.add(
            b.set_parent(
                var
            )
        )