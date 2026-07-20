import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import cover
from esphome.components import sensor
from esphome import pins

from esphome.const import (
    CONF_ID,
)

CODEOWNERS = ["@shammysha"]

DEPENDENCIES = ["cover"]

jalouzee_ns = cg.esphome_ns.namespace(
    "jalouzee_blinds"
)

JalouzeeBlinds = jalouzee_ns.class_(
    "JalouzeeBlinds",
    cover.Cover,
    cg.Component
)

CONF_MOTOR = "motor"
CONF_IN1 = "in1"
CONF_IN2 = "in2"

CONF_ENCODER = "encoder"
CONF_A = "a"
CONF_B = "b"

CONF_MPU6050 = "mpu6050"
CONF_ANGLE_SENSOR = "angle_sensor"

CONF_CAL_BUTTON = (
    "calibration_button"
)

CONF_CANCEL_BUTTON = (
    "cancel_button"
)

CONF_PIN = "pin"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID():
                cv.declare_id(
                    JalouzeeBlinds
                ),
            cv.GenerateID(): cv.declare_id(
                JalouzeeBlinds
            ),
            cv.Required(CONF_MOTOR): cv.Schema({
                cv.Required(CONF_IN1): pins.gpio_output_pin_schema,
                cv.Required(CONF_IN2): pins.gpio_output_pin_schema,
            }),
            cv.Required(CONF_ENCODER): cv.Schema({
                cv.Required(CONF_A): pins.gpio_input_pin_schema,
                cv.Required(CONF_B): pins.gpio_input_pin_schema,
            }),
            cv.Optional(CONF_MPU6050): cv.Schema({
                cv.Required(CONF_ANGLE_SENSOR): cv.use_id(sensor.Sensor)
            }),
            cv.Optional(CONF_CAL_BUTTON): cv.Schema({
                cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
            }),
            cv.Optional(CONF_CANCEL_BUTTON): cv.Schema({
                cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
            }),
        }
    )
)


async def to_code(config):
    var = cg.new_Pvariable( config[CONF_ID])

    await cg.register_component( var, config )
    await cover.register_cover( var, config )

    #
    # Motor
    #

    motor = config[CONF_MOTOR]

    in1 = await cg.gpio_pin_expression( motor[CONF_IN1] )
    in2 = await cg.gpio_pin_expression( motor[CONF_IN2] )

    cg.add(
        var.set_motor_pins( in1, in2 )
    )

    #
    # Encoder
    #

    encoder = config[CONF_ENCODER]

    enc_a = await cg.gpio_pin_expression( encoder[CONF_A] )
    enc_b = await cg.gpio_pin_expression( encoder[CONF_B] )

    cg.add(
        var.set_encoder_pins( enc_a, enc_b )
    )

    #
    # MPU6050 angle source
    #

    if CONF_MPU6050 in config:

        mpu = config[CONF_MPU6050]

        angle_sensor = await cg.get_variable( mpu[CONF_ANGLE_SENSOR] )

        cg.add(
            var.set_angle_sensor( angle_sensor )
        )

    #
    # Calibration button
    #

    if CONF_CAL_BUTTON in config:

        pin = await cg.gpio_pin_expression( config[CONF_CAL_BUTTON][CONF_PIN] )

        cg.add(
            var.set_calibration_button( pin )
        )

    #
    # Cancel button
    #

    if CONF_CANCEL_BUTTON in config:

        pin = await cg.gpio_pin_expression( config[CONF_CANCEL_BUTTON][CONF_PIN] )

        cg.add(
            var.set_cancel_button( pin )
        )