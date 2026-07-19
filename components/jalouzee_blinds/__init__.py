import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import cover
from esphome.components import sensor
from esphome import pins

from esphome.const import (
    CONF_ID,
)

DEPENDENCIES = [
    "cover",
]


jalouzee_blinds_ns = cg.esphome_ns.namespace(
    "jalouzee_blinds"
)


JalouzeeBlinds = jalouzee_blinds_ns.class_(
    "JalouzeeBlinds",
    cover.Cover,
    cg.Component,
)


AUTO_LOAD = [
    "button",
    "sensor",
    "binary_sensor",
    "select",
]

CONF_OPEN_PIN = "open_pin"
CONF_CLOSE_PIN = "close_pin"

CONF_PRIMARY_SENSOR = "primary_sensor"
CONF_SECONDARY_SENSOR = "secondary_sensor"

CONF_STALL_TIMEOUT = "stall_timeout"


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID():
                cv.declare_id(JalouzeeBlinds),
        }
    )
    .extend(
        cover.cover_schema(JalouzeeBlinds)
    )
    .extend(
        {
            cv.Required(CONF_OPEN_PIN):
                pins.gpio_output_pin_schema,

            cv.Required(CONF_CLOSE_PIN):
                pins.gpio_output_pin_schema,


            cv.Optional(CONF_PRIMARY_SENSOR):
                cv.use_id(sensor.Sensor),


            cv.Optional(CONF_SECONDARY_SENSOR):
                cv.use_id(sensor.Sensor),


            cv.Optional(
                CONF_STALL_TIMEOUT,
                default="10s"
            ):
                cv.positive_time_period_milliseconds,

        }
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


    open_pin = await cg.gpio_pin_expression(
        config[CONF_OPEN_PIN]
    )


    close_pin = await cg.gpio_pin_expression(
        config[CONF_CLOSE_PIN]
    )


    cg.add(
        var.set_motor_pins(
            open_pin,
            close_pin
        )
    )


    if CONF_PRIMARY_SENSOR in config:

        primary = await cg.get_variable(
            config[CONF_PRIMARY_SENSOR]
        )

        cg.add(
            var.set_primary_sensor(
                primary
            )
        )


    if CONF_SECONDARY_SENSOR in config:

        secondary = await cg.get_variable(
            config[CONF_SECONDARY_SENSOR]
        )

        cg.add(
            var.set_secondary_sensor(
                secondary
            )
        )


    cg.add(
        var.set_stall_timeout(
            config[CONF_STALL_TIMEOUT]
        )
    )