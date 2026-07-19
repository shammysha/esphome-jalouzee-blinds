import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor

from .. import JalouzeeBlinds

CONF_PARENT = "parent"
CONF_TYPE = "type"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.Required(CONF_PARENT):
            cv.use_id(JalouzeeBlinds),

        cv.Required(CONF_TYPE):
        cv.enum(
            {
                "angle": "angle",
                "position": "position",
            },
            lower=True,
        ),
    }
)


async def to_code(config):

    parent = await cg.get_variable(
        config[CONF_PARENT]
    )

    sens = await sensor.new_sensor(
        config
    )

    if config[CONF_TYPE] == "angle":

        cg.add(
            parent.set_angle_output(
                sens
            )
        )

    elif config[CONF_TYPE] == "position":

        cg.add(
            parent.set_position_output(
                sens
            )
        )
