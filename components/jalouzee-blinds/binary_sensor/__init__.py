import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import binary_sensor

from .. import JalouzeeBlinds


CONF_PARENT = "parent"
CONF_TYPE = "type"


CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.Required(CONF_PARENT):
            cv.use_id(JalouzeeBlinds),


        cv.Required(CONF_TYPE):
            cv.enum(
                {
                    "fault",
                    "calibrated",
                },
                lower=True
            ),
    }
)


async def to_code(config):

    parent = await cg.get_variable(
        config[CONF_PARENT]
    )


    bs = await binary_sensor.new_binary_sensor(
        config
    )


    if config[CONF_TYPE] == "fault":

        cg.add(
            parent.set_fault_output(
                bs
            )
        )


    elif config[CONF_TYPE] == "calibrated":

        cg.add(
            parent.set_calibrated_output(
                bs
            )
        )