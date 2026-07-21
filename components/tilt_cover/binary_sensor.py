import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import CONF_TILT_COVER_ID, TiltCover

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_TILT_COVER_ID): cv.use_id(TiltCover),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_TILT_COVER_ID])
    var = await binary_sensor.new_binary_sensor(config)
    cg.add(hub.set_calibrated_binary_sensor(var))
