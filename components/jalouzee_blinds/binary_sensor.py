import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import CONF_JALOUZEE_BLINDS_ID, JalouzeeBlinds

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_JALOUZEE_BLINDS_ID): cv.use_id(JalouzeeBlinds),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JALOUZEE_BLINDS_ID])
    var = await binary_sensor.new_binary_sensor(config)
    cg.add(hub.set_calibrated_binary_sensor(var))
