import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import CONF_JALOUZEE_BLINDS_ID, JalouzeeBlinds

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(CONF_JALOUZEE_BLINDS_ID): cv.use_id(JalouzeeBlinds),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JALOUZEE_BLINDS_ID])
    var = await text_sensor.new_text_sensor(config)
    cg.add(hub.set_calibrate_message_sensor(var))
