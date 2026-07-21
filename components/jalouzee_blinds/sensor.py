import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from . import CONF_JALOUZEE_BLINDS_ID, JalouzeeBlinds

CONFIG_SCHEMA = sensor.sensor_schema(
    icon="mdi:tune-vertical",
    accuracy_decimals=0,
).extend(
    {
        cv.GenerateID(CONF_JALOUZEE_BLINDS_ID): cv.use_id(JalouzeeBlinds),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JALOUZEE_BLINDS_ID])
    var = await sensor.new_sensor(config)
    cg.add(hub.set_calibrate_step_sensor(var))
