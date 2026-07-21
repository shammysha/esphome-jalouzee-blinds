import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import CONF_JALOUZEE_BLINDS_ID, JalouzeeBlinds, jalouzee_blinds_ns

CalibrateButton = jalouzee_blinds_ns.class_("CalibrateButton", button.Button, cg.Component)

CONFIG_SCHEMA = button.button_schema(
    CalibrateButton,
    icon="mdi:tune-vertical",
).extend(
    {
        cv.GenerateID(CONF_JALOUZEE_BLINDS_ID): cv.use_id(JalouzeeBlinds),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JALOUZEE_BLINDS_ID])
    var = await button.new_button(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(hub))
