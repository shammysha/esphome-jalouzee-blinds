import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import CONF_TILT_COVER_ID, TiltCover, tilt_cover_ns

CalibrateButton = tilt_cover_ns.class_("CalibrateButton", button.Button, cg.Component)

CONFIG_SCHEMA = button.button_schema(
    CalibrateButton,
    icon="mdi:tune-vertical",
).extend(
    {
        cv.GenerateID(CONF_TILT_COVER_ID): cv.use_id(TiltCover),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_TILT_COVER_ID])
    var = await button.new_button(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(hub))
