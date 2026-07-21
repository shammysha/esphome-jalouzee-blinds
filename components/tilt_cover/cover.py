import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from . import CONF_TILT_COVER_ID, TiltCover, tilt_cover_ns

TiltCoverOutput = tilt_cover_ns.class_("TiltCoverOutput", cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.cover_schema(TiltCoverOutput).extend(
    {
        cv.GenerateID(CONF_TILT_COVER_ID): cv.use_id(TiltCover),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_TILT_COVER_ID])
    var = await cover.new_cover(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(hub))
    cg.add(hub.set_cover(var))
