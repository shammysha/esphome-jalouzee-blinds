import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from . import CONF_JALOUZEE_BLINDS_ID, JalouzeeBlinds, jalouzee_blinds_ns

JalouzeeBlindsOutput = jalouzee_blinds_ns.class_("JalouzeeBlindsOutput", cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.cover_schema(JalouzeeBlindsOutput).extend(
    {
        cv.GenerateID(CONF_JALOUZEE_BLINDS_ID): cv.use_id(JalouzeeBlinds),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JALOUZEE_BLINDS_ID])
    var = await cover.new_cover(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(hub))
    cg.add(hub.set_cover(var))
