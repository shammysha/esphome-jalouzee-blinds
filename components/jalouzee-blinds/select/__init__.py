import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import select

from .. import jalouzee_blinds_ns, JalouzeeBlinds


CONF_PARENT = "parent"


AngleSourceSelect = jalouzee_blinds_ns.class_(
    "AngleSourceSelect",
    select.Select,
    includes=[
        "entities.h",
    ],    
)


CONFIG_SCHEMA = select.select_schema(
    AngleSourceSelect
).extend(
    {
        cv.Required(CONF_PARENT):
            cv.use_id(JalouzeeBlinds),
    }
)


async def to_code(config):

    var = await select.new_select(
        config
    )


    parent = await cg.get_variable(
        config[CONF_PARENT]
    )


    cg.add(
        var.set_parent(
            parent
        )
    )