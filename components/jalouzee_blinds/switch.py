import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_TYPE, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_JALOUZEE_BLINDS_ID, JalouzeeBlinds, jalouzee_blinds_ns

UseAngleSwitch = jalouzee_blinds_ns.class_("UseAngleSwitch", switch.Switch, cg.Component)
HasProblemSwitch = jalouzee_blinds_ns.class_("HasProblemSwitch", switch.Switch, cg.Component)

CONF_USE_ANGLE_SENSOR = "use_angle_sensor"
CONF_HAS_PROBLEM = "has_problem"

TYPES = {
    CONF_USE_ANGLE_SENSOR: UseAngleSwitch,
    CONF_HAS_PROBLEM: HasProblemSwitch,
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        CONF_USE_ANGLE_SENSOR: switch.switch_schema(UseAngleSwitch).extend(
            {cv.GenerateID(CONF_JALOUZEE_BLINDS_ID): cv.use_id(JalouzeeBlinds)}
        ),
        CONF_HAS_PROBLEM: switch.switch_schema(
            HasProblemSwitch,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend({cv.GenerateID(CONF_JALOUZEE_BLINDS_ID): cv.use_id(JalouzeeBlinds)}),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JALOUZEE_BLINDS_ID])
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(hub))
    if config[CONF_TYPE] == CONF_USE_ANGLE_SENSOR:
        cg.add(hub.set_use_angle_switch(var))
    else:
        cg.add(hub.set_has_problem_switch(var))
