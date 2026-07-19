import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import button

from .. import jalouzee_blinds_ns, JalouzeeBlinds


CONF_PARENT = "parent"
CONF_ACTION = "action"


JalouzeeButton = jalouzee_blinds_ns.class_(
    "JalouzeeButton",
    button.Button,
    includes=[
        "entities.h",
    ],    
)


Action = JalouzeeButton.enum("Action")


ACTION_MAP = {
    "start_calibration":
        Action.START_CALIBRATION,

    "save_closed":
        Action.SAVE_CLOSED,

    "save_open":
        Action.SAVE_OPEN,

    "clear_fault":
        Action.CLEAR_FAULT,
}


CONFIG_SCHEMA = button.button_schema(
    JalouzeeButton
).extend(
    {

        cv.Required(CONF_PARENT):
            cv.use_id(JalouzeeBlinds),


        cv.Required(CONF_ACTION):
            cv.enum(
                ACTION_MAP,
                lower=True
            ),

    }
)



async def to_code(config):

    var = await button.new_button(
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


    cg.add(
        var.set_action(
            config[CONF_ACTION]
        )
    )