import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import (
    cover,
    sensor,
    binary_sensor,
    button,
    select,
)

from esphome import pins
from esphome.const import CONF_ID

DEPENDENCIES = [
    "cover",
]

AUTO_LOAD = [
    "sensor",
    "binary_sensor",
    "button",
    "select",
]

CONF_OPEN_PIN = "open_pin"
CONF_CLOSE_PIN = "close_pin"

CONF_PRIMARY_SENSOR = "primary_sensor"
CONF_SECONDARY_SENSOR = "secondary_sensor"

CONF_STALL_TIMEOUT = "stall_timeout"

CONF_ENTITIES = "entities"
CONF_BUTTONS = "buttons"
CONF_ANGLE_SOURCE = "angle_source"

jalouzee_blinds_ns = cg.esphome_ns.namespace(
    "jalouzee_blinds"
)

JalouzeeBlinds = jalouzee_blinds_ns.class_(
    "JalouzeeBlinds",
    cover.Cover,
    cg.Component,
    is_class=True,
)

JalouzeeButton = jalouzee_blinds_ns.class_(
    "JalouzeeButton",
    button.Button,
)

AngleSourceSelect = jalouzee_blinds_ns.class_(
    "AngleSourceSelect",
    select.Select,
)

ButtonAction = JalouzeeButton.enum(
    "Action"
)

BUTTON_ACTIONS = {

    "calibration":
        ButtonAction.START_CALIBRATION,

    "save_closed":
        ButtonAction.SAVE_CLOSED,

    "save_open":
        ButtonAction.SAVE_OPEN,

    "clear_fault":
        ButtonAction.CLEAR_FAULT,

}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID():
                cv.declare_id(
                    JalouzeeBlinds
                ),

            cv.Required(CONF_OPEN_PIN):
                pins.gpio_output_pin_schema,

            cv.Required(CONF_CLOSE_PIN):
                pins.gpio_output_pin_schema,

            cv.Optional(CONF_PRIMARY_SENSOR):
                cv.use_id(sensor.Sensor),

            cv.Optional(CONF_SECONDARY_SENSOR):
                cv.use_id(sensor.Sensor),

            cv.Optional(
                CONF_STALL_TIMEOUT,
                default="10s"
            ):
                cv.positive_time_period_milliseconds,

            cv.Optional(CONF_ENTITIES):
                cv.Schema(
                    {
                        cv.Optional("angle"):
                            cv.boolean,

                        cv.Optional("position"):
                            cv.boolean,

                        cv.Optional("fault"):
                            cv.boolean,

                        cv.Optional("calibrated"):
                            cv.boolean,
                    }
                ),

            cv.Optional(CONF_BUTTONS):
                cv.Schema(
                    {
                        cv.Optional(name):
                            cv.boolean
                        for name in BUTTON_ACTIONS
                    }
                ),

            cv.Optional(CONF_ANGLE_SOURCE):
                cv.use_id(
                    AngleSourceSelect
                ),

        }
    )
    .extend(
        cover.cover_schema(
            JalouzeeBlinds
        )
    )
)


async def to_code(config):

    var = cg.new_Pvariable(
        config[CONF_ID]
    )

    await cg.register_component(
        var,
        config
    )

    await cover.register_cover(
        var,
        config
    )

    #
    # Motor GPIO
    #

    open_pin = await cg.gpio_pin_expression(
        config[CONF_OPEN_PIN]
    )

    close_pin = await cg.gpio_pin_expression(
        config[CONF_CLOSE_PIN]
    )

    cg.add(
        var.set_motor_pins(
            open_pin,
            close_pin
        )
    )

    #
    # Angle sensors
    #

    if CONF_PRIMARY_SENSOR in config:

        primary = await cg.get_variable(
            config[CONF_PRIMARY_SENSOR]
        )

        cg.add(
            var.set_primary_sensor(
                primary
            )
        )

    if CONF_SECONDARY_SENSOR in config:

        secondary = await cg.get_variable(
            config[CONF_SECONDARY_SENSOR]
        )

        cg.add(
            var.set_secondary_sensor(
                secondary
            )
        )

    #
    # Stall timeout
    #

    cg.add(
        var.set_stall_timeout(
            config[CONF_STALL_TIMEOUT]
        )
    )

    #
    # Internal entities
    #

    entities = config.get(
        CONF_ENTITIES,
        {}
    )

    if entities.get(
        "angle",
        False
    ):

        sens = await sensor.new_sensor(
            {
                "name":
                    "Angle",
            }
        )

        cg.add(
            var.set_angle_output(
                sens
            )
        )

    if entities.get(
        "position",
        False
    ):

        sens = await sensor.new_sensor(
            {
                "name":
                    "Position",
            }
        )

        cg.add(
            var.set_position_output(
                sens
            )
        )

    if entities.get(
        "fault",
        False
    ):

        bsens = await binary_sensor.new_binary_sensor(
            {
                "name":
                    "Fault",
            }
        )

        cg.add(
            var.set_fault_output(
                bsens
            )
        )

    if entities.get(
        "calibrated",
        False
    ):

        bsens = await binary_sensor.new_binary_sensor(
            {
                "name":
                    "Calibrated",
            }
        )

        cg.add(
            var.set_calibrated_output(
                bsens
            )
        )

    #
    # Buttons
    #

    buttons = config.get(
        CONF_BUTTONS,
        {}
    )

    for name, action in BUTTON_ACTIONS.items():

        if buttons.get(
            name,
            False
        ):

            btn = await button.new_button(
                {
                    "name":
                        name.replace(
                            "_",
                            " "
                        ).title()
                }
            )

            cg.add(
                btn.set_parent(
                    var
                )
            )

            cg.add(
                btn.set_action(
                    action
                )
            )

    #
    # Angle source select
    #

    if CONF_ANGLE_SOURCE in config:

        select = await select.new_select(
            {
                "name":
                    "Angle Source"
            }
        )

        cg.add(
            select.set_parent(
                var
            )
        )
