"""jalouzee_blinds: external component for a hall/encoder or MPU6050-angle driven tilt blind.

This is the hub. It owns the motor pins, the encoder pins and (optionally) a
reference to an already-configured angle sensor (e.g. the accel_x sub-sensor
of a `sensor.mpu6050`). All of the actual control logic lives in
JalouzeeBlinds (see jalouzee_blinds.h / jalouzee_blinds.cpp).

Child entities (cover, binary_sensor, sensor, text_sensor, switch, button)
are declared in their own platform files in this directory and attach
themselves to this hub via `jalouzee_blinds_id`.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.pins as pins
from esphome.components import sensor
from esphome.const import CONF_ID

CODEOWNERS = ["@shammysha"]

jalouzee_blinds_ns = cg.esphome_ns.namespace("jalouzee_blinds")
JalouzeeBlinds = jalouzee_blinds_ns.class_("JalouzeeBlinds", cg.Component)

CONF_PIN_CW = "pin_cw"
CONF_PIN_CCW = "pin_ccw"
CONF_PIN_PH_A = "pin_ph_a"
CONF_PIN_PH_B = "pin_ph_b"
CONF_ANGLE_SENSOR = "angle_sensor"
CONF_JALOUZEE_BLINDS_ID = "jalouzee_blinds_id"

MULTI_CONF = True

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(JalouzeeBlinds),
        cv.Required(CONF_PIN_CW): pins.gpio_output_pin_schema,
        cv.Required(CONF_PIN_CCW): pins.gpio_output_pin_schema,
        cv.Required(CONF_PIN_PH_A): pins.gpio_input_pin_schema,
        cv.Required(CONF_PIN_PH_B): pins.gpio_input_pin_schema,
        # Optional: point this at the accel_x sub-sensor of an mpu6050
        # (or any other sensor.* you want to drive position from).
        # If omitted, the hub falls back to counting encoder pulses.
        cv.Optional(CONF_ANGLE_SENSOR): cv.use_id(sensor.Sensor),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cw_pin = await cg.gpio_pin_expression(config[CONF_PIN_CW])
    cg.add(var.set_cw_pin(cw_pin))
    ccw_pin = await cg.gpio_pin_expression(config[CONF_PIN_CCW])
    cg.add(var.set_ccw_pin(ccw_pin))
    ph_a_pin = await cg.gpio_pin_expression(config[CONF_PIN_PH_A])
    cg.add(var.set_ph_a_pin(ph_a_pin))
    ph_b_pin = await cg.gpio_pin_expression(config[CONF_PIN_PH_B])
    cg.add(var.set_ph_b_pin(ph_b_pin))

    if CONF_ANGLE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_ANGLE_SENSOR])
        cg.add(var.set_angle_sensor(sens))
