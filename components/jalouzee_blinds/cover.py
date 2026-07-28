import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import cover, sensor, adc
from esphome.components.esp32 import add_idf_sdkconfig_option
from esphome.const import CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@your-github-handle"]
# 'adc' is auto-loaded (used internally to read the resistor on the motor
# shaft via ESPHome's built-in ADC component, without having to declare a
# separate 'sensor: platform: adc' platform in YAML)
AUTO_LOAD = [
    "sensor",
    "adc",
    "voltage_sampler",
    "button",
    "select",
    "number",
    "text_sensor",
    "binary_sensor",
]

jalouzee_blinds_ns = cg.esphome_ns.namespace("jalouzee_blinds")
JalouzeeBlinds = jalouzee_blinds_ns.class_("JalouzeeBlinds", cover.Cover, cg.Component)

# --- config keys ------------------------------------------------------------
CONF_MOTOR = "motor"
CONF_IN1 = "in1"
CONF_IN2 = "in2"

CONF_ENCODER = "encoder"
CONF_A = "a"
CONF_B = "b"
CONF_ADC = "adc"

CONF_ANGLE = "angle"

CONF_POSITION = "position"
CONF_FAULT_TIMEOUT = "fault_timeout"

# Physical BLE remote over a custom NimBLE advertising-relay protocol (see
# ble_relay.h/.cpp; not Bluetooth SIG BLE Mesh -- see
# docs/plans/vivid-noodling-gray.md for why) — opt-in since it pulls in
# NimBLE (CONFIG_BT_ENABLED etc.), which every existing config shouldn't
# have to pay for.
CONF_BLE_RELAY = "ble_relay"
# Shared fleet-wide NetKey (see docs/plans/vivid-noodling-gray.md's "Модель
# ключей и безопасность") -- 16 raw bytes, given in YAML as a 32-char hex
# string, meant to come from `!secret` like ota_pass/wifi_pass already do in
# this project (see config-template.yaml). Optional: this is the path for
# builds where whoever flashes it has ESPHome access (this maintainer's own
# installs). Left unset, the blind boots unprovisioned and gets its NetKey
# later over BLE (provision_chr_uuid_ in ble_relay.cpp) -- the path for a
# real customer with no ESPHome access, see "Provisioning NetKey без
# ESPHome" in the plan doc. No unsafe default either way: there's no
# placeholder key compiled in when this is left unset.
CONF_NET_KEY = "net_key"

# --- angle source mode options (see the "Angle Source" select) -------------
ANGLE_SOURCE_MODES = {
    "auto": 0,       # automatic: 1) angle (MPU6050)  2) Hall/ADC
    "angle": 1,      # force the angle sensor (MPU6050)
    "encoder": 2,    # force the Hall encoder OR resistor (whichever is set in YAML)
}

MOTOR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_IN1): pins.gpio_output_pin_schema,
        cv.Required(CONF_IN2): pins.gpio_output_pin_schema,
    }
)


def _validate_encoder(config):
    has_a = CONF_A in config
    has_b = CONF_B in config
    has_adc = CONF_ADC in config

    if has_a != has_b:
        raise cv.Invalid(
            "Both Hall encoder pins must be set: 'a' and 'b'"
        )

    has_hall = has_a and has_b

    if has_hall and has_adc:
        raise cv.Invalid(
            "'a'/'b' (Hall sensor) and 'adc' (resistor on the motor shaft) are "
            "mutually exclusive — specify only one way to determine the angle "
            "in the 'encoder' block"
        )
    if not has_hall and not has_adc:
        raise cv.Invalid(
            "The 'encoder' block needs either 'a' and 'b' (Hall sensor), "
            "or 'adc' (resistor on the motor shaft)"
        )
    return config


ENCODER_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_A): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_B): pins.internal_gpio_input_pin_schema,
            # adc.validate_adc_pin (not internal_gpio_input_pin_schema) — checks
            # ADC compatibility of the pin for the specific platform (e.g. on
            # ESP8266 it must be strictly A0/GPIO17).
            cv.Optional(CONF_ADC): adc.validate_adc_pin,
        }
    ),
    _validate_encoder,
)


def _validate_net_key(value):
    value = cv.string_strict(value)
    if len(value) != 32 or not all(c in "0123456789abcdefABCDEF" for c in value):
        raise cv.Invalid(
            "net_key must be exactly 32 hex characters (16 bytes), e.g. from "
            "`python3 -c \"import secrets; print(secrets.token_hex(16))\"`"
        )
    return value.lower()


def _validate_root(config):
    if CONF_ENCODER not in config and CONF_ANGLE not in config:
        raise cv.Invalid(
            "At least one angle source must be specified: 'encoder' and/or 'angle'"
        )
    mode = config[CONF_POSITION]
    if mode == "angle" and CONF_ANGLE not in config:
        raise cv.Invalid("position: angle is set, but the 'angle' parameter is missing")
    if mode == "encoder" and CONF_ENCODER not in config:
        raise cv.Invalid("position: encoder is set, but the 'encoder' block is missing")
    if config[CONF_BLE_RELAY] and not CORE.is_esp32:
        raise cv.Invalid("'ble_relay' requires an ESP32 (no BLE on this platform)")
    return config


CONFIG_SCHEMA = cv.All(
    cover.cover_schema(JalouzeeBlinds)
    .extend(
        {
            cv.Required(CONF_MOTOR): MOTOR_SCHEMA,
            cv.Optional(CONF_ENCODER): ENCODER_SCHEMA,
            # id of an existing sensor (e.g. from the esphome::mpu6050 platform,
            # a specific accelerometer axis, or a ready-made angle computed by a
            # separate template sensor). The component only calibrates the
            # "closed..open" range against this sensor's values.
            cv.Optional(CONF_ANGLE): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_POSITION, default="auto"): cv.enum(
                ANGLE_SOURCE_MODES, lower=True
            ),
            # how long the angle may stay unchanged while the motor is actively
            # moving before the component declares a fault (also available via
            # the API as a number)
            cv.Optional(CONF_FAULT_TIMEOUT, default="10s"): cv.All(
                cv.positive_time_period_seconds, cv.Range(min=cv.TimePeriod(seconds=1))
            ),
            cv.Optional(CONF_BLE_RELAY, default=False): cv.boolean,
            cv.Optional(CONF_NET_KEY): _validate_net_key,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _validate_root,
    cv.only_on(["esp32", "esp8266"]),
)


async def to_code(config):
    # calibration_button_, cancel_calibration_button_, fault_reset_button_,
    # angle_source_select_, fault_timeout_number_, calibration_text_sensor_,
    # calibration_step_sensor_, calibrated_binary_sensor_ and
    # fault_binary_sensor_ are created directly in C++ (register_sub_entities_)
    # rather than via YAML platforms. Without this, CORE.platform_counts stays
    # at 0/undersized for these domains, so USE_SELECT/USE_NUMBER (and the
    # ESPHOME_ENTITY_*_COUNT StaticVector sizing) never get emitted, and
    # register_select()/register_number() won't exist.
    for _ in range(3):
        CORE.register_platform_component("button", None)
    CORE.register_platform_component("select", None)
    CORE.register_platform_component("number", None)
    CORE.register_platform_component("text_sensor", None)
    # "sensor" is already used by the user's own YAML (e.g. the mpu6050
    # platform) when angle is configured, but that only accounts for THAT
    # sensor's slot — calibration_step_sensor_ is an extra one created here in
    # C++, so it needs its own count bump same as text_sensor/select/number
    # above.
    CORE.register_platform_component("sensor", None)
    # 2 base sensors ("Calibrated", "Fault") + per-source calibration
    # diagnostics, created only for sources actually configured (see
    # SubEntities::setup): 1 extra for encoder (Hall or ADC — mutually
    # exclusive) + 1 extra for angle.
    binary_sensor_count = 2
    if CONF_ENCODER in config:
        binary_sensor_count += 1
    if CONF_ANGLE in config:
        binary_sensor_count += 1
    for _ in range(binary_sensor_count):
        CORE.register_platform_component("binary_sensor", None)

    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    motor = config[CONF_MOTOR]
    in1 = await cg.gpio_pin_expression(motor[CONF_IN1])
    in2 = await cg.gpio_pin_expression(motor[CONF_IN2])
    cg.add(var.set_motor_pins(in1, in2))

    if CONF_ENCODER in config:
        enc = config[CONF_ENCODER]
        if CONF_A in enc:
            a = await cg.gpio_pin_expression(enc[CONF_A])
            b = await cg.gpio_pin_expression(enc[CONF_B])
            cg.add(var.set_hall_encoder_pins(a, b))
        else:
            adc_pin = await cg.gpio_pin_expression(enc[CONF_ADC])
            cg.add(var.set_adc_pin(adc_pin))

    if CONF_ANGLE in config:
        mpu_sens = await cg.get_variable(config[CONF_ANGLE])
        cg.add(var.set_mpu6050_sensor(mpu_sens))

    cg.add(var.set_angle_source_mode(ANGLE_SOURCE_MODES[config[CONF_POSITION]]))
    cg.add(var.set_fault_timeout(config[CONF_FAULT_TIMEOUT]))

    if config[CONF_BLE_RELAY]:
        cg.add(var.set_ble_relay_enabled(True))

        if CONF_NET_KEY in config:
            net_key_hex = config[CONF_NET_KEY]
            net_key_bytes = ", ".join(f"0x{net_key_hex[i:i + 2]}" for i in range(0, 32, 2))
            cg.add(var.set_net_key(cg.RawExpression(f"std::array<uint8_t, 16>{{{net_key_bytes}}}")))
        # else: no net_key in YAML -- factory firmware, boots unprovisioned
        # and gets its NetKey later over BLE (see ble_relay.cpp's
        # provision_chr_uuid_ and docs/plans/vivid-noodling-gray.md).

        # NimBLE host -- lighter-weight than Bluedroid, used directly (no
        # esp_ble_mesh, see ble_relay.h/.cpp and
        # docs/plans/vivid-noodling-gray.md for why). Mutually exclusive with
        # ESPHome's own esp32_ble component (which hardcodes Bluedroid) --
        # fine here since this project doesn't use esp32_ble.
        add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", False)
