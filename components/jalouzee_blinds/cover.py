import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import cover, sensor, adc
from esphome.const import CONF_ID

CODEOWNERS = ["@your-github-handle"]
DEPENDENCIES = ["esp32"]
# 'adc' подключаем автоматически (используется внутри компонента для чтения
# резистора на оси мотора через штатный ESP-IDF ADC-драйвер ESPHome, без
# необходимости объявлять отдельную платформу 'sensor: platform: adc' в YAML)
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

# --- ключи конфигурации ---------------------------------------------------
CONF_MOTOR = "motor"
CONF_IN1 = "in1"
CONF_IN2 = "in2"

CONF_ENCODER = "encoder"
CONF_A = "a"
CONF_B = "b"
CONF_ADC = "adc"

CONF_MPU6050 = "mpu6050"
CONF_ANGLE_SENSOR = "angle_sensor"

CONF_ANGLE_SOURCE = "angle_source"
CONF_FAULT_TIMEOUT = "fault_timeout"

# --- варианты режима определения угла (см. select "Angle Source") --------
ANGLE_SOURCE_MODES = {
    "auto": 0,       # автоматический выбор: 1) MPU6050  2) Hall/ADC
    "mpu6050": 1,    # принудительно MPU6050
    "encoder": 2,    # принудительно Hall-энкодер ИЛИ резистор (что задано в YAML)
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
            "Для датчика Холла нужно указать ОБА пина энкодера: 'a' и 'b'"
        )

    has_hall = has_a and has_b

    if has_hall and has_adc:
        raise cv.Invalid(
            "'a'/'b' (датчик Холла) и 'adc' (резистор на оси мотора) взаимоисключающие "
            "— укажите только один способ определения угла в блоке 'encoder'"
        )
    if not has_hall and not has_adc:
        raise cv.Invalid(
            "В блоке 'encoder' нужно указать либо 'a' и 'b' (датчик Холла), "
            "либо 'adc' (резистор на оси мотора)"
        )
    return config


ENCODER_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_A): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_B): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ADC): pins.internal_gpio_input_pin_schema,
        }
    ),
    _validate_encoder,
)

MPU6050_SCHEMA = cv.Schema(
    {
        # angle_sensor — id уже существующего sensor (например, из платформы
        # esphome::mpu6050, конкретная ось акселерометра либо готовый угол,
        # вычисленный отдельным template sensor). Компонент лишь калибрует
        # диапазон "закрыто..открыто" по значениям этого сенсора.
        cv.Required(CONF_ANGLE_SENSOR): cv.use_id(sensor.Sensor),
    }
)


def _validate_root(config):
    if CONF_ENCODER not in config and CONF_MPU6050 not in config:
        raise cv.Invalid(
            "Нужно указать хотя бы один источник угла наклона: 'encoder' и/или 'mpu6050'"
        )
    mode = config[CONF_ANGLE_SOURCE]
    if mode == "mpu6050" and CONF_MPU6050 not in config:
        raise cv.Invalid("angle_source: mpu6050 указан, но блок 'mpu6050' отсутствует")
    if mode == "encoder" and CONF_ENCODER not in config:
        raise cv.Invalid("angle_source: encoder указан, но блок 'encoder' отсутствует")
    return config


CONFIG_SCHEMA = cv.All(
    cover.cover_schema(JalouzeeBlinds)
    .extend(
        {
            cv.Required(CONF_MOTOR): MOTOR_SCHEMA,
            cv.Optional(CONF_ENCODER): ENCODER_SCHEMA,
            cv.Optional(CONF_MPU6050): MPU6050_SCHEMA,
            cv.Optional(CONF_ANGLE_SOURCE, default="auto"): cv.enum(
                ANGLE_SOURCE_MODES, lower=True
            ),
            # период, за который угол должен измениться при движении,
            # иначе компонент объявит аварию (доступно и в API как number)
            cv.Optional(CONF_FAULT_TIMEOUT, default="10s"): cv.All(
                cv.positive_time_period_seconds, cv.Range(min=cv.TimePeriod(seconds=1))
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _validate_root,
    cv.only_on(["esp32"]),
)


async def to_code(config):
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

    if CONF_MPU6050 in config:
        mpu_sens = await cg.get_variable(config[CONF_MPU6050][CONF_ANGLE_SENSOR])
        cg.add(var.set_mpu6050_sensor(mpu_sens))

    cg.add(var.set_angle_source_mode(ANGLE_SOURCE_MODES[config[CONF_ANGLE_SOURCE]]))
    cg.add(var.set_fault_timeout(config[CONF_FAULT_TIMEOUT]))
