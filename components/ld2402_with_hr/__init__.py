"""
LD2402 with Heart Rate Detection Component for ESPHome
包含心率检测功能的LD2402毫米波雷达组件
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import (
    CONF_ID,
    CONF_DISTANCE,
    UNIT_CENTIMETER,
    UNIT_BEATS_PER_MINUTE,
    ICON_MOTION_SENSOR,
    ICON_HEART_PULSE,
    ICON_LUNGS,
    DEVICE_CLASS_DISTANCE,
)

CODEOWNERS = ["@your_username"]

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

# 命名空间
ld2402_with_hr_ns = cg.esphome_ns.namespace("ld2402_with_hr")
LD2402WithHR = ld2402_with_hr_ns.class_("LD2402WithHR", cg.Component, uart.UARTDevice)

# 配置常量
CONF_DETECTION_STATE = "detection_state"
CONF_HEART_RATE = "heart_rate"
CONF_BREATH_RATE = "breath_rate"

# 配置模式 - 顶级组件
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LD2402WithHR),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CENTIMETER,
            icon=ICON_MOTION_SENSOR,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DISTANCE,
        ),
        cv.Optional(CONF_DETECTION_STATE): sensor.sensor_schema(
            icon=ICON_MOTION_SENSOR,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_HEART_RATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_BEATS_PER_MINUTE,
            icon=ICON_HEART_PULSE,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_BREATH_RATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_BEATS_PER_MINUTE,
            icon=ICON_LUNGS,
            accuracy_decimals=1,
        ),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

# UART配置验证
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2402_with_hr",
    require_tx=True,
    require_rx=True,
    baud_rate=115200,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    """生成代码"""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    # 距离传感器
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    
    # 检测状态传感器
    if CONF_DETECTION_STATE in config:
        sens = await sensor.new_sensor(config[CONF_DETECTION_STATE])
        cg.add(var.set_state_sensor(sens))
    
    # 心率传感器
    if CONF_HEART_RATE in config:
        sens = await sensor.new_sensor(config[CONF_HEART_RATE])
        cg.add(var.set_heart_rate_sensor(sens))
    
    # 呼吸率传感器
    if CONF_BREATH_RATE in config:
        sens = await sensor.new_sensor(config[CONF_BREATH_RATE])
        cg.add(var.set_breath_rate_sensor(sens))
