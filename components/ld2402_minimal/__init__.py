"""
LD2402 Minimal Component for ESPHome
极简的LD2402毫米波雷达组件，包含心率检测功能（实验性）
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import (
    CONF_ID,
    CONF_DISTANCE,
    UNIT_CENTIMETER,
    ICON_MOTION_SENSOR,
    DEVICE_CLASS_DISTANCE,
)

CODEOWNERS = ["@your_username"]

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

# 命名空间
ld2402_minimal_ns = cg.esphome_ns.namespace("ld2402_minimal")
LD2402Minimal = ld2402_minimal_ns.class_("LD2402Minimal", cg.Component, uart.UARTDevice)

# 配置常量
CONF_DETECTION_STATE = "detection_state"
CONF_HEART_RATE = "heart_rate"
CONF_BREATH_RATE = "breath_rate"
CONF_HEART_RATE_RAW = "heart_rate_raw"

# 配置模式
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LD2402Minimal),
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
            unit_of_measurement="BPM",
            icon="mdi:heart-pulse",
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_BREATH_RATE): sensor.sensor_schema(
            unit_of_measurement="BPM",
            icon="mdi:lungs",
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_HEART_RATE_RAW): sensor.sensor_schema(
            name="Heart Rate Raw",
            icon="mdi:chart-line",
            accuracy_decimals=0,
        ),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

# UART配置验证
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2402_minimal",
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
    
    # 设置距离传感器
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    
    # 设置状态传感器
    if CONF_DETECTION_STATE in config:
        sens = await sensor.new_sensor(config[CONF_DETECTION_STATE])
        cg.add(var.set_state_sensor(sens))
    
    # 设置心率传感器
    if CONF_HEART_RATE in config:
        sens = await sensor.new_sensor(config[CONF_HEART_RATE])
        cg.add(var.set_heart_rate_sensor(sens))
    
    # 设置呼吸率传感器
    if CONF_BREATH_RATE in config:
        sens = await sensor.new_sensor(config[CONF_BREATH_RATE])
        cg.add(var.set_breath_rate_sensor(sens))
    
    # 设置原始心率传感器
    if CONF_HEART_RATE_RAW in config:
        sens = await sensor.new_sensor(config[CONF_HEART_RATE_RAW])
        cg.add(var.set_heart_rate_raw_sensor(sens))
