import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_DISTANCE,
    UNIT_CENTIMETER,
    ICON_MOTION_SENSOR,
    DEVICE_CLASS_DISTANCE,
)

from . import LD2402Minimal

DEPENDENCIES = ["ld2402_minimal"]

CONF_DETECTION_STATE = "detection_state"

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
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # 距离传感器
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    
    # 状态传感器
    if CONF_DETECTION_STATE in config:
        sens = await sensor.new_sensor(config[CONF_DETECTION_STATE])
        cg.add(var.set_state_sensor(sens))
