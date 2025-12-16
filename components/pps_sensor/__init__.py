import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    UNIT_SECOND,
    ICON_PULSE,
    STATE_CLASS_MEASUREMENT,
)

pps_sensor_ns = cg.esphome_ns.namespace("pps_sensor")
PPSSensor = pps_sensor_ns.class_("PPSSensor", sensor.Sensor, cg.PollingComponent)

CONF_PIN = "pin"

CONFIG_SCHEMA = sensor.sensor_schema(
    PPSSensor,
    unit_of_measurement=UNIT_SECOND,
    icon=ICON_PULSE,
    accuracy_decimals=6,
    state_class=STATE_CLASS_MEASUREMENT,
).extend({
    cv.Required(CONF_PIN): cv.int_range(min=0, max=35),
}).extend(cv.polling_component_schema("60s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    
    cg.add(var.set_pin(config[CONF_PIN]))
