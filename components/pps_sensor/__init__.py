import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_PIN

DEPENDENCIES = []
AUTO_LOAD = []

pps_sensor_ns = cg.esphome_ns.namespace("pps_sensor")
PPSSensor = pps_sensor_ns.class_("PPSSensor", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(PPSSensor),
    cv.Required(CONF_PIN): cv.int_range(min=0, max=35),
    cv.Optional("interval_sensor"): sensor.sensor_schema(
        unit_of_measurement="s",
        icon="mdi:pulse",
        accuracy_decimals=6,
    ),
}).extend(cv.polling_component_schema("1s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pin(config[CONF_PIN]))
    
    if "interval_sensor" in config:
        sens = await sensor.new_sensor(config["interval_sensor"])
        cg.add(var.set_sensor(sens))
