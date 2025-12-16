import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import gps
from esphome.const import CONF_ID

DEPENDENCIES = ['gps']
AUTO_LOAD = []

custom_gps_time_ns = cg.esphome_ns.namespace('custom_gps_time')
CustomGPSTime = custom_gps_time_ns.class_('CustomGPSTime', cg.Component, gps.GPSListener)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CustomGPSTime),
    cv.Required('gps_id'): cv.use_id(gps.GPS),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # 关联到GPS父组件
    gps_component = await cg.get_variable(config['gps_id'])
    cg.add(var.set_gps_parent(gps_component))
