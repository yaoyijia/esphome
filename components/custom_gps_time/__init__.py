import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import time, sensor
from esphome.const import CONF_ID, CONF_TIME_ID

DEPENDENCIES = ['gps']
AUTO_LOAD = []

custom_gps_time_ns = cg.esphome_ns.namespace('custom_gps_time')
CustomGPSTime = custom_gps_time_ns.class_('CustomGPSTime', time.RealTimeClock, cg.Component)

CONFIG_SCHEMA = time.TIME_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(CustomGPSTime),
    cv.Required('gps_id'): cv.use_id(sensor.Sensor), # 关联GPS传感器组件
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await time.register_time(var, config)
    
    # 将配置中的 gps_id 与我们的C++组件关联
    gps_component = await cg.get_variable(config['gps_id'])
    cg.add(var.set_gps_parent(gps_component))
