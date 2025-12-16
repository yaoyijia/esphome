import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ['wifi']
AUTO_LOAD = []

ntp_server_ns = cg.esphome_ns.namespace('ntp_server')
NTP_Server = ntp_server_ns.class_('NTP_Server', cg.Component)

# 定义新的配置项，用于接收其他组件的引用
CONF_CUSTOM_GPS_TIME = 'custom_gps_time_id'
CONF_PPS_SENSOR = 'pps_sensor_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(NTP_Server),
    cv.Optional(CONF_CUSTOM_GPS_TIME): cv.use_id(cg.esphome_ns.namespace('custom_gps_time').CustomGPSTime),
    cv.Optional(CONF_PPS_SENSOR): cv.use_id(cg.esphome_ns.namespace('pps_sensor').PPSSensor),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # 如果配置中提供了custom_gps_time组件的ID，则获取其变量并传递给C++
    if CONF_CUSTOM_GPS_TIME in config:
        time_comp = await cg.get_variable(config[CONF_CUSTOM_GPS_TIME])
        cg.add(var.set_time_component(time_comp))
    
    # 如果配置中提供了pps_sensor组件的ID，则获取其变量并传递给C++
    if CONF_PPS_SENSOR in config:
        pps_comp = await cg.get_variable(config[CONF_PPS_SENSOR])
        cg.add(var.set_pps_component(pps_comp))
