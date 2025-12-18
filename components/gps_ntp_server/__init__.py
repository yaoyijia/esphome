import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import gps
from esphome.const import CONF_ID

DEPENDENCIES = ['wifi']

gps_ntp_server_ns = cg.esphome_ns.namespace('gps_ntp_server')
GPSNTPServer = gps_ntp_server_ns.class_('GPSNTPServer', cg.Component)

CONF_PPS_PIN = 'pps_pin'
CONF_GPS_ID = 'gps_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GPSNTPServer),
    cv.Optional(CONF_PPS_PIN, default=0): cv.int_range(min=0, max=35),
    cv.Optional(CONF_GPS_ID): cv.use_id(gps.GPS),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    
    # GPS组件是可选的，用于兼容性
    if CONF_GPS_ID in config:
        gps_component = await cg.get_variable(config[CONF_GPS_ID])
        cg.add(var.set_gps(gps_component))
