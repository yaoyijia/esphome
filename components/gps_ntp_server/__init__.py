
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import gps
from esphome.const import CONF_ID

DEPENDENCIES = ['wifi', 'gps']

gps_ntp_server_ns = cg.esphome_ns.namespace('gps_ntp_server')
GPSNTPServer = gps_ntp_server_ns.class_('GPSNTPServer', cg.Component, gps.GPSListener)

CONF_PPS_PIN = 'pps_pin'
CONF_GPS_ID = 'gps_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GPSNTPServer),
    cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=35),
    cv.Required(CONF_GPS_ID): cv.use_id(gps.GPS),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    
    gps_component = await cg.get_variable(config[CONF_GPS_ID])
    cg.add(var.set_gps(gps_component))

