import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PIN, CONF_BAUD_RATE

DEPENDENCIES = ['wifi', 'uart']
AUTO_LOAD = []

gps_pps_ntp_ns = cg.esphome_ns.namespace('gps_pps_ntp')
GPSPPSNTPServer = gps_pps_ntp_ns.class_('GPSPPSNTPServer', cg.Component, uart.UARTDevice)

CONF_PPS_PIN = 'pps_pin'
CONF_GPS_BAUD_RATE = 'gps_baud_rate'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GPSPPSNTPServer),
    cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=35),
    cv.Optional(CONF_GPS_BAUD_RATE, default=9600): cv.positive_int,
}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    cg.add(var.set_gps_baud_rate(config[CONF_GPS_BAUD_RATE]))
