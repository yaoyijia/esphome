
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ['wifi', 'uart']

gps_ntp_server_ns = cg.esphome_ns.namespace('gps_ntp_server')
GPSNTPServer = gps_ntp_server_ns.class_('GPSNTPServer', cg.Component)

CONF_UART_ID = 'uart_id'
CONF_PPS_PIN = 'pps_pin'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GPSNTPServer),
    cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=35),
    cv.Optional(CONF_UART_ID): cv.use_id(uart.UARTComponent),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    
    if CONF_UART_ID in config:
        uart_component = await cg.get_variable(config[CONF_UART_ID])
        cg.add(var.set_uart(uart_component))
