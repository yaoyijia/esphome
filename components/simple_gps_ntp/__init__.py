
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PIN

DEPENDENCIES = ['wifi', 'uart']

simple_gps_ntp_ns = cg.esphome_ns.namespace('simple_gps_ntp')
SimpleGPSNTPServer = simple_gps_ntp_ns.class_('SimpleGPSNTPServer', cg.Component)

CONF_PPS_PIN = 'pps_pin'
CONF_UART_ID = 'uart_id'
CONF_DEBUG_LEVEL = 'debug_level'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SimpleGPSNTPServer),
    cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=35),
    
    cv.Optional(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_DEBUG_LEVEL, default=1): cv.int_range(min=0, max=3),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    cg.add(var.set_debug_level(config[CONF_DEBUG_LEVEL]))
    
    if CONF_UART_ID in config:
        uart_component = await cg.get_variable(config[CONF_UART_ID])
        cg.add(var.set_uart(uart_component))
