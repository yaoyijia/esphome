import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PIN

DEPENDENCIES = ['wifi']

simple_gps_ntp_ns = cg.esphome_ns.namespace('simple_gps_ntp')
SimpleGPSNTPServer = simple_gps_ntp_ns.class_('SimpleGPSNTPServer', cg.Component)

CONF_PPS_PIN = 'pps_pin'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SimpleGPSNTPServer),
    cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=35),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    
    # 自动查找可用的UART
    for uart_key, uart_conf in config.get('uart', {}).items():
        if isinstance(uart_key, str) and uart_key != 'id':
            uart_component = await cg.get_variable(uart_conf['id'])
            cg.add(var.set_uart(uart_component))
