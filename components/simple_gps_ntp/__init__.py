import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PIN

DEPENDENCIES = ['wifi', 'uart']

simple_gps_ntp_ns = cg.esphome_ns.namespace('simple_gps_ntp')
SimpleGPSNTPServer = simple_gps_ntp_ns.class_('SimpleGPSNTPServer', cg.Component)

CONF_PPS_PIN = 'pps_pin'
CONF_UART_ID = 'uart_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SimpleGPSNTPServer),
    cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=35),
    
    # 可选：指定特定的UART组件
    cv.Optional(CONF_UART_ID): cv.use_id(uart.UARTComponent),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    
    # 如果指定了特定的UART组件，使用它
    if CONF_UART_ID in config:
        uart_component = await cg.get_variable(config[CONF_UART_ID])
        cg.add(var.set_uart(uart_component))
    else:
        # 否则，使用默认的UART组件
        # 需要确保配置中有uart组件
        pass
