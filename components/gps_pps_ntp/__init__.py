import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PIN, CONF_BAUD_RATE

DEPENDENCIES = ['wifi']
AUTO_LOAD = []

gps_pps_ntp_ns = cg.esphome_ns.namespace('gps_pps_ntp')
GPSPPSNTPServer = gps_pps_ntp_ns.class_('GPSPPSNTPServer', cg.Component)

CONF_PPS_PIN = 'pps_pin'
CONF_UART_ID = 'uart_id'  # 新增：支持引用外部UART

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(GPSPPSNTPServer),
    cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=35),
    
    # 支持两种方式：要么直接配置引脚，要么引用外部UART
    cv.Exclusive(CONF_BAUD_RATE, 'uart_config'): cv.positive_int,
    cv.Exclusive(CONF_UART_ID, 'uart_config'): cv.use_id(uart.UARTComponent),
    
    cv.Optional('rx_pin'): cv.int_range(min=0, max=35),
    cv.Optional('tx_pin'): cv.int_range(min=0, max=35),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_pps_pin(config[CONF_PPS_PIN]))
    
    # 如果配置了单独的UART总线
    if CONF_UART_ID in config:
        uart_component = await cg.get_variable(config[CONF_UART_ID])
        cg.add(var.set_uart_parent(uart_component))
    # 如果直接配置了引脚和波特率
    elif CONF_BAUD_RATE in config:
        cg.add(var.set_gps_baud_rate(config[CONF_BAUD_RATE]))
        if 'rx_pin' in config:
            cg.add(var.set_rx_pin(config['rx_pin']))
        if 'tx_pin' in config:
            cg.add(var.set_tx_pin(config['tx_pin']))
