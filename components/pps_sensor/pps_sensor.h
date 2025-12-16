#pragma once
#include "esphome.h"

namespace esphome {
namespace pps_sensor {

class PPSSensor : public PollingComponent, public Sensor {
 public:
  // 设置PPS引脚
  void set_pin(uint8_t pin) { pps_pin_ = pin; }
  
  void setup() override;
  void update() override; // PollingComponent的循环函数
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }

  // 在中断中更新脉冲计数（必须用ICACHE_RAM_ATTR）
  static void IRAM_ATTR pps_interrupt_handler();

 protected:
  uint8_t pps_pin_;
  volatile uint32_t last_pps_micros_ = 0;    // 最近一次PPS发生时的微秒数
  volatile uint32_t pps_interval_us_ = 0;    // 计算的脉冲间隔（微秒）
  volatile uint32_t pulse_count_ = 0;        // 脉冲计数器，用于检测新脉冲
  volatile bool pps_updated_ = false;        // 标志位，表示有新数据
  static PPSSensor *instance_;               // 静态实例指针，用于中断回调
};

}  // namespace pps_sensor
}  // namespace esphome
