#pragma once
#include "esphome.h"

namespace esphome {
namespace pps_sensor {

class PPSSensor : public PollingComponent {
 public:
  // 构造函数：初始化传感器
  PPSSensor() : PollingComponent(1000) {} // 默认1秒更新间隔

  // 设置引脚和传感器指针
  void set_pin(uint8_t pin) { pps_pin_ = pin; }
  void set_sensor(sensor::Sensor *sensor) { interval_sensor_ = sensor; }
  
  void setup() override;
  void update() override;
  
  // 硬件优先级
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }
  // 添加一个公共方法，用于获取最新的间隔值（秒）

  float get_interval_s() const { return last_calculated_interval_s_; }
  uint32_t get_last_pps_micros() const { return last_pps_micros_; }

 protected:
  // 中断处理函数（必须是静态的）
  static void IRAM_ATTR pps_interrupt_handler();
  
  // 成员变量
  uint8_t pps_pin_;
  sensor::Sensor *interval_sensor_{nullptr};
  
  // 中断共享变量（必须用volatile）
  volatile uint32_t last_pps_micros_ = 0;
  volatile uint32_t pps_interval_us_ = 0;
  volatile uint32_t pulse_count_ = 0;
  volatile bool pps_updated_ = false;
  
  // 静态实例指针，用于中断回调
  static PPSSensor *instance_;
  // +++ 新增：用于稳定读取的间隔值 +++
  float last_calculated_interval_s_{0.0f};
};

}  // namespace pps_sensor
}  // namespace esphome
