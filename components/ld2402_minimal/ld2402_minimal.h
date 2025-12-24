#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::ld2402_minimal {

class LD2402Minimal : public Component, public uart::UARTDevice {
 public:
  // 基础传感器
  void set_distance_sensor(sensor::Sensor *sensor) { distance_sensor_ = sensor; }
  void set_state_sensor(sensor::Sensor *sensor) { state_sensor_ = sensor; }
  
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

 private:
  // 帧标识
  static const uint32_t ENERGY_FRAME_HEADER = 0xF1F2F3F4;  // 小端: F4 F3 F2 F1
  static const uint32_t ENERGY_FRAME_FOOTER = 0xF5F6F7F8;  // 小端: F8 F7 F6 F5
  
  // 缓冲区大小
  static const uint16_t MAX_BUFFER_SIZE = 300;
  
  // 解析数据帧
  void parse_energy_frame(const uint8_t *buffer, uint16_t len);
  
  // 发送配置命令
  void send_config_command(const uint8_t *cmd, size_t len);
  
  // 传感器指针
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *state_sensor_{nullptr};
  
  // 缓冲区
  uint8_t buffer_[MAX_BUFFER_SIZE];
  uint16_t buffer_pos_{0};
  
  // 当前状态
  bool initialized_{false};
  uint16_t distance_{0};
  uint8_t detection_state_{0};
};

}  // namespace esphome::ld2402_minimal
