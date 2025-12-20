#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <cmath>

namespace esphome::ld2402_with_hr {

class LD2402WithHR : public Component, public uart::UARTDevice {
 public:
  // 传感器设置
  void set_distance_sensor(sensor::Sensor *sensor) { distance_sensor_ = sensor; }
  void set_state_sensor(sensor::Sensor *sensor) { state_sensor_ = sensor; }
  void set_heart_rate_sensor(sensor::Sensor *sensor) { heart_rate_sensor_ = sensor; }
  void set_breath_rate_sensor(sensor::Sensor *sensor) { breath_rate_sensor_ = sensor; }
  
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

 private:
  // 检测状态枚举
  enum DetectionState : uint8_t {
    DETECTION_NONE = 0x00,    // 无人
    DETECTION_MOVING = 0x01,  // 有人移动
    DETECTION_STILL = 0x02    // 有人静止
  };
  
  // 帧标识
  static const uint32_t ENERGY_FRAME_HEADER = 0xF1F2F3F4;
  static const uint32_t ENERGY_FRAME_FOOTER = 0xF5F6F7F8;
  static const uint16_t MAX_BUFFER_SIZE = 300;
  
  // 心率检测参数
  static const uint16_t SIGNAL_BUFFER_SIZE = 64;    // 约10.6秒数据 (64 * 165ms)
  static const float SAMPLING_RATE;                 // 6.06Hz (1000ms/165ms)
  static const uint16_t HR_ANALYSIS_INTERVAL_MS;    // 每10秒分析一次
  
  // 发送配置命令
  void send_config_command(uint16_t command, const uint8_t* data = nullptr, uint8_t data_len = 0);
  
  // 解析数据帧
  void parse_energy_frame(const uint8_t* buffer, uint16_t len);
  
  // 心率检测
  void check_and_analyze_vital_signs(uint16_t gate_energy);
  void analyze_vital_signs();
  float calculate_heart_rate();
  float calculate_breath_rate();
  void apply_hanning_window(float *signal, uint16_t length);
  float goertzel_algorithm(const float *signal, uint16_t length, float target_freq, float sampling_rate);
  
  // 传感器
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *state_sensor_{nullptr};
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *breath_rate_sensor_{nullptr};
  
  // 缓冲区
  uint8_t buffer_[MAX_BUFFER_SIZE];
  uint16_t buffer_pos_{0};
  
  // 当前状态
  uint16_t distance_{0};
  uint8_t detection_state_{DETECTION_NONE};
  
  // 心率检测相关
  std::vector<float> energy_history_;
  uint16_t history_index_{0};
  uint32_t last_analysis_time_{0};
  bool tracking_person_{false};
  uint8_t consecutive_person_frames_{0};
  
  // 初始化标志
  bool initialized_{false};
};

}  // namespace esphome::ld2402_with_hr
