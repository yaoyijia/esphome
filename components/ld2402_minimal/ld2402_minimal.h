#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <deque>

namespace esphome::ld2402_minimal {

class LD2402Minimal : public Component, public uart::UARTDevice {
 public:
  // 基础传感器
  void set_distance_sensor(sensor::Sensor *sensor) { distance_sensor_ = sensor; }
  void set_state_sensor(sensor::Sensor *sensor) { state_sensor_ = sensor; }
  
  // 生命体征传感器
  void set_heart_rate_sensor(sensor::Sensor *sensor) { heart_rate_sensor_ = sensor; }
  void set_breath_rate_sensor(sensor::Sensor *sensor) { breath_rate_sensor_ = sensor; }
  void set_heart_rate_raw_sensor(sensor::Sensor *sensor) { heart_rate_raw_sensor_ = sensor; }
  
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

 private:
  // 帧标识
  static const uint32_t ENERGY_FRAME_HEADER = 0xF1F2F3F4;  // 小端: F4 F3 F2 F1
  static const uint32_t ENERGY_FRAME_FOOTER = 0xF5F6F7F8;  // 小端: F8 F7 F6 F5
  
  // 缓冲区大小
  static const uint16_t MAX_BUFFER_SIZE = 300;
  
  // 心率检测参数
  static const uint16_t HEART_RATE_SAMPLES = 64;    // 用于心率分析的数据点数
  static const uint16_t HEART_RATE_INTERVAL = 5000; // 心率分析间隔(ms)
  static const float SAMPLING_RATE;                 // 采样率 ~6.06Hz
  
  // 解析数据帧
  void parse_energy_frame(const uint8_t *buffer, uint16_t len);
  
  // 发送配置命令
  void send_config_command(const uint8_t *cmd, size_t len);
  
  // 心率检测函数
  void analyze_vital_signs();
  float calculate_heart_rate();
  float calculate_breath_rate();
  void apply_bandpass_filter(float *data, uint16_t len, float low_freq, float high_freq);
  float find_peak_frequency(const float *data, uint16_t len, float min_freq, float max_freq);
  
  // 传感器指针
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *state_sensor_{nullptr};
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *breath_rate_sensor_{nullptr};
  sensor::Sensor *heart_rate_raw_sensor_{nullptr};
  
  // 缓冲区
  uint8_t buffer_[MAX_BUFFER_SIZE];
  uint16_t buffer_pos_{0};
  
  // 心率检测相关
  std::deque<float> energy_history_;
  uint32_t last_analysis_time_{0};
  uint32_t last_sample_time_{0};
  bool initialized_{false};
  
  // 当前状态
  uint16_t distance_{0};
  uint8_t detection_state_{0};
  float current_heart_rate_{0.0f};
  float current_breath_rate_{0.0f};
};

}  // namespace esphome::ld2402_minimal
