#include "ld2402_with_hr_fixed.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include <cstring>

namespace esphome::ld2402_with_hr {

static const char *const TAG = "ld2402_with_hr";

// 常量定义
const float LD2402WithHR::SAMPLING_RATE = 6.06f;        // 6.06Hz (1000ms/165ms)
const uint16_t LD2402WithHR::HR_ANALYSIS_INTERVAL_MS = 5000;  // 5秒

// 构造函数定义
LD2402WithHR::LD2402WithHR() : energy_history_(SIGNAL_BUFFER_SIZE, 0.0f) {}

void LD2402WithHR::setup() {
  // 等待模块启动
  delay(500);
  
  ESP_LOGI(TAG, "Initializing LD2402 with heart rate detection...");
  
  // 1. 进入配置模式
  uint8_t enable_cmd[] = {0x02, 0x00};  // 协议版本2.0
  send_config_command(0x00FF, enable_cmd, 2);
  delay(100);
  
  // 2. 设置为工程模式（能量输出模式）
  uint8_t mode_cmd[] = {0x00, 0x00, 0x04, 0x00, 0x00, 0x00};
  send_config_command(0x0012, mode_cmd, 6);
  delay(100);
  
  // 3. 退出配置模式
  send_config_command(0x00FE);
  delay(100);
  
  initialized_ = true;
  ESP_LOGI(TAG, "LD2402 initialized successfully");
}

void LD2402WithHR::loop() {
  // 读取串口数据
  while (available()) {
    uint8_t byte = read();
    
    // 添加到缓冲区
    if (buffer_pos_ < MAX_BUFFER_SIZE - 1) {
      buffer_[buffer_pos_++] = byte;
    } else {
      buffer_pos_ = 0;
      ESP_LOGW(TAG, "Buffer overflow, resetting");
      continue;
    }
    
    // 检查是否有完整的数据帧（检查帧尾 F8 F7 F6 F5）
    if (buffer_pos_ >= 8) {
      uint32_t footer;
      memcpy(&footer, &buffer_[buffer_pos_ - 4], 4);
      
      if (footer == ENERGY_FRAME_FOOTER) {
        // 检查帧头 F4 F3 F2 F1
        if (buffer_pos_ >= 12) {
          uint32_t header;
          memcpy(&header, buffer_, 4);
          
          if (header == ENERGY_FRAME_HEADER) {
            parse_energy_frame(buffer_, buffer_pos_);
          } else {
            ESP_LOGW(TAG, "Invalid header: 0x%08X", header);
          }
        }
        buffer_pos_ = 0;  // 清空缓冲区
      }
    }
  }
  
  // 定期更新心率传感器数据（即使没有新数据）
  uint32_t now = millis();
  if (now - last_hr_update_time_ > 2000) {  // 每2秒更新一次
    last_hr_update_time_ = now;
    
    // 如果正在追踪人员且有有效数据，更新传感器
    if (tracking_person_ && detection_state_ == DETECTION_STILL) {
      if (!isnan(current_heart_rate_) && heart_rate_sensor_) {
        heart_rate_sensor_->publish_state(current_heart_rate_);
      }
      if (!isnan(current_breath_rate_) && breath_rate_sensor_) {
        breath_rate_sensor_->publish_state(current_breath_rate_);
      }
    }
  }
}

void LD2402WithHR::send_config_command(uint16_t command, const uint8_t* data, uint8_t data_len) {
  uint8_t frame[64];
  uint8_t pos = 0;
  
  // 帧头
  uint32_t header = 0xFAFBFCFD;
  memcpy(&frame[pos], &header, 4);
  pos += 4;
  
  // 长度
  uint16_t length = 2 + data_len;
  memcpy(&frame[pos], &length, 2);
  pos += 2;
  
  // 命令
  memcpy(&frame[pos], &command, 2);
  pos += 2;
  
  // 数据
  if (data && data_len > 0) {
    memcpy(&frame[pos], data, data_len);
    pos += data_len;
  }
  
  // 帧尾
  uint32_t footer = 0x01020304;
  memcpy(&frame[pos], &footer, 4);
  pos += 4;
  
  write_array(frame, pos);
  delay(50);
}

void LD2402WithHR::parse_energy_frame(const uint8_t* buffer, uint16_t len) {
  if (len < 12) {
    return;
  }
  
  // 获取数据长度
  uint16_t data_length;
  memcpy(&data_length, &buffer[4], 2);
  
  // 验证长度
  uint16_t expected_len = 4 + 2 + data_length + 4;
  if (len != expected_len) {
    ESP_LOGW(TAG, "Length mismatch: expected %d, got %d", expected_len, len);
    return;
  }
  
  // 解析检测结果
  detection_state_ = buffer[6];
  
  // 解析距离
  uint16_t distance;
  memcpy(&distance, &buffer[7], 2);
  distance_ = distance;
  
  // 解析能量值（第一个距离门）
  memcpy(&current_energy_, &buffer[9], sizeof(current_energy_));
  
  // 发布基础传感器数据
  if (distance_sensor_) {
    distance_sensor_->publish_state(distance_);
  }
  
  if (state_sensor_) {
    state_sensor_->publish_state(detection_state_);
  }
  
  // 只在检测到有人时进行心率检测
  if (detection_state_ == DETECTION_MOVING || detection_state_ == DETECTION_STILL) {
    consecutive_person_frames_++;
    if (consecutive_person_frames_ > 5) {  // 连续5帧检测到人
      tracking_person_ = true;
    }
    
    // 检查并分析生命体征（主要针对静止状态）
    if (detection_state_ == DETECTION_STILL && distance_ >= 50 && distance_ <= 400) {
      check_and_analyze_vital_signs(current_energy_);
    }
    
    // 调试输出
    static uint32_t last_log_ms = 0;
    uint32_t now = millis();
    if (now - last_log_ms > 2000) {  // 每2秒输出一次
      const char* state_str = detection_state_ == DETECTION_MOVING ? "有人移动" : "有人静止";
      ESP_LOGI(TAG, "状态: %s, 距离: %d cm, 能量: %d", state_str, distance_, current_energy_);
      
      // 如果正在检测心率，也输出心率信息
      if (!isnan(current_heart_rate_)) {
        ESP_LOGI(TAG, "心率: %.1f BPM", current_heart_rate_);
      }
      if (!isnan(current_breath_rate_)) {
        ESP_LOGI(TAG, "呼吸率: %.1f BPM", current_breath_rate_);
      }
      
      last_log_ms = now;
    }
  } else {
    // 无人状态，重置追踪
    consecutive_person_frames_ = 0;
    tracking_person_ = false;
    
    // 清除心率数据
    current_heart_rate_ = NAN;
    current_breath_rate_ = NAN;
    
    // 发布NaN到传感器
    if (heart_rate_sensor_) {
      heart_rate_sensor_->publish_state(NAN);
    }
    if (breath_rate_sensor_) {
      breath_rate_sensor_->publish_state(NAN);
    }
  }
}

void LD2402WithHR::check_and_analyze_vital_signs(uint16_t gate_energy) {
  // 收集能量值到历史缓冲区
  if (history_index_ < SIGNAL_BUFFER_SIZE) {
    energy_history_[history_index_++] = static_cast<float>(gate_energy);
  } else {
    // 缓冲区已满，循环使用
    for (uint16_t i = 0; i < SIGNAL_BUFFER_SIZE - 1; i++) {
      energy_history_[i] = energy_history_[i + 1];
    }
    energy_history_[SIGNAL_BUFFER_SIZE - 1] = static_cast<float>(gate_energy);
  }
  
  // 定期分析（每5秒）
  uint32_t now = millis();
  if (now - last_analysis_time_ > HR_ANALYSIS_INTERVAL_MS) {
    last_analysis_time_ = now;
    
    // 确保有足够的数据
    if (history_index_ >= SIGNAL_BUFFER_SIZE / 2) {
      analyze_vital_signs();
      
      // 立即更新传感器值
      if (!isnan(current_heart_rate_) && heart_rate_sensor_) {
        heart_rate_sensor_->publish_state(current_heart_rate_);
      }
      if (!isnan(current_breath_rate_) && breath_rate_sensor_) {
        breath_rate_sensor_->publish_state(current_breath_rate_);
      }
    }
  }
}

void LD2402WithHR::analyze_vital_signs() {
  // 检查是否有人且静止
  if (detection_state_ != DETECTION_STILL || !tracking_person_) {
    current_heart_rate_ = NAN;
    current_breath_rate_ = NAN;
    return;
  }
  
  // 使用简化算法计算心率
  float heart_rate = simple_heart_rate_detection();
  float breath_rate = simple_breath_rate_detection();
  
  // 有效性检查
  if (heart_rate > 40.0f && heart_rate < 180.0f) {
    current_heart_rate_ = heart_rate;
  } else {
    current_heart_rate_ = NAN;
  }
  
  if (breath_rate > 6.0f && breath_rate < 30.0f) {
    current_breath_rate_ = breath_rate;
  } else {
    current_breath_rate_ = NAN;
  }
  
  ESP_LOGD(TAG, "心率分析完成: HR=%.1f BPM, BR=%.1f BPM", heart_rate, breath_rate);
}

float LD2402WithHR::simple_heart_rate_detection() {
  // 简单的心率检测算法：基于能量变化的峰值检测
  
  if (history_index_ < 32) {  // 需要至少32个数据点
    return NAN;
  }
  
  // 使用最近的数据
  uint16_t effective_length = std::min(history_index_, static_cast<uint16_t>(32));
  uint16_t start_idx = history_index_ - effective_length;
  
  // 计算能量变化的均值
  float sum = 0;
  for (uint16_t i = 0; i < effective_length; i++) {
    sum += energy_history_[start_idx + i];
  }
  float mean = sum / effective_length;
  
  // 寻找峰值（简化算法）
  int peak_count = 0;
  float last_peak_val = 0;
  int last_peak_idx = -1;
  
  // 检测峰值
  for (uint16_t i = 1; i < effective_length - 1; i++) {
    float current = energy_history_[start_idx + i];
    float prev = energy_history_[start_idx + i - 1];
    float next = energy_history_[start_idx + i + 1];
    
    // 简单的峰值检测
    if (current > prev && current > next && current > mean * 1.05f) {
      // 确保峰值之间有足够的时间间隔（对应心率）
      if (last_peak_idx == -1 || (i - last_peak_idx) > 3) {  // 至少间隔0.5秒（3个样本）
        peak_count++;
        last_peak_idx = i;
        last_peak_val = current;
      }
    }
  }
  
  // 计算心率
  if (peak_count >= 2) {
    // 估计心率（基于峰值数量和时间）
    float total_time = effective_length / SAMPLING_RATE;  // 总时间（秒）
    float bpm = (peak_count - 1) * 60.0f / (total_time * (peak_count / (float)effective_length));
    
    // 限制在合理范围内
    if (bpm >= 40.0f && bpm <= 180.0f) {
      return bpm;
    }
  }
  
  return NAN;
}

float LD2402WithHR::simple_breath_rate_detection() {
  // 简单的呼吸率检测：寻找更低频的信号
  
  if (history_index_ < 48) {  // 需要更多数据点
    return NAN;
  }
  
  // 使用最近的数据
  uint16_t effective_length = std::min(history_index_, static_cast<uint16_t>(48));
  uint16_t start_idx = history_index_ - effective_length;
  
  // 计算移动平均以平滑数据
  std::vector<float> smoothed(effective_length, 0.0f);
  int window_size = 5;  // 5点移动平均
  
  for (uint16_t i = window_size; i < effective_length - window_size; i++) {
    float sum = 0;
    for (int j = -window_size; j <= window_size; j++) {
      sum += energy_history_[start_idx + i + j];
    }
    smoothed[i] = sum / (2 * window_size + 1);
  }
  
  // 寻找呼吸峰值（更低频）
  int breath_peaks = 0;
  int last_breath_peak = -1;
  
  for (uint16_t i = 10; i < effective_length - 10; i++) {
    // 检查是否为局部最大值（更宽窗口）
    bool is_peak = true;
    for (int j = -8; j <= 8; j++) {
      if (j != 0 && smoothed[i] <= smoothed[i + j]) {
        is_peak = false;
        break;
      }
    }
    
    if (is_peak && (last_breath_peak == -1 || (i - last_breath_peak) > 12)) {  // 至少间隔2秒
      breath_peaks++;
      last_breath_peak = i;
    }
  }
  
  // 计算呼吸率
  if (breath_peaks >= 2) {
    float total_time = effective_length / SAMPLING_RATE;
    float breaths_per_minute = (breath_peaks - 1) * 60.0f / total_time;
    
    if (breaths_per_minute >= 6.0f && breaths_per_minute <= 30.0f) {
      return breaths_per_minute;
    }
  }
  
  return NAN;
}

}  // namespace esphome::ld2402_with_hr
