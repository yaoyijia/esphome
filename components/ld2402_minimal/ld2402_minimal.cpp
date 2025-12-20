#include "ld2402_with_hr.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include <cstring>

namespace esphome::ld2402_with_hr {

static const char *const TAG = "ld2402_with_hr";

// 常量定义
const float LD2402WithHR::SAMPLING_RATE = 6.06f;        // 6.06Hz (1000ms/165ms)
const uint16_t LD2402WithHR::HR_ANALYSIS_INTERVAL_MS = 10000;  // 10秒

// 初始化向量
static std::vector<float> initialize_vector() {
  return std::vector<float>(LD2402WithHR::SIGNAL_BUFFER_SIZE, 0.0f);
}

LD2402WithHR::LD2402WithHR() : energy_history_(initialize_vector()) {}

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
    tracking_person_ = true;
    
    // 使用第一个距离门的能量值（假设人体在最近的距离门）
    uint16_t gate_energy;
    memcpy(&gate_energy, &buffer[9], sizeof(gate_energy));
    
    // 检查并分析生命体征
    check_and_analyze_vital_signs(gate_energy);
    
    // 调试输出
    static uint32_t last_log_ms = 0;
    uint32_t now = millis();
    if (now - last_log_ms > 2000) {  // 每2秒输出一次
      const char* state_str = detection_state_ == DETECTION_MOVING ? "有人移动" : "有人静止";
      ESP_LOGI(TAG, "状态: %s, 距离: %d cm", state_str, distance_);
      last_log_ms = now;
    }
  } else {
    // 无人状态，重置追踪
    consecutive_person_frames_ = 0;
    tracking_person_ = false;
    
    // 清空心率数据
    if (heart_rate_sensor_) {
      heart_rate_sensor_->publish_state(NAN);
    }
    if (breath_rate_sensor_) {
      breath_rate_sensor_->publish_state(NAN);
    }
  }
}

void LD2402WithHR::check_and_analyze_vital_signs(uint16_t gate_energy) {
  // 只在静止状态下进行心率检测（移动时心率检测不可靠）
  if (detection_state_ != DETECTION_STILL) {
    return;
  }
  
  // 距离太远或太近都不适合检测
  if (distance_ < 50 || distance_ > 400) {  // 0.5米 - 4米
    return;
  }
  
  // 收集能量值到历史缓冲区
  if (history_index_ < SIGNAL_BUFFER_SIZE) {
    energy_history_[history_index_++] = static_cast<float>(gate_energy);
  } else {
    // 缓冲区已满，循环使用
    std::rotate(energy_history_.begin(), energy_history_.begin() + 1, energy_history_.end());
    energy_history_[SIGNAL_BUFFER_SIZE - 1] = static_cast<float>(gate_energy);
  }
  
  // 定期分析
  uint32_t now = millis();
  if (now - last_analysis_time_ > HR_ANALYSIS_INTERVAL_MS) {
    last_analysis_time_ = now;
    
    // 确保有足够的数据
    if (history_index_ >= SIGNAL_BUFFER_SIZE / 2) {
      analyze_vital_signs();
    }
  }
}

void LD2402WithHR::analyze_vital_signs() {
  // 检查是否有人且静止
  if (detection_state_ != DETECTION_STILL || !tracking_person_) {
    return;
  }
  
  // 预处理信号
  float signal[SIGNAL_BUFFER_SIZE];
  float sum = 0;
  
  // 复制并去直流
  for (uint16_t i = 0; i < SIGNAL_BUFFER_SIZE; i++) {
    signal[i] = energy_history_[i];
    sum += signal[i];
  }
  float mean = sum / SIGNAL_BUFFER_SIZE;
  
  for (uint16_t i = 0; i < SIGNAL_BUFFER_SIZE; i++) {
    signal[i] -= mean;
  }
  
  // 应用窗函数
  apply_hanning_window(signal, SIGNAL_BUFFER_SIZE);
  
  // 计算心率和呼吸率
  float heart_rate = calculate_heart_rate();
  float breath_rate = calculate_breath_rate();
  
  // 发布传感器数据
  if (heart_rate_sensor_ && heart_rate > 0) {
    heart_rate_sensor_->publish_state(heart_rate);
  }
  
  if (breath_rate_sensor_ && breath_rate > 0) {
    breath_rate_sensor_->publish_state(breath_rate);
  }
  
  ESP_LOGD(TAG, "心率分析完成: HR=%.1f BPM, BR=%.1f BPM", heart_rate, breath_rate);
}

void LD2402WithHR::apply_hanning_window(float *signal, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    float window = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (length - 1)));
    signal[i] *= window;
  }
}

float LD2402WithHR::goertzel_algorithm(const float *signal, uint16_t length, float target_freq, float sampling_rate) {
  // Goertzel算法：计算特定频率的信号强度
  float omega = 2.0f * M_PI * target_freq / sampling_rate;
  float coeff = 2.0f * cosf(omega);
  
  float s_prev = 0.0f;
  float s_prev2 = 0.0f;
  
  for (uint16_t i = 0; i < length; i++) {
    float s = signal[i] + coeff * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev = s;
  }
  
  float real = s_prev - s_prev2 * cosf(omega);
  float imag = s_prev2 * sinf(omega);
  
  return sqrtf(real * real + imag * imag);
}

float LD2402WithHR::calculate_heart_rate() {
  // 使用前一半数据（较新）进行分析
  uint16_t effective_length = std::min(history_index_, SIGNAL_BUFFER_SIZE);
  if (effective_length < 32) {
    return 0.0f;  // 数据不足
  }
  
  // 心率频率范围：0.8-3.0 Hz (48-180 BPM)
  const float hr_min_freq = 0.8f;  // 48 BPM
  const float hr_max_freq = 3.0f;  // 180 BPM
  const float hr_step = 0.1f;      // 分辨率
  
  float max_magnitude = 0.0f;
  float best_freq = 0.0f;
  
  // 复制信号
  std::vector<float> signal(effective_length);
  float sum = 0;
  for (uint16_t i = 0; i < effective_length; i++) {
    signal[i] = energy_history_[i];
    sum += signal[i];
  }
  float mean = sum / effective_length;
  
  // 去直流
  for (uint16_t i = 0; i < effective_length; i++) {
    signal[i] -= mean;
  }
  
  // 搜索最佳频率
  for (float freq = hr_min_freq; freq <= hr_max_freq; freq += hr_step) {
    float magnitude = goertzel_algorithm(signal.data(), effective_length, freq, SAMPLING_RATE);
    
    if (magnitude > max_magnitude) {
      max_magnitude = magnitude;
      best_freq = freq;
    }
  }
  
  // 转换为BPM
  float heart_rate_bpm = best_freq * 60.0f;
  
  // 有效性检查
  if (heart_rate_bpm >= 40.0f && heart_rate_bpm <= 180.0f && max_magnitude > 100.0f) {
    return heart_rate_bpm;
  }
  
  return 0.0f;
}

float LD2402WithHR::calculate_breath_rate() {
  // 呼吸率检测（类似心率）
  uint16_t effective_length = std::min(history_index_, SIGNAL_BUFFER_SIZE);
  if (effective_length < 32) {
    return 0.0f;
  }
  
  // 呼吸频率范围：0.1-0.5 Hz (6-30 BPM)
  const float br_min_freq = 0.1f;  // 6 BPM
  const float br_max_freq = 0.5f;  // 30 BPM
  const float br_step = 0.05f;     // 分辨率
  
  float max_magnitude = 0.0f;
  float best_freq = 0.0f;
  
  // 复制并预处理信号
  std::vector<float> signal(effective_length);
  float sum = 0;
  for (uint16_t i = 0; i < effective_length; i++) {
    signal[i] = energy_history_[i];
    sum += signal[i];
  }
  float mean = sum / effective_length;
  
  // 去直流
  for (uint16_t i = 0; i < effective_length; i++) {
    signal[i] -= mean;
  }
  
  // 搜索最佳频率
  for (float freq = br_min_freq; freq <= br_max_freq; freq += br_step) {
    float magnitude = goertzel_algorithm(signal.data(), effective_length, freq, SAMPLING_RATE);
    
    if (magnitude > max_magnitude) {
      max_magnitude = magnitude;
      best_freq = freq;
    }
  }
  
  // 转换为BPM
  float breath_rate_bpm = best_freq * 60.0f;
  
  // 有效性检查
  if (breath_rate_bpm >= 6.0f && breath_rate_bpm <= 30.0f && max_magnitude > 50.0f) {
    return breath_rate_bpm;
  }
  
  return 0.0f;
}

}  // namespace esphome::ld2402_with_hr
