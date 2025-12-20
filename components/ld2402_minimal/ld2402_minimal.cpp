#include "ld2402_minimal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cmath>
#include <algorithm>

namespace esphome::ld2402_minimal {

static const char *const TAG = "ld2402_minimal";

// 静态常量定义
const float LD2402Minimal::SAMPLING_RATE = 6.06f;  // 165ms间隔 ≈ 6.06Hz

void LD2402Minimal::setup() {
  ESP_LOGI(TAG, "Initializing LD2402...");
  
  // 命令：进入配置模式
  uint8_t enable_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  // 帧头
    0x04, 0x00,              // 长度: 4字节
    0xFF, 0x00,              // 命令: 0x00FF (使能配置)
    0x02, 0x00,              // 协议版本: 2
    0x04, 0x03, 0x02, 0x01   // 帧尾
  };
  send_config_command(enable_cmd, sizeof(enable_cmd));
  delay(50);
  
  // 命令：设置为工程模式（能量输出模式）
  uint8_t mode_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  // 帧头
    0x08, 0x00,              // 长度: 8字节
    0x12, 0x00,              // 命令: 0x0012 (配置系统参数)
    0x00, 0x00,              // 参数ID: 0x0000
    0x04, 0x00, 0x00, 0x00,  // 参数值: 0x00000004 (工程模式)
    0x04, 0x03, 0x02, 0x01   // 帧尾
  };
  send_config_command(mode_cmd, sizeof(mode_cmd));
  delay(50);
  
  // 命令：退出配置模式
  uint8_t disable_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  // 帧头
    0x02, 0x00,              // 长度: 2字节
    0xFE, 0x00,              // 命令: 0x00FE (禁用配置)
    0x04, 0x03, 0x02, 0x01   // 帧尾
  };
  send_config_command(disable_cmd, sizeof(disable_cmd));
  delay(50);
  
  // 初始化能量历史缓冲区
  energy_history_.clear();
  
  initialized_ = true;
  ESP_LOGI(TAG, "LD2402 initialized in energy mode");
}

void LD2402Minimal::send_config_command(const uint8_t *cmd, size_t len) {
  write_array(cmd, len);
  ESP_LOGV(TAG, "Sent config command, length: %d", len);
}

void LD2402Minimal::loop() {
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
        uint32_t header;
        memcpy(&header, buffer_, 4);
        
        if (header == ENERGY_FRAME_HEADER) {
          parse_energy_frame(buffer_, buffer_pos_);
        } else {
          ESP_LOGW(TAG, "Invalid header: 0x%08X", header);
        }
        
        buffer_pos_ = 0;
      }
    }
  }
  
  // 定期分析生命体征
  if (initialized_ && millis() - last_analysis_time_ > HEART_RATE_INTERVAL) {
    analyze_vital_signs();
    last_analysis_time_ = millis();
  }
}

void LD2402Minimal::parse_energy_frame(const uint8_t *buffer, uint16_t len) {
  if (len < 12) {
    ESP_LOGW(TAG, "Frame too short: %d bytes", len);
    return;
  }
  
  // 获取数据长度 (小端)
  uint16_t data_length;
  memcpy(&data_length, &buffer[4], 2);
  
  // 验证长度
  uint16_t expected_len = 4 + 2 + data_length + 4;
  if (len != expected_len) {
    ESP_LOGW(TAG, "Length mismatch: expected %d, got %d", expected_len, len);
    return;
  }
  
  // 解析检测结果 (位置: 6)
  detection_state_ = buffer[6];
  
  // 解析距离 (位置: 7-8，小端)
  memcpy(&distance_, &buffer[7], 2);
  
  // 发布基础传感器数据
  if (distance_sensor_) {
    distance_sensor_->publish_state(distance_);
  }
  
  if (state_sensor_) {
    state_sensor_->publish_state(detection_state_);
  }
  
  // 采集能量数据用于心率分析（仅在有人且距离合适时）
  if (detection_state_ != 0x00 && distance_ > 0 && distance_ < 400) {
    // 计算要使用的距离门位置
    const uint8_t ENERGY_BYTES_PER_GATE = 4;  // 每个距离门4字节
    
    // 确保距离门编号在有效范围内 (1-16)
    uint8_t gate_to_use = gate_number_;
    if (gate_to_use < 1) gate_to_use = 1;
    if (gate_to_use > 16) gate_to_use = 16;
    
    // 计算距离门位置：帧头4 + 长度2 + 检测结果1 + 距离2 + (gate-1)*4
    uint16_t energy_start_pos = 9 + ((gate_to_use - 1) * ENERGY_BYTES_PER_GATE);
    
    if (len >= energy_start_pos + ENERGY_BYTES_PER_GATE) {
      // 读取4字节的能量值（小端格式）
      uint32_t gate_energy = 0;
      memcpy(&gate_energy, &buffer[energy_start_pos], sizeof(gate_energy));
      
      // 存储到历史缓冲区
      energy_history_.push_back(static_cast<float>(gate_energy));
      
      // 保持缓冲区大小
      if (energy_history_.size() > HEART_RATE_SAMPLES * 2) {
        energy_history_.pop_front();
      }
      
      // 发布原始心率值（用于调试）
      if (heart_rate_raw_sensor_) {
        heart_rate_raw_sensor_->publish_state(gate_energy);
      }
      
      // 调试输出
      static uint32_t last_debug_time = 0;
      uint32_t now = millis();
      if (now - last_debug_time > 5000) {
        // 还可以读取并显示前几个距离门的能量值进行比较
        uint32_t gate1_energy = 0, gate2_energy = 0, gate3_energy = 0;
        
        if (len >= 13) memcpy(&gate1_energy, &buffer[9], 4);
        if (len >= 17) memcpy(&gate2_energy, &buffer[13], 4);
        if (len >= 21) memcpy(&gate3_energy, &buffer[17], 4);
        
        const char* state_str = "未知";
        if (detection_state_ == 0x00) state_str = "无人";
        else if (detection_state_ == 0x01) state_str = "有人移动";
        else if (detection_state_ == 0x02) state_str = "有人静止";
        
        ESP_LOGD(TAG, "状态: %s, 距离: %d cm, 使用距离门: %d", 
                 state_str, distance_, gate_to_use);
        ESP_LOGD(TAG, "能量值 - 门1: %lu, 门2: %lu, 门3: %lu", 
                 gate1_energy, gate2_energy, gate3_energy);
        last_debug_time = now;
      }
    } else {
      ESP_LOGW(TAG, "数据帧太短，无法读取第%d个距离门", gate_to_use);
    }
  } else {
    // 无人时清空历史数据
    if (!energy_history_.empty()) {
      ESP_LOGD(TAG, "无人状态，清空能量历史数据");
      energy_history_.clear();
    }
  }
}
  
  // 调试日志
  static uint32_t last_log_ms = 0;
  uint32_t now = millis();
  if (now - last_log_ms > 2000) {
    const char* state_str = "未知";
    if (detection_state_ == 0x00) state_str = "无人";
    else if (detection_state_ == 0x01) state_str = "有人移动";
    else if (detection_state_ == 0x02) state_str = "有人静止";
    
    ESP_LOGD(TAG, "状态: %s, 距离: %d cm, 能量历史: %d", 
             state_str, distance_, energy_history_.size());
    last_log_ms = now;
  }
}

void LD2402Minimal::analyze_vital_signs() {
  // 检查是否有足够的数据
  if (energy_history_.size() < HEART_RATE_SAMPLES || detection_state_ == 0x00) {
    if (heart_rate_sensor_ && current_heart_rate_ > 0) {
      heart_rate_sensor_->publish_state(0.0f);
      current_heart_rate_ = 0.0f;
    }
    if (breath_rate_sensor_ && current_breath_rate_ > 0) {
      breath_rate_sensor_->publish_state(0.0f);
      current_breath_rate_ = 0.0f;
    }
    return;
  }
  
  // 计算心率和呼吸率
  float heart_rate = calculate_heart_rate();
  float breath_rate = calculate_breath_rate();
  
  // 更新传感器值
  if (heart_rate_sensor_ && heart_rate > 0) {
    heart_rate_sensor_->publish_state(heart_rate);
    current_heart_rate_ = heart_rate;
  }
  
  if (breath_rate_sensor_ && breath_rate > 0) {
    breath_rate_sensor_->publish_state(breath_rate);
    current_breath_rate_ = breath_rate;
  }
  
  ESP_LOGD(TAG, "生命体征分析: 心率=%.1f BPM, 呼吸率=%.1f BPM", 
           heart_rate, breath_rate);
}

float LD2402Minimal::calculate_heart_rate() {
  if (energy_history_.size() < HEART_RATE_SAMPLES) {
    return 0.0f;
  }
  
  // 提取最近的样本
  std::vector<float> signal(HEART_RATE_SAMPLES);
  auto it = energy_history_.end();
  std::advance(it, -HEART_RATE_SAMPLES);
  
  for (size_t i = 0; i < HEART_RATE_SAMPLES && it != energy_history_.end(); ++i, ++it) {
    signal[i] = *it;
  }
  
  // 移除直流分量
  float sum = 0.0f;
  for (float sample : signal) {
    sum += sample;
  }
  float mean = sum / signal.size();
  
  for (float &sample : signal) {
    sample -= mean;
  }
  
  // 应用带通滤波器（心率频率：0.8-3.0 Hz，对应48-180 BPM）
  apply_bandpass_filter(signal.data(), signal.size(), 0.8f, 3.0f);
  
  // 寻找峰值频率
  float peak_freq = find_peak_frequency(signal.data(), signal.size(), 0.8f, 3.0f);
  
  if (peak_freq > 0) {
    float heart_rate_bpm = peak_freq * 60.0f;  // 转换为BPM
    
    // 有效性检查
    if (heart_rate_bpm >= 40.0f && heart_rate_bpm <= 180.0f) {
      return heart_rate_bpm;
    }
  }
  
  return 0.0f;
}

float LD2402Minimal::calculate_breath_rate() {
  if (energy_history_.size() < HEART_RATE_SAMPLES) {
    return 0.0f;
  }
  
  // 提取最近的样本
  std::vector<float> signal(HEART_RATE_SAMPLES);
  auto it = energy_history_.end();
  std::advance(it, -HEART_RATE_SAMPLES);
  
  for (size_t i = 0; i < HEART_RATE_SAMPLES && it != energy_history_.end(); ++i, ++it) {
    signal[i] = *it;
  }
  
  // 移除直流分量
  float sum = 0.0f;
  for (float sample : signal) {
    sum += sample;
  }
  float mean = sum / signal.size();
  
  for (float &sample : signal) {
    sample -= mean;
  }
  
  // 应用带通滤波器（呼吸频率：0.1-0.5 Hz，对应6-30 BPM）
  apply_bandpass_filter(signal.data(), signal.size(), 0.1f, 0.5f);
  
  // 寻找峰值频率
  float peak_freq = find_peak_frequency(signal.data(), signal.size(), 0.1f, 0.5f);
  
  if (peak_freq > 0) {
    float breath_rate_bpm = peak_freq * 60.0f;  // 转换为BPM
    
    // 有效性检查
    if (breath_rate_bpm >= 4.0f && breath_rate_bpm <= 40.0f) {
      return breath_rate_bpm;
    }
  }
  
  return 0.0f;
}

void LD2402Minimal::apply_bandpass_filter(float *data, uint16_t len, float low_freq, float high_freq) {
  // 简单的IIR带通滤波器实现
  // 注意：这是一个简化版本，实际应用可能需要更复杂的滤波器
  
  if (len < 2) return;
  
  // 计算滤波器系数
  float dt = 1.0f / SAMPLING_RATE;
  float rc_low = 1.0f / (2.0f * M_PI * high_freq);
  float rc_high = 1.0f / (2.0f * M_PI * low_freq);
  float alpha_low = dt / (rc_low + dt);
  float alpha_high = rc_high / (rc_high + dt);
  
  // 应用滤波器
  float prev_low = data[0];
  float prev_high = data[0];
  
  for (uint16_t i = 1; i < len; i++) {
    // 低通部分
    float lowpass = prev_low + alpha_low * (data[i] - prev_low);
    
    // 高通部分
    float highpass = alpha_high * (prev_high + data[i] - data[i-1]);
    
    // 带通 = 高通 + 低通
    data[i] = highpass + (lowpass - prev_low);
    
    prev_low = lowpass;
    prev_high = highpass;
  }
}

float LD2402Minimal::find_peak_frequency(const float *data, uint16_t len, float min_freq, float max_freq) {
  // 简单峰值检测算法（零交叉法）
  // 注意：这是一个简化版本，对于复杂信号可能需要FFT
  
  if (len < 4) return 0.0f;
  
  // 寻找过零点
  int zero_crossings = 0;
  for (uint16_t i = 1; i < len; i++) {
    if (data[i-1] * data[i] < 0) {  // 符号变化
      zero_crossings++;
    }
  }
  
  if (zero_crossings < 2) {
    return 0.0f;  // 没有足够的周期
  }
  
  // 计算平均周期长度
  float avg_period_samples = static_cast<float>(len) / (static_cast<float>(zero_crossings) / 2.0f);
  float frequency_hz = SAMPLING_RATE / avg_period_samples;
  
  // 检查频率是否在有效范围内
  if (frequency_hz >= min_freq && frequency_hz <= max_freq) {
    return frequency_hz;
  }
  
  return 0.0f;
}

}  // namespace esphome::ld2402_minimal
