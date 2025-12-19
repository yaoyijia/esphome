#include "ld2402_minimal.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome::ld2402_minimal {

static const char *const TAG = "ld2402_minimal";

void LD2402Minimal::setup() {
  // 发送配置命令，设置模块为工程模式
  ESP_LOGI(TAG, "Initializing LD2402...");
  
  // 命令：进入配置模式
  uint8_t enable_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  // 帧头
    0x04, 0x00,              // 长度: 4字节
    0xFF, 0x00,              // 命令: 0x00FF (使能配置)
    0x02, 0x00,              // 协议版本: 2
    0x04, 0x03, 0x02, 0x01   // 帧尾
  };
  write_array(enable_cmd, sizeof(enable_cmd));
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
  write_array(mode_cmd, sizeof(mode_cmd));
  delay(50);
  
  // 命令：退出配置模式
  uint8_t disable_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  // 帧头
    0x02, 0x00,              // 长度: 2字节
    0xFE, 0x00,              // 命令: 0x00FE (禁用配置)
    0x04, 0x03, 0x02, 0x01   // 帧尾
  };
  write_array(disable_cmd, sizeof(disable_cmd));
  delay(50);
  
  initialized_ = true;
  ESP_LOGI(TAG, "LD2402 initialized in energy mode");
}

void LD2402Minimal::loop() {
  // 读取串口数据
  while (available()) {
    uint8_t byte = read();
    
    // 添加到缓冲区
    if (buffer_pos_ < MAX_BUFFER_SIZE - 1) {
      buffer_[buffer_pos_++] = byte;
    } else {
      // 缓冲区溢出，重置
      buffer_pos_ = 0;
      ESP_LOGW(TAG, "Buffer overflow, resetting");
      continue;
    }
    
    // 检查是否有完整的数据帧（检查帧尾 F8 F7 F6 F5）
    if (buffer_pos_ >= 8) {
      uint32_t footer;
      memcpy(&footer, &buffer_[buffer_pos_ - 4], 4);
      
      // 检查帧尾
      if (footer == ENERGY_FRAME_FOOTER) {
        // 检查帧头 F4 F3 F2 F1
        uint32_t header;
        memcpy(&header, buffer_, 4);
        
        if (header == ENERGY_FRAME_HEADER) {
          // 解析数据帧
          parse_energy_frame(buffer_, buffer_pos_);
        } else {
          ESP_LOGW(TAG, "Invalid header: 0x%08X, expected: 0x%08X", header, ENERGY_FRAME_HEADER);
        }
        
        // 清空缓冲区
        buffer_pos_ = 0;
      }
    }
  }
}

void LD2402Minimal::parse_energy_frame(const uint8_t *buffer, uint16_t len) {
  // 数据帧格式：
  // 帧头: F4 F3 F2 F1 (4字节)
  // 长度: 2字节 (小端，不包括帧头和帧尾的数据长度)
  // 检测结果: 1字节 (00:无人, 01:有人移动, 02:有人静止)
  // 目标距离: 2字节 (小端，单位cm)
  // 运动能量值: 16个距离门 × 4字节 = 64字节
  // 微动&静止能量值: 16个距离门 × 4字节 = 64字节
  // 帧尾: F8 F7 F6 F5 (4字节)
  
  if (len < 4 + 2 + 1 + 2 + 4) {  // 最小长度
    ESP_LOGW(TAG, "Frame too short: %d bytes", len);
    return;
  }
  
  // 获取数据长度 (小端)
  uint16_t data_length;
  memcpy(&data_length, &buffer[4], 2);
  
  // 验证长度
  // 总长度 = 帧头4 + 长度2 + data_length + 帧尾4
  uint16_t expected_len = 4 + 2 + data_length + 4;
  if (len != expected_len) {
    ESP_LOGW(TAG, "Length mismatch: expected %d, got %d", expected_len, len);
    return;
  }
  
  // 解析检测结果 (位置: 6)
  uint8_t detection_state = buffer[6];
  
  // 解析距离 (位置: 7-8，小端)
  uint16_t distance;
  memcpy(&distance, &buffer[7], 2);
  
  // 发布到传感器
  if (distance_sensor_) {
    distance_sensor_->publish_state(distance);
  }
  
  if (state_sensor_) {
    // 将检测状态转换为数值：0=无人, 1=有人移动, 2=有人静止
    state_sensor_->publish_state(detection_state);
  }
  
  // 调试日志（每秒最多输出一次）
  static uint32_t last_log_ms = 0;
  uint32_t now = millis();
  if (now - last_log_ms > 1000) {
    const char* state_str;
    switch (detection_state) {
      case 0x00: state_str = "无人"; break;
      case 0x01: state_str = "有人移动"; break;
      case 0x02: state_str = "有人静止"; break;
      default: state_str = "未知"; break;
    }
    
    ESP_LOGD(TAG, "状态: %s, 距离: %d cm, 帧长度: %d bytes", 
             state_str, distance, len);
    last_log_ms = now;
  }
}

}  // namespace esphome::ld2402_minimal
