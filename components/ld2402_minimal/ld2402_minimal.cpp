#include "ld2402_minimal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome::ld2402_minimal {

static const char *const TAG = "ld2402_minimal";

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
  
  initialized_ = true;
  ESP_LOGI(TAG, "LD2402 initialized in energy mode");
}

void LD2402Minimal::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2402 Minimal Component (Simplified):");
  ESP_LOGCONFIG(TAG, "  UART: TX=%d, RX=%d, Baud=115200", 1, 3);  // 根据实际引脚修改
  ESP_LOGCONFIG(TAG, "  Features: Distance and presence detection only");
  ESP_LOGCONFIG(TAG, "  Heart rate/Breath rate: Disabled");
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
  
  // 调试日志（每秒最多输出一次）
  static uint32_t last_log_ms = 0;
  uint32_t now = millis();
  if (now - last_log_ms > 2000) {
    const char* state_str = "Unknown";
    if (detection_state_ == 0x00) state_str = "无人";
    else if (detection_state_ == 0x01) state_str = "有人移动";
    else if (detection_state_ == 0x02) state_str = "有人静止";
    
    ESP_LOGD(TAG, "状态: %s, 距离: %d cm", state_str, distance_);
    last_log_ms = now;
  }
}

}  // namespace esphome::ld2402_minimal
