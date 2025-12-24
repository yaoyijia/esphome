#include "ld2402_minimal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome::ld2402_minimal {

static const char *const TAG = "ld2402_minimal";

void LD2402Minimal::setup() {
  ESP_LOGI(TAG, "Initializing LD2402...");
  

  uint8_t enable_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  
    0x04, 0x00,              
    0xFF, 0x00,              
    0x02, 0x00,              
    0x04, 0x03, 0x02, 0x01   
  };
  send_config_command(enable_cmd, sizeof(enable_cmd));
  delay(50);
  

  uint8_t mode_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  
    0x08, 0x00,              
    0x12, 0x00,              
    0x00, 0x00,              
    0x01, 0x00, 0x00, 0x00,  
    0x04, 0x03, 0x02, 0x01   
  };
  send_config_command(mode_cmd, sizeof(mode_cmd));
  delay(50);
  

  uint8_t disable_cmd[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  
    0x02, 0x00,              
    0xFE, 0x00,              
    0x04, 0x03, 0x02, 0x01   
  };
  send_config_command(disable_cmd, sizeof(disable_cmd));
  delay(50);
  
  initialized_ = true;
  ESP_LOGI(TAG, "LD2402 initialized in detection mode");
}

void LD2402Minimal::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2402 Minimal Component:");
  ESP_LOGCONFIG(TAG, "  UART: TX=%d, RX=%d, Baud=115200", 1, 3);  
}

void LD2402Minimal::send_config_command(const uint8_t *cmd, size_t len) {
  write_array(cmd, len);
  ESP_LOGV(TAG, "Sent config command, length: %d", len);
}

void LD2402Minimal::loop() {

  while (available()) {
    uint8_t byte = read();
    

    if (buffer_pos_ < MAX_BUFFER_SIZE - 1) {
      buffer_[buffer_pos_++] = byte;
    } else {
      buffer_pos_ = 0;
      ESP_LOGW(TAG, "Buffer overflow, resetting");
      continue;
    }
    

    if (buffer_pos_ >= 8) {
      uint32_t footer;
      memcpy(&footer, &buffer_[buffer_pos_ - 4], 4);
      
      if (footer == ENERGY_FRAME_FOOTER) {

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
  

  uint16_t data_length;
  memcpy(&data_length, &buffer[4], 2);
  

  uint16_t expected_len = 4 + 2 + data_length + 4;
  if (len != expected_len) {
    ESP_LOGW(TAG, "Length mismatch: expected %d, got %d", expected_len, len);
    return;
  }
  

  detection_state_ = buffer[6];
  

  memcpy(&distance_, &buffer[7], 2);
  

  if (distance_sensor_) {
    distance_sensor_->publish_state(distance_);
  }
  
  if (state_sensor_) {
    state_sensor_->publish_state(detection_state_);
  }
  
  static uint32_t last_log_ms = 0;
  uint32_t now = millis();
  if (now - last_log_ms > 2000) {
    const char* state_str = "Unknown";
    if (detection_state_ == 0x00) state_str = "无人";
    else if (detection_state_ == 0x01) state_str = "有人移动";
    else if (detection_state_ == 0x02) state_str = "有人静止";
    
    ESP_LOGD(TAG, "状态: %s, 距离: %d cm", 
             state_str, distance_);
    last_log_ms = now;
  }
}

}  // namespace esphome::ld2402_minimal
