#include "simple_gps_ntp.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace simple_gps_ntp {

SimpleGPSNTPServer *SimpleGPSNTPServer::instance_ = nullptr;

void IRAM_ATTR SimpleGPSNTPServer::pps_isr() {
  if (SimpleGPSNTPServer::instance_) {
    SimpleGPSNTPServer::instance_->last_pps_us_ = micros();
    SimpleGPSNTPServer::instance_->pps_count_++;
    SimpleGPSNTPServer::instance_->pps_active_ = true;
  }
}

void SimpleGPSNTPServer::setup() {
  ESP_LOGI("SimpleNTP", "Initializing Simple GPS NTP Server");
  
  // 设置实例指针
  instance_ = this;
  
  // 配置PPS引脚
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   &SimpleGPSNTPServer::pps_isr, 
                   FALLING);
    ESP_LOGI("SimpleNTP", "PPS on GPIO %d", pps_pin_);
  }
  
  // 初始化GPS缓冲区
  memset(gps_buffer_, 0, sizeof(gps_buffer_));
  gps_idx_ = 0;
  
  // 启动NTP服务器
  if (!ntp_started_) {
    udp_.begin(123);
    ntp_started_ = true;
    ESP_LOGI("SimpleNTP", "NTP server started");
  }
}

void SimpleGPSNTPServer::parse_gps() {
  if (!uart_) return;
  
  while (uart_->available()) {
    char c = uart_->read();
    
    // 简单解析GPRMC语句获取时间
    if (c == '$') {
      gps_idx_ = 0;
    }
    
    if (gps_idx_ < sizeof(gps_buffer_) - 1) {
      gps_buffer_[gps_idx_++] = c;
      gps_buffer_[gps_idx_] = 0;
      
      // 检查是否收到完整句子
      if (c == '\n') {
        if (strstr(gps_buffer_, "$GPRMC")) {
          // 简单解析时间 (HHMMSS.XXX)
          char *ptr = strchr(gps_buffer_, ',');
          if (ptr) {
            ptr++; // 跳过第一个逗号
            
            // 获取时间字段
            char time_str[16];
            int i = 0;
            while (*ptr != ',' && *ptr != '.' && i < 15) {
              time_str[i++] = *ptr++;
            }
            time_str[i] = 0;
            
            if (strlen(time_str) >= 6) {
              // 解析时、分、秒
              int hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
              int minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
              int second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
              
              // 简单转换为秒数（假设为UTC当天时间）
              gps_seconds_ = hour * 3600 + minute * 60 + second;
              gps_valid_ = true;
              
              ESP_LOGI("SimpleNTP", "GPS time: %02d:%02d:%02d UTC", hour, minute, second);
            }
          }
        }
        gps_idx_ = 0;
      }
    } else {
      gps_idx_ = 0; // 缓冲区溢出，重置
    }
  }
}

void SimpleGPSNTPServer::handle_pps() {
  if (pps_active_) {
    pps_active_ = false;
    
    // 每次PPS脉冲，GPS秒数加1
    if (gps_valid_) {
      gps_seconds_++;
      
      // 处理溢出（超过24小时）
      if (gps_seconds_ >= 86400) {
        gps_seconds_ -= 86400;
      }
    }
  }
}

bool SimpleGPSNTPServer::get_ntp_time(uint32_t &seconds, uint32_t &fraction) {
  if (!gps_valid_ || !pps_active_) {
    // 回退到系统时间
    struct timeval tv;
    gettimeofday(&tv, NULL);
    seconds = tv.tv_sec + 2208988800UL;
    fraction = (tv.tv_usec * 4294967296UL) / 1000000UL;
    return false;
  }
  
  // 计算从上次PPS到现在的微秒偏移
  uint32_t now_us = micros();
  uint32_t offset_us = (now_us - last_pps_us_) & 0xFFFFFFFFUL;
  
  // 计算NTP时间
  seconds = gps_seconds_ + 2208988800UL + (offset_us / 1000000UL);
  fraction = ((offset_us % 1000000UL) * 4294967296UL) / 1000000UL;
  
  return true;
}

void SimpleGPSNTPServer::handle_ntp() {
  if (!ntp_started_) return;
  
  int packetSize = udp_.parsePacket();
  if (packetSize) {
    byte packet[48];
    udp_.read(packet, 48);
    
    uint32_t ntp_seconds, ntp_fraction;
    bool high_precision = get_ntp_time(ntp_seconds, ntp_fraction);
    
    // 构建响应包（简化版）
    packet[0] = 0b00100100; // LI, Version, Mode
    packet[1] = high_precision ? 1 : 4; // Stratum
    
    // 使用计算的时间填充Transmit Timestamp
    packet[40] = (ntp_seconds >> 24) & 0xFF;
    packet[41] = (ntp_seconds >> 16) & 0xFF;
    packet[42] = (ntp_seconds >> 8) & 0xFF;
    packet[43] = ntp_seconds & 0xFF;
    
    packet[44] = (ntp_fraction >> 24) & 0xFF;
    packet[45] = (ntp_fraction >> 16) & 0xFF;
    packet[46] = (ntp_fraction >> 8) & 0xFF;
    packet[47] = ntp_fraction & 0xFF;
    
    // 发送响应
    udp_.beginPacket(udp_.remoteIP(), udp_.remotePort());
    udp_.write(packet, 48);
    udp_.endPacket();
    
    ESP_LOGD("SimpleNTP", "NTP response sent");
  }
}

void SimpleGPSNTPServer::loop() {
  uint32_t now = millis();
  
  // 限制循环频率
  if (now - last_loop_ < 10) return;
  last_loop_ = now;
  
  // 处理GPS数据
  parse_gps();
  
  // 处理PPS
  handle_pps();
  
  // 处理NTP请求
  handle_ntp();
}

void SimpleGPSNTPServer::dump_config() {
  ESP_LOGCONFIG("SimpleNTP", "Simple GPS NTP Server");
  ESP_LOGCONFIG("SimpleNTP", "  PPS Pin: %d", pps_pin_);
  ESP_LOGCONFIG("SimpleNTP", "  GPS Valid: %s", gps_valid_ ? "YES" : "NO");
  ESP_LOGCONFIG("SimpleNTP", "  PPS Active: %s", pps_active_ ? "YES" : "NO");
  ESP_LOGCONFIG("SimpleNTP", "  PPS Count: %u", pps_count_);
}

}  // namespace simple_gps_ntp
}  // namespace esphome
