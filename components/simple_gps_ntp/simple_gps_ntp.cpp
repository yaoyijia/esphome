
#include "simple_gps_ntp.h"
#include "esphome/components/network/util.h"
#include <time.h>

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
  
  // 初始化时间相关变量
  gps_valid_ = false;
  time_calibration_ = 0.0f;
  
  // 启动NTP服务器
  if (!ntp_started_) {
    udp_.begin(123);
    ntp_started_ = true;
    ESP_LOGI("SimpleNTP", "NTP server started on port 123");
  }
  
  // 初始化系统时间（设置为2020年，避免1970年问题）
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (tv.tv_sec < 1609459200) { // 如果时间早于2021年1月1日
    // 设置为2020年1月1日
    tv.tv_sec = 1577836800;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    ESP_LOGW("SimpleNTP", "System time reset to 2020-01-01");
  }
}

void SimpleGPSNTPServer::parse_gps() {
  if (!uart_) return;
  
  uint8_t buffer[32];
  int available = uart_->available();
  
  while (available > 0) {
    int to_read = available < 32 ? available : 32;
    uart_->read_array(buffer, to_read);
    
    for (int i = 0; i < to_read; i++) {
      char c = buffer[i];
      
      if (c == '$') {
        gps_idx_ = 0;
      }
      
      if (gps_idx_ < sizeof(gps_buffer_) - 1) {
        gps_buffer_[gps_idx_++] = c;
        gps_buffer_[gps_idx_] = 0;
        
        if (c == '\n') {
          // 检查是否有RMC或GGA语句
          if (strstr(gps_buffer_, "$GPRMC") || strstr(gps_buffer_, "$GNRMC") || 
              strstr(gps_buffer_, "$GPGGA") || strstr(gps_buffer_, "$GNGGA")) {
            
            char *ptr = gps_buffer_;
            int field_count = 0;
            
            // 解析日期和时间
            uint8_t hour = 0, minute = 0, second = 0;
            uint8_t day = 0, month = 0;
            uint16_t year = 0;
            bool has_date = false;
            bool has_time = false;
            
            while (*ptr) {
              if (*ptr == ',') {
                field_count++;
                
                // 解析时间 (字段2)
                if (field_count == 2) {
                  ptr++;
                  char time_str[16];
                  int j = 0;
                  
                  while (*ptr != ',' && *ptr != '.' && *ptr != '\0' && j < 15) {
                    time_str[j++] = *ptr++;
                  }
                  time_str[j] = 0;
                  
                  if (strlen(time_str) == 6) {
                    hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
                    minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
                    second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
                    has_time = true;
                  }
                }
                
                // 解析日期 (字段10，在RMC语句中)
                if ((strstr(gps_buffer_, "$GPRMC") || strstr(gps_buffer_, "$GNRMC")) && field_count == 10) {
                  ptr++;
                  char date_str[16];
                  int j = 0;
                  
                  while (*ptr != ',' && *ptr != '\0' && j < 15) {
                    date_str[j++] = *ptr++;
                  }
                  date_str[j] = 0;
                  
                  if (strlen(date_str) == 6) {
                    day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
                    month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
                    year = (date_str[4] - '0') * 10 + (date_str[5] - '0') + 2000;
                    has_date = true;
                  }
                }
              }
              ptr++;
            }
            
            // 更新成员变量
            if (has_time) {
              gps_hour_ = hour;
              gps_minute_ = minute;
              gps_second_ = second;
              
              if (has_date) {
                gps_day_ = day;
                gps_month_ = month;
                gps_year_ = year;
                
                // 设置系统时间
                struct tm timeinfo = {0};
                timeinfo.tm_year = year - 1900;
                timeinfo.tm_mon = month - 1;
                timeinfo.tm_mday = day;
                timeinfo.tm_hour = hour;
                timeinfo.tm_min = minute;
                timeinfo.tm_sec = second;
                
                time_t epoch = mktime(&timeinfo);
                
                // 检查时间是否合理（晚于2020年）
                if (epoch > 1577836800) {
                  struct timeval tv = {epoch, 0};
                  settimeofday(&tv, NULL);
                  gps_valid_ = true;
                  
                  if (debug_level_ >= 1) {
                    ESP_LOGI("SimpleNTP", "GPS时间已设置: %04d-%02d-%02d %02d:%02d:%02d UTC",
                            year, month, day, hour, minute, second);
                  }
                }
              } else {
                // 只有时间没有日期
                gps_valid_ = true;
                if (debug_level_ >= 2) {
                  ESP_LOGD("SimpleNTP", "GPS时间: %02d:%02d:%02d UTC", hour, minute, second);
                }
              }
            }
          }
          gps_idx_ = 0;
        }
      } else {
        gps_idx_ = 0;
      }
    }
    
    available = uart_->available();
  }
}

void SimpleGPSNTPServer::handle_pps() {
  static uint32_t last_pps_count = 0;
  
  if (pps_active_ && pps_count_ > last_pps_count) {
    last_pps_count = pps_count_;
    pps_active_ = false;
    
    uint32_t now_us = micros();
    uint32_t interval_us = (now_us - last_pps_us_) & 0xFFFFFFFFUL;
    
    if (debug_level_ >= 3) {
      if (interval_us > 900000 && interval_us < 1100000) {
        ESP_LOGD("SimpleNTP", "PPS #%u, 间隔: %.6f秒", pps_count_, interval_us / 1000000.0f);
      } else {
        ESP_LOGW("SimpleNTP", "PPS间隔异常: %u us", interval_us);
      }
    }
  }
}

bool SimpleGPSNTPServer::get_ntp_time(uint32_t &seconds, uint32_t &fraction) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 如果系统时间太早（小于2020年），说明未同步，返回错误
  if (tv.tv_sec < 1577836800) {
    if (debug_level_ >= 1) {
      ESP_LOGW("SimpleNTP", "系统时间未同步: %u", tv.tv_sec);
    }
    return false;
  }
  
  // Unix时间（1970年1月1日）转换为NTP时间（1900年1月1日）
  // NTP时间 = Unix时间 + 2208988800
  seconds = tv.tv_sec + 2208988800UL;
  
  // 微秒转换为2^-32秒的分数部分
  fraction = (uint32_t)((uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL);
  
  // 应用PPS微调
  if (pps_count_ > 0) {
    uint32_t now_us = micros();
    uint32_t offset_us = (now_us - last_pps_us_) & 0xFFFFFFFFUL;
    
    if (offset_us < 1100000) {
      // 使用PPS进行更精确的微秒调整
      uint32_t pps_micros = offset_us % 1000000;
      fraction = (uint32_t)((uint64_t)pps_micros * 4294967296ULL / 1000000ULL);
      
      // 如果PPS发生在上一秒，调整秒数
      if (offset_us >= 1000000) {
        seconds += offset_us / 1000000UL;
      }
      
      if (debug_level_ >= 3) {
        ESP_LOGD("SimpleNTP", "PPS微调: 偏移=%uus, 分数=%u", offset_us, fraction);
      }
    }
  }
  
  // 定期记录日志（每5秒一次）
  uint32_t now_ms = millis();
  if (now_ms - last_ntp_log_ > 5000) {
    last_ntp_log_ = now_ms;
    
    time_t unix_time = seconds - 2208988800UL;
    struct tm *tm_info = gmtime(&unix_time);
    
    ESP_LOGI("SimpleNTP", 
             "时间状态: UTC=%04d-%02d-%02d %02d:%02d:%02d, GPS=%s, PPS=%u",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             gps_valid_ ? "YES" : "NO", pps_count_);
  }
  
  return gps_valid_ || (pps_count_ > 0);
}

void SimpleGPSNTPServer::handle_ntp() {
  if (!ntp_started_) return;
  
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    byte packet[48];
    udp_.read(packet, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    if (debug_level_ >= 2) {
      ESP_LOGI("SimpleNTP", "NTP请求来自: %s:%d", remote.toString().c_str(), remotePort);
    }
    
    // 获取当前时间
    uint32_t ntp_seconds, ntp_fraction;
    bool high_precision = get_ntp_time(ntp_seconds, ntp_fraction);
    
    if (!high_precision && !gps_valid_) {
      if (debug_level_ >= 1) {
        ESP_LOGW("SimpleNTP", "时间未同步，不响应NTP请求");
      }
      return;
    }
    
    // 构建响应包
    memset(packet, 0, 48);
    
    // LI = 0 (无警告), Version = 4, Mode = 4 (服务器)
    packet[0] = 0x24;
    
    // Stratum: 1表示一级时间服务器
    packet[1] = high_precision ? 1 : 2;
    
    // Poll interval
    packet[2] = 4; // 16秒
    
    // Precision: 2^-8 ≈ 3.9ms
    packet[3] = 0xE8;
    
    // Root Delay (0)
    packet[4] = 0;
    packet[5] = 0;
    packet[6] = 0;
    packet[7] = 0;
    
    // Root Dispersion (0.5秒，固定点小数表示)
    packet[8] = 0;
    packet[9] = 0;
    packet[10] = 0x80;
    packet[11] = 0;
    
    // Reference Identifier (自定义标识)
    packet[12] = 'G';
    packet[13] = 'P';
    packet[14] = 'S';
    packet[15] = 'N';
    
    // Reference Timestamp (参考时间戳，使用当前时间)
    uint32_t ref_seconds = ntp_seconds;
    uint32_t ref_fraction = ntp_fraction;
    
    // 大端序存储
    packet[16] = (ref_seconds >> 24) & 0xFF;
    packet[17] = (ref_seconds >> 16) & 0xFF;
    packet[18] = (ref_seconds >> 8) & 0xFF;
    packet[19] = ref_seconds & 0xFF;
    packet[20] = (ref_fraction >> 24) & 0xFF;
    packet[21] = (ref_fraction >> 16) & 0xFF;
    packet[22] = (ref_fraction >> 8) & 0xFF;
    packet[23] = ref_fraction & 0xFF;
    
    // Originate Timestamp (复制客户端发送时间)
    for (int i = 0; i < 8; i++) {
      packet[24 + i] = packet[40 + i];
    }
    
    // Receive Timestamp (接收时间)
    packet[32] = (ntp_seconds >> 24) & 0xFF;
    packet[33] = (ntp_seconds >> 16) & 0xFF;
    packet[34] = (ntp_seconds >> 8) & 0xFF;
    packet[35] = ntp_seconds & 0xFF;
    packet[36] = (ntp_fraction >> 24) & 0xFF;
    packet[37] = (ntp_fraction >> 16) & 0xFF;
    packet[38] = (ntp_fraction >> 8) & 0xFF;
    packet[39] = ntp_fraction & 0xFF;
    
    // Transmit Timestamp (发送时间，与接收时间相同)
    packet[40] = (ntp_seconds >> 24) & 0xFF;
    packet[41] = (ntp_seconds >> 16) & 0xFF;
    packet[42] = (ntp_seconds >> 8) & 0xFF;
    packet[43] = ntp_seconds & 0xFF;
    packet[44] = (ntp_fraction >> 24) & 0xFF;
    packet[45] = (ntp_fraction >> 16) & 0xFF;
    packet[46] = (ntp_fraction >> 8) & 0xFF;
    packet[47] = ntp_fraction & 0xFF;
    
    // 发送响应
    udp_.beginPacket(remote, remotePort);
    udp_.write(packet, 48);
    udp_.endPacket();
    
    if (debug_level_ >= 3) {
      time_t unix_time = ntp_seconds - 2208988800UL;
      struct tm *tm_info = gmtime(&unix_time);
      
      ESP_LOGD("SimpleNTP", 
               "NTP响应: 时间=%04d-%02d-%02d %02d:%02d:%02d UTC",
               tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    }
  }
}

void SimpleGPSNTPServer::loop() {
  uint32_t now = millis();
  
  if (now - last_loop_ < 10) return;
  last_loop_ = now;
  
  parse_gps();
  handle_pps();
  handle_ntp();
}

void SimpleGPSNTPServer::dump_config() {
  ESP_LOGCONFIG("SimpleNTP", "Simple GPS NTP Server");
  ESP_LOGCONFIG("SimpleNTP", "  PPS Pin: %d", pps_pin_);
  ESP_LOGCONFIG("SimpleNTP", "  Debug Level: %d", debug_level_);
  ESP_LOGCONFIG("SimpleNTP", "  GPS Valid: %s", gps_valid_ ? "YES" : "NO");
  ESP_LOGCONFIG("SimpleNTP", "  PPS Count: %u", pps_count_);
  
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm *tm_info = gmtime(&tv.tv_sec);
  
  ESP_LOGCONFIG("SimpleNTP", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
               tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  
  if (gps_valid_) {
    if (gps_year_ > 0) {
      ESP_LOGCONFIG("SimpleNTP", "  GPS时间: %04d-%02d-%02d %02d:%02d:%02d UTC", 
                   gps_year_, gps_month_, gps_day_, 
                   gps_hour_, gps_minute_, gps_second_);
    } else {
      ESP_LOGCONFIG("SimpleNTP", "  GPS时间: %02d:%02d:%02d UTC", 
                   gps_hour_, gps_minute_, gps_second_);
    }
  }
}

}  // namespace simple_gps_ntp
}  // namespace esphome

