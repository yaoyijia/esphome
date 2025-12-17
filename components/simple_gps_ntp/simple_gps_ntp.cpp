#include "simple_gps_ntp.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace simple_gps_ntp {

SimpleGPSNTPServer *SimpleGPSNTPServer::instance_ = nullptr;

// 添加常量定义
const uint32_t NTP_TIMESTAMP_DELTA = 2208988800UL;  // 1900-1970的秒数差

void IRAM_ATTR SimpleGPSNTPServer::pps_isr() {
  if (SimpleGPSNTPServer::instance_) {
    SimpleGPSNTPServer::instance_->last_pps_us_ = micros();
    SimpleGPSNTPServer::instance_->pps_count_++;
    SimpleGPSNTPServer::instance_->pps_active_ = true;
  }
}

void SimpleGPSNTPServer::setup() {
  ESP_LOGI("SimpleNTP", "Initializing Simple GPS NTP Server");
  
  instance_ = this;
  
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   &SimpleGPSNTPServer::pps_isr, 
                   FALLING);
    ESP_LOGI("SimpleNTP", "PPS on GPIO %d", pps_pin_);
  }
  
  memset(gps_buffer_, 0, sizeof(gps_buffer_));
  gps_idx_ = 0;
  
  // 初始化时间校准
  time_calibration_ = 0.0f;
  
  // 启动NTP服务器
  if (!ntp_started_) {
    udp_.begin(123);
    ntp_started_ = true;
    ESP_LOGI("SimpleNTP", "NTP server started on port 123");
  }
  
  // 检查系统时间
  time_t now;
  time(&now);
  if (now < 1609459200) { // 早于2021年
    ESP_LOGW("SimpleNTP", "System time not set: %ld", now);
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
          if (strstr(gps_buffer_, "$GPRMC") || strstr(gps_buffer_, "$GNRMC")) {
            // 解析RMC语句（包含日期和时间）
            char *ptr = gps_buffer_;
            int field_count = 0;
            char time_str[16] = {0};
            char date_str[16] = {0};
            
            while (*ptr) {
              if (*ptr == ',') {
                field_count++;
                if (field_count == 2) {
                  // 时间字段 HHMMSS.sss
                  ptr++;
                  int j = 0;
                  while (*ptr != ',' && *ptr != '.' && *ptr != '\0' && j < 15) {
                    time_str[j++] = *ptr++;
                  }
                  time_str[j] = 0;
                }
                else if (field_count == 10) {
                  // 日期字段 DDMMYY
                  ptr++;
                  int j = 0;
                  while (*ptr != ',' && *ptr != '\0' && j < 15) {
                    date_str[j++] = *ptr++;
                  }
                  date_str[j] = 0;
                }
              }
              ptr++;
            }
            
            // 如果有时间和日期，尝试设置系统时间
            if (strlen(time_str) == 6 && strlen(date_str) == 6) {
              // 解析时间
              int hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
              int minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
              int second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
              
              // 解析日期
              int day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
              int month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
              int year = (date_str[4] - '0') * 10 + (date_str[5] - '0') + 2000;
              
              // 构建tm结构（UTC时间）
              struct tm gps_tm;
              memset(&gps_tm, 0, sizeof(gps_tm));
              gps_tm.tm_year = year - 1900;
              gps_tm.tm_mon = month - 1;
              gps_tm.tm_mday = day;
              gps_tm.tm_hour = hour;
              gps_tm.tm_min = minute;
              gps_tm.tm_sec = second;
              gps_tm.tm_isdst = 0;
              
              // 直接使用mktime，假设系统时区为UTC
              // 在ESP32上，默认时区通常是UTC，或者可以通过环境变量设置
              time_t gps_time = mktime(&gps_tm);
              
              // 获取当前系统时间
              time_t now;
              time(&now);
              
              // 如果时间差大于10秒，则设置系统时间
              if (abs(gps_time - now) > 10) {
                struct timeval tv;
                tv.tv_sec = gps_time;
                tv.tv_usec = 0;
                settimeofday(&tv, NULL);
                
                // 同时更新内部GPS时间记录
                gps_hour_ = hour;
                gps_minute_ = minute;
                gps_second_ = second;
                gps_valid_ = true;
                
                ESP_LOGI("SimpleNTP", "System time set from GPS: %04d-%02d-%02d %02d:%02d:%02d UTC",
                         year, month, day, hour, minute, second);
              }
              
              ESP_LOGI("SimpleNTP", "GPS time: %02d:%02d:%02d, date: %04d-%02d-%02d UTC", 
                       hour, minute, second, year, month, day);
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
    
    if (interval_us > 900000 && interval_us < 1100000) {
      ESP_LOGD("SimpleNTP", "PPS #%u, interval: %.6f s", pps_count_, interval_us / 1000000.0f);
    } else {
      ESP_LOGW("SimpleNTP", "PPS abnormal interval: %u us", interval_us);
    }
  }
}

// 辅助函数：将tm结构转换为UTC时间（忽略时区）
time_t tm_to_utc(const struct tm *tm) {
  // 使用简单的算法计算UTC时间
  // 这个算法假设输入是UTC时间
  static const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  
  // 计算年份
  int year = tm->tm_year + 1900;
  time_t days = 0;
  
  // 1970年之前的天数
  for (int y = 1970; y < year; y++) {
    days += 365;
    if ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) {
      days++; // 闰年
    }
  }
  
  // 当年的月份天数
  for (int m = 0; m < tm->tm_mon; m++) {
    days += days_per_month[m];
    if (m == 1 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) {
      days++; // 闰年的二月
    }
  }
  
  // 当月天数
  days += (tm->tm_mday - 1);
  
  // 转换为秒数
  time_t seconds = days * 24 * 3600;
  seconds += tm->tm_hour * 3600;
  seconds += tm->tm_min * 60;
  seconds += tm->tm_sec;
  
  return seconds;
}

bool SimpleGPSNTPServer::get_ntp_time(uint32_t &seconds, uint32_t &fraction, uint32_t recv_sec, uint32_t recv_frac) {
  // 获取当前系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  uint32_t base_seconds = tv.tv_sec;
  uint32_t base_micros = tv.tv_usec;
  
  // 应用PPS微调
  if (pps_count_ > 0 && last_pps_us_ > 0) {
    uint32_t now_us = micros();
    uint32_t offset_us = (now_us - last_pps_us_) & 0xFFFFFFFFUL;
    
    if (offset_us < 1100000) {
      base_micros = offset_us;
      ESP_LOGD("SimpleNTP", "PPS adjusted: offset=%uus", offset_us);
    }
  }
  
  // 转换为NTP时间（基准1900年）
  seconds = base_seconds + NTP_TIMESTAMP_DELTA;
  fraction = (base_micros * 4294967296UL) / 1000000UL;
  
  return true;
}

void SimpleGPSNTPServer::handle_ntp() {
  if (!ntp_started_) return;
  
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    byte packet[48];
    udp_.read(packet, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    // 记录接收时间戳（在收到包后立即获取）
    struct timeval tv_recv;
    gettimeofday(&tv_recv, NULL);
    uint32_t recv_seconds = tv_recv.tv_sec + NTP_TIMESTAMP_DELTA;
    uint32_t recv_fraction = (tv_recv.tv_usec * 4294967296UL) / 1000000UL;
    
    ESP_LOGD("SimpleNTP", "NTP request from: %s:%d", remote.toString().c_str(), remotePort);
    
    // 获取传输时间戳
    uint32_t tx_seconds, tx_fraction;
    get_ntp_time(tx_seconds, tx_fraction, recv_seconds, recv_fraction);
    
    // 构建响应包
    memset(packet, 0, 48);
    
    // NTP头部
    packet[0] = 0b00100100;  // LI=0, Version=4, Mode=4 (server)
    packet[1] = 1;           // Stratum 1 (primary reference)
    packet[2] = 6;           // Poll interval: 64 seconds
    packet[3] = 0xFA;        // Precision: ~15.6 ms
    
    // Root delay (0.0000 seconds)
    packet[4] = 0x00;
    packet[5] = 0x00;
    packet[6] = 0x00;
    packet[7] = 0x00;
    
    // Root dispersion (0.0000 seconds)
    packet[8] = 0x00;
    packet[9] = 0x00;
    packet[10] = 0x00;
    packet[11] = 0x00;
    
    // Reference identifier (使用GPS)
    packet[12] = 'G';
    packet[13] = 'P';
    packet[14] = 'S';
    packet[15] = '0';
    
    // Reference timestamp (使用当前系统时间)
    uint32_t ref_seconds, ref_fraction;
    get_ntp_time(ref_seconds, ref_fraction, recv_seconds, recv_fraction);
    
    // 写入参考时间戳
    packet[16] = (ref_seconds >> 24) & 0xFF;
    packet[17] = (ref_seconds >> 16) & 0xFF;
    packet[18] = (ref_seconds >> 8) & 0xFF;
    packet[19] = ref_seconds & 0xFF;
    
    packet[20] = (ref_fraction >> 24) & 0xFF;
    packet[21] = (ref_fraction >> 16) & 0xFF;
    packet[22] = (ref_fraction >> 8) & 0xFF;
    packet[23] = ref_fraction & 0xFF;
    
    // Originate timestamp (复制客户端的时间戳)
    for (int i = 0; i < 8; i++) {
      packet[24 + i] = packet[40 + i];
    }
    
    // Receive timestamp (记录收到请求的时间)
    packet[32] = (recv_seconds >> 24) & 0xFF;
    packet[33] = (recv_seconds >> 16) & 0xFF;
    packet[34] = (recv_seconds >> 8) & 0xFF;
    packet[35] = recv_seconds & 0xFF;
    
    packet[36] = (recv_fraction >> 24) & 0xFF;
    packet[37] = (recv_fraction >> 16) & 0xFF;
    packet[38] = (recv_fraction >> 8) & 0xFF;
    packet[39] = recv_fraction & 0xFF;
    
    // Transmit timestamp (发送时间)
    packet[40] = (tx_seconds >> 24) & 0xFF;
    packet[41] = (tx_seconds >> 16) & 0xFF;
    packet[42] = (tx_seconds >> 8) & 0xFF;
    packet[43] = tx_seconds & 0xFF;
    
    packet[44] = (tx_fraction >> 24) & 0xFF;
    packet[45] = (tx_fraction >> 16) & 0xFF;
    packet[46] = (tx_fraction >> 8) & 0xFF;
    packet[47] = tx_fraction & 0xFF;
    
    // 发送响应
    udp_.beginPacket(remote, remotePort);
    udp_.write(packet, 48);
    udp_.endPacket();
    
    ESP_LOGD("SimpleNTP", "NTP response sent to %s", remote.toString().c_str());
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
  ESP_LOGCONFIG("SimpleNTP", "  GPS Valid: %s", gps_valid_ ? "YES" : "NO");
  ESP_LOGCONFIG("SimpleNTP", "  PPS Count: %u", pps_count_);
  ESP_LOGCONFIG("SimpleNTP", "  GPS Time: %02d:%02d:%02d UTC", gps_hour_, gps_minute_, gps_second_);
}

}  // namespace simple_gps_ntp
}  // namespace esphome
