
#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <time.h>
#include <sys/time.h>

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// ==================== 中断处理函数 ====================
void IRAM_ATTR GPSNTPServer::pps_interrupt_handler() {
  if (GPSNTPServer::instance_) {
    GPSNTPServer::instance_->pps_last_edge_us_ = micros();
    GPSNTPServer::instance_->pps_count_++;
    GPSNTPServer::instance_->pps_triggered_ = true;
  }
}

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器");
  
  // 设置实例指针
  instance_ = this;
  
  // 配置PPS引脚
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   &GPSNTPServer::pps_interrupt_handler, 
                   FALLING);
    ESP_LOGI("gps_ntp", "PPS引脚配置在GPIO %d", pps_pin_);
  }
  
  // 初始化NMEA缓冲区
  memset(nmea_buffer_, 0, sizeof(nmea_buffer_));
  nmea_index_ = 0;
  
  // 启动NTP服务器
  if (!ntp_running_) {
    udp_.begin(123);
    ntp_running_ = true;
    ESP_LOGI("gps_ntp", "NTP服务器已启动，端口123");
  }
  
  // 初始化系统时间（如果时间太旧）
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < 1609459200) {  // 早于2021年
    // 设置为2024年1月1日
    tv.tv_sec = 1704067200;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    ESP_LOGW("gps_ntp", "系统时间重置为2024-01-01 00:00:00");
  }
}

// ==================== NMEA解析 ====================
void GPSNTPServer::parse_nmea() {
  if (!uart_) return;
  
  // 使用字节数组读取，效率更高
  uint8_t buffer[32];
  int available = uart_->available();
  
  while (available > 0) {
    int to_read = available < 32 ? available : 32;
    uart_->read_array(buffer, to_read);
    
    for (int i = 0; i < to_read; i++) {
      char c = buffer[i];
      
      // 开始新的NMEA语句
      if (c == '$') {
        nmea_index_ = 0;
        nmea_buffer_[nmea_index_++] = c;
      }
      // 处理语句内容
      else if (nmea_index_ > 0 && nmea_index_ < sizeof(nmea_buffer_) - 1) {
        nmea_buffer_[nmea_index_++] = c;
        nmea_buffer_[nmea_index_] = '\0';
        
        // 完整语句
        if (c == '\n') {
          // 根据语句类型解析
          if (strstr(nmea_buffer_, "$GPRMC") || strstr(nmea_buffer_, "$GNRMC")) {
            parse_gprmc(nmea_buffer_);
          }
          else if (strstr(nmea_buffer_, "$GPGGA") || strstr(nmea_buffer_, "$GNGGA")) {
            parse_gpgga(nmea_buffer_);
          }
          
          nmea_index_ = 0;
        }
      }
      // 缓冲区溢出
      else if (nmea_index_ >= sizeof(nmea_buffer_) - 1) {
        nmea_index_ = 0;
      }
    }
    
    available = uart_->available();
  }
}

bool GPSNTPServer::parse_gprmc(const char *data) {
  // $GPRMC,hhmmss.sss,A,llll.llll,N,xxxxx.xxxx,W,spd,cog,ddmmyy,xxx,E,mode*cs
  char time_str[16] = {0};
  char date_str[16] = {0};
  char status;
  
  // 简单解析时间字段
  int fields = sscanf(data, "$GPRMC,%15[^,],%c,", time_str, &status);
  
  if (fields >= 2 && status == 'A') {  // A=有效定位
    // 解析时间
    if (strlen(time_str) >= 6) {
      gps_hour_ = (time_str[0] - '0') * 10 + (time_str[1] - '0');
      gps_minute_ = (time_str[2] - '0') * 10 + (time_str[3] - '0');
      gps_second_ = (time_str[4] - '0') * 10 + (time_str[5] - '0');
      
      // 查找日期字段（第9个字段）
      const char *ptr = data;
      int comma_count = 0;
      while (*ptr && comma_count < 9) {
        if (*ptr == ',') comma_count++;
        ptr++;
      }
      
      if (comma_count == 9 && strlen(ptr) >= 6) {
        // 解析日期 ddmmyy
        gps_day_ = (ptr[0] - '0') * 10 + (ptr[1] - '0');
        gps_month_ = (ptr[2] - '0') * 10 + (ptr[3] - '0');
        gps_year_ = (ptr[4] - '0') * 10 + (ptr[5] - '0') + 2000;
        
        // 设置系统时间
        struct tm timeinfo = {0};
        timeinfo.tm_year = gps_year_ - 1900;
        timeinfo.tm_mon = gps_month_ - 1;
        timeinfo.tm_mday = gps_day_;
        timeinfo.tm_hour = gps_hour_;
        timeinfo.tm_min = gps_minute_;
        timeinfo.tm_sec = gps_second_;
        
        time_t epoch = mktime(&timeinfo);
        
        // 检查时间是否合理（晚于2020年）
        if (epoch > 1577836800) {
          struct timeval tv = {epoch, 0};
          settimeofday(&tv, nullptr);
          gps_valid_ = true;
          last_gps_update_ = millis();
          
          ESP_LOGI("gps_ntp", "GPS时间已设置: %04d-%02d-%02d %02d:%02d:%02d UTC",
                   gps_year_, gps_month_, gps_day_,
                   gps_hour_, gps_minute_, gps_second_);
        }
        return true;
      }
    }
  }
  return false;
}

bool GPSNTPServer::parse_gpgga(const char *data) {
  // $GPGGA,hhmmss.sss,llll.llll,N,xxxxx.xxxx,W,fix,num_sats,hdop,alt,M,geoid,M,dgps_age,dgps_id*cs
  char time_str[16] = {0};
  char fix_char;
  
  int fields = sscanf(data, "$GPGGA,%15[^,],", time_str);
  
  if (fields >= 1 && strlen(time_str) >= 6) {
    // 只更新时间，不更新日期
    gps_hour_ = (time_str[0] - '0') * 10 + (time_str[1] - '0');
    gps_minute_ = (time_str[2] - '0') * 10 + (time_str[3] - '0');
    gps_second_ = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    
    // 如果已经有日期信息，更新时间部分
    if (gps_year_ > 0) {
      gps_valid_ = true;
      last_gps_update_ = millis();
    }
    return true;
  }
  return false;
}

// ==================== PPS处理 ====================
void GPSNTPServer::handle_pps() {
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_active_ = true;
    
    uint32_t now_us = micros();
    uint32_t interval_us = (now_us - pps_last_edge_us_) & 0xFFFFFFFFUL;
    
    // 检查PPS间隔是否稳定
    if (interval_us > 900000 && interval_us < 1100000) {
      pps_last_stable_ = millis();
      
      // 调试信息
      if (pps_count_ % 60 == 0) {  // 每分钟输出一次
        ESP_LOGI("gps_ntp", "PPS #%u, 间隔: %.6f秒", pps_count_, interval_us / 1000000.0f);
      }
    } else {
      ESP_LOGW("gps_ntp", "PPS间隔异常: %u us", interval_us);
    }
  }
  
  // 检查PPS是否活跃
  if (pps_active_ && (millis() - pps_last_stable_ > 2000)) {
    pps_active_ = false;
  }
}

// ==================== NTP时间戳计算 ====================
bool GPSNTPServer::get_ntp_time(uint32_t &seconds, uint32_t &fraction) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  
  // 检查系统时间是否合理
  if (tv.tv_sec < 1609459200) {  // 早于2021年
    return false;
  }
  
  // Unix时间（1970年1月1日）转换为NTP时间（1900年1月1日）
  // NTP时间 = Unix时间 + 2208988800
  seconds = tv.tv_sec + 2208988800UL;
  
  // 微秒转换为2^-32秒的分数部分
  fraction = (uint32_t)((uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL);
  
  // 如果有活跃的PPS信号，进行微秒级调整
  if (pps_active_ && pps_count_ > 0) {
    uint32_t now_us = micros();
    uint32_t offset_us = (now_us - pps_last_edge_us_) & 0xFFFFFFFFUL;
    
    if (offset_us < 1100000) {  // 在合理范围内
      uint32_t pps_micros = offset_us % 1000000UL;
      fraction = (uint32_t)((uint64_t)pps_micros * 4294967296ULL / 1000000ULL);
      
      // 如果跨越了秒边界
      if (offset_us >= 1000000) {
        seconds += offset_us / 1000000UL;
      }
    }
  }
  
  return true;
}

int GPSNTPServer::get_time_source() const {
  if (gps_valid_ && pps_active_) return TIME_SOURCE_GPS_PPS;
  if (pps_active_) return TIME_SOURCE_PPS;
  if (gps_valid_) return TIME_SOURCE_GPS;
  return TIME_SOURCE_SYSTEM;
}

// ==================== NTP请求处理 ====================
void GPSNTPServer::handle_ntp() {
  if (!ntp_running_) return;
  
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    uint8_t packet[48];
    udp_.read(packet, 48);
    
    IPAddress remote = udp_.remoteIP();
    uint16_t remotePort = udp_.remotePort();
    
    ntp_requests_++;
    
    // 获取当前时间
    uint32_t ntp_seconds, ntp_fraction;
    bool time_valid = get_ntp_time(ntp_seconds, ntp_fraction);
    
    if (!time_valid) {
      return;  // 时间无效，不响应
    }
    
    // 构建响应包
    memset(packet, 0, 48);
    
    // NTP头部
    packet[0] = 0x24;  // LI=0, Version=4, Mode=4 (服务器)
    
    // Stratum (根据时间源质量)
    int source = get_time_source();
    switch (source) {
      case TIME_SOURCE_GPS_PPS:
        packet[1] = 1;  // 主参考源
        break;
      case TIME_SOURCE_PPS:
      case TIME_SOURCE_GPS:
        packet[1] = 2;  // 二级参考源
        break;
      default:
        packet[1] = 3;  // 三级参考源
    }
    
    // Poll interval (2^4 = 16秒)
    packet[2] = 4;
    
    // Precision (2^-8 ≈ 3.9ms)
    packet[3] = 0xE8;
    
    // Root Delay (0)
    packet[4] = 0;
    packet[5] = 0;
    packet[6] = 0;
    packet[7] = 0;
    
    // Root Dispersion (0.5秒)
    packet[8] = 0;
    packet[9] = 0;
    packet[10] = 0x80;
    packet[11] = 0;
    
    // Reference Identifier (GPS NTP)
    packet[12] = 'G';
    packet[13] = 'P';
    packet[14] = 'S';
    packet[15] = 'N';
    
    // Reference Timestamp (参考时间戳，使用当前时间)
    packet[16] = (ntp_seconds >> 24) & 0xFF;
    packet[17] = (ntp_seconds >> 16) & 0xFF;
    packet[18] = (ntp_seconds >> 8) & 0xFF;
    packet[19] = ntp_seconds & 0xFF;
    packet[20] = (ntp_fraction >> 24) & 0xFF;
    packet[21] = (ntp_fraction >> 16) & 0xFF;
    packet[22] = (ntp_fraction >> 8) & 0xFF;
    packet[23] = ntp_fraction & 0xFF;
    
    // Originate Timestamp (复制客户端时间)
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
    
    // Transmit Timestamp (发送时间)
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
    
    // 记录日志
    if (ntp_requests_ % 10 == 0) {
      time_t unix_time = ntp_seconds - 2208988800UL;
      struct tm *tm_info = gmtime(&unix_time);
      
      ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%04d-%02d-%02d %02d:%02d:%02d",
               ntp_requests_, remote.toString().c_str(), remotePort,
               tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    }
  }
}

// ==================== 主循环 ====================
void GPSNTPServer::loop() {
  uint32_t now = millis();
  
  // 限制循环频率（10ms）
  if (now - last_loop_ < 10) return;
  last_loop_ = now;
  
  // 解析GPS数据
  parse_nmea();
  
  // 处理PPS
  handle_pps();
  
  // 处理NTP请求
  handle_ntp();
  
  // 定期状态更新
  if (now - last_status_log_ > 30000) {  // 每30秒
    last_status_log_ = now;
    
    // 检查GPS超时
    if (gps_valid_ && (now - last_gps_update_ > 10000)) {
      gps_valid_ = false;
      ESP_LOGW("gps_ntp", "GPS信号丢失");
    }
    
    // 输出状态
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    struct tm *tm_info = gmtime(&tv.tv_sec);
    
    ESP_LOGI("gps_ntp", "状态: GPS=%s, PPS=%s, 源=%d, UTC=%02d:%02d:%02d, NTP请求=%u",
             gps_valid_ ? "有效" : "无效", 
             pps_active_ ? "活跃" : "无效",
             get_time_source(),
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             ntp_requests_);
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器:");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  GPS有效: %s", gps_valid_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  时间源: %d", get_time_source());
  ESP_LOGCONFIG("gps_ntp", "  NTP请求: %u", ntp_requests_);
  
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  struct tm *tm_info = gmtime(&tv.tv_sec);
  
  ESP_LOGCONFIG("gps_ntp", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
               tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  
  if (gps_valid_) {
    ESP_LOGCONFIG("gps_ntp", "  GPS时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 gps_year_, gps_month_, gps_day_,
                 gps_hour_, gps_minute_, gps_second_);
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome

