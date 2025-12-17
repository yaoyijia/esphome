
#include "gps_ntp_server.h"
#include <cmath>

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// ==================== 中断处理 ====================
void IRAM_ATTR GPSNTPServer::pps_interrupt_handler() {
  if (GPSNTPServer::instance_) {
    GPSNTPServer::instance_->pps_.last_edge_us = micros();
    GPSNTPServer::instance_->pps_.count++;
    GPSNTPServer::instance_->pps_.triggered = true;
  }
}

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("GPSNTP", "初始化GPS NTP服务器");
  
  instance_ = this;
  
  // 配置PPS引脚
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                    pps_interrupt_handler, 
                    FALLING);
    ESP_LOGI("GPSNTP", "PPS引脚配置在GPIO %d", pps_pin_);
  }
  
  // 初始化NMEA缓冲区
  memset(nmea_buffer_, 0, sizeof(nmea_buffer_));
  nmea_index_ = 0;
  
  // 启动NTP服务器
  if (!ntp_running_) {
    ntp_socket_.begin(123);
    ntp_running_ = true;
    ESP_LOGI("GPSNTP", "NTP服务器已启动，端口123");
  }
  
  // 设置初始系统时间（如果未设置）
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < 1609459200) {  // 早于2021年
    // 设置为2024年1月1日
    tv.tv_sec = 1704067200;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    ESP_LOGW("GPSNTP", "系统时间重置为2024-01-01");
  }
}

// ==================== NMEA解析 ====================
void GPSNTPServer::parse_nmea() {
  if (!uart_) return;
  
  while (uart_->available()) {
    char c = uart_->read();
    
    // 寻找句子开始
    if (c == '$') {
      nmea_index_ = 0;
      nmea_buffer_[nmea_index_++] = c;
    }
    // 处理句子内容
    else if (nmea_index_ > 0 && nmea_index_ < sizeof(nmea_buffer_) - 1) {
      nmea_buffer_[nmea_index_++] = c;
      nmea_buffer_[nmea_index_] = '\0';
      
      // 完整句子
      if (c == '\n') {
        stats_gps_sentences_++;
        
        // 验证校验和（可选）
        bool checksum_ok = true;
        char *asterisk = strchr(nmea_buffer_, '*');
        if (asterisk) {
          // 这里可以添加校验和验证
        }
        
        // 解析不同类型的NMEA语句
        if (strstr(nmea_buffer_, "$GPRMC") || strstr(nmea_buffer_, "$GNRMC")) {
          checksum_ok &= parse_rmc(nmea_buffer_);
        }
        else if (strstr(nmea_buffer_, "$GPGGA") || strstr(nmea_buffer_, "$GNGGA")) {
          checksum_ok &= parse_gga(nmea_buffer_);
        }
        else if (strstr(nmea_buffer_, "$GPZDA") || strstr(nmea_buffer_, "$GNZDA")) {
          checksum_ok &= parse_zda(nmea_buffer_);
        }
        
        if (checksum_ok) {
          // 更新系统时间
          update_system_time();
        }
        
        nmea_index_ = 0;
      }
    }
    // 缓冲区溢出
    else if (nmea_index_ >= sizeof(nmea_buffer_) - 1) {
      nmea_index_ = 0;
      ESP_LOGW("GPSNTP", "NMEA缓冲区溢出");
    }
  }
}

bool GPSNTPServer::parse_rmc(const char *data) {
  // $GPRMC,hhmmss.sss,A,llll.llll,N,xxxxx.xxxx,W,spd,cog,ddmmyy,xxx,E,mode*cs
  char time_str[10] = {0};
  char date_str[10] = {0};
  char status;
  
  // 简单解析，实际应该使用更健壮的解析器
  int fields = sscanf(data, "$GPRMC,%9[^,],%c,", time_str, &status);
  
  if (fields >= 2 && status == 'A') {  // A=有效定位
    // 解析时间
    if (strlen(time_str) >= 6) {
      gps_time_.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
      gps_time_.minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
      gps_time_.second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
      
      // 解析毫秒部分
      char *dot = strchr(time_str, '.');
      if (dot) {
        gps_time_.microsecond = atoi(dot + 1) * 1000;  // 转换为微秒
      }
      
      // 查找日期字段（第9个字段）
      const char *ptr = data;
      int comma_count = 0;
      while (*ptr && comma_count < 9) {
        if (*ptr == ',') comma_count++;
        ptr++;
      }
      
      if (comma_count == 9 && strlen(ptr) >= 6) {
        // 解析日期 ddmmyy
        gps_time_.day = (ptr[0] - '0') * 10 + (ptr[1] - '0');
        gps_time_.month = (ptr[2] - '0') * 10 + (ptr[3] - '0');
        gps_time_.year = (ptr[4] - '0') * 10 + (ptr[5] - '0') + 2000;
        
        gps_time_.valid = true;
        gps_time_.last_update = millis();
        
        if (!gps_valid_) {
          gps_valid_ = true;
          ESP_LOGI("GPSNTP", "GPS时间有效: %04d-%02d-%02d %02d:%02d:%02d.%06d",
                   gps_time_.year, gps_time_.month, gps_time_.day,
                   gps_time_.hour, gps_time_.minute, gps_time_.second,
                   gps_time_.microsecond);
        }
        
        return true;
      }
    }
  }
  return false;
}

bool GPSNTPServer::parse_gga(const char *data) {
  // $GPGGA,hhmmss.sss,llll.llll,N,xxxxx.xxxx,W,fix,num_sats,hdop,alt,M,geoid,M,dgps_age,dgps_id*cs
  char time_str[10] = {0};
  char fix_char;
  
  int fields = sscanf(data, "$GPGGA,%9[^,],", time_str);
  
  if (fields >= 1 && strlen(time_str) >= 6) {
    // 只更新时间，不更新日期
    gps_time_.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
    gps_time_.minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
    gps_time_.second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    
    // 如果有日期信息，只更新时间部分
    if (gps_time_.valid) {
      gps_time_.last_update = millis();
      return true;
    }
  }
  return false;
}

bool GPSNTPServer::parse_zda(const char *data) {
  // $GPZDA,hhmmss.sss,dd,mm,yyyy,zz,zz*cs
  char time_str[10] = {0};
  uint8_t day, month;
  uint16_t year;
  
  int fields = sscanf(data, "$GPZDA,%9[^,],%hhu,%hhu,%hu,", 
                     time_str, &day, &month, &year);
  
  if (fields >= 4 && strlen(time_str) >= 6) {
    gps_time_.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
    gps_time_.minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
    gps_time_.second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    gps_time_.day = day;
    gps_time_.month = month;
    gps_time_.year = year;
    
    gps_time_.valid = true;
    gps_time_.last_update = millis();
    
    if (!gps_valid_) {
      gps_valid_ = true;
      ESP_LOGI("GPSNTP", "ZDA时间: %04d-%02d-%02d %02d:%02d:%02d",
               year, month, day,
               gps_time_.hour, gps_time_.minute, gps_time_.second);
    }
    
    return true;
  }
  return false;
}

// ==================== PPS处理 ====================
void GPSNTPServer::process_pps() {
  if (!pps_.triggered) return;
  
  pps_.triggered = false;
  uint32_t now_us = micros();
  uint32_t interval_us = (now_us - pps_.last_edge_us) & 0xFFFFFFFFUL;
  
  // 统计PPS间隔
  if (pps_.count > 1) {
    // 计算移动平均和标准差
    float alpha = 0.1f;  // 平滑因子
    float diff = interval_us - pps_.interval_avg;
    pps_.interval_avg = pps_.interval_avg * (1 - alpha) + interval_us * alpha;
    pps_.interval_std = pps_.interval_std * (1 - alpha) + fabs(diff) * alpha;
    
    // 检查PPS锁定状态
    if (pps_.interval_std < 10000.0f &&  // 标准差小于10ms
        fabs(pps_.interval_avg - 1000000.0f) < 50000.0f) {  // 平均接近1秒
      if (!pps_.locked) {
        pps_.locked = true;
        pps_.last_stable_count = pps_.count;
        ESP_LOGI("GPSNTP", "PPS已锁定: 平均间隔=%.1fus, 标准差=%.1fus",
                 pps_.interval_avg, pps_.interval_std);
      }
      pps_locked_ = true;
      time_accuracy_ = pps_.interval_std / 1000000.0f;  // 转换为秒
    } else {
      pps_locked_ = false;
    }
    
    // 校准系统时钟
    if (pps_locked_ && gps_time_.valid) {
      // PPS应该在整秒时刻发生
      uint32_t expected_us = 1000000UL;
      int64_t error_ns = (int64_t)(interval_us - expected_us) * 1000LL;
      
      // 使用简单的PI控制器校准
      float Kp = 0.1f;
      float Ki = 0.01f;
      static int64_t integral_error = 0;
      
      integral_error += error_ns;
      calibration_.offset_ns = (int64_t)(Kp * error_ns + Ki * integral_error);
      calibration_.accuracy_ns = pps_.interval_std * 1000.0f;
      
      if (pps_.count % 10 == 0) {
        ESP_LOGD("GPSNTP", "PPS校准: 误差=%lldns, 偏移=%lldns, 精度=%.0fns",
                 error_ns, calibration_.offset_ns, calibration_.accuracy_ns);
      }
    }
  }
  
  // 更新调试信息
  if (pps_.count % 60 == 0) {  // 每分钟
    ESP_LOGI("GPSNTP", "PPS统计: 计数=%u, 平均间隔=%.1fus, 标准差=%.1fus",
             pps_.count, pps_.interval_avg, pps_.interval_std);
  }
}

// ==================== 系统时间更新 ====================
void GPSNTPServer::update_system_time() {
  if (!gps_time_.valid) return;
  
  // 转换为Unix时间戳
  struct tm timeinfo = {0};
  timeinfo.tm_year = gps_time_.year - 1900;
  timeinfo.tm_mon = gps_time_.month - 1;
  timeinfo.tm_mday = gps_time_.day;
  timeinfo.tm_hour = gps_time_.hour;
  timeinfo.tm_min = gps_time_.minute;
  timeinfo.tm_sec = gps_time_.second;
  
  time_t unix_time = mktime(&timeinfo);
  
  // 检查时间是否合理
  if (unix_time > 1609459200) {  // 晚于2021年
    struct timeval tv;
    tv.tv_sec = unix_time;
    tv.tv_usec = gps_time_.microsecond;
    
    // 应用PPS校准
    if (pps_locked_) {
      tv.tv_usec += calibration_.offset_ns / 1000LL;
      if (tv.tv_usec >= 1000000) {
        tv.tv_sec += tv.tv_usec / 1000000;
        tv.tv_usec %= 1000000;
      }
    }
    
    settimeofday(&tv, nullptr);
    
    static uint32_t last_log = 0;
    if (millis() - last_log > 5000) {
      last_log = millis();
      ESP_LOGI("GPSNTP", "系统时间已更新: %04d-%02d-%02d %02d:%02d:%02d.%06d",
               gps_time_.year, gps_time_.month, gps_time_.day,
               gps_time_.hour, gps_time_.minute, gps_time_.second,
               tv.tv_usec);
    }
  }
}

// ==================== NTP时间戳计算 ====================
uint32_t GPSNTPServer::calculate_ntp_timestamp(uint32_t unix_seconds, uint32_t microseconds) {
  // NTP时间从1900-01-01开始，Unix时间从1970-01-01开始
  // 差值: 2208988800 秒
  static const uint32_t NTP_OFFSET = 2208988800UL;
  
  uint32_t ntp_seconds = unix_seconds + NTP_OFFSET;
  uint32_t ntp_fraction = (uint32_t)((uint64_t)microseconds * 4294967296ULL / 1000000ULL);
  
  return ntp_seconds;
}

bool GPSNTPServer::get_ntp_timestamp(uint32_t &seconds, uint32_t &fraction) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  
  // 检查时间是否合理
  if (tv.tv_sec < 1609459200) {  // 早于2021年
    return false;
  }
  
  // 转换为NTP时间
  seconds = tv.tv_sec + 2208988800UL;
  fraction = (uint32_t)((uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL);
  
  // 应用PPS校准
  if (pps_locked_) {
    // PPS校准提供了更精确的微秒级时间
    uint32_t now_us = micros();
    uint32_t since_pps = (now_us - pps_.last_edge_us) & 0xFFFFFFFFUL;
    
    if (since_pps < 1100000) {  // 在合理范围内
      uint32_t calibrated_us = since_pps % 1000000UL;
      fraction = (uint32_t)((uint64_t)calibrated_us * 4294967296ULL / 1000000ULL);
      
      // 如果跨越了秒边界
      if (since_pps >= 1000000) {
        seconds += since_pps / 1000000UL;
      }
    }
  }
  
  return true;
}

GPSNTPServer::TimeSource GPSNTPServer::get_time_source() const {
  if (gps_valid_ && pps_locked_) return TIME_SOURCE_GPS_PPS;
  if (pps_locked_) return TIME_SOURCE_PPS;
  if (gps_valid_) return TIME_SOURCE_GPS;
  return TIME_SOURCE_SYSTEM;
}

// ==================== NTP请求处理 ====================
void GPSNTPServer::handle_ntp_request() {
  if (!ntp_running_) return;
  
  int packet_size = ntp_socket_.parsePacket();
  if (packet_size >= 48) {
    uint8_t packet[48];
    ntp_socket_.read(packet, sizeof(packet));
    
    IPAddress remote_ip = ntp_socket_.remoteIP();
    uint16_t remote_port = ntp_socket_.remotePort();
    
    stats_ntp_requests_++;
    
    // 获取当前时间
    uint32_t ntp_seconds, ntp_fraction;
    bool time_valid = get_ntp_timestamp(ntp_seconds, ntp_fraction);
    
    if (!time_valid) {
      ESP_LOGW("GPSNTP", "时间无效，拒绝NTP请求");
      return;
    }
    
    // 构建响应包
    memset(packet, 0, 48);
    
    // NTP头部
    packet[0] = 0x24;  // LI=0, Version=4, Mode=4 (服务器)
    
    // Stratum (根据时间源质量)
    TimeSource source = get_time_source();
    switch (source) {
      case TIME_SOURCE_GPS_PPS:
        packet[1] = 1;  // 主参考源
        break;
      case TIME_SOURCE_PPS:
        packet[1] = 2;  // 二级参考源
        break;
      case TIME_SOURCE_GPS:
        packet[1] = 3;  // 三级参考源
        break;
      default:
        packet[1] = 4;  // 未知
    }
    
    // Poll interval (2^4 = 16秒)
    packet[2] = 4;
    
    // Precision (2^-8 ≈ 3.9ms)
    packet[3] = 0xE8;
    
    // Root Delay 和 Root Dispersion
    // 根据时间精度设置
    float accuracy_sec = time_accuracy_;
    uint32_t dispersion = (uint32_t)(accuracy_sec * 65536.0f);
    
    packet[4] = 0;  // Root Delay (整数部分)
    packet[5] = 0;
    packet[6] = 0;
    packet[7] = 0;
    
    packet[8] = (dispersion >> 24) & 0xFF;  // Root Dispersion
    packet[9] = (dispersion >> 16) & 0xFF;
    packet[10] = (dispersion >> 8) & 0xFF;
    packet[11] = dispersion & 0xFF;
    
    // Reference Identifier (GPS NTP)
    packet[12] = 'G';
    packet[13] = 'P';
    packet[14] = 'S';
    packet[15] = 'N';
    
    // 时间戳
    // Reference Timestamp (当前时间)
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
    ntp_socket_.beginPacket(remote_ip, remote_port);
    ntp_socket_.write(packet, sizeof(packet));
    ntp_socket_.endPacket();
    
    // 记录日志
    static uint32_t last_log = 0;
    if (stats_ntp_requests_ % 10 == 0 && millis() - last_log > 10000) {
      last_log = millis();
      time_t unix_time = ntp_seconds - 2208988800UL;
      struct tm *tm_info = gmtime(&unix_time);
      
      ESP_LOGI("GPSNTP", "NTP响应 #%u: %s:%d, 时间=%04d-%02d-%02d %02d:%02d:%02d, 源=%d",
               stats_ntp_requests_, remote_ip.toString().c_str(), remote_port,
               tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               source);
    }
  }
}

// ==================== 主循环 ====================
void GPSNTPServer::loop() {
  static uint32_t last_loop = 0;
  uint32_t now = millis();
  
  // 限制循环频率
  if (now - last_loop < 10) return;
  last_loop = now;
  
  // 处理GPS数据
  parse_nmea();
  
  // 处理PPS
  process_pps();
  
  // 处理NTP请求
  handle_ntp_request();
  
  // 定期状态更新
  static uint32_t last_status = 0;
  if (now - last_status > 30000) {  // 每30秒
    last_status = now;
    
    // 检查GPS超时
    if (gps_time_.valid && (now - gps_time_.last_update > 10000)) {
      gps_time_.valid = false;
      gps_valid_ = false;
      ESP_LOGW("GPSNTP", "GPS信号丢失");
    }
    
    // 输出状态
    TimeSource source = get_time_source();
    ESP_LOGI("GPSNTP", "状态: GPS=%s, PPS=%s, 源=%d, 精度=%.6fs, NTP请求=%u",
             gps_valid_ ? "是" : "否", 
             pps_locked_ ? "锁定" : "未锁定",
             source,
             time_accuracy_,
             stats_ntp_requests_);
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("GPSNTP", "GPS NTP服务器配置:");
  ESP_LOGCONFIG("GPSNTP", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("GPSNTP", "  GPS有效: %s", gps_valid_ ? "是" : "否");
  ESP_LOGCONFIG("GPSNTP", "  PPS锁定: %s", pps_locked_ ? "是" : "否");
  ESP_LOGCONFIG("GPSNTP", "  PPS计数: %u", pps_.count);
  ESP_LOGCONFIG("GPSNTP", "  时间精度: %.6f秒", time_accuracy_);
  ESP_LOGCONFIG("GPSNTP", "  时间源: %d", get_time_source());
  ESP_LOGCONFIG("GPSNTP", "  NTP请求: %u", stats_ntp_requests_);
  
  if (gps_time_.valid) {
    ESP_LOGCONFIG("GPSNTP", "  GPS时间: %04d-%02d-%02d %02d:%02d:%02d",
                 gps_time_.year, gps_time_.month, gps_time_.day,
                 gps_time_.hour, gps_time_.minute, gps_time_.second);
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome
