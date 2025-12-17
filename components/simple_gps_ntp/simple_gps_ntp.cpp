#include "simple_gps_ntp.h"
#include "esphome/components/network/util.h"
#include <ctime>      // 添加ctime头文件
#include <cstring>    // 添加cstring头文件

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
  
  uint8_t buffer[64];
  int available = uart_->available();
  
  while (available > 0) {
    int to_read = available < 64 ? available : 64;
    uart_->read_array(buffer, to_read);
    
    for (int i = 0; i < to_read; i++) {
      char c = buffer[i];
      
      // 简单解析NMEA语句获取时间
      if (c == '$') {
        gps_idx_ = 0;
      }
      
      if (gps_idx_ < sizeof(gps_buffer_) - 1) {
        gps_buffer_[gps_idx_++] = c;
        gps_buffer_[gps_idx_] = 0;
        
        // 检查是否收到完整句子
        if (c == '\n' || c == '\r') {
          // 检查是否是RMC语句（推荐最小定位信息）
          if (strstr(gps_buffer_, "$GPRMC") || strstr(gps_buffer_, "$GNRMC")) {
            // 解析完整的RMC语句
            parse_rmc_sentence(gps_buffer_);
          }
          gps_idx_ = 0;
        }
      } else {
        gps_idx_ = 0; // 缓冲区溢出，重置
      }
    }
    
    available = uart_->available(); // 重新检查可用数据
  }
}

void SimpleGPSNTPServer::parse_rmc_sentence(char* sentence) {
  // 复制句子，因为strtok会修改原始字符串
  char sentence_copy[256];
  strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
  sentence_copy[sizeof(sentence_copy) - 1] = 0;
  
  // 使用strtok分割逗号
  char* tokens[20];
  int token_count = 0;
  
  char* token = strtok(sentence_copy, ",");
  while (token != NULL && token_count < 20) {
    tokens[token_count++] = token;
    token = strtok(NULL, ",");
  }
  
  // GPRMC格式: $GPRMC,hhmmss.sss,A,llll.ll,N,...
  // 字段索引: 0=类型, 1=时间, 2=状态, 3=纬度, 4=N/S, 5=经度, 6=E/W, 
  //          7=速度, 8=航向, 9=日期, 10=磁偏角, ...
  
  if (token_count >= 10 && tokens[2] && strcmp(tokens[2], "A") == 0) {  // A=有效定位
    // 解析时间 (hhmmss.sss)
    if (tokens[1] && strlen(tokens[1]) >= 6) {
      char hour_str[3] = {tokens[1][0], tokens[1][1], 0};
      char minute_str[3] = {tokens[1][2], tokens[1][3], 0};
      char second_str[3] = {tokens[1][4], tokens[1][5], 0};
      
      gps_hour_ = atoi(hour_str);
      gps_minute_ = atoi(minute_str);
      gps_second_ = atoi(second_str);
      
      // 计算当天秒数（用于PPS同步）
      gps_seconds_ = gps_hour_ * 3600 + gps_minute_ * 60 + gps_second_;
      
      ESP_LOGD("SimpleNTP", "GPS时间: %02d:%02d:%02d", 
               gps_hour_, gps_minute_, gps_second_);
    }
    
    // 解析日期 (ddmmyy)
    if (tokens[9] && strlen(tokens[9]) == 6) {
      char day_str[3] = {tokens[9][0], tokens[9][1], 0};
      char month_str[3] = {tokens[9][2], tokens[9][3], 0};
      char year_str[3] = {tokens[9][4], tokens[9][5], 0};
      
      gps_day_ = atoi(day_str);
      gps_month_ = atoi(month_str);
      int year_short = atoi(year_str);
      
      // 两位年份转四位年份
      if (year_short >= 80) {
        gps_year_ = 1900 + year_short;  // 1980-1999
      } else {
        gps_year_ = 2000 + year_short;  // 2000-2079
      }
      
      ESP_LOGD("SimpleNTP", "GPS日期: %04d-%02d-%02d", 
               gps_year_, gps_month_, gps_day_);
      
      // 计算Unix时间戳
      calculate_unix_timestamp();
      
      gps_valid_ = true;
      
      ESP_LOGI("SimpleNTP", "GPS时间有效: %04d-%02d-%02d %02d:%02d:%02d", 
               gps_year_, gps_month_, gps_day_,
               gps_hour_, gps_minute_, gps_second_);
    }
  }
}

void SimpleGPSNTPServer::calculate_unix_timestamp() {
  // 使用标准库的tm结构和mktime计算时间戳
  struct tm timeinfo;
  
  // 清零结构体
  memset(&timeinfo, 0, sizeof(timeinfo));
  
  // 设置时间（注意：tm_year从1900年开始，tm_month从0开始）
  timeinfo.tm_year = gps_year_ - 1900;  // 年份从1900开始
  timeinfo.tm_mon = gps_month_ - 1;     // 月份从0开始
  timeinfo.tm_mday = gps_day_;
  timeinfo.tm_hour = gps_hour_;
  timeinfo.tm_min = gps_minute_;
  timeinfo.tm_sec = gps_second_;
  timeinfo.tm_isdst = 0;  // 不使用夏令时
  
  // 使用mktime计算时间戳
  // 注意：mktime假设输入的是本地时间，但GPS提供的是UTC时间
  // 在ESP8266/ESP32上，mktime会根据时区设置转换，所以我们需要确保时区正确
  // 或者使用timegm（如果可用）
  
  // 临时方法：设置时区为UTC
  setenv("TZ", "UTC0", 1);
  tzset();
  
  gps_timestamp_ = mktime(&timeinfo);
  
  // 恢复默认时区（如果需要）
  // setenv("TZ", "CST-8", 1);  // 例如中国标准时间
  // tzset();
  
  ESP_LOGD("SimpleNTP", "Unix时间戳: %u", gps_timestamp_);
  
  // 验证时间戳是否合理（应该大于1609459200 = 2021-01-01 00:00:00）
  if (gps_timestamp_ < 1609459200) {
    ESP_LOGW("SimpleNTP", "时间戳可能无效: %u", gps_timestamp_);
  }
}

void SimpleGPSNTPServer::handle_pps() {
  static uint32_t last_pps_count = 0;
  
  if (pps_active_ && pps_count_ > last_pps_count) {
    last_pps_count = pps_count_;
    pps_active_ = false;
    
    // 每次PPS脉冲，GPS秒数加1
    if (gps_valid_) {
      gps_seconds_++;
      
      // 处理溢出（超过24小时）
      if (gps_seconds_ >= 86400) {
        gps_seconds_ -= 86400;
        // 如果跨越了一天，增加时间戳的日期部分
        if (gps_timestamp_ > 0) {
          gps_timestamp_ += 86400; // 增加一天
        }
      }
      
      // 更新时间戳的秒数部分
      if (gps_timestamp_ > 0) {
        // 获取当前时间戳对应的日期
        time_t current_time = gps_timestamp_;
        struct tm *timeinfo = gmtime(&current_time);
        
        if (timeinfo) {
          // 设置新的秒数
          timeinfo->tm_hour = gps_seconds_ / 3600;
          timeinfo->tm_min = (gps_seconds_ % 3600) / 60;
          timeinfo->tm_sec = gps_seconds_ % 60;
          
          // 重新计算时间戳
          setenv("TZ", "UTC0", 1);
          tzset();
          gps_timestamp_ = mktime(timeinfo);
        }
      }
      
      ESP_LOGD("SimpleNTP", "PPS #%u, GPS seconds: %u, Unix timestamp: %u", 
               pps_count_, gps_seconds_, gps_timestamp_);
    }
  }
}

bool SimpleGPSNTPServer::get_ntp_time(uint32_t &seconds, uint32_t &fraction) {
  // 检查是否有有效的GPS时间戳
  if (gps_timestamp_ == 0 || pps_count_ == 0) {
    // 回退到系统时间
    struct timeval tv;
    gettimeofday(&tv, NULL);
    seconds = tv.tv_sec + 2208988800UL;  // Unix转NTP
    fraction = (tv.tv_usec * 4294967296UL) / 1000000UL;
    ESP_LOGD("SimpleNTP", "使用系统时间: Unix=%u, NTP=%u", 
             tv.tv_sec, seconds);
    return false;
  }
  
  // 计算从上次PPS到现在的微秒偏移
  uint32_t now_us = micros();
  uint32_t offset_us = (now_us - last_pps_us_) & 0xFFFFFFFFUL;
  
  // 检查偏移是否合理
  if (offset_us > 1100000) {
    ESP_LOGW("SimpleNTP", "Large offset: %u us, using system time", offset_us);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    seconds = tv.tv_sec + 2208988800UL;
    fraction = (tv.tv_usec * 4294967296UL) / 1000000UL;
    return false;
  }
  
  // 计算NTP时间
  // 注意：gps_timestamp_ 已经是Unix时间戳
  // Unix时间戳（从1970-01-01开始）转NTP时间戳（从1900-01-01开始）：加2208988800秒
  seconds = gps_timestamp_ + 2208988800UL + (offset_us / 1000000UL);
  fraction = ((offset_us % 1000000UL) * 4294967296UL) / 1000000UL;
  
  ESP_LOGD("SimpleNTP", "高精度时间: Unix=%u, NTP=%u, 偏移=%uus", 
           gps_timestamp_, seconds, offset_us);
  
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
    
    ESP_LOGD("SimpleNTP", "NTP response sent to %s", udp_.remoteIP().toString().c_str());
  }
}

void SimpleGPSNTPServer::loop() {
  uint32_t now = millis();
  
  // 限制循环频率（每10ms执行一次）
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
  ESP_LOGCONFIG("SimpleNTP", "  PPS Count: %u", pps_count_);
  if (gps_valid_) {
    ESP_LOGCONFIG("SimpleNTP", "  GPS Date: %04d-%02d-%02d", 
                  gps_year_, gps_month_, gps_day_);
    ESP_LOGCONFIG("SimpleNTP", "  GPS Time: %02d:%02d:%02d", 
                  gps_hour_, gps_minute_, gps_second_);
    ESP_LOGCONFIG("SimpleNTP", "  Unix Timestamp: %u", gps_timestamp_);
  }
}

}  // namespace simple_gps_ntp
}  // namespace esphome
