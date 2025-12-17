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
  
  // 初始化系统时间（确保有合理的时间）
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (tv.tv_sec < 1609459200) { // 如果时间早于2021年
    ESP_LOGW("SimpleNTP", "System time is too old: %u", tv.tv_sec);
    // 可以设置一个默认时间，但最好让网络时间协议先同步
  }
}

void SimpleGPSNTPServer::parse_gps() {
  if (!uart_) return;
  
  // 使用数组读取，效率更高
  uint8_t buffer[32];
  int available = uart_->available();
  
  while (available > 0) {
    int to_read = available < 32 ? available : 32;
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
        if (c == '\n') {
          // 检查是否是RMC语句（推荐最小定位信息）
          if (strstr(gps_buffer_, "$GPRMC") || strstr(gps_buffer_, "$GNRMC") || 
              strstr(gps_buffer_, "$GPGGA") || strstr(gps_buffer_, "$GNGGA")) {
            
            // 查找时间字段（通常在第2个字段）
            char *ptr = gps_buffer_;
            int field_count = 0;
            
            while (*ptr) {
              if (*ptr == ',') {
                field_count++;
                if (field_count == 2) {
                  // 找到时间字段
                  ptr++;
                  char time_str[16];
                  int j = 0;
                  
                  // 提取时间 HHMMSS.sss
                  while (*ptr != ',' && *ptr != '.' && *ptr != '\0' && j < 15) {
                    time_str[j++] = *ptr++;
                  }
                  time_str[j] = 0;
                  
                  if (strlen(time_str) == 6) {
                    // 解析时、分、秒
                    int hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
                    int minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
                    int second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
                    
                    // 记录GPS时间
                    gps_hour_ = hour;
                    gps_minute_ = minute;
                    gps_second_ = second;
                    gps_valid_ = true;
                    
                    // 获取当前系统时间，计算校准值
                    struct timeval tv;
                    gettimeofday(&tv, NULL);
                    struct tm *tm_info = gmtime(&tv.tv_sec);
                    
                    // 计算GPS时间与系统时间的秒数差（忽略日期）
                    int gps_total_seconds = hour * 3600 + minute * 60 + second;
                    int sys_total_seconds = tm_info->tm_hour * 3600 + tm_info->tm_min * 60 + tm_info->tm_sec;
                    
                    // 时间差（秒），注意处理跨天情况
                    int time_diff = gps_total_seconds - sys_total_seconds;
                    if (time_diff > 43200) time_diff -= 86400;  // 如果差超过12小时，可能是跨天
                    if (time_diff < -43200) time_diff += 86400; // 同上
                    
                    // 更新校准值（简单平均）
                    if (abs(time_diff) < 10) {  // 只在校准小于10秒时更新
                      time_calibration_ = time_calibration_ * 0.8 + time_diff * 0.2;
                      ESP_LOGI("SimpleNTP", "GPS校准: GPS=%02d:%02d:%02d, 系统=%02d:%02d:%02d, 校准值=%d秒", 
                               hour, minute, second,
                               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
                               (int)time_calibration_);
                    } else {
                      ESP_LOGW("SimpleNTP", "GPS时间与系统时间差异过大: %d秒", time_diff);
                    }
                    
                    ESP_LOGD("SimpleNTP", "GPS时间: %02d:%02d:%02d UTC", hour, minute, second);
                  }
                  break;
                }
              }
              ptr++;
            }
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

void SimpleGPSNTPServer::handle_pps() {
  static uint32_t last_pps_count = 0;
  
  if (pps_active_ && pps_count_ > last_pps_count) {
    last_pps_count = pps_count_;
    pps_active_ = false;
    
    // 每次PPS脉冲，记录调试信息
    uint32_t now_us = micros();
    uint32_t interval_us = (now_us - last_pps_us_) & 0xFFFFFFFFUL;
    
    if (interval_us > 900000 && interval_us < 1100000) { // 合理范围
      ESP_LOGD("SimpleNTP", "PPS #%u, 间隔: %.6f秒", pps_count_, interval_us / 1000000.0f);
    } else {
      ESP_LOGW("SimpleNTP", "PPS间隔异常: %u us", interval_us);
    }
  }
}

bool SimpleGPSNTPServer::get_ntp_time(uint32_t &seconds, uint32_t &fraction) {
  // 总是使用系统时间作为基础
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  uint32_t base_seconds = tv.tv_sec;
  uint32_t base_micros = tv.tv_usec;
  
  // 应用GPS时间校准（如果GPS有效）
  if (gps_valid_) {
    base_seconds += (int)time_calibration_;
  }
  
  // 应用PPS微调（如果PPS信号正常）
  if (pps_count_ > 0) {
    uint32_t now_us = micros();
    uint32_t offset_us = (now_us - last_pps_us_) & 0xFFFFFFFFUL;
    
    // 检查偏移是否在合理范围内（小于1.1秒）
    if (offset_us < 1100000) {
      // 使用微秒偏移进行更精确的调整
      base_micros = offset_us % 1000000UL;
      
      // 如果有跨越秒边界，调整秒数
      if (offset_us >= 1000000) {
        base_seconds += offset_us / 1000000UL;
      }
      
      ESP_LOGD("SimpleNTP", "PPS微调: 偏移=%uus, 调整后秒=%u.%06u", 
               offset_us, base_seconds, base_micros);
    } else {
      ESP_LOGD("SimpleNTP", "PPS偏移过大: %uus, 使用系统时间", offset_us);
    }
  }
  
  // 转换为NTP时间
  // Unix时间(1970)转NTP时间(1900)需要加2208988800秒
  seconds = base_seconds + 2208988800UL;
  fraction = (base_micros * 4294967296UL) / 1000000UL;
  
  // 调试信息
  static uint32_t last_log = 0;
  if (millis() - last_log > 5000) {
    last_log = millis();
    
    // 显示当前时间
    time_t now_time = base_seconds;
    struct tm *tm_info = gmtime(&now_time);
    
    ESP_LOGI("SimpleNTP", 
             "时间状态: 系统=%u, GPS校准=%+.1fs, PPS=%u, 最终时间=%04d-%02d-%02d %02d:%02d:%02d",
             tv.tv_sec, time_calibration_, pps_count_,
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  }
  
  return gps_valid_ || (pps_count_ > 0); // 返回是否使用了高精度源
}

void SimpleGPSNTPServer::handle_ntp() {
  if (!ntp_started_) return;
  
  int packetSize = udp_.parsePacket();
  if (packetSize) {
    byte packet[48];
    udp_.read(packet, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    ESP_LOGI("SimpleNTP", "NTP请求来自: %s:%d", remote.toString().c_str(), remotePort);
    
    uint32_t ntp_seconds, ntp_fraction;
    bool high_precision = get_ntp_time(ntp_seconds, ntp_fraction);
    
    // 构建响应包（完整NTPv4格式）
    memset(packet, 0, 48);
    
    // LI, Version, Mode (0b00=无警告, 0b100=版本4, 0b11=服务器模式)
    packet[0] = 0b00100100;
    
    // Stratum (时间层级): 高精度用1，普通用4
    packet[1] = high_precision ? 1 : 4;
    
    // Poll Interval (轮询间隔)
    packet[2] = 6; // 2^6 = 64秒
    
    // Precision (精度): 2^-6 ≈ 15.6毫秒
    packet[3] = 0xFA;
    
    // Root Delay和Root Dispersion（简单设置）
    packet[4] = 0; packet[5] = 0; packet[6] = 8; packet[7] = 0;
    packet[8] = 0; packet[9] = 0; packet[10] = 0xC; packet[11] = 0;
    
    // Reference Identifier（使用本机IP）
    IPAddress myIP = network::get_ip_addresses()[0];
    packet[12] = myIP[0];
    packet[13] = myIP[1];
    packet[14] = myIP[2];
    packet[15] = myIP[3];
    
    // Reference Timestamp（参考时间戳，使用发送时间戳）
    for (int i = 0; i < 8; i++) {
      packet[16 + i] = packet[40 + i];
    }
    
    // Originate Timestamp（复制客户端的时间戳）
    for (int i = 0; i < 8; i++) {
      packet[24 + i] = packet[40 + i];
    }
    
    // Receive Timestamp（接收时间戳）
    uint32_t receive_seconds = ntp_seconds;
    uint32_t receive_fraction = ntp_fraction;
    
    packet[32] = (receive_seconds >> 24) & 0xFF;
    packet[33] = (receive_seconds >> 16) & 0xFF;
    packet[34] = (receive_seconds >> 8) & 0xFF;
    packet[35] = receive_seconds & 0xFF;
    
    packet[36] = (receive_fraction >> 24) & 0xFF;
    packet[37] = (receive_fraction >> 16) & 0xFF;
    packet[38] = (receive_fraction >> 8) & 0xFF;
    packet[39] = receive_fraction & 0xFF;
    
    // Transmit Timestamp（发送时间戳，使用我们计算的时间）
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
    
    ESP_LOGD("SimpleNTP", "NTP响应发送到 %s, 时间: %u.%u", 
             remote.toString().c_str(), ntp_seconds, ntp_fraction);
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
  ESP_LOGCONFIG("SimpleNTP", "  GPS时间: %02d:%02d:%02d", gps_hour_, gps_minute_, gps_second_);
  ESP_LOGCONFIG("SimpleNTP", "  时间校准值: %.1f秒", time_calibration_);
}

}  // namespace simple_gps_ntp
}  // namespace esphome
