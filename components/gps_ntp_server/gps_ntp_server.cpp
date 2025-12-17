[file name]: gps_ntp_server.cpp
[file content begin]
#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <sys/time.h>

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// PPS中断处理
void IRAM_ATTR GPSNTPServer::pps_interrupt_handler() {
  if (GPSNTPServer::instance_) {
    GPSNTPServer::instance_->pps_last_edge_us_ = micros();
    GPSNTPServer::instance_->pps_count_++;
    GPSNTPServer::instance_->pps_triggered_ = true;
  }
}

void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器");
  
  instance_ = this;
  
  // 设置PPS引脚中断
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   pps_interrupt_handler, 
                   FALLING);
    ESP_LOGI("gps_ntp", "PPS引脚配置在GPIO %d", pps_pin_);
  }
  
  // 启动NTP服务器
  udp_.begin(123);
  ntp_started_ = true;
  ESP_LOGI("gps_ntp", "NTP服务器已启动，端口123");
}

void GPSNTPServer::on_update(gps::GPS &gps) {
  auto &tiny_gps = gps.get_tiny_gps();
  
  if (tiny_gps.time.isValid() && tiny_gps.date.isValid() && 
      tiny_gps.date.year() >= 2024) {
    
    // 使用GPS时间设置系统时间（类似gps_time组件的逻辑）
    struct tm timeinfo = {0};
    timeinfo.tm_year = tiny_gps.date.year() - 1900;
    timeinfo.tm_mon = tiny_gps.date.month() - 1;
    timeinfo.tm_mday = tiny_gps.date.day();
    timeinfo.tm_hour = tiny_gps.time.hour();
    timeinfo.tm_min = tiny_gps.time.minute();
    timeinfo.tm_sec = tiny_gps.time.second();
    
    // 设置时区为UTC
    setenv("TZ", "UTC", 1);
    tzset();
    
    time_t epoch = mktime(&timeinfo);
    
    if (epoch > 1609459200L) {  // 晚于2021年
      struct timeval tv = {epoch, 0};
      settimeofday(&tv, nullptr);
      
      gps_valid_ = true;
      last_gps_update_ = millis();
      
      if (!pps_active_) {  // 如果没有PPS，记录GPS时间
        ESP_LOGI("gps_ntp", "GPS时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 tiny_gps.date.year(), tiny_gps.date.month(), tiny_gps.date.day(),
                 tiny_gps.time.hour(), tiny_gps.time.minute(), tiny_gps.time.second());
      }
    }
  }
}

void GPSNTPServer::handle_pps() {
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_active_ = true;
    pps_last_stable_ = millis();
    
    uint32_t now_us = micros();
    uint32_t interval_us = (now_us - pps_last_edge_us_) & 0xFFFFFFFFUL;
    
    // 检查PPS间隔
    if (interval_us > 900000 && interval_us < 1100000) {
      if (pps_count_ % 60 == 0) {
        ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %.3fms", 
                 pps_count_, interval_us / 1000.0f);
      }
    } else {
      ESP_LOGW("gps_ntp", "PPS间隔异常: %.3fms", interval_us / 1000.0f);
    }
  }
  
  // 检查PPS是否丢失
  if (pps_active_ && (millis() - pps_last_stable_ > 2000)) {
    pps_active_ = false;
  }
}

uint8_t GPSNTPServer::get_time_quality() const {
  if (gps_valid_ && pps_active_) return QUALITY_GPS_PPS;
  if (pps_active_) return QUALITY_PPS;
  if (gps_valid_) return QUALITY_GPS;
  return QUALITY_SYSTEM;
}

void GPSNTPServer::process_ntp() {
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    byte packetBuffer[48];
    udp_.read(packetBuffer, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    ntp_requests_++;
    
    // 获取当前时间
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    // Unix时间转换为NTP时间（关键！）
    const unsigned long seventyYears = 2208988800UL;
    time_t timestamp = tv.tv_sec + seventyYears;
    
    // 根据时间质量设置stratum
    uint8_t quality = get_time_quality();
    uint8_t stratum;
    if (tv.tv_sec < seventyYears / 2) {
      stratum = 16;  // 时间无效，阻止同步
    } else {
      switch (quality) {
        case QUALITY_GPS_PPS:
          stratum = 1;   // 一级参考源
          break;
        case QUALITY_PPS:
        case QUALITY_GPS:
          stratum = 2;   // 二级参考源
          break;
        default:
          stratum = 4;   // 未知参考源
      }
    }
    
    // 构建NTP响应包（参考工作代码）
    memset(packetBuffer, 0, 48);
    
    packetBuffer[0] = 0b00100100;  // LI, Version, Mode
    packetBuffer[1] = stratum;
    packetBuffer[2] = 6;           // polling minimum
    packetBuffer[3] = 0xFA;        // precision
    
    // root delay
    packetBuffer[4] = 0;
    packetBuffer[5] = 0;
    packetBuffer[6] = 8;
    packetBuffer[7] = 0;
    
    // root dispersion
    packetBuffer[8] = 0;
    packetBuffer[9] = 0;
    packetBuffer[10] = 0xC;
    packetBuffer[11] = 0;
    
    // reference identifier (使用IP地址)
    IPAddress myIP = network::get_ip_addresses()[0];
    packetBuffer[12] = myIP[0];
    packetBuffer[13] = myIP[1];
    packetBuffer[14] = myIP[2];
    packetBuffer[15] = myIP[3];
    
    // reference timestamp
    uint32_t tempval = timestamp;
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    packetBuffer[19] = (tempval) & 0xFF;
    
    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;
    
    // copy originate timestamp from incoming packet
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];
    
    // receive timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    packetBuffer[35] = (tempval) & 0xFF;
    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;
    
    // transmit timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    packetBuffer[43] = (tempval) & 0xFF;
    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;
    
    // 发送响应
    udp_.beginPacket(remote, remotePort);
    udp_.write(packetBuffer, 48);
    udp_.endPacket();
    
    // 记录日志
    if (ntp_requests_ % 10 == 0) {
      time_t unix_time = timestamp - seventyYears;
      struct tm *tm_info = gmtime(&unix_time);
      
      ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d, 质量=%d",
               ntp_requests_, remote.toString().c_str(), remotePort,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               quality);
    }
  }
}

void GPSNTPServer::loop() {
  uint32_t now = millis();
  
  // 限制循环频率
  if (now - last_loop_ < 10) return;
  last_loop_ = now;
  
  // 处理PPS
  handle_pps();
  
  // 处理NTP请求
  process_ntp();
  
  // 定期状态更新
  static uint32_t last_status = 0;
  if (now - last_status > 30000) {
    last_status = now;
    
    // 检查GPS超时
    if (gps_valid_ && (now - last_gps_update_ > 10000)) {
      gps_valid_ = false;
    }
    
    // 输出状态
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      struct tm *tm_info = gmtime(&tv.tv_sec);
      
      ESP_LOGI("gps_ntp", "状态: GPS=%s, PPS=%s, PPS计数=%u, 质量=%d, UTC=%02d:%02d:%02d, NTP请求=%u",
               gps_valid_ ? "有效" : "无效",
               pps_active_ ? "活跃" : "无效",
               pps_count_,
               get_time_quality(),
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               ntp_requests_);
    }
  }
}

void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器配置:");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  GPS有效: %s", gps_valid_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  时间质量: %d", get_time_quality());
  ESP_LOGCONFIG("gps_ntp", "  NTP请求: %u", ntp_requests_);
  
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) == 0) {
    struct tm *tm_info = gmtime(&tv.tv_sec);
    
    ESP_LOGCONFIG("gps_ntp", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                 tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome
[file content end]
