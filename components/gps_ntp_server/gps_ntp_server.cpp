
#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <sys/time.h>

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// ==================== PPS中断处理 ====================
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
  
  // 设置时区为UTC
  setenv("TZ", "UTC", 1);
  tzset();
}

// ==================== GPS更新回调 ====================
void GPSNTPServer::on_update(TinyGPSPlus &tiny_gps) {
  // 检查GPS数据是否有效
  if (!tiny_gps.time.isValid() || !tiny_gps.date.isValid() || 
      !tiny_gps.time.isUpdated() || !tiny_gps.date.isUpdated() || 
      tiny_gps.date.year() < 2024) {
    return;
  }
  
  // 记录GPS时间（只记录整数秒）
  gps_time_.year = tiny_gps.date.year();
  gps_time_.month = tiny_gps.date.month();
  gps_time_.day = tiny_gps.date.day();
  gps_time_.hour = tiny_gps.time.hour();
  gps_time_.minute = tiny_gps.time.minute();
  gps_time_.second = tiny_gps.time.second();
  gps_time_.valid = true;
  gps_time_.last_update = millis();
  
  ESP_LOGI("gps_ntp", "GPS时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
           gps_time_.year, gps_time_.month, gps_time_.day,
           gps_time_.hour, gps_time_.minute, gps_time_.second);
}

// ==================== PPS处理（新逻辑） ====================
void GPSNTPServer::handle_pps() {
  static uint32_t last_pps_count = 0;
  
  if (pps_triggered_ && pps_count_ > last_pps_count) {
    pps_triggered_ = false;
    last_pps_count = pps_count_;
    pps_active_ = true;
    pps_last_stable_ = millis();
    
    uint32_t now_us = micros();
    uint32_t interval_us = (now_us - pps_last_edge_us_) & 0xFFFFFFFFUL;
    
    // 检查PPS间隔是否稳定（0.9-1.1秒）
    if (interval_us > 900000 && interval_us < 1100000) {
      // ========== 关键逻辑：PPS脉冲时设置系统时间 ==========
      if (gps_time_.valid) {
        // GPS秒数加1，使用mktime自动处理所有溢出
        struct tm timeinfo = {0};
        timeinfo.tm_year = gps_time_.year - 1900;
        timeinfo.tm_mon = gps_time_.month - 1;
        timeinfo.tm_mday = gps_time_.day;
        timeinfo.tm_hour = gps_time_.hour;
        timeinfo.tm_min = gps_time_.minute;
        timeinfo.tm_sec = gps_time_.second + 1;  // GPS秒数加1
        
        // 设置时区为UTC
        setenv("TZ", "UTC", 1);
        tzset();
        
        // mktime会自动处理所有溢出：
        // - 60秒 -> 1分钟
        // - 60分 -> 1小时  
        // - 24小时 -> 1天
        // - 超过当月天数 -> 下个月
        // - 12月 -> 1月，年份+1
        // - 闰年处理
        time_t epoch = mktime(&timeinfo);
        
        if (epoch == -1) {
          ESP_LOGE("gps_ntp", "mktime转换失败");
          return;
        }
        
        // 检查时间是否合理（晚于2020年）
        if (epoch > 1609459200L) {
          struct timeval tv = {epoch, 0};  // 微秒设为0
          
          if (settimeofday(&tv, nullptr) == 0) {
            time_valid_ = true;
            last_pps_sync_ = millis();
            last_pps_second_ = pps_count_;
            
            // 获取规范化后的时间用于日志（mktime可能修改了timeinfo）
            ESP_LOGI("gps_ntp", "PPS同步时间: %04d-%02d-%02d %02d:%02d:%02d.%06u UTC",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                     0);
          } else {
            ESP_LOGE("gps_ntp", "设置系统时间失败");
          }
        }
      }
      
      // 调试信息
      if (pps_count_ % 60 == 0) {
        ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %.6f秒", pps_count_, interval_us / 1000000.0f);
      }
    } else {
      ESP_LOGW("gps_ntp", "PPS间隔异常: %.6f秒", interval_us / 1000000.0f);
    }
  }
  
  // 检查PPS是否丢失
  if (pps_active_ && (millis() - pps_last_stable_ > 2000)) {
    pps_active_ = false;
  }
}

// ==================== 时间质量评估 ====================
uint8_t GPSNTPServer::get_time_quality() const {
  if (time_valid_ && pps_active_) return QUALITY_GPS_PPS;
  if (pps_active_) return QUALITY_PPS;
  if (time_valid_) return QUALITY_GPS;
  return QUALITY_SYSTEM;
}

// ==================== NTP请求处理 ====================
void GPSNTPServer::process_ntp() {
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    byte packetBuffer[48];
    udp_.read(packetBuffer, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    ntp_requests_++;
    
    // 保存客户端的Transmit Timestamp
    byte clientTransmit[8];
    for (int i = 0; i < 8; i++) {
      clientTransmit[i] = packetBuffer[40 + i];
    }
    
    // 获取当前系统时间
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    // Unix时间转换为NTP时间
    const unsigned long seventyYears = 2208988800UL;
    uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
    
    // 计算微秒部分
    uint64_t ntp_fraction = (uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL;
    
    // 如果PPS活跃，使用PPS提供的更精确的微秒计数
    if (pps_active_ && pps_count_ > last_pps_second_) {
      uint32_t now_us = micros();
      uint32_t since_pps = (now_us - pps_last_edge_us_) & 0xFFFFFFFFUL;
      
      // 只使用当前秒内的微秒计数
      if (since_pps < 1000000) {
        ntp_fraction = (uint64_t)since_pps * 4294967296ULL / 1000000ULL;
      }
    }
    
    // 根据时间质量设置stratum
    uint8_t quality = get_time_quality();
    uint8_t stratum;
    
    if (tv.tv_sec < seventyYears / 2) {
      stratum = 16;  // 时间无效
    } else {
      switch (quality) {
        case QUALITY_GPS_PPS:
          stratum = 1;
          break;
        case QUALITY_PPS:
        case QUALITY_GPS:
          stratum = 2;
          break;
        default:
          stratum = 4;
      }
    }
    
    // 构建NTP响应包
    memset(packetBuffer, 0, 48);
    
    // NTP头部
    packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
    packetBuffer[1] = stratum;
    packetBuffer[2] = 6;     // Poll interval: 64秒
    packetBuffer[3] = 0xFA;  // Precision: 2^-6 ≈ 15.6ms
    
    // Root Delay (0)
    packetBuffer[4] = 0;
    packetBuffer[5] = 0;
    packetBuffer[6] = 8;
    packetBuffer[7] = 0;
    
    // Root Dispersion (0.5秒)
    packetBuffer[8] = 0;
    packetBuffer[9] = 0;
    packetBuffer[10] = 0xC;
    packetBuffer[11] = 0;
    
    // Reference Identifier (GPS NTP)
    packetBuffer[12] = 'G';
    packetBuffer[13] = 'P';
    packetBuffer[14] = 'S';
    packetBuffer[15] = 'N';
    
    // Reference Timestamp
    uint32_t ref_seconds = (uint32_t)ntp_seconds;
    uint32_t ref_fraction = (uint32_t)ntp_fraction;
    
    packetBuffer[16] = (ref_seconds >> 24) & 0xFF;
    packetBuffer[17] = (ref_seconds >> 16) & 0xFF;
    packetBuffer[18] = (ref_seconds >> 8) & 0xFF;
    packetBuffer[19] = ref_seconds & 0xFF;
    
    packetBuffer[20] = (ref_fraction >> 24) & 0xFF;
    packetBuffer[21] = (ref_fraction >> 16) & 0xFF;
    packetBuffer[22] = (ref_fraction >> 8) & 0xFF;
    packetBuffer[23] = ref_fraction & 0xFF;
    
    // Origin Timestamp（复制客户端时间）
    for (int i = 0; i < 8; i++) {
      packetBuffer[24 + i] = clientTransmit[i];
    }
    
    // Receive Timestamp
    packetBuffer[32] = (ref_seconds >> 24) & 0xFF;
    packetBuffer[33] = (ref_seconds >> 16) & 0xFF;
    packetBuffer[34] = (ref_seconds >> 8) & 0xFF;
    packetBuffer[35] = ref_seconds & 0xFF;
    
    packetBuffer[36] = (ref_fraction >> 24) & 0xFF;
    packetBuffer[37] = (ref_fraction >> 16) & 0xFF;
    packetBuffer[38] = (ref_fraction >> 8) & 0xFF;
    packetBuffer[39] = ref_fraction & 0xFF;
    
    // Transmit Timestamp
    packetBuffer[40] = (ref_seconds >> 24) & 0xFF;
    packetBuffer[41] = (ref_seconds >> 16) & 0xFF;
    packetBuffer[42] = (ref_seconds >> 8) & 0xFF;
    packetBuffer[43] = ref_seconds & 0xFF;
    
    packetBuffer[44] = (ref_fraction >> 24) & 0xFF;
    packetBuffer[45] = (ref_fraction >> 16) & 0xFF;
    packetBuffer[46] = (ref_fraction >> 8) & 0xFF;
    packetBuffer[47] = ref_fraction & 0xFF;
    
    // 发送响应
    udp_.beginPacket(remote, remotePort);
    udp_.write(packetBuffer, 48);
    udp_.endPacket();
    
    // 记录日志
    if (ntp_requests_ % 10 == 0) {
      time_t unix_time = ref_seconds - seventyYears;
      struct tm *tm_info = gmtime(&unix_time);
      uint32_t microseconds = (uint64_t)ref_fraction * 1000000ULL / 4294967296ULL;
      
      ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d.%06u, 质量=%d",
               ntp_requests_, remote.toString().c_str(), remotePort,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               microseconds,
               quality);
    }
  }
}

// ==================== 主循环 ====================
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
    
    // 检查GPS数据是否过期
    if (gps_time_.valid && (now - gps_time_.last_update > 10000)) {
      gps_time_.valid = false;
      ESP_LOGW("gps_ntp", "GPS数据过期");
    }
    
    // 检查时间是否有效（PPS同步后5秒内）
    if (time_valid_ && (now - last_pps_sync_ > 5000)) {
      time_valid_ = false;
    }
    
    // 输出状态
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      struct tm *tm_info = gmtime(&tv.tv_sec);
      
      ESP_LOGI("gps_ntp", "状态: GPS=%s, PPS=%s, PPS计数=%u, 同步=%s, UTC=%02d:%02d:%02d.%06u",
               gps_time_.valid ? "有效" : "无效",
               pps_active_ ? "活跃" : "无效",
               pps_count_,
               time_valid_ ? "是" : "否",
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               tv.tv_usec);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器配置:");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  GPS有效: %s", gps_time_.valid ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  时间有效: %s", time_valid_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  时间质量: %d", get_time_quality());
  ESP_LOGCONFIG("gps_ntp", "  NTP请求: %u", ntp_requests_);
  
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) == 0) {
    struct tm *tm_info = gmtime(&tv.tv_sec);
    
    ESP_LOGCONFIG("gps_ntp", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d.%06u UTC",
                 tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                 tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
                 tv.tv_usec);
  }
  
  if (gps_time_.valid) {
    ESP_LOGCONFIG("gps_ntp", "  GPS时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 gps_time_.year, gps_time_.month, gps_time_.day,
                 gps_time_.hour, gps_time_.minute, gps_time_.second);
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome

