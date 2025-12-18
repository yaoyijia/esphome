#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <sys/time.h>
#include "esphome/core/hal.h"

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// ==================== PPS中断处理 ====================
void IRAM_ATTR GPSNTPServer::pps_interrupt_handler() {
  if (GPSNTPServer::instance_) {
    GPSNTPServer::instance_->pps_last_edge_us_ = micros();  // ESP8266兼容版本
    GPSNTPServer::instance_->pps_count_++;
    GPSNTPServer::instance_->pps_triggered_ = true;
  }
}

// ==================== 设置GPS组件 ====================
void GPSNTPServer::set_gps(gps::GPS *gps) { 
  gps_ = gps;
  if (gps_) {
    gps_->register_listener(this);
    ESP_LOGI("gps_ntp", "GPS组件已注册");
  }
}

// ==================== 获取高精度时间（跨平台） ====================
uint64_t GPSNTPServer::get_precise_time_us() {
#ifdef USE_ESP32
  // ESP32可以使用高精度定时器
  return esp_timer_get_time();  // 注意：需要包含正确的头文件
#else
  // ESP8266使用micros()，但需要处理溢出
  return micros();  // 返回32位值，会溢出，但用于计算间隔是可以的
#endif
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
  } else {
    ESP_LOGW("gps_ntp", "未配置PPS引脚，时间精度将受限");
  }
  
  // 启动NTP服务器
  udp_.begin(123);
  ntp_started_ = true;
  ESP_LOGI("gps_ntp", "NTP服务器已启动，端口123");
}

// ==================== GPS更新回调 ====================
void GPSNTPServer::on_update(TinyGPSPlus &tiny_gps) {
  // 检查GPS数据是否有效
  if (!tiny_gps.time.isValid() || !tiny_gps.date.isValid() || 
      tiny_gps.date.year() < 2024) {
    return;
  }
  
  // 缓存GPS时间，但不立即设置系统时间
  gps_time_cache_.year = tiny_gps.date.year();
  gps_time_cache_.month = tiny_gps.date.month();
  gps_time_cache_.day = tiny_gps.date.day();
  gps_time_cache_.hour = tiny_gps.time.hour();
  gps_time_cache_.minute = tiny_gps.time.minute();
  gps_time_cache_.second = tiny_gps.time.second();
  
  // 构建tm结构
  struct tm timeinfo = {0};
  timeinfo.tm_year = gps_time_cache_.year - 1900;
  timeinfo.tm_mon = gps_time_cache_.month - 1;
  timeinfo.tm_mday = gps_time_cache_.day;
  timeinfo.tm_hour = gps_time_cache_.hour;
  timeinfo.tm_min = gps_time_cache_.minute;
  timeinfo.tm_sec = gps_time_cache_.second;
  
  // 转换为Unix时间
  time_t epoch = mktime(&timeinfo);
  
  // 检查时间是否合理
  if (epoch > 1609459200L) {
    gps_time_cache_.epoch = epoch;
    gps_time_cache_.valid = true;
    
    // 标记需要PPS同步
    if (pps_pin_ > 0) {
      pps_sync_.awaiting_sync = true;
      ESP_LOGD("gps_ntp", "GPS时间有效，等待PPS同步: %04d-%02d-%02d %02d:%02d:%02d",
               gps_time_cache_.year, gps_time_cache_.month, gps_time_cache_.day,
               gps_time_cache_.hour, gps_time_cache_.minute, gps_time_cache_.second);
    } else {
      // 没有PPS引脚，直接设置系统时间
      struct timeval tv = {epoch, 0};
      if (settimeofday(&tv, nullptr) == 0) {
        gps_valid_ = true;
        last_gps_update_ = millis();
        ESP_LOGI("gps_ntp", "GPS时间已设置（无PPS）: %04d-%02d-%02d %02d:%02d:%02d",
                 gps_time_cache_.year, gps_time_cache_.month, gps_time_cache_.day,
                 gps_time_cache_.hour, gps_time_cache_.minute, gps_time_cache_.second);
      }
    }
  }
}

// ==================== 用GPS和PPS同步系统时间 ====================
void GPSNTPServer::sync_system_time_with_gps_and_pps() {
  if (!gps_time_cache_.valid) {
    return;
  }
  
  // 重要：PPS标记的是秒的开始，所以使用GPS时间的整秒部分
  // 假设PPS在GPS时间的秒边界到达
  time_t epoch_for_pps = gps_time_cache_.epoch;  // 使用GPS的整秒时间
  
  struct timeval tv;
  tv.tv_sec = epoch_for_pps;  // 整秒
  tv.tv_usec = 0;              // 微秒归零，由PPS精确对齐
  
  if (settimeofday(&tv, nullptr) == 0) {
    gps_valid_ = true;
    pps_sync_.synced_once = true;
    pps_sync_.last_sync_millis = millis();
    pps_sync_.awaiting_sync = false;
    last_gps_update_ = millis();
    
    // 记录精确的同步时间
    uint32_t pps_us = pps_last_edge_us_ % 1000000;
    ESP_LOGI("gps_ntp", "GPS+PPS时间同步完成: %04d-%02d-%02d %02d:%02d:%02d.%06u UTC",
             gps_time_cache_.year, gps_time_cache_.month, gps_time_cache_.day,
             gps_time_cache_.hour, gps_time_cache_.minute, gps_time_cache_.second,
             pps_us);
  } else {
    ESP_LOGE("gps_ntp", "GPS+PPS时间同步失败");
  }
}

// ==================== PPS处理 ====================
void GPSNTPServer::handle_pps() {
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_active_ = true;
    pps_last_stable_ = millis();
    
    uint32_t now_us = micros();
    uint32_t pps_edge_us = pps_last_edge_us_;
    
    // 正确处理32位溢出
    uint32_t interval_us;
    if (now_us >= pps_edge_us) {
      interval_us = now_us - pps_edge_us;
    } else {
      // 处理微秒计数器溢出（大约每71分钟发生一次）
      interval_us = (0xFFFFFFFFUL - pps_edge_us) + now_us + 1;
    }
    
    // 检查PPS间隔是否稳定（900ms-1100ms）
    if (interval_us > 900000 && interval_us < 1100000) {
      // 每分钟输出一次详细日志
      if (pps_count_ % 60 == 0) {
        ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %.3fms", 
                 pps_count_, interval_us / 1000.0f);
      }
      
      // 如果有等待同步的GPS时间，现在同步
      if (pps_sync_.awaiting_sync) {
        sync_system_time_with_gps_and_pps();
      }
    } else {
      ESP_LOGW("gps_ntp", "PPS间隔异常: %.3fms", interval_us / 1000.0f);
    }
  }
  
  // 检查PPS是否丢失（2秒无更新）
  if (pps_active_ && (millis() - pps_last_stable_ > 2000)) {
    pps_active_ = false;
  }
}

// ==================== 时间质量评估 ====================
uint8_t GPSNTPServer::get_time_quality() const {
  if (gps_valid_ && pps_active_ && pps_sync_.synced_once) return QUALITY_GPS_PPS;
  if (pps_active_ && pps_sync_.synced_once) return QUALITY_PPS;
  if (gps_valid_) return QUALITY_GPS;
  return QUALITY_SYSTEM;
}

// ==================== NTP请求处理（完整微秒精度） ====================
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
    memcpy(clientTransmit, &packetBuffer[40], 8);
    
    // 获取当前系统时间
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    // Unix时间转换为NTP时间
    const unsigned long seventyYears = 2208988800UL;
    uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
    uint64_t ntp_fraction = (uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL;
    
    // 如果PPS有效，使用PPS校准微秒部分
    if (pps_active_ && pps_count_ > 0 && pps_sync_.synced_once) {
      uint32_t now_us = micros();
      uint32_t pps_edge_us = pps_last_edge_us_;
      
      // 计算自上次PPS以来的微秒数（处理溢出）
      uint32_t since_pps;
      if (now_us >= pps_edge_us) {
        since_pps = now_us - pps_edge_us;
      } else {
        since_pps = (0xFFFFFFFFUL - pps_edge_us) + now_us + 1;
      }
      
      // 确保在合理范围内（< 1.1秒）
      if (since_pps < 1100000) {
        // 计算微秒部分（取余，确保在0-999999微秒内）
        uint32_t microseconds = since_pps % 1000000UL;
        
        // 如果超过1秒，调整秒数
        if (since_pps >= 1000000UL) {
          ntp_seconds += since_pps / 1000000UL;
        }
        
        // 使用PPS校准的微秒部分
        ntp_fraction = (uint64_t)microseconds * 4294967296ULL / 1000000ULL;
        
        // 调试日志（每20个请求输出一次）
        if (ntp_requests_ % 20 == 0) {
          ESP_LOGD("gps_ntp", "PPS校准: since_pps=%uus, micros=%uus", 
                  since_pps, microseconds);
        }
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
          stratum = 1;   // 一级参考源
          break;
        case QUALITY_PPS:
        case QUALITY_GPS:
          stratum = 2;   // 二级参考源
          break;
        default:
          stratum = 4;   // 普通服务器
      }
    }
    
    // 构建NTP响应包
    memset(packetBuffer, 0, 48);
    
    // NTP头部
    packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
    packetBuffer[1] = stratum;
    packetBuffer[2] = 6;     // Poll interval: 64秒
    packetBuffer[3] = 0xEC;  // Precision: 2^-20 ≈ 1微秒
    
    // Root Delay (0)
    memset(&packetBuffer[4], 0, 4);
    
    // Root Dispersion (小值表示高精度)
    uint32_t root_dispersion = 100 << 16;  // 100微秒
    packetBuffer[8] = (root_dispersion >> 24) & 0xFF;
    packetBuffer[9] = (root_dispersion >> 16) & 0xFF;
    packetBuffer[10] = (root_dispersion >> 8) & 0xFF;
    packetBuffer[11] = root_dispersion & 0xFF;
    
    // Reference Identifier (GPS NTP)
    packetBuffer[12] = 'G';
    packetBuffer[13] = 'P';
    packetBuffer[14] = 'S';
    packetBuffer[15] = 'P';  // 表示GPS+PPS
    
    // Reference Timestamp（使用当前时间）
    uint32_t ref_seconds = (uint32_t)ntp_seconds;
    uint32_t ref_fraction = (uint32_t)ntp_fraction;
    
    for (int i = 0; i < 4; i++) {
      packetBuffer[16 + i] = (ref_seconds >> (24 - i*8)) & 0xFF;
      packetBuffer[20 + i] = (ref_fraction >> (24 - i*8)) & 0xFF;
    }
    
    // Origin Timestamp（复制客户端时间）
    memcpy(&packetBuffer[24], clientTransmit, 8);
    
    // Receive Timestamp（当前时间）
    memcpy(&packetBuffer[32], &packetBuffer[16], 8);
    
    // Transmit Timestamp（当前时间）
    memcpy(&packetBuffer[40], &packetBuffer[16], 8);
    
    // 发送响应
    udp_.beginPacket(remote, remotePort);
    udp_.write(packetBuffer, 48);
    udp_.endPacket();
    
    // 记录日志
    if (ntp_requests_ % 20 == 0 || ntp_requests_ == 1) {
      time_t unix_time = ref_seconds - seventyYears;
      struct tm *tm_info = gmtime(&unix_time);
      
      ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d.%06u, 质量=%d",
               ntp_requests_, remote.toString().c_str(), remotePort,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               (uint32_t)((uint64_t)ref_fraction * 1000000ULL / 4294967296ULL),
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
    
    // 检查GPS超时（10秒）
    if (gps_valid_ && (now - last_gps_update_ > 10000)) {
      gps_valid_ = false;
      ESP_LOGW("gps_ntp", "GPS信号丢失");
    }
    
    // 输出状态
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      struct tm *tm_info = gmtime(&tv.tv_sec);
      
      ESP_LOGI("gps_ntp", "状态: GPS=%s, PPS=%s, 已同步=%s, PPS计数=%u, 质量=%d, UTC=%02d:%02d:%02d.%06u, NTP请求=%u",
               gps_valid_ ? "有效" : "无效",
               pps_active_ ? "活跃" : "无效",
               pps_sync_.synced_once ? "是" : "否",
               pps_count_,
               get_time_quality(),
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               tv.tv_usec,
               ntp_requests_);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器配置:");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  GPS有效: %s", gps_valid_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS已同步: %s", pps_sync_.synced_once ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
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
}

}  // namespace gps_ntp_server
}  // namespace esphome
