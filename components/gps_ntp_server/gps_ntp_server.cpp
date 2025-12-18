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
    uint32_t current_us = micros();
    
    // 简单的防抖：忽略200ms内的重复中断
    static uint32_t last_interrupt_us = 0;
    if (last_interrupt_us > 0) {
      uint32_t elapsed = (current_us >= last_interrupt_us) ? 
                         (current_us - last_interrupt_us) :
                         (0xFFFFFFFFUL - last_interrupt_us + current_us + 1);
      if (elapsed < 200000) return;  // 200ms内忽略
    }
    last_interrupt_us = current_us;
    
    GPSNTPServer::instance_->pps_last_edge_us_ = current_us;
    GPSNTPServer::instance_->pps_count_++;
    GPSNTPServer::instance_->pps_triggered_ = true;
    
    // 标记PPS活跃
    GPSNTPServer::instance_->pps_active_ = true;
    GPSNTPServer::instance_->pps_last_stable_ = millis();
  }
}

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器 (简化版)");
  
  instance_ = this;
  
  // 设置PPS引脚中断
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    
    // 尝试不同的触发方式
    int trigger_mode = RISING;  // 先用上升沿
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   pps_interrupt_handler, 
                   trigger_mode);
    
    ESP_LOGI("gps_ntp", "PPS引脚配置在GPIO %d (%s触发)", 
             pps_pin_, 
             trigger_mode == RISING ? "上升沿" : 
             trigger_mode == FALLING ? "下降沿" : "变化沿");
  } else {
    ESP_LOGW("gps_ntp", "未配置PPS引脚，将使用系统时间");
  }
  
  // 启动NTP服务器
  udp_.begin(123);
  ntp_started_ = true;
  ESP_LOGI("gps_ntp", "NTP服务器已启动，端口123");
}

// ==================== GPS更新回调 ====================
void GPSNTPServer::on_update(TinyGPSPlus &tiny_gps) {
  // 仅用于状态显示，不用于时间同步
  // 可以在这里记录GPS状态，但不设置系统时间
}

// ==================== PPS处理 ====================
void GPSNTPServer::handle_pps() {
  if (pps_triggered_) {
    pps_triggered_ = false;
    
    uint32_t now_us = micros();
    uint32_t pps_edge_us = pps_last_edge_us_;
    
    // 计算间隔
    uint32_t interval_us;
    if (now_us >= pps_edge_us) {
      interval_us = now_us - pps_edge_us;
    } else {
      interval_us = (0xFFFFFFFFUL - pps_edge_us) + now_us + 1;
    }
    
    // 记录PPS间隔
    static uint32_t last_interval = 0;
    if (pps_count_ > 1) {
      if (interval_us > 700000 && interval_us < 1300000) {
        // 间隔正常（0.7-1.3秒）
        if (pps_count_ % 60 == 0) {
          ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %.3fms", 
                   pps_count_, interval_us / 1000.0f);
        }
      } else {
        ESP_LOGW("gps_ntp", "PPS间隔异常: %.3fms", interval_us / 1000.0f);
      }
    }
    last_interval = interval_us;
  }
  
  // 检查PPS是否丢失（3秒无更新）
  if (pps_active_ && (millis() - pps_last_stable_ > 3000)) {
    pps_active_ = false;
    ESP_LOGW("gps_ntp", "PPS信号丢失");
  }
}

// ==================== 时间质量评估 ====================
uint8_t GPSNTPServer::get_time_quality() const {
  if (pps_active_) return QUALITY_PPS;
  return QUALITY_SYSTEM;
}

// ==================== 发送NTP响应 ====================
void GPSNTPServer::send_ntp_response(WiFiUDP &udp, IPAddress remote, 
                                     int remotePort, byte *clientTransmit) {
  // 获取当前系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 检查系统时间是否有效（不是1970年）
  if (tv.tv_sec < 1609459200L) {  // 2021年之前认为无效
    ESP_LOGW("gps_ntp", "系统时间无效 (%lu秒)，使用PPS计数作为时间源", tv.tv_sec);
    // 如果系统时间无效，使用启动时间+PPS计数
    static uint32_t start_time = millis();
    uint32_t uptime_seconds = (millis() - start_time) / 1000;
    tv.tv_sec = 1700000000L + uptime_seconds;  // 使用一个基准时间
    tv.tv_usec = 0;
  }
  
  // 计算NTP时间
  const unsigned long seventyYears = 2208988800UL;
  uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
  uint64_t ntp_fraction = 0;
  
  // 使用PPS校准微秒部分
  if (pps_active_ && pps_count_ > 0) {
    uint32_t now_us = micros();
    uint32_t pps_edge_us = pps_last_edge_us_;
    
    // 计算自上次PPS以来的微秒数
    uint32_t since_pps;
    if (now_us >= pps_edge_us) {
      since_pps = now_us - pps_edge_us;
    } else {
      since_pps = (0xFFFFFFFFUL - pps_edge_us) + now_us + 1;
    }
    
    // 确保在合理范围内（< 1.1秒）
    if (since_pps < 1100000) {
      // 微秒部分
      uint32_t microseconds = since_pps % 1000000UL;
      
      // 如果超过1秒，调整秒数
      if (since_pps >= 1000000UL) {
        ntp_seconds += since_pps / 1000000UL;
      }
      
      // 使用PPS校准的微秒部分
      ntp_fraction = (uint64_t)microseconds * 4294967296ULL / 1000000ULL;
      
      // 调试日志
      if (ntp_requests_ % 10 == 0) {
        ESP_LOGD("gps_ntp", "NTP响应: PPS校准，距上次PPS=%uus", since_pps);
      }
    } else {
      // PPS间隔异常，使用系统时间的微秒
      ntp_fraction = (uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL;
    }
  } else {
    // 无PPS，使用系统时间的微秒
    ntp_fraction = (uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL;
  }
  
  // 根据时间质量设置stratum
  uint8_t quality = get_time_quality();
  uint8_t stratum;
  
  switch (quality) {
    case QUALITY_PPS:
      stratum = 2;   // 二级参考源
      break;
    default:
      stratum = 4;   // 普通服务器
  }
  
  // 构建NTP响应包
  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  
  // NTP头部
  packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
  packetBuffer[1] = stratum;
  packetBuffer[2] = 6;     // Poll interval: 64秒
  packetBuffer[3] = 0xEC;  // Precision: 2^-20 ≈ 1微秒
  
  // Root Delay (0)
  memset(&packetBuffer[4], 0, 4);
  
  // Root Dispersion
  uint32_t root_dispersion = (pps_active_ ? 100 : 1000) << 16;  // 微秒
  packetBuffer[8] = (root_dispersion >> 24) & 0xFF;
  packetBuffer[9] = (root_dispersion >> 16) & 0xFF;
  packetBuffer[10] = (root_dispersion >> 8) & 0xFF;
  packetBuffer[11] = root_dispersion & 0xFF;
  
  // Reference Identifier
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = pps_active_ ? 'P' : 'S';  // PPS或系统时间
  
  // Reference Timestamp
  uint32_t ref_seconds = (uint32_t)ntp_seconds;
  uint32_t ref_fraction = (uint32_t)ntp_fraction;
  
  for (int i = 0; i < 4; i++) {
    packetBuffer[16 + i] = (ref_seconds >> (24 - i*8)) & 0xFF;
    packetBuffer[20 + i] = (ref_fraction >> (24 - i*8)) & 0xFF;
  }
  
  // Origin Timestamp（复制客户端时间）
  memcpy(&packetBuffer[24], clientTransmit, 8);
  
  // Receive Timestamp
  memcpy(&packetBuffer[32], &packetBuffer[16], 8);
  
  // Transmit Timestamp
  memcpy(&packetBuffer[40], &packetBuffer[16], 8);
  
  // 发送响应
  udp.beginPacket(remote, remotePort);
  udp.write(packetBuffer, 48);
  udp.endPacket();
  
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
    memcpy(clientTransmit, &packetBuffer[40], 8);
    
    // 立即发送响应（不等待PPS）
    // 对于精确应用，可以等待下一个PPS，但这会增加延迟
    // 这里我们立即响应，时间已经用最近的PPS校准过
    send_ntp_response(udp_, remote, remotePort, clientTransmit);
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
    
    // 获取系统时间用于状态显示
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      struct tm *tm_info = gmtime(&tv.tv_sec);
      
      ESP_LOGI("gps_ntp", "状态: PPS=%s, PPS计数=%u, 质量=%d, 系统时间=%02d:%02d:%02d, NTP请求=%u",
               pps_active_ ? "活跃" : "无效",
               pps_count_,
               get_time_quality(),
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               ntp_requests_);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器配置 (简化版):");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  时间质量: %d", get_time_quality());
  ESP_LOGCONFIG("gps_ntp", "  NTP请求: %u", ntp_requests_);
  
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) == 0) {
    struct tm *tm_info = gmtime(&tv.tv_sec);
    
    ESP_LOGCONFIG("gps_ntp", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d.%06u",
                 tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                 tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
                 tv.tv_usec);
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome
