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
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器 (极简版)");
  
  instance_ = this;
  
  // 设置PPS引脚中断
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   pps_interrupt_handler, 
                   RISING);  // 使用上升沿触发
    
    ESP_LOGI("gps_ntp", "PPS引脚: GPIO%d", pps_pin_);
  } else {
    ESP_LOGI("gps_ntp", "未配置PPS引脚，将使用系统时间");
  }
  
  // 启动NTP服务器
  udp_.begin(123);
  ESP_LOGI("gps_ntp", "NTP服务器已启动，端口123");
}

// ==================== 发送NTP响应 ====================
void send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                      byte *clientTransmit, uint32_t pps_last_edge_us, 
                      bool pps_active) {
  // 获取当前系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 转换Unix时间为NTP时间
  const unsigned long seventyYears = 2208988800UL;
  uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
  uint64_t ntp_fraction = 0;
  
  // 如果有PPS，使用PPS校准微秒部分
  if (pps_active) {
    uint32_t now_us = micros();
    uint32_t pps_edge_us = pps_last_edge_us;
    
    // 计算自上次PPS以来的微秒数（处理溢出）
    uint32_t since_pps;
    if (now_us >= pps_edge_us) {
      since_pps = now_us - pps_edge_us;
    } else {
      since_pps = (0xFFFFFFFFUL - pps_edge_us) + now_us + 1;
    }
    
    // 微秒部分（取模1秒）
    uint32_t microseconds = since_pps % 1000000UL;
    
    // 如果超过1秒，调整秒数
    if (since_pps >= 1000000UL) {
      ntp_seconds += since_pps / 1000000UL;
    }
    
    // 使用PPS校准的微秒部分
    ntp_fraction = (uint64_t)microseconds * 4294967296ULL / 1000000ULL;
  } else {
    // 无PPS，使用系统时间的微秒
    ntp_fraction = (uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL;
  }
  
  // 构建NTP响应包
  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  
  // NTP头部
  packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
  packetBuffer[1] = pps_active ? 2 : 4;  // stratum: PPS=2, 系统时间=4
  packetBuffer[2] = 6;     // Poll interval: 64秒
  packetBuffer[3] = 0xEC;  // Precision: 2^-20 ≈ 1微秒
  
  // Root Delay (0)
  memset(&packetBuffer[4], 0, 4);
  
  // Root Dispersion
  uint32_t root_dispersion = pps_active ? (100 << 16) : (1000 << 16);  // 微秒
  packetBuffer[8] = (root_dispersion >> 24) & 0xFF;
  packetBuffer[9] = (root_dispersion >> 16) & 0xFF;
  packetBuffer[10] = (root_dispersion >> 8) & 0xFF;
  packetBuffer[11] = root_dispersion & 0xFF;
  
  // Reference Identifier
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = pps_active ? 'P' : 'S';  // PPS或系统时间
  
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
}

// ==================== 处理NTP请求 ====================
void handle_ntp_request(WiFiUDP &udp, uint32_t &ntp_requests, 
                       uint32_t pps_last_edge_us, bool pps_active) {
  int packetSize = udp.parsePacket();
  if (packetSize >= 48) {
    byte packetBuffer[48];
    udp.read(packetBuffer, 48);
    IPAddress remote = udp.remoteIP();
    int remotePort = udp.remotePort();
    
    ntp_requests++;
    
    // 保存客户端的Transmit Timestamp
    byte clientTransmit[8];
    memcpy(clientTransmit, &packetBuffer[40], 8);
    
    // 立即发送响应
    send_ntp_response(udp, remote, remotePort, clientTransmit, 
                     pps_last_edge_us, pps_active);
    
    // 每20个请求记录一次日志
    if (ntp_requests % 20 == 0 || ntp_requests == 1) {
      // 获取当前时间用于日志
      struct timeval tv;
      gettimeofday(&tv, NULL);
      time_t unix_time = tv.tv_sec;
      struct tm *tm_info = gmtime(&unix_time);
      
      ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d",
               ntp_requests, remote.toString().c_str(), remotePort,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    }
  }
}

// ==================== 处理PPS ====================
void handle_pps(bool &pps_active, volatile bool &pps_triggered, 
               uint32_t &pps_last_stable) {
  if (pps_triggered) {
    pps_triggered = false;
    pps_active = true;
    pps_last_stable = millis();
  }
  
  // 检查PPS是否丢失（3秒无更新）
  if (pps_active && (millis() - pps_last_stable > 3000)) {
    pps_active = false;
  }
}

// ==================== 主循环 ====================
void GPSNTPServer::loop() {
  uint32_t now = millis();
  
  // 限制循环频率
  if (now - last_loop_ < 10) return;
  last_loop_ = now;
  
  // 处理PPS
  handle_pps(pps_active_, pps_triggered_, pps_last_stable_);
  
  // 处理NTP请求
  handle_ntp_request(udp_, ntp_requests_, pps_last_edge_us_, pps_active_);
  
  // 定期状态更新
  static uint32_t last_status = 0;
  if (now - last_status > 30000) {
    last_status = now;
    
    // 获取系统时间用于状态显示
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      struct tm *tm_info = gmtime(&tv.tv_sec);
      
      ESP_LOGI("gps_ntp", "状态: PPS=%s, PPS计数=%u, NTP请求=%u, 系统时间=%02d:%02d:%02d",
               pps_active_ ? "活跃" : "无效",
               pps_count_,
               ntp_requests_,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器 (极简版):");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  NTP请求: %u", ntp_requests_);
  
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) == 0) {
    struct tm *tm_info = gmtime(&tv.tv_sec);
    
    ESP_LOGCONFIG("gps_ntp", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d",
                 tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                 tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome
