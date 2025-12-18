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
                      bool pps_active, uint32_t &ntp_requests) {
  // 记录接收时间（尽可能早）
  struct timeval receive_tv;
  gettimeofday(&receive_tv, NULL);
  
  // 转换Unix时间为NTP时间
  const unsigned long seventyYears = 2208988800UL;
  
  // Reference Timestamp（使用系统启动时间或固定时间）
  static time_t ref_time = 0;
  if (ref_time == 0) {
    ref_time = receive_tv.tv_sec - 3600; // 1小时前，表示稳定运行
  }
  uint64_t ref_ntp_seconds = (uint64_t)ref_time + seventyYears;
  uint64_t ref_ntp_fraction = 0;
  
  // Receive Timestamp
  uint64_t recv_ntp_seconds = (uint64_t)receive_tv.tv_sec + seventyYears;
  uint64_t recv_ntp_fraction = (uint64_t)receive_tv.tv_usec * 4294967296ULL / 1000000ULL;
  
  // 等待一点点时间，让Transmit Timestamp不同
  // 实际上NTP服务器处理请求需要时间
  delayMicroseconds(100);
  
  // Transmit Timestamp（当前时间）
  struct timeval transmit_tv;
  gettimeofday(&transmit_tv, NULL);
  uint64_t tx_ntp_seconds = (uint64_t)transmit_tv.tv_sec + seventyYears;
  uint64_t tx_ntp_fraction = 0;
  
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
      tx_ntp_seconds += since_pps / 1000000UL;
    }
    
    // 使用PPS校准的微秒部分
    tx_ntp_fraction = (uint64_t)microseconds * 4294967296ULL / 1000000ULL;
    
    // 同时校准Receive Timestamp的微秒部分
    uint32_t recv_microseconds = (receive_tv.tv_usec + since_pps) % 1000000UL;
    recv_ntp_fraction = (uint64_t)recv_microseconds * 4294967296ULL / 1000000ULL;
  } else {
    // 无PPS，使用系统时间的微秒
    tx_ntp_fraction = (uint64_t)transmit_tv.tv_usec * 4294967296ULL / 1000000ULL;
  }
  
  // 构建NTP响应包
  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  
  // NTP头部
  packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
  packetBuffer[1] = pps_active ? 2 : 3;  // stratum: PPS=2(secondary), 无PPS=3(tertiary)
  packetBuffer[2] = 4;     // Poll interval: 16秒（更合理）
  packetBuffer[3] = 0xFA;  // Precision: 2^-6 ≈ 15.6ms（更实际）
  
  // Root Delay (0.001秒)
  uint32_t root_delay = 1 << 16;  // 1 * 2^-16 = 0.001秒
  packetBuffer[4] = (root_delay >> 24) & 0xFF;
  packetBuffer[5] = (root_delay >> 16) & 0xFF;
  packetBuffer[6] = (root_delay >> 8) & 0xFF;
  packetBuffer[7] = root_delay & 0xFF;
  
  // Root Dispersion (0.01秒)
  uint32_t root_dispersion = 10 << 16;  // 10 * 2^-16 = 0.00015秒 ≈ 0.15ms
  packetBuffer[8] = (root_dispersion >> 24) & 0xFF;
  packetBuffer[9] = (root_dispersion >> 16) & 0xFF;
  packetBuffer[10] = (root_dispersion >> 8) & 0xFF;
  packetBuffer[11] = root_dispersion & 0xFF;
  
  // Reference Identifier (GPSP)
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = 'P';  // GPS with PPS
  
  // Reference Timestamp
  uint32_t ref_seconds = (uint32_t)ref_ntp_seconds;
  uint32_t ref_fraction = (uint32_t)ref_ntp_fraction;
  
  packetBuffer[16] = (ref_seconds >> 24) & 0xFF;
  packetBuffer[17] = (ref_seconds >> 16) & 0xFF;
  packetBuffer[18] = (ref_seconds >> 8) & 0xFF;
  packetBuffer[19] = ref_seconds & 0xFF;
  
  packetBuffer[20] = (ref_fraction >> 24) & 0xFF;
  packetBuffer[21] = (ref_fraction >> 16) & 0xFF;
  packetBuffer[22] = (ref_fraction >> 8) & 0xFF;
  packetBuffer[23] = ref_fraction & 0xFF;
  
  // Origin Timestamp（复制客户端时间）
  memcpy(&packetBuffer[24], clientTransmit, 8);
  
  // Receive Timestamp
  uint32_t recv_seconds = (uint32_t)recv_ntp_seconds;
  uint32_t recv_fraction = (uint32_t)recv_ntp_fraction;
  
  packetBuffer[32] = (recv_seconds >> 24) & 0xFF;
  packetBuffer[33] = (recv_seconds >> 16) & 0xFF;
  packetBuffer[34] = (recv_seconds >> 8) & 0xFF;
  packetBuffer[35] = recv_seconds & 0xFF;
  
  packetBuffer[36] = (recv_fraction >> 24) & 0xFF;
  packetBuffer[37] = (recv_fraction >> 16) & 0xFF;
  packetBuffer[38] = (recv_fraction >> 8) & 0xFF;
  packetBuffer[39] = recv_fraction & 0xFF;
  
  // Transmit Timestamp
  uint32_t tx_seconds = (uint32_t)tx_ntp_seconds;
  uint32_t tx_fraction = (uint32_t)tx_ntp_fraction;
  
  packetBuffer[40] = (tx_seconds >> 24) & 0xFF;
  packetBuffer[41] = (tx_seconds >> 16) & 0xFF;
  packetBuffer[42] = (tx_seconds >> 8) & 0xFF;
  packetBuffer[43] = tx_seconds & 0xFF;
  
  packetBuffer[44] = (tx_fraction >> 24) & 0xFF;
  packetBuffer[45] = (tx_fraction >> 16) & 0xFF;
  packetBuffer[46] = (tx_fraction >> 8) & 0xFF;
  packetBuffer[47] = tx_fraction & 0xFF;
  
  // 发送响应
  udp.beginPacket(remote, remotePort);
  udp.write(packetBuffer, 48);
  udp.endPacket();
  
  ntp_requests++;
  
  // 记录日志（每10个请求）
  if (ntp_requests % 10 == 0 || ntp_requests == 1) {
    time_t unix_time = tx_seconds - seventyYears;
    struct tm *tm_info = gmtime(&unix_time);
    
    ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d.%06u, 质量=%s",
             ntp_requests, remote.toString().c_str(), remotePort,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             (uint32_t)((uint64_t)tx_fraction * 1000000ULL / 4294967296ULL),
             pps_active ? "PPS" : "系统时间");
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
