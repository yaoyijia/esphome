#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <sys/time.h>
#include <math.h>  // 添加math.h头文件

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

// ==================== 发送NTP响应 ====================
void GPSNTPServer::send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                                    byte *clientTransmit) {
  // 获取当前系统时间（不进行PPS校准，因为校准已在驯服过程中完成）
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 转换Unix时间为NTP时间
  const unsigned long seventyYears = 2208988800UL;
  uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
  uint64_t ntp_fraction = (uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL;
  
  // 构建NTP响应包
  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  
  // NTP头部
  packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
  packetBuffer[1] = pps_active_ ? 2 : 4;  // stratum: PPS=2, 无PPS=4
  packetBuffer[2] = 6;     // Poll interval: 64秒
  packetBuffer[3] = 0xF6;  // Precision: 2^-10 ≈ 0.98ms
  
  // Root Delay (0.001秒)
  uint32_t root_delay = 1 << 16;
  packetBuffer[4] = (root_delay >> 24) & 0xFF;
  packetBuffer[5] = (root_delay >> 16) & 0xFF;
  packetBuffer[6] = (root_delay >> 8) & 0xFF;
  packetBuffer[7] = root_delay & 0xFF;
  
  // Root Dispersion (0.01秒)
  uint32_t root_dispersion = 10 << 16;
  packetBuffer[8] = (root_dispersion >> 24) & 0xFF;
  packetBuffer[9] = (root_dispersion >> 16) & 0xFF;
  packetBuffer[10] = (root_dispersion >> 8) & 0xFF;
  packetBuffer[11] = root_dispersion & 0xFF;
  
  // Reference Identifier
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = pps_active_ ? 'P' : 'N';
  
  // Reference Timestamp（使用系统启动时间）
  static uint64_t ref_ntp_seconds = 0;
  if (ref_ntp_seconds == 0) {
    struct timeval ref_tv;
    gettimeofday(&ref_tv, NULL);
    ref_ntp_seconds = (uint64_t)ref_tv.tv_sec + seventyYears;
  }
  uint32_t ref_seconds = (uint32_t)ref_ntp_seconds;
  uint32_t ref_fraction = 0;
  
  // 设置Reference Timestamp
  for (int i = 0; i < 4; i++) {
    packetBuffer[16 + i] = (ref_seconds >> (24 - i*8)) & 0xFF;
    packetBuffer[20 + i] = (ref_fraction >> (24 - i*8)) & 0xFF;
  }
  
  // Origin Timestamp（复制客户端时间）
  memcpy(&packetBuffer[24], clientTransmit, 8);
  
  // Receive Timestamp（接收时间）
  // 注意：这里我们使用相同的当前时间，因为处理非常快
  uint32_t recv_seconds = (uint32_t)ntp_seconds;
  uint32_t recv_fraction = (uint32_t)ntp_fraction;
  
  for (int i = 0; i < 4; i++) {
    packetBuffer[32 + i] = (recv_seconds >> (24 - i*8)) & 0xFF;
    packetBuffer[36 + i] = (recv_fraction >> (24 - i*8)) & 0xFF;
  }
  
  // Transmit Timestamp（发送时间，与Receive时间相同）
  for (int i = 0; i < 4; i++) {
    packetBuffer[40 + i] = (recv_seconds >> (24 - i*8)) & 0xFF;
    packetBuffer[44 + i] = (recv_fraction >> (24 - i*8)) & 0xFF;
  }
  
  // 发送响应
  udp.beginPacket(remote, remotePort);
  udp.write(packetBuffer, 48);
  udp.endPacket();
  
  ntp_requests_++;
  
  // 记录日志（每20个请求）
  if (ntp_requests_ % 20 == 0 || ntp_requests_ == 1) {
    time_t unix_time = recv_seconds - seventyYears;
    struct tm *tm_info = gmtime(&unix_time);
    
    ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d.%06u",
             ntp_requests_, remote.toString().c_str(), remotePort,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             (uint32_t)((uint64_t)recv_fraction * 1000000ULL / 4294967296ULL));
  }
}

// ==================== 时间驯服函数 ====================
void GPSNTPServer::discipline_time() {
  if (!pps_active_) return;
  
  uint32_t now = millis();
  
  // 每秒驯服一次
  if (now - time_discipline_.last_discipline < 1000) return;
  
  // 获取当前系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 计算PPS与系统时间的误差
  // 理想情况：PPS发生时，系统时间的毫秒部分应为0
  uint32_t current_us = micros();
  uint32_t pps_edge_us = pps_last_edge_us_;
  
  // 计算自上次PPS以来的微秒数
  uint32_t since_pps;
  if (current_us >= pps_edge_us) {
    since_pps = current_us - pps_edge_us;
  } else {
    since_pps = (0xFFFFFFFFUL - pps_edge_us) + current_us + 1;
  }
  
  // 计算误差（毫秒）
  float error_ms = (since_pps % 1000000) / 1000.0f;
  
  // 如果超过500ms，说明我们可能在下半秒，调整误差
  if (error_ms > 500.0f) {
    error_ms = error_ms - 1000.0f;
  }
  
  // 更新误差统计
  time_discipline_.last_error = time_discipline_.error_ms;
  time_discipline_.error_ms = error_ms;
  time_discipline_.accumulated_error += error_ms;
  
  // 简单的PI控制算法
  float adjustment = 0.0f;
  
  // 比例项
  float Kp = 0.5f;  // 比例系数
  adjustment += Kp * error_ms;
  
  // 积分项（抗积分饱和）
  float Ki = 0.01f;  // 积分系数
  if (fabs(time_discipline_.accumulated_error) < 1000.0f) {  // 限制积分项
    adjustment += Ki * time_discipline_.accumulated_error;
  }
  
  // 限制调整幅度（最大10ms）
  if (adjustment > 10.0f) adjustment = 10.0f;
  if (adjustment < -10.0f) adjustment = -10.0f;
  
  // 如果误差大于5ms，进行驯服
  if (fabs(error_ms) > 5.0f) {
    time_discipline_.disciplining = true;
    
    // 调整系统时间（微秒级调整）
    int32_t adjust_us = (int32_t)(adjustment * 1000.0f);  // 转换为微秒
    
    struct timeval new_tv;
    gettimeofday(&new_tv, NULL);
    
    // 应用调整
    new_tv.tv_usec += adjust_us;
    
    // 处理进位
    if (new_tv.tv_usec >= 1000000) {
      new_tv.tv_sec += new_tv.tv_usec / 1000000;
      new_tv.tv_usec %= 1000000;
    } else if (new_tv.tv_usec < 0) {
      new_tv.tv_sec -= (-new_tv.tv_usec / 1000000) + 1;
      new_tv.tv_usec = 1000000 + (new_tv.tv_usec % 1000000);
    }
    
    // 设置新时间
    if (settimeofday(&new_tv, NULL) == 0) {
      ESP_LOGD("gps_ntp", "时间驯服: 误差=%.2fms, 调整=%.2fms", 
               error_ms, adjustment);
    }
  } else {
    time_discipline_.disciplining = false;
  }
  
  time_discipline_.last_discipline = now;
}

// ==================== 处理NTP请求 ====================
void GPSNTPServer::handle_ntp_request() {
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    byte packetBuffer[48];
    udp_.read(packetBuffer, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    // 保存客户端的Transmit Timestamp
    byte clientTransmit[8];
    memcpy(clientTransmit, &packetBuffer[40], 8);
    
    // 立即发送响应
    send_ntp_response(udp_, remote, remotePort, clientTransmit);
  }
}

// ==================== 处理PPS ====================
void GPSNTPServer::handle_pps() {
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_active_ = true;
    pps_last_stable_ = millis();
    
    // 记录PPS间隔（用于检测PPS质量）
    static uint32_t last_pps_count = 0;
    if (pps_count_ > 0) {
      uint32_t interval = millis() - pps_last_stable_;
      
      // 检查间隔是否在合理范围内（900-1100ms）
      if (interval > 900 && interval < 1100) {
        if (pps_count_ % 60 == 0) {
          ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %ums", pps_count_, interval);
        }
      } else {
        ESP_LOGW("gps_ntp", "PPS间隔异常: %ums", interval);
      }
    }
    last_pps_count = pps_count_;
  }
  
  // 检查PPS是否丢失（2秒无更新）
  if (pps_active_ && (millis() - pps_last_stable_ > 2000)) {
    pps_active_ = false;
    ESP_LOGW("gps_ntp", "PPS信号丢失");
  }
}

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器 (驯服版)");
  
  instance_ = this;
  
  // 设置PPS引脚中断
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   pps_interrupt_handler, 
                   RISING);
    
    ESP_LOGI("gps_ntp", "PPS引脚: GPIO%d (上升沿触发)", pps_pin_);
  } else {
    ESP_LOGI("gps_ntp", "未配置PPS引脚，将使用纯系统时间");
  }
  
  // 启动NTP服务器
  udp_.begin(123);
  ESP_LOGI("gps_ntp", "NTP服务器已启动，端口123");
}

// ==================== 主循环 ====================
void GPSNTPServer::loop() {
  uint32_t now = millis();
  
  // 限制循环频率
  if (now - last_loop_ < 10) return;
  last_loop_ = now;
  
  // 处理PPS
  handle_pps();
  
  // 时间驯服
  discipline_time();
  
  // 处理NTP请求
  handle_ntp_request();
  
  // 定期状态更新
  static uint32_t last_status = 0;
  if (now - last_status > 30000) {
    last_status = now;
    
    // 获取系统时间用于状态显示
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      struct tm *tm_info = gmtime(&tv.tv_sec);
      
      ESP_LOGI("gps_ntp", "状态: PPS=%s, PPS计数=%u, 驯服=%s, 误差=%.2fms, NTP请求=%u, 时间=%02d:%02d:%02d",
               pps_active_ ? "活跃" : "无效",
               pps_count_,
               time_discipline_.disciplining ? "进行中" : "稳定",
               time_discipline_.error_ms,
               ntp_requests_,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器 (驯服版):");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  时间误差: %.2fms", time_discipline_.error_ms);
  ESP_LOGCONFIG("gps_ntp", "  驯服状态: %s", time_discipline_.disciplining ? "进行中" : "稳定");
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
