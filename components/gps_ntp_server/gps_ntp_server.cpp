#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <sys/time.h>
#include <math.h>

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// ==================== PPS中断处理 ====================
void IRAM_ATTR GPSNTPServer::pps_interrupt_handler() {
  if (GPSNTPServer::instance_) {
    static uint32_t last_interrupt_time = 0;
    uint32_t now = micros();
    
    // 防抖：忽略100ms内的重复中断
    if (last_interrupt_time > 0) {
      uint32_t elapsed;
      if (now >= last_interrupt_time) {
        elapsed = now - last_interrupt_time;
      } else {
        elapsed = (0xFFFFFFFFUL - last_interrupt_time) + now + 1;
      }
      if (elapsed < 100000) return;  // 100ms内忽略
    }
    last_interrupt_time = now;
    
    GPSNTPServer::instance_->pps_last_edge_us_ = now;
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

// ==================== 时间驯服函数（简化版） ====================
void GPSNTPServer::discipline_time() {
  // 获取当前系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 计算误差（微秒部分，转换为毫秒）
  float error_ms = tv.tv_usec / 1000.0f;
  
  // 如果误差大于500ms，转换为负数
  if (error_ms > 500.0f) {
    error_ms = error_ms - 1000.0f;
  }
  
  // 更新误差统计
  time_discipline_.last_error = time_discipline_.error_ms;
  time_discipline_.error_ms = error_ms;
  
  // 根据误差大小决定调整量（反向调整）
  float adjustment = 0.0f;
  
  // 简单的反向调整逻辑
  if (fabs(error_ms) > 400.0f) {
    adjustment = -20.0f;  // 大误差，中等调整
  } else if (fabs(error_ms) > 200.0f) {
    adjustment = -10.0f;  // 中等误差，较小调整
  } else if (fabs(error_ms) > 100.0f) {
    adjustment = -5.0f;   // 较小误差，温和调整
  } else if (fabs(error_ms) > 50.0f) {
    adjustment = -2.0f;   // 小误差，轻微调整
  } else if (fabs(error_ms) > 10.0f) {
    adjustment = -1.0f;   // 微小误差，微调
  } else {
    adjustment = 0.0f;    // 忽略微小误差
    time_discipline_.disciplining = false;
    return;
  }
  
  // 确保调整方向正确（反向调整）
  if (error_ms > 0) {
    adjustment = -fabs(adjustment);  // 正误差，负调整
  } else {
    adjustment = fabs(adjustment);   // 负误差，正调整
  }
  
  time_discipline_.disciplining = true;
  time_discipline_.discipline_count++;
  
  // 调整系统时间
  int32_t adjust_us = (int32_t)(adjustment * 1000.0f);
  
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
    // 获取调整后的误差
    struct timeval after_tv;
    gettimeofday(&after_tv, NULL);
    float after_error_ms = after_tv.tv_usec / 1000.0f;
    if (after_error_ms > 500.0f) after_error_ms = after_error_ms - 1000.0f;
    
    ESP_LOGI("gps_ntp", "时间驯服 #%u: 前误差=%.2fms, 调整=%.2fms, 后误差=%.2fms", 
             time_discipline_.discipline_count,
             error_ms, adjustment, after_error_ms);
  } else {
    ESP_LOGE("gps_ntp", "时间驯服失败");
  }
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
    need_discipline_ = true;  // 触发驯服
    
    // 记录PPS间隔（用于检测PPS质量）
    static uint32_t last_pps_time = 0;
    static uint32_t last_pps_count = 0;
    
    if (last_pps_count > 0) {
      uint32_t interval = millis() - last_pps_time;
      float interval_sec = interval / 1000.0f;
      
      // 读取引脚电平状态
      int pin_state = digitalRead(pps_pin_);
      
      // 记录详细的PPS信息
      ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %.3fs, 引脚电平: %d", 
               pps_count_, interval_sec, pin_state);
      
      // 检查间隔是否在合理范围内
      if (interval > 900 && interval < 1100) {
        // 正常
        if (pps_count_ % 60 == 0) {
          ESP_LOGI("gps_ntp", "PPS正常 #%u, 间隔: %.3fs", 
                   pps_count_, interval_sec);
        }
      } else if (interval < 100) {
        // 极短的间隔，可能是抖动或错误触发
        ESP_LOGW("gps_ntp", "PPS间隔极短: %ums，可能是干扰", interval);
      } else if (interval > 1500) {
        // 间隔太长，可能丢失PPS
        ESP_LOGW("gps_ntp", "PPS间隔过长: %ums", interval);
      } else {
        ESP_LOGW("gps_ntp", "PPS间隔异常: %ums (预期1000ms)", interval);
      }
    }
    
    last_pps_time = millis();
    last_pps_count = pps_count_;
  }
  
  // 检查PPS是否丢失（3秒无更新）
  if (pps_active_ && (millis() - pps_last_stable_ > 3000)) {
    pps_active_ = false;
    ESP_LOGW("gps_ntp", "PPS信号丢失");
  }
}

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器 (下降沿触发)");
  
  instance_ = this;
  
  // 设置PPS引脚中断
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    
    // 使用下降沿触发
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   pps_interrupt_handler, 
                   FALLING);
    
    ESP_LOGI("gps_ntp", "PPS引脚: GPIO%d (下降沿触发)", pps_pin_);
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
  
  // 时间驯服（由PPS触发）
  if (need_discipline_) {
    need_discipline_ = false;
    discipline_time();
  }
  
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
      
      ESP_LOGI("gps_ntp", "状态: PPS=%s, PPS计数=%u, 驯服=%s, 误差=%.2fms, NTP请求=%u, 时间=%02d:%02d:%02d.%06u",
               pps_active_ ? "活跃" : "无效",
               pps_count_,
               time_discipline_.disciplining ? "进行中" : "稳定",
               time_discipline_.error_ms,
               ntp_requests_,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               tv.tv_usec);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器 (下降沿触发):");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  时间误差: %.2fms", time_discipline_.error_ms);
  ESP_LOGCONFIG("gps_ntp", "  驯服状态: %s", time_discipline_.disciplining ? "进行中" : "稳定");
  ESP_LOGCONFIG("gps_ntp", "  驯服次数: %u", time_discipline_.discipline_count);
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
