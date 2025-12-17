
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
}

// ==================== GPS更新回调 ====================
void GPSNTPServer::on_update(TinyGPSPlus &tiny_gps) {
  // 检查GPS数据是否有效
  if (!tiny_gps.time.isValid() || !tiny_gps.date.isValid() || 
      !tiny_gps.time.isUpdated() || !tiny_gps.date.isUpdated() || 
      tiny_gps.date.year() < 2024) {
    return;
  }
  
  // 设置时区为UTC
  setenv("TZ", "UTC", 1);
  tzset();
  
  // 构建tm结构
  struct tm timeinfo = {0};
  timeinfo.tm_year = tiny_gps.date.year() - 1900;
  timeinfo.tm_mon = tiny_gps.date.month() - 1;
  timeinfo.tm_mday = tiny_gps.date.day();
  timeinfo.tm_hour = tiny_gps.time.hour();
  timeinfo.tm_min = tiny_gps.time.minute();
  timeinfo.tm_sec = tiny_gps.time.second();
  
  // 转换为Unix时间
  time_t epoch = mktime(&timeinfo);
  
  // 检查时间是否合理（晚于2020年）
  if (epoch > 1609459200L) {
    struct timeval tv = {epoch, 0};
    
    // 设置系统时间
    if (settimeofday(&tv, nullptr) == 0) {
      gps_valid_ = true;
      last_gps_update_ = millis();
      
      // 记录GPS时间
      ESP_LOGI("gps_ntp", "GPS时间: %04d-%02d-%02d %02d:%02d:%02d UTC",
               tiny_gps.date.year(), tiny_gps.date.month(), tiny_gps.date.day(),
               tiny_gps.time.hour(), tiny_gps.time.minute(), tiny_gps.time.second());
    } else {
      ESP_LOGE("gps_ntp", "设置系统时间失败");
    }
  }
}

// ==================== PPS处理 ====================
void GPSNTPServer::handle_pps() {
  static uint32_t last_pps_count = 0;
  
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_active_ = true;
    pps_last_stable_ = millis();
    
    uint32_t now_us = micros();
    uint32_t interval_us = (now_us - pps_last_edge_us_) & 0xFFFFFFFFUL;
    
    // 检查PPS间隔是否稳定（900ms-1100ms）
    if (interval_us > 900000 && interval_us < 1100000) {
      if (pps_count_ > last_pps_count) {
        last_pps_count = pps_count_;
        
        if (pps_count_ % 60 == 0) {  // 每分钟输出一次
          ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %.3fms", 
                   pps_count_, interval_us / 1000.0f);
        }
        
        // 关键：PPS发生时，系统时间的秒部分应该是整数
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        
        // 如果系统时间的微秒部分不是接近0，可能需要调整
        if (tv.tv_usec > 500000) {
          // PPS发生在秒的开始，微秒应该是0
          // 但我们不自动调整，让GPS时间决定
        }
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
  if (gps_valid_ && pps_active_) return QUALITY_GPS_PPS;
  if (pps_active_) return QUALITY_PPS;
  if (gps_valid_) return QUALITY_GPS;
  return QUALITY_SYSTEM;
}

// ==================== NTP请求处理（修复秒边界错误） ====================
void GPSNTPServer::process_ntp() {
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    byte packetBuffer[48];
    udp_.read(packetBuffer, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    ntp_requests_++;
    
    // 保存客户端的Transmit Timestamp（字节40-47）
    byte clientTransmit[8];
    for (int i = 0; i < 8; i++) {
      clientTransmit[i] = packetBuffer[40 + i];
    }
    
    // 获取当前系统时间（包括微秒）
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    // Unix时间转换为NTP时间
    const unsigned long seventyYears = 2208988800UL;
    uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
    
    // 计算NTP分数部分（微秒转换为2^32分数）
    uint64_t ntp_fraction = (uint64_t)tv.tv_usec * 4294967296ULL / 1000000ULL;
    
    // ========== 关键修复：PPS微秒校准（无秒边界调整） ==========
    if (pps_active_ && pps_count_ > 0) {
      uint32_t now_us = micros();
      uint32_t since_pps = (now_us - pps_last_edge_us_) & 0xFFFFFFFFUL;
      
      // 只使用PPS提供当前秒内的微秒计数
      // 永远不增加秒数，秒数由系统时间决定
      if (since_pps < 1000000) {  // 在当前秒内
        uint32_t pps_micros = since_pps;
        ntp_fraction = (uint64_t)pps_micros * 4294967296ULL / 1000000ULL;
        
        ESP_LOGD("gps_ntp", "PPS校准: 微秒=%u, 分数=%llu", 
                 pps_micros, ntp_fraction);
      }
      // 如果since_pps >= 1000000，说明PPS信号可能有问题，不使用PPS校准
    }
    
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
          stratum = 4;   // 普通服务器
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
    
    // ========== 时间戳设置 ==========
    uint32_t ref_seconds = (uint32_t)ntp_seconds;
    uint32_t ref_fraction = (uint32_t)ntp_fraction;
    
    // Reference Timestamp
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
    
    // 记录日志（显示详细时间信息）
    static uint32_t last_log_time = 0;
    uint32_t now = millis();
    if (now - last_log_time > 1000) {  // 每秒最多记录一次
      last_log_time = now;
      
      time_t unix_time = ref_seconds - seventyYears;
      struct tm *tm_info = gmtime(&unix_time);
      
      uint32_t microseconds = (uint64_t)ref_fraction * 1000000ULL / 4294967296ULL;
      
      // 解码客户端的Transmit Timestamp
      uint32_t client_seconds = (clientTransmit[0] << 24) | 
                                (clientTransmit[1] << 16) | 
                                (clientTransmit[2] << 8) | 
                                clientTransmit[3];
      uint32_t client_fraction = (clientTransmit[4] << 24) | 
                                 (clientTransmit[5] << 16) | 
                                 (clientTransmit[6] << 8) | 
                                 clientTransmit[7];
      uint32_t client_micros = (uint64_t)client_fraction * 1000000ULL / 4294967296ULL;
      time_t client_unix_time = client_seconds - seventyYears;
      struct tm *client_tm = gmtime(&client_unix_time);
      
      ESP_LOGI("gps_ntp", "NTP响应: 客户端=%02d:%02d:%02d.%06u, 服务器=%02d:%02d:%02d.%06u, 差值=%.3fms",
               client_tm->tm_hour, client_tm->tm_min, client_tm->tm_sec, client_micros,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec, microseconds,
               (microseconds - client_micros) / 1000.0f);
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
    
    // 检查GPS超时
    if (gps_valid_ && (now - last_gps_update_ > 10000)) {
      gps_valid_ = false;
      ESP_LOGW("gps_ntp", "GPS信号丢失");
    }
    
    // 输出状态
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      struct tm *tm_info = gmtime(&tv.tv_sec);
      
      ESP_LOGI("gps_ntp", "状态: GPS=%s, PPS=%s, PPS计数=%u, 质量=%d, UTC=%02d:%02d:%02d.%06u, NTP请求=%u",
               gps_valid_ ? "有效" : "无效",
               pps_active_ ? "活跃" : "无效",
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

