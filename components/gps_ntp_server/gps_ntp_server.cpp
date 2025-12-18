#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <sys/time.h>
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// ==================== PPS中断处理 ====================
void IRAM_ATTR GPSNTPServer::pps_interrupt_handler() {
  if (GPSNTPServer::instance_) {
    uint32_t current_us = micros();
    
    // 简单的防抖：忽略100ms内的重复中断
    static uint32_t last_interrupt_us = 0;
    if (last_interrupt_us > 0) {
      uint32_t elapsed = (current_us >= last_interrupt_us) ? 
                         (current_us - last_interrupt_us) :
                         (0xFFFFFFFFUL - last_interrupt_us + current_us + 1);
      if (elapsed < 100000) return;  // 100ms内忽略
    }
    last_interrupt_us = current_us;
    
    GPSNTPServer::instance_->pps_last_edge_us_ = current_us;
    GPSNTPServer::instance_->pps_count_++;
    GPSNTPServer::instance_->pps_triggered_ = true;
  }
}

// ==================== 获取精确时间戳 ====================
GPSNTPServer::PreciseTimestamp GPSNTPServer::get_precise_timestamp() {
  PreciseTimestamp ts;
  
  // 原子操作：禁用中断，确保数据一致性
  noInterrupts();
  
  // 同时获取所有时间相关变量
  ts.micros_counter = micros();
  ts.pps_edge_us = pps_last_edge_us_;
  ts.pps_count = pps_count_;
  
  interrupts();
  
  // 获取系统时间（不能在中断中调用）
  gettimeofday(&ts.system_time, nullptr);
  
  return ts;
}

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化高精度GPS NTP服务器");
  ESP_LOGI("gps_ntp", "版本: 2.0 - 原子操作时间戳");
  
  instance_ = this;
  
  // 设置PPS引脚中断
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    
    // 使用上升沿触发（0.2秒脉冲的开始）
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   pps_interrupt_handler, 
                   RISING);
    
    ESP_LOGI("gps_ntp", "PPS引脚: GPIO%d (上升沿触发)", pps_pin_);
  } else {
    ESP_LOGW("gps_ntp", "未配置PPS引脚，时间精度将受限");
  }
  
  // 启动NTP服务器
  if (udp_.begin(123)) {
    ntp_started_ = true;
    ESP_LOGI("gps_ntp", "NTP服务器已启动，端口123");
  } else {
    ESP_LOGE("gps_ntp", "NTP服务器启动失败");
  }
  
  // 初始化时间状态
  time_state_.system_time_valid = false;
  time_state_.last_system_check = millis();
}

// ==================== GPS更新回调 ====================
void GPSNTPServer::on_update(TinyGPSPlus &tiny_gps) {
  // 仅用于状态显示和验证
  if (tiny_gps.time.isValid() && tiny_gps.date.isValid()) {
    gps_status_.valid = true;
    gps_status_.year = tiny_gps.date.year();
    gps_status_.month = tiny_gps.date.month();
    gps_status_.day = tiny_gps.date.day();
    gps_status_.hour = tiny_gps.time.hour();
    gps_status_.minute = tiny_gps.time.minute();
    gps_status_.second = tiny_gps.time.second();
    gps_status_.last_update = millis();
    
    // 每10秒输出一次GPS状态
    static uint32_t last_gps_log = 0;
    if (millis() - last_gps_log > 10000) {
      last_gps_log = millis();
      ESP_LOGD("gps_ntp", "GPS状态: %04d-%02d-%02d %02d:%02d:%02d, 卫星: %d",
               gps_status_.year, gps_status_.month, gps_status_.day,
               gps_status_.hour, gps_status_.minute, gps_status_.second,
               tiny_gps.satellites.value());
    }
  }
}

// ==================== PPS信号处理 ====================
void GPSNTPServer::handle_pps_signal() {
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_active_ = true;
    pps_last_stable_ = millis();
    
    uint32_t now_us = micros();
    uint32_t pps_edge_us = pps_last_edge_us_;
    
    // 计算PPS间隔（处理32位溢出）
    uint32_t interval_us;
    if (now_us >= pps_edge_us) {
      interval_us = now_us - pps_edge_us;
    } else {
      interval_us = (0xFFFFFFFFUL - pps_edge_us) + now_us + 1;
    }
    
    // 验证PPS间隔是否合理
    if (interval_us > 700000 && interval_us < 1300000) {
      // 正常PPS间隔
      static uint32_t last_logged_pps = 0;
      if (pps_count_ - last_logged_pps >= 60) {  // 每分钟记录一次
        last_logged_pps = pps_count_;
        ESP_LOGD("gps_ntp", "PPS #%u, 间隔: %.3fms", 
                 pps_count_, interval_us / 1000.0f);
      }
      
      // 校准PPS
      if (!pps_calibration_.calibrated || pps_count_ % 60 == 0) {
        // 获取当前系统时间
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        
        // 计算PPS微秒与系统微秒的差异
        uint32_t pps_micros = interval_us % 1000000UL;
        int32_t offset_us = (int32_t)pps_micros - (int32_t)tv.tv_usec;
        
        // 调整超过±500ms的差异
        if (offset_us > 500000) {
          offset_us -= 1000000;
        } else if (offset_us < -500000) {
          offset_us += 1000000;
        }
        
        pps_calibration_.accumulated_offset_us = offset_us;
        pps_calibration_.last_calibration_us = now_us;
        pps_calibration_.calibrated = true;
        
        if (abs(offset_us) > 1000) {  // 差异大于1ms才记录
          ESP_LOGD("gps_ntp", "PPS校准: 偏移=%dus", offset_us);
        }
      }
    } else {
      // PPS间隔异常
      ESP_LOGW("gps_ntp", "PPS间隔异常: %.3fms (预期1000ms)", interval_us / 1000.0f);
      
      // 如果是极短的间隔，可能是硬件问题
      if (interval_us < 100000) {
        ESP_LOGE("gps_ntp", "检测到快速脉冲，请检查PPS引脚配置");
      }
    }
  }
  
  // 检查PPS是否丢失（3秒无更新）
  if (pps_active_ && (millis() - pps_last_stable_ > 3000)) {
    pps_active_ = false;
    ESP_LOGW("gps_ntp", "PPS信号丢失");
  }
}

// ==================== 时间质量评估 ====================
uint8_t GPSNTPServer::get_time_quality() const {
  // 检查系统时间是否有效
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  
  bool system_valid = (tv.tv_sec > 1609459200L);  // 2021年之后
  
  if (system_valid && pps_active_ && pps_calibration_.calibrated) {
    return QUALITY_GPS_PPS;
  } else if (pps_active_ && pps_calibration_.calibrated) {
    return QUALITY_PPS;
  } else if (system_valid) {
    return QUALITY_SYSTEM;
  } else {
    return QUALITY_NO_SYNC;
  }
}

// ==================== 发送NTP响应 ====================
void GPSNTPServer::send_ntp_response(WiFiUDP &udp, IPAddress remote, 
                                     int remotePort, byte *client_transmit,
                                     const PreciseTimestamp &ts) {
  // 基本参数
  const unsigned long seventyYears = 2208988800UL;
  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  
  // 计算时间
  struct timeval tv = ts.system_time;
  
  // 使用PPS校准微秒部分
  uint32_t calibrated_microseconds = tv.tv_usec;
  
  if (pps_active_ && pps_count_ > 0) {
    // 计算自PPS以来的微秒数
    uint32_t since_pps;
    if (ts.micros_counter >= ts.pps_edge_us) {
      since_pps = ts.micros_counter - ts.pps_edge_us;
    } else {
      since_pps = (0xFFFFFFFFUL - ts.pps_edge_us) + ts.micros_counter + 1;
    }
    
    // 获取PPS微秒部分
    uint32_t pps_micros = since_pps % 1000000UL;
    
    // 检查是否在秒边界附近
    uint32_t system_micros = tv.tv_usec;
    int32_t diff = (int32_t)pps_micros - (int32_t)system_micros;
    
    // 处理超过±500ms的差异（秒边界跳变）
    if (diff > 500000) {
      // PPS显示时间比系统时间快超过500ms
      // 系统时间可能慢了1秒
      tv.tv_sec -= 1;
      diff -= 1000000;
    } else if (diff < -500000) {
      // PPS显示时间比系统时间慢超过500ms
      // 系统时间可能快了1秒
      tv.tv_sec += 1;
      diff += 1000000;
    }
    
    // 如果PPS校准可用且差异合理，使用PPS微秒
    if (pps_calibration_.calibrated && abs(diff) < 100000) {
      calibrated_microseconds = pps_micros;
      
      // 应用PPS校准偏移
      calibrated_microseconds = (calibrated_microseconds + 
                                1000000UL - pps_calibration_.accumulated_offset_us) % 1000000UL;
    }
  }
  
  // 转换为NTP时间
  uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
  uint64_t ntp_fraction = (uint64_t)calibrated_microseconds * 4294967296ULL / 1000000ULL;
  
  uint32_t ref_seconds = (uint32_t)ntp_seconds;
  uint32_t ref_fraction = (uint32_t)ntp_fraction;
  
  // 根据时间质量设置stratum
  uint8_t quality = get_time_quality();
  uint8_t stratum;
  
  switch (quality) {
    case QUALITY_GPS_PPS:
      stratum = 1;   // 一级参考源
      break;
    case QUALITY_PPS:
      stratum = 2;   // 二级参考源
      break;
    default:
      stratum = 4;   // 普通服务器
  }
  
  // NTP头部
  packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
  packetBuffer[1] = stratum;
  packetBuffer[2] = 6;     // Poll interval: 64秒
  packetBuffer[3] = 0xEC;  // Precision: 2^-20 ≈ 1微秒
  
  // Root Delay (0)
  memset(&packetBuffer[4], 0, 4);
  
  // Root Dispersion
  uint32_t root_dispersion = 100 << 16;  // 100微秒
  for (int i = 0; i < 4; i++) {
    packetBuffer[8 + i] = (root_dispersion >> (24 - i*8)) & 0xFF;
  }
  
  // Reference Identifier
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = quality >= QUALITY_PPS ? 'P' : 'S';
  
  // Reference Timestamp
  for (int i = 0; i < 4; i++) {
    packetBuffer[16 + i] = (ref_seconds >> (24 - i*8)) & 0xFF;
    packetBuffer[20 + i] = (ref_fraction >> (24 - i*8)) & 0xFF;
  }
  
  // Origin Timestamp（复制客户端时间）
  memcpy(&packetBuffer[24], client_transmit, 8);
  
  // Receive Timestamp（当前时间）
  memcpy(&packetBuffer[32], &packetBuffer[16], 8);
  
  // Transmit Timestamp（当前时间）
  memcpy(&packetBuffer[40], &packetBuffer[16], 8);
  
  // 发送响应
  udp.beginPacket(remote, remotePort);
  udp.write(packetBuffer, 48);
  
  if (udp.endPacket() == 1) {
    // 成功发送，记录统计
    ntp_requests_++;
    
    // 每10个请求或首个请求记录详细日志
    if (ntp_requests_ % 10 == 1) {
      time_t unix_time = ref_seconds - seventyYears;
      struct tm *tm_info = gmtime(&unix_time);
      
      ESP_LOGI("gps_ntp", "NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d.%06u, 质量=%d",
               ntp_requests_, remote.toString().c_str(), remotePort,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               calibrated_microseconds,
               quality);
    }
  } else {
    ntp_errors_++;
    ESP_LOGE("gps_ntp", "NTP响应发送失败");
  }
}

// ==================== NTP请求处理 ====================
void GPSNTPServer::process_ntp_requests() {
  int packetSize = udp_.parsePacket();
  if (packetSize >= 48) {
    byte packetBuffer[48];
    udp_.read(packetBuffer, 48);
    IPAddress remote = udp_.remoteIP();
    int remotePort = udp_.remotePort();
    
    // 保存客户端的Transmit Timestamp
    byte clientTransmit[8];
    memcpy(clientTransmit, &packetBuffer[40], 8);
    
    // 获取精确时间戳
    PreciseTimestamp ts = get_precise_timestamp();
    
    // 验证系统时间有效性
    if (ts.system_time.tv_sec < 1609459200L) {
      ESP_LOGW("gps_ntp", "系统时间无效 (%lu秒)，无法提供NTP服务", ts.system_time.tv_sec);
      return;
    }
    
    // 发送NTP响应
    send_ntp_response(udp_, remote, remotePort, clientTransmit, ts);
  }
}

// ==================== 主循环 ====================
void GPSNTPServer::loop() {
  uint32_t now = millis();
  
  // 限制循环频率（10ms）
  if (now - last_loop_ < 10) return;
  last_loop_ = now;
  
  // 处理PPS信号
  handle_pps_signal();
  
  // 处理NTP请求
  process_ntp_requests();
  
  // 定期状态更新（每30秒）
  static uint32_t last_status = 0;
  if (now - last_status > 30000) {
    last_status = now;
    
    // 获取系统时间
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      // 检查系统时间有效性
      bool system_valid = (tv.tv_sec > 1609459200L);
      time_state_.system_time_valid = system_valid;
      time_state_.last_system_check = now;
      
      // 输出状态
      struct tm *tm_info = gmtime(&tv.tv_sec);
      uint8_t quality = get_time_quality();
      
      const char* quality_str = "";
      switch (quality) {
        case QUALITY_GPS_PPS: quality_str = "GPS+PPS"; break;
        case QUALITY_PPS: quality_str = "PPS"; break;
        case QUALITY_SYSTEM: quality_str = "系统时间"; break;
        default: quality_str = "未同步";
      }
      
      ESP_LOGI("gps_ntp", "========================================");
      ESP_LOGI("gps_ntp", "NTP服务器状态:");
      ESP_LOGI("gps_ntp", "  时间质量: %s (等级 %d)", quality_str, quality);
      ESP_LOGI("gps_ntp", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d.%06u",
               tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               tv.tv_usec);
      ESP_LOGI("gps_ntp", "  PPS状态: %s, 计数: %u",
               pps_active_ ? "活跃" : "无效", pps_count_);
      
      if (gps_status_.valid && (now - gps_status_.last_update < 10000)) {
        ESP_LOGI("gps_ntp", "  GPS时间: %04d-%02d-%02d %02d:%02d:%02d",
                 gps_status_.year, gps_status_.month, gps_status_.day,
                 gps_status_.hour, gps_status_.minute, gps_status_.second);
      }
      
      ESP_LOGI("gps_ntp", "  NTP统计: 请求=%u, 错误=%u", 
               ntp_requests_, ntp_errors_);
      ESP_LOGI("gps_ntp", "========================================");
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "高精度GPS NTP服务器配置:");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  
  uint8_t quality = get_time_quality();
  const char* quality_str = "";
  switch (quality) {
    case QUALITY_GPS_PPS: quality_str = "GPS+PPS"; break;
    case QUALITY_PPS: quality_str = "PPS"; break;
    case QUALITY_SYSTEM: quality_str = "系统时间"; break;
    default: quality_str = "未同步";
  }
  
  ESP_LOGCONFIG("gps_ntp", "  时间质量: %s (等级 %d)", quality_str, quality);
  ESP_LOGCONFIG("gps_ntp", "  PPS状态: %s", pps_active_ ? "活跃" : "无效");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  NTP统计: 请求=%u, 错误=%u", ntp_requests_, ntp_errors_);
  
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) == 0 && tv.tv_sec > 1609459200L) {
    struct tm *tm_info = gmtime(&tv.tv_sec);
    
    ESP_LOGCONFIG("gps_ntp", "  当前时间: %04d-%02d-%02d %02d:%02d:%02d.%06u",
                 tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                 tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
                 tv.tv_usec);
  } else {
    ESP_LOGCONFIG("gps_ntp", "  当前时间: 无效 (请先同步系统时间)");
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome
