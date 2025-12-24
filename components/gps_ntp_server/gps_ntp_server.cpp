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
    
    // 进入临界区
    portENTER_CRITICAL_ISR(&GPSNTPServer::instance_->time_mutex_);
    
    // 记录PPS时间
    GPSNTPServer::instance_->pps_last_edge_us_ = now;
    GPSNTPServer::instance_->pps_count_++;
    
    // PPS作为1Hz时钟源：每个脉冲增加1秒
    GPSNTPServer::instance_->pps_base_seconds_++;
    
    // 记录此时的微秒计数器值
    GPSNTPServer::instance_->last_pps_micros_ = now;
    
    // 离开临界区
    portEXIT_CRITICAL_ISR(&GPSNTPServer::instance_->time_mutex_);
    
    GPSNTPServer::instance_->pps_triggered_ = true;
  }
}

// ==================== 获取精确时间（基于PPS和微秒计数器） ====================
uint64_t GPSNTPServer::get_precise_time_us() {
  portENTER_CRITICAL(&time_mutex_);
  
  // 如果没有PPS信号，回退到系统时间
  if (!pps_active_ || pps_count_ < 3) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t result = (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
    portEXIT_CRITICAL(&time_mutex_);
    return result;
  }
  
  // 获取当前微秒计数器值（应用校准因子）
  uint32_t current_micros = (uint32_t)(micros() * micros_calibration_factor_);
  
  // 计算自上次PPS以来的微秒数
  uint32_t elapsed_micros;
  if (current_micros >= last_pps_micros_) {
    elapsed_micros = current_micros - last_pps_micros_;
  } else {
    // 处理微秒计数器溢出
    elapsed_micros = (0xFFFFFFFFUL - last_pps_micros_) + current_micros + 1;
  }
  
  // 限制在1秒内（理论上不应该超过）
  if (elapsed_micros >= 1000000) {
    elapsed_micros = 999999;
  }
  
  // 计算总时间：PPS秒数 + 微秒部分
  uint64_t result = pps_base_seconds_ * 1000000ULL + elapsed_micros;
  
  portEXIT_CRITICAL(&time_mutex_);
  return result;
}

// ==================== 校准微秒计数器 ====================
void GPSNTPServer::calibrate_microsecond_counter() {
  if (pps_count_ < 10) return;  // 需要足够的PPS样本
  
  static uint32_t last_calibration_pps = 0;
  static uint32_t last_calibration_micros = 0;
  
  portENTER_CRITICAL(&time_mutex_);
  
  uint32_t current_pps = pps_count_;
  uint32_t current_micros = micros();
  
  // 计算自上次校准以来的PPS数和微秒数
  if (last_calibration_pps > 0) {
    uint32_t pps_elapsed = current_pps - last_calibration_pps;
    uint32_t micros_elapsed;
    
    if (current_micros >= last_calibration_micros) {
      micros_elapsed = current_micros - last_calibration_micros;
    } else {
      micros_elapsed = (0xFFFFFFFFUL - last_calibration_micros) + current_micros + 1;
    }
    
    // 理想情况下，每个PPS间隔应该是1,000,000微秒
    if (pps_elapsed >= 10) {  // 至少10个PPS间隔
      float expected_micros = pps_elapsed * 1000000.0f;
      float actual_micros = (float)micros_elapsed;
      
      // 计算频率误差（ppm）
      frequency_error_ppm_ = ((actual_micros - expected_micros) / expected_micros) * 1000000.0f;
      
      // 更新校准因子
      micros_calibration_factor_ = expected_micros / actual_micros;
      
      // 限制校准因子在合理范围内（±1000ppm）
      if (micros_calibration_factor_ > 1.001f) micros_calibration_factor_ = 1.001f;
      if (micros_calibration_factor_ < 0.999f) micros_calibration_factor_ = 0.999f;
      
      ESP_LOGD("gps_ntp", "微秒计数器校准: 误差=%.1fppm, 校准因子=%.6f", 
               frequency_error_ppm_, micros_calibration_factor_);
    }
  }
  
  last_calibration_pps = current_pps;
  last_calibration_micros = current_micros;
  
  portEXIT_CRITICAL(&time_mutex_);
}

// ==================== 更新系统时间（基于PPS虚拟RTC） ====================
void GPSNTPServer::update_system_time() {
  if (!pps_active_ || pps_count_ < 3) return;
  
  // 获取基于PPS的精确时间
  uint64_t precise_time_us = get_precise_time_us();
  
  // 转换为秒和微秒
  uint64_t seconds = precise_time_us / 1000000ULL;
  uint64_t microseconds = precise_time_us % 1000000ULL;
  
  // 设置系统时间
  struct timeval tv;
  tv.tv_sec = seconds;
  tv.tv_usec = microseconds;
  
  if (settimeofday(&tv, NULL) == 0) {
    // 记录最后一次同步时间
    last_sync_us_ = microseconds;
    
    // 仅在误差较大时记录日志
    static uint64_t last_log_seconds = 0;
    if (seconds - last_log_seconds >= 60) {  // 每分钟记录一次
      last_log_seconds = seconds;
      
      struct tm *tm_info = gmtime(&tv.tv_sec);
      ESP_LOGD("gps_ntp", "PPS时间同步: %02d:%02d:%02d.%06u UTC",
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               (uint32_t)microseconds);
    }
  }
}

// ==================== 时间驯服函数（现在更简单了） ====================
void GPSNTPServer::discipline_time() {
  if (!pps_active_) return;
  
  // 不再需要复杂的调整逻辑，直接基于PPS更新时间
  update_system_time();
  
  // 定期校准微秒计数器
  uint32_t now = micros();
  if (now - last_calibration_us_ > calibration_interval_us_) {
    last_calibration_us_ = now;
    calibrate_microsecond_counter();
  }
  
  // 更新状态（计算当前误差）
  uint64_t precise_time_us = get_precise_time_us();
  
  // 计算误差（微秒）
  time_discipline_.error_us = (float)(precise_time_us % 1000000ULL);
  if (time_discipline_.error_us > 500000.0f) {
    time_discipline_.error_us -= 1000000.0f;
  }
  
  time_discipline_.disciplining = true;
  time_discipline_.discipline_count++;
}

// ==================== 发送NTP响应 ====================
void GPSNTPServer::send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                                    byte *clientTransmit) {
  // 获取基于PPS的精确时间
  uint64_t precise_time_us = get_precise_time_us();
  uint64_t seconds = precise_time_us / 1000000ULL;
  uint64_t microseconds = precise_time_us % 1000000ULL;
  
  // 转换Unix时间为NTP时间
  const unsigned long seventyYears = 2208988800UL;
  uint64_t ntp_seconds = seconds + seventyYears;
  uint64_t ntp_fraction = microseconds * 4294967296ULL / 1000000ULL;
  
  // 构建NTP响应包
  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  
  // NTP头部
  packetBuffer[0] = 0x24;  // LI=0, Version=4, Mode=4
  packetBuffer[1] = pps_active_ ? 1 : 4;  // stratum: GPS+PPS=1, 无PPS=4
  packetBuffer[2] = 6;     // Poll interval: 64秒
  packetBuffer[3] = 0xEC;  // Precision: 2^-20 ≈ 0.95μs
  
  // Root Delay (0.0001秒 = 100μs)
  uint32_t root_delay = 655;  // 0.0001 * 65536
  packetBuffer[4] = (root_delay >> 24) & 0xFF;
  packetBuffer[5] = (root_delay >> 16) & 0xFF;
  packetBuffer[6] = (root_delay >> 8) & 0xFF;
  packetBuffer[7] = root_delay & 0xFF;
  
  // Root Dispersion (0.001秒 = 1ms)
  uint32_t root_dispersion = 6554;  // 0.001 * 65536
  packetBuffer[8] = (root_dispersion >> 24) & 0xFF;
  packetBuffer[9] = (root_dispersion >> 16) & 0xFF;
  packetBuffer[10] = (root_dispersion >> 8) & 0xFF;
  packetBuffer[11] = root_dispersion & 0xFF;
  
  // Reference Identifier (GPS+PPS)
  packetBuffer[12] = 'G';
  packetBuffer[13] = 'P';
  packetBuffer[14] = 'S';
  packetBuffer[15] = 'P';
  
  // Reference Timestamp（使用第一个PPS时间）
  static uint64_t ref_ntp_seconds = 0;
  if (ref_ntp_seconds == 0 && pps_active_) {
    ref_ntp_seconds = seconds + seventyYears;
  }
  if (ref_ntp_seconds == 0) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    ref_ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
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
             (uint32_t)microseconds);
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
    
    // 至少需要3个PPS信号才开始判断
    if (pps_count_ >= 3) {
      pps_active_ = true;
      
      // PPS发生时立即更新时间
      discipline_time();
    }
    
    pps_last_stable_ = millis();
    
    // 记录PPS间隔质量
    static uint32_t last_pps_time = 0;
    static uint32_t last_pps_count = 0;
    
    if (last_pps_count > 0) {
      uint32_t interval = millis() - last_pps_time;
      
      // 检查间隔是否在合理范围内
      if (interval < 900 || interval > 1100) {
        ESP_LOGW("gps_ntp", "PPS间隔异常: %ums (预期1000ms)", interval);
      } else if (pps_count_ % 60 == 0) {
        ESP_LOGD("gps_ntp", "PPS正常 #%u, 间隔: %ums", pps_count_, interval);
      }
    }
    
    last_pps_time = millis();
    last_pps_count = pps_count_;
  }
  
  // 检查PPS是否丢失（3秒无更新）
  if (pps_active_ && (millis() - pps_last_stable_ > 3000)) {
    pps_active_ = false;
    ESP_LOGW("gps_ntp", "PPS信号丢失，回退到系统RTC");
  }
}

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器 (PPS虚拟RTC模式)");
  
  instance_ = this;
  
  // 初始化时间变量
  pps_base_seconds_ = 0;
  last_pps_micros_ = 0;
  micros_calibration_factor_ = 1.0f;
  frequency_error_ppm_ = 0.0f;
  
  // 设置PPS引脚中断
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    
    // 使用下降沿触发
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                   pps_interrupt_handler, 
                   FALLING);
    
    ESP_LOGI("gps_ntp", "PPS引脚: GPIO%d (下降沿触发)", pps_pin_);
    ESP_LOGI("gps_ntp", "使用PPS作为虚拟1Hz RTC时钟源");
  } else {
    ESP_LOGI("gps_ntp", "未配置PPS引脚，将使用系统RTC");
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
  
  // 处理NTP请求
  handle_ntp_request();
  
  // 定期状态更新（10秒一次）
  static uint32_t last_status = 0;
  if (now - last_status > 10000) {
    last_status = now;
    
    // 获取基于PPS的精确时间
    uint64_t precise_time_us = get_precise_time_us();
    uint64_t seconds = precise_time_us / 1000000ULL;
    uint64_t microseconds = precise_time_us % 1000000ULL;
    time_t unix_time = seconds;
    struct tm *tm_info = gmtime(&unix_time);
    
    ESP_LOGI("gps_ntp", "状态: PPS=%s, 计数=%u, 频率误差=%.1fppm, 校准因子=%.6f, NTP请求=%u, PPS时间=%02d:%02d:%02d.%06u",
             pps_active_ ? "活跃" : "无效",
             pps_count_,
             frequency_error_ppm_,
             micros_calibration_factor_,
             ntp_requests_,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             (uint32_t)microseconds);
    
    // 显示系统时间用于对比
    if (pps_active_) {
      struct timeval tv;
      gettimeofday(&tv, NULL);
      struct tm *sys_tm = gmtime(&tv.tv_sec);
      
      int64_t time_diff_us = (int64_t)precise_time_us - ((int64_t)tv.tv_sec * 1000000LL + tv.tv_usec);
      ESP_LOGD("gps_ntp", "系统时间: %02d:%02d:%02d.%06u, 与PPS时间差: %lldμs",
               sys_tm->tm_hour, sys_tm->tm_min, sys_tm->tm_sec, tv.tv_usec,
               time_diff_us);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器配置 (PPS虚拟RTC模式):");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  频率误差: %.1f ppm", frequency_error_ppm_);
  ESP_LOGCONFIG("gps_ntp", "  校准因子: %.6f", micros_calibration_factor_);
  ESP_LOGCONFIG("gps_ntp", "  时间误差: %.3fms", time_discipline_.error_us / 1000.0f);
  ESP_LOGCONFIG("gps_ntp", "  驯服次数: %u", time_discipline_.discipline_count);
  ESP_LOGCONFIG("gps_ntp", "  NTP请求: %u", ntp_requests_);
  
  // 显示基于PPS的时间
  uint64_t precise_time_us = get_precise_time_us();
  uint64_t seconds = precise_time_us / 1000000ULL;
  uint64_t microseconds = precise_time_us % 1000000ULL;
  time_t unix_time = seconds;
  struct tm *tm_info = gmtime(&unix_time);
  
  ESP_LOGCONFIG("gps_ntp", "  PPS时间: %04d-%02d-%02d %02d:%02d:%02d.%06u UTC",
               tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               (uint32_t)microseconds);
  
  // 同时显示系统时间用于对比
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) == 0) {
    struct tm *sys_tm = gmtime(&tv.tv_sec);
    
    ESP_LOGCONFIG("gps_ntp", "  系统时间: %04d-%02d-%02d %02d:%02d:%02d.%06u UTC",
                 sys_tm->tm_year + 1900, sys_tm->tm_mon + 1, sys_tm->tm_mday,
                 sys_tm->tm_hour, sys_tm->tm_min, sys_tm->tm_sec,
                 tv.tv_usec);
  }
}

}  // namespace gps_ntp_server
}  // namespace esphome
