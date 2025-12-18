#include "gps_ntp_server.h"
#include "esphome/components/network/util.h"
#include <sys/time.h>
#include <math.h>

namespace esphome {
namespace gps_ntp_server {

GPSNTPServer *GPSNTPServer::instance_ = nullptr;

// ==================== PPS中断处理（高精度版本） ====================
void IRAM_ATTR GPSNTPServer::pps_interrupt_handler() {
  if (GPSNTPServer::instance_) {
    static uint32_t last_interrupt_time = 0;
    uint32_t now = micros();
    
    // 防抖：忽略50ms内的重复中断
    if (last_interrupt_time > 0) {
      uint32_t elapsed;
      if (now >= last_interrupt_time) {
        elapsed = now - last_interrupt_time;
      } else {
        elapsed = (0xFFFFFFFFUL - last_interrupt_time) + now + 1;
      }
      if (elapsed < 50000) return;  // 50ms内忽略
    }
    last_interrupt_time = now;
    
    GPSNTPServer::instance_->pps_last_edge_us_ = now;
    GPSNTPServer::instance_->pps_count_++;
    GPSNTPServer::instance_->pps_triggered_ = true;
  }
}

// ==================== 精密驯服函数 ====================
void GPSNTPServer::precise_discipline() {
  // 获取当前系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 计算当前误差（微秒）
  uint32_t current_us = micros();
  uint32_t pps_edge_us = pps_last_edge_us_;
  
  // 计算自上次PPS以来的微秒数
  uint32_t since_pps;
  if (current_us >= pps_edge_us) {
    since_pps = current_us - pps_edge_us;
  } else {
    since_pps = (0xFFFFFFFFUL - pps_edge_us) + current_us + 1;
  }
  
  // 归一化误差到±500,000微秒范围
  float error_us = (float)(since_pps % 1000000);
  if (error_us > 500000.0f) {
    error_us = error_us - 1000000.0f;
  }
  
  // 计算误差变化率（微秒/秒）
  static uint32_t last_error_time = 0;
  if (last_error_time > 0) {
    uint32_t elapsed = (current_us >= last_error_time) ? 
                      (current_us - last_error_time) : 
                      (0xFFFFFFFFUL - last_error_time + current_us + 1);
    if (elapsed > 0) {
      float time_since_last = elapsed / 1000000.0f;  // 秒
      time_discipline_.error_rate_us = 
        (error_us - time_discipline_.last_error_us) / time_since_last;
    }
  }
  last_error_time = current_us;
  
  // 更新误差统计
  time_discipline_.last_error_us = time_discipline_.error_us;
  time_discipline_.error_us = error_us;
  
  // 计算时钟漂移（PPM = 百万分之一）
  static float last_pps_time = 0;
  if (last_pps_time > 0 && pps_count_ > 1) {
    float pps_interval = (current_us - last_pps_time) / 1000000.0f;
    float expected_interval = 1.0f;  // 期望1秒
    float drift = (pps_interval - expected_interval) / expected_interval;
    time_discipline_.clock_drift_ppm = drift * 1000000.0f;  // 转换为PPM
  }
  last_pps_time = current_us;
  
  // 精密PI控制器（微秒级控制）
  float adjustment_us = 0.0f;
  
  // 比例项：误差补偿
  float Kp = 0.7f;  // 比例系数（根据误差动态调整）
  adjustment_us -= Kp * error_us;
  
  // 积分项：消除稳态误差
  time_discipline_.accumulated_error_us += error_us;
  
  // 抗积分饱和：限制积分项
  const float MAX_INTEGRAL = 1000000.0f;  // 最大累积误差1秒
  if (time_discipline_.accumulated_error_us > MAX_INTEGRAL) {
    time_discipline_.accumulated_error_us = MAX_INTEGRAL;
  } else if (time_discipline_.accumulated_error_us < -MAX_INTEGRAL) {
    time_discipline_.accumulated_error_us = -MAX_INTEGRAL;
  }
  
  float Ki = 0.02f;  // 积分系数（较小，避免振荡）
  adjustment_us -= Ki * time_discipline_.accumulated_error_us;
  
  // 前馈补偿：根据时钟漂移预测调整
  if (fabs(time_discipline_.clock_drift_ppm) > 1.0f) {
    // 漂移补偿：每秒需要调整的微秒数
    float drift_compensation = time_discipline_.clock_drift_ppm / 1000000.0f * 1000000.0f;  // 转换为微秒/秒
    adjustment_us += drift_compensation;
  }
  
  // 智能调整限制（根据误差大小动态调整）
  float max_adjustment = 0.0f;
  
  if (fabs(error_us) > 100000.0f) {  // >100ms误差
    max_adjustment = 10000.0f;  // 10ms
    Kp = 0.9f;  // 增大比例系数
  } else if (fabs(error_us) > 10000.0f) {  // >10ms误差
    max_adjustment = 1000.0f;  // 1ms
    Kp = 0.8f;
  } else if (fabs(error_us) > 1000.0f) {  // >1ms误差
    max_adjustment = 100.0f;  // 0.1ms
    Kp = 0.7f;
  } else if (fabs(error_us) > 100.0f) {  // >0.1ms误差
    max_adjustment = 10.0f;  // 0.01ms
    Kp = 0.6f;
  } else {
    max_adjustment = 1.0f;  // 1微秒调整
    Kp = 0.5f;
    
    // 误差很小，增加稳定计数
    time_discipline_.stable_count++;
    if (time_discipline_.stable_count > 10) {
      // 长时间稳定，降低积分项
      time_discipline_.accumulated_error_us *= 0.9f;
    }
  }
  
  // 应用限制
  if (adjustment_us > max_adjustment) adjustment_us = max_adjustment;
  if (adjustment_us < -max_adjustment) adjustment_us = -max_adjustment;
  
  // 死区控制：误差很小时不调整，避免抖动
  const float DEAD_ZONE = 10.0f;  // 10微秒死区
  if (fabs(error_us) < DEAD_ZONE && fabs(adjustment_us) < DEAD_ZONE) {
    time_discipline_.disciplining = false;
    return;  // 不调整
  }
  
  time_discipline_.disciplining = true;
  time_discipline_.discipline_count++;
  
  // 应用调整
  struct timeval new_tv;
  gettimeofday(&new_tv, NULL);
  
  // 微秒级调整
  int32_t adjust_microseconds = (int32_t)adjustment_us;
  new_tv.tv_usec += adjust_microseconds;
  
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
    // 获取调整后的误差进行验证
    gettimeofday(&tv, NULL);
    uint32_t after_us = micros();
    uint32_t after_since_pps;
    if (after_us >= pps_edge_us) {
      after_since_pps = after_us - pps_edge_us;
    } else {
      after_since_pps = (0xFFFFFFFFUL - pps_edge_us) + after_us + 1;
    }
    
    float after_error_us = (float)(after_since_pps % 1000000);
    if (after_error_us > 500000.0f) {
      after_error_us = after_error_us - 1000000.0f;
    }
    
    // 选择性记录日志（避免日志过多）
    static uint32_t log_counter = 0;
    if (log_counter % 10 == 0 || fabs(error_us) > 1000.0f) {
      ESP_LOGI("gps_ntp", "精密驯服 #%u: 误差=%.1fus->%.1fus, 调整=%.1fus, 漂移=%.1fPPM, 稳定=%u", 
               time_discipline_.discipline_count,
               error_us, after_error_us, adjustment_us,
               time_discipline_.clock_drift_ppm,
               time_discipline_.stable_count);
    }
    log_counter++;
  } else {
    ESP_LOGE("gps_ntp", "精密驯服失败");
  }
}

// ==================== 时间驯服函数（主函数） ====================
void GPSNTPServer::discipline_time() {
  if (!pps_active_) return;
  
  uint32_t now = millis();
  
  // 每秒驯服一次（与PPS同步）
  if (now - time_discipline_.last_discipline < 1000) return;
  
  precise_discipline();
  
  time_discipline_.last_discipline = now;
}

// ==================== 处理PPS（精密版本） ====================
void GPSNTPServer::handle_pps() {
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_active_ = true;
    pps_last_stable_ = millis();
    
    // 立即触发一次精密驯服
    if (pps_active_) {
      precise_discipline();
    }
    
    // 记录PPS间隔和质量
    static uint32_t last_pps_time = 0;
    static uint32_t last_pps_count = 0;
    static float interval_sum = 0.0f;
    static uint32_t interval_count = 0;
    
    if (last_pps_count > 0) {
      uint32_t interval = millis() - last_pps_time;
      float interval_sec = interval / 1000.0f;
      
      // 统计平均间隔
      interval_sum += interval_sec;
      interval_count++;
      
      if (interval_count >= 60) {
        float avg_interval = interval_sum / interval_count;
        float jitter = fabs(interval_sec - avg_interval);
        
        if (pps_count_ % 60 == 0) {
          ESP_LOGI("gps_ntp", "PPS质量: 平均间隔=%.6fs, 抖动=%.6fs", 
                   avg_interval, jitter);
        }
        
        interval_sum = 0.0f;
        interval_count = 0;
      }
    }
    
    last_pps_time = millis();
    last_pps_count = pps_count_;
  }
  
  // 检查PPS是否丢失（2秒无更新）
  if (pps_active_ && (millis() - pps_last_stable_ > 2000)) {
    pps_active_ = false;
    time_discipline_.stable_count = 0;  // 重置稳定计数
    ESP_LOGW("gps_ntp", "PPS信号丢失");
  }
}

// ==================== 发送NTP响应 ====================
void GPSNTPServer::send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                                    byte *clientTransmit) {
  // 获取当前精确时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  // 如果PPS活跃，使用更精确的时间计算
  if (pps_active_) {
    uint32_t current_us = micros();
    uint32_t pps_edge_us = pps_last_edge_us_;
    
    // 计算自PPS以来的微秒数
    uint32_t since_pps;
    if (current_us >= pps_edge_us) {
      since_pps = current_us - pps_edge_us;
    } else {
      since_pps = (0xFFFFFFFFUL - pps_edge_us) + current_us + 1;
    }
    
    // 使用PPS校准的微秒部分
    uint32_t microseconds = since_pps % 1000000;
    
    // 如果误差超过0.5秒，调整秒数
    if (since_pps >= 500000) {
      tv.tv_sec++;
    }
    
    tv.tv_usec = microseconds;
  }
  
  // 转换Unix时间为NTP时间
  const unsigned long seventyYears = 2208988800UL;
  uint64_t ntp_seconds = (uint64_t)tv.tv_sec + seventyYears;
  
  // 使用64位整数确保精度
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
  
  // 记录日志（每50个请求或误差较大时）
  if (ntp_requests_ % 50 == 0 || ntp_requests_ == 1) {
    time_t unix_time = (uint32_t)ntp_seconds - seventyYears;
    struct tm *tm_info = gmtime(&unix_time);
    
    ESP_LOGI("gps_ntp", "精密NTP响应 #%u: %s:%d, UTC=%02d:%02d:%02d.%06u, 误差=%.1fus", 
             ntp_requests_, remote.toString().c_str(), remotePort,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             tv.tv_usec, time_discipline_.error_us);
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

// ==================== 初始化 ====================
void GPSNTPServer::setup() {
  ESP_LOGI("gps_ntp", "初始化GPS NTP服务器 (精密驯服版)");
  
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
      
      ESP_LOGI("gps_ntp", "精密状态: PPS=%s, 计数=%u, 驯服=%s, 误差=%.1fus, 漂移=%.1fPPM, NTP请求=%u, 时间=%02d:%02d:%02d.%06u",
               pps_active_ ? "活跃" : "无效",
               pps_count_,
               time_discipline_.disciplining ? "进行中" : "稳定",
               time_discipline_.error_us,
               time_discipline_.clock_drift_ppm,
               ntp_requests_,
               tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
               tv.tv_usec);
    }
  }
}

// ==================== 配置输出 ====================
void GPSNTPServer::dump_config() {
  ESP_LOGCONFIG("gps_ntp", "GPS NTP服务器 (精密驯服版):");
  ESP_LOGCONFIG("gps_ntp", "  PPS引脚: GPIO%d", pps_pin_);
  ESP_LOGCONFIG("gps_ntp", "  PPS活跃: %s", pps_active_ ? "是" : "否");
  ESP_LOGCONFIG("gps_ntp", "  PPS计数: %u", pps_count_);
  ESP_LOGCONFIG("gps_ntp", "  时间误差: %.1f微秒", time_discipline_.error_us);
  ESP_LOGCONFIG("gps_ntp", "  时钟漂移: %.1f PPM", time_discipline_.clock_drift_ppm);
  ESP_LOGCONFIG("gps_ntp", "  驯服状态: %s", time_discipline_.disciplining ? "进行中" : "稳定");
  ESP_LOGCONFIG("gps_ntp", "  驯服次数: %u", time_discipline_.discipline_count);
  ESP_LOGCONFIG("gps_ntp", "  稳定计数: %u", time_discipline_.stable_count);
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
