#include "gps_pps_ntp.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace gps_pps_ntp {

// 初始化静态实例指针
GPSPPSNTPServer* GPSPPSNTPServer::instance_ = nullptr;

// PPS中断服务程序
void IRAM_ATTR GPSPPSNTPServer::pps_interrupt_handler() {
  if (GPSPPSNTPServer::instance_ != nullptr) {
    GPSPPSNTPServer::instance_->handle_pps_interrupt();
  }
}

void IRAM_ATTR GPSPPSNTPServer::handle_pps_interrupt() {
  uint32_t now = micros();
  
  // 检查是否是有效的PPS间隔（应在0.9s-1.1s之间）
  uint32_t interval = now - last_pps_micros_;
  
  if (interval > 900000 && interval < 1100000) {
    last_pps_micros_ = now;
    pps_count_++;
    pps_triggered_ = true;
    pps_active_ = true;
  }
}

void GPSPPSNTPServer::setup() {
  // 设置静态实例
  instance_ = this;
  
  // 配置串口（GPS）
  if (gps_baud_rate_ > 0) {
    ESP_LOGI("GPSPPSNTP", "Initializing GPS at %u baud", gps_baud_rate_);
  }
  
  // 配置PPS引脚
  if (pps_pin_ > 0) {
    pinMode(pps_pin_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pps_pin_), 
                    &GPSPPSNTPServer::pps_interrupt_handler, 
                    FALLING); // 根据实际连接调整
    
    ESP_LOGI("GPSPPSNTP", "PPS sensor on GPIO %d", pps_pin_);
  }
  
  // 启动NTP服务器
  start_ntp_server();
  
  ESP_LOGI("GPSPPSNTP", "High Precision NTP Server initialized");
}

void GPSPPSNTPServer::loop() {
  // 解析GPS数据
  parse_gps_data();
  
  // 处理PPS事件
  if (pps_triggered_) {
    pps_triggered_ = false;
    pps_pulses_++;
    
    // 如果有有效的GPS时间，更新时间基准
    if (gps_locked_) {
      // GPS时间在PPS时刻增加1秒
      gps_epoch_seconds_++;
      gps_epoch_micros_ = 0;
      last_gps_update_micros_ = last_pps_micros_;
      
      ESP_LOGD("GPSPPSNTP", "PPS #%u at %u us, GPS epoch: %u", 
               pps_count_, last_pps_micros_, gps_epoch_seconds_);
    }
    
    // 更新卡尔曼滤波器
    calculate_precise_time();
  }
  
  // 处理NTP请求
  handle_ntp_request();
}

void GPSPPSNTPServer::dump_config() {
  ESP_LOGCONFIG("GPSPPSNTP", "High Precision GPS+PPS NTP Server");
  ESP_LOGCONFIG("GPSPPSNTP", "  PPS Pin: %d", pps_pin_);
  ESP_LOGCONFIG("GPSPPSNTP", "  GPS Baud Rate: %u", gps_baud_rate_);
  
  if (gps_locked_) {
    ESP_LOGCONFIG("GPSPPSNTP", "  GPS Status: LOCKED (%d satellites)", get_satellites());
    ESP_LOGCONFIG("GPSPPSNTP", "  HDOP: %.2f", get_hdop());
  } else {
    ESP_LOGCONFIG("GPSPPSNTP", "  GPS Status: Searching");
  }
  
  ESP_LOGCONFIG("GPSPPSNTP", "  PPS Status: %s", pps_active_ ? "ACTIVE" : "INACTIVE");
  ESP_LOGCONFIG("GPSPPSNTP", "  Time Uncertainty: %.6f s", time_uncertainty_);
}

void GPSPPSNTPServer::parse_gps_data() {
  // 读取串口数据
  while (available()) {
    uint8_t c = read();
    if (gps_.encode(c)) {
      // 当GPS解析出完整句子时，更新GPS时间
      update_gps_time();
    }
  }
}

void GPSPPSNTPServer::update_gps_time() {
  // 检查GPS是否有有效的时间和日期
  if (gps_.time.isValid() && gps_.date.isValid() && gps_.date.year() >= 2025) {
    // 转换为ESPTime结构
    ESPTime gps_time{};
    gps_time.year = gps_.date.year();
    gps_time.month = gps_.date.month();
    gps_time.day_of_month = gps_.date.day();
    gps_time.hour = gps_.time.hour();
    gps_time.minute = gps_.time.minute();
    gps_time.second = gps_.time.second();
    gps_time.day_of_week = 1;  // 简化处理
    gps_time.day_of_year = 1;  // 简化处理
    
    // 计算UTC时间戳
    gps_time.recalc_timestamp_utc(true);
    
    // 更新GPS时间基准
    if (gps_time.timestamp > 1609459200) {  // 确保在2021年之后
      gps_epoch_seconds_ = gps_time.timestamp;
      gps_epoch_micros_ = 0;
      last_gps_update_micros_ = micros();
      gps_locked_ = true;
      gps_updates_++;
      
      ESP_LOGI("GPSPPSNTP", "GPS time updated: %04d-%02d-%02d %02d:%02d:%02d UTC", 
               gps_time.year, gps_time.month, gps_time.day_of_month,
               gps_time.hour, gps_time.minute, gps_time.second);
    }
  }
}

void GPSPPSNTPServer::calculate_precise_time() {
  if (!gps_locked_ || !pps_active_) {
    time_uncertainty_ = 1.0f;  // 1秒不确定性
    return;
  }
  
  // 计算当前微秒偏移
  uint32_t now_micros = micros();
  uint32_t offset_since_pps = (now_micros - last_pps_micros_) & 0xFFFFFFFFUL;
  
  // 检查偏移是否合理
  if (offset_since_pps > 1100000) {  // 超过1.1秒
    time_uncertainty_ += 0.1f;  // 增加不确定性
    return;
  }
  
  // 计算本秒内的分数
  float fraction = offset_since_pps / 1000000.0f;
  
  // 使用卡尔曼滤波器平滑时间偏移
  float filtered_fraction = offset_filter_.update(fraction);
  
  // 计算时间不确定性
  float error = fabsf(fraction - filtered_fraction);
  accumulated_error_ = 0.9f * accumulated_error_ + 0.1f * error;
  time_uncertainty_ = accumulated_error_;
  
  ESP_LOGD("GPSPPSNTP", "Time offset: %.6f s, Filtered: %.6f s, Uncertainty: %.6f s",
           fraction, filtered_fraction, time_uncertainty_);
}

bool GPSPPSNTPServer::get_ntp_timestamp(uint32_t &seconds, uint32_t &fraction) {
  if (!gps_locked_ || !pps_active_) {
    // 回退到系统时间
    struct timeval tv;
    gettimeofday(&tv, NULL);
    seconds = tv.tv_sec + NTP_OFFSET;
    fraction = (tv.tv_usec * 4294967296UL) / US_PER_SECOND;
    return false;
  }
  
  // 计算当前精确时间
  uint32_t now_micros = micros();
  uint32_t offset_since_pps = (now_micros - last_pps_micros_) & 0xFFFFFFFFUL;
  
  // 确保偏移在合理范围内
  if (offset_since_pps > 1100000) {
    ESP_LOGW("GPSPPSNTP", "Large offset: %u us", offset_since_pps);
    offset_since_pps = offset_since_pps % US_PER_SECOND;
  }
  
  // 计算NTP时间戳
  seconds = gps_epoch_seconds_ + NTP_OFFSET + (offset_since_pps / US_PER_SECOND);
  fraction = ((offset_since_pps % US_PER_SECOND) * 4294967296UL) / US_PER_SECOND;
  
  return true;
}

void GPSPPSNTPServer::start_ntp_server() {
  if (!ntp_running_) {
    ntp_udp_.begin(NTP_PORT);
    ntp_running_ = true;
    ESP_LOGI("GPSPPSNTP", "NTP server started on port %d", NTP_PORT);
  }
}

void GPSPPSNTPServer::handle_ntp_request() {
  if (!ntp_running_) return;
  
  int packetSize = ntp_udp_.parsePacket();
  if (packetSize) {
    ntp_requests_++;
    
    // 读取请求数据
    ntp_udp_.read(ntp_packet_buffer_, NTP_PACKET_SIZE);
    IPAddress remote = ntp_udp_.remoteIP();
    int remotePort = ntp_udp_.remotePort();
    
    ESP_LOGD("GPSPPSNTP", "NTP request from %s:%d", remote.toString().c_str(), remotePort);
    
    // 获取当前精确时间
    uint32_t ntp_seconds, ntp_fraction;
    bool high_precision = get_ntp_timestamp(ntp_seconds, ntp_fraction);
    
    // 构建NTP响应包
    // LI, Version, Mode (服务器模式)
    ntp_packet_buffer_[0] = 0b00100100;
    
    // Stratum (时间层级)
    ntp_packet_buffer_[1] = high_precision ? 1 : 3;
    
    // Poll Interval
    ntp_packet_buffer_[2] = 6; // 64秒
    
    // Precision (精度)
    if (high_precision) {
      // 高精度模式：根据时间不确定性设置精度
      int8_t precision = (int8_t)log2f(time_uncertainty_);
      ntp_packet_buffer_[3] = precision < -20 ? 0xE0 : precision;
    } else {
      ntp_packet_buffer_[3] = 0xFA; // 2^-6 ≈ 15.6毫秒
    }
    
    // Root Delay和Root Dispersion（设置为0）
    memset(&ntp_packet_buffer_[4], 0, 8);
    
    // Reference Identifier（使用本机IP）
    IPAddress localIP = network::get_ip_addresses()[0];
    memcpy(&ntp_packet_buffer_[12], &localIP[0], 4);
    
    // Reference Timestamp（使用发送时间戳）
    for (int i = 0; i < 8; i++) {
      ntp_packet_buffer_[16 + i] = ntp_packet_buffer_[40 + i];
    }
    
    // Receive Timestamp（当前时间）
    ntp_packet_buffer_[32] = (ntp_seconds >> 24) & 0xFF;
    ntp_packet_buffer_[33] = (ntp_seconds >> 16) & 0xFF;
    ntp_packet_buffer_[34] = (ntp_seconds >> 8) & 0xFF;
    ntp_packet_buffer_[35] = ntp_seconds & 0xFF;
    
    ntp_packet_buffer_[36] = (ntp_fraction >> 24) & 0xFF;
    ntp_packet_buffer_[37] = (ntp_fraction >> 16) & 0xFF;
    ntp_packet_buffer_[38] = (ntp_fraction >> 8) & 0xFF;
    ntp_packet_buffer_[39] = ntp_fraction & 0xFF;
    
    // Transmit Timestamp（当前时间，再次）
    ntp_packet_buffer_[40] = (ntp_seconds >> 24) & 0xFF;
    ntp_packet_buffer_[41] = (ntp_seconds >> 16) & 0xFF;
    ntp_packet_buffer_[42] = (ntp_seconds >> 8) & 0xFF;
    ntp_packet_buffer_[43] = ntp_seconds & 0xFF;
    
    ntp_packet_buffer_[44] = (ntp_fraction >> 24) & 0xFF;
    ntp_packet_buffer_[45] = (ntp_fraction >> 16) & 0xFF;
    ntp_packet_buffer_[46] = (ntp_fraction >> 8) & 0xFF;
    ntp_packet_buffer_[47] = ntp_fraction & 0xFF;
    
    // 发送响应
    ntp_udp_.beginPacket(remote, remotePort);
    ntp_udp_.write(ntp_packet_buffer_, NTP_PACKET_SIZE);
    ntp_udp_.endPacket();
    
    ntp_responses_++;
    ESP_LOGD("GPSPPSNTP", "NTP response sent (Precision: %s)", 
             high_precision ? "HIGH" : "SYSTEM");
  }
}

}  // namespace gps_pps_ntp
}  // namespace esphome
