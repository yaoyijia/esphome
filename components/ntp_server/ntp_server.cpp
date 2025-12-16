#include "ntp_server.h"
#include "esphome/components/network/util.h"
#include <WiFiUdp.h>

// 包含相关组件的头文件
// 如果组件在同一目录下，使用相对路径
#include "../custom_gps_time/custom_gps_time.h"
#include "../pps_sensor/pps_sensor.h"

namespace esphome {
namespace ntp_server {

void NTP_Server::setup() {
  // 组件初始化
  ESP_LOGI("NTP", "NTP Server component initialized");
}

void NTP_Server::startNTP() {
  udp_.begin(NTP_PORT);
  ESP_LOGI("NTP", "NTP server started on port %d", NTP_PORT);
}

void NTP_Server::startNTPIfNeeded() {
  if (first_loop_flag_) {
    first_loop_flag_ = false;
    startNTP();
  }
}

/**
 * 获取高精度NTP时间戳
 * 成功返回true，使用高精度源
 * 失败返回false，使用系统时间回退
 */
bool NTP_Server::get_high_precision_timestamp(uint32_t &ntp_seconds, uint32_t &ntp_fraction) {
  // 检查组件指针是否有效
  if (time_comp_ != nullptr && pps_comp_ != nullptr) {
    uint32_t epoch_secs, epoch_micros;
    
    // 从自定义GPS时间组件获取基准时间
    if (time_comp_->get_precise_time(epoch_secs, epoch_micros)) {
      // 从PPS组件获取最近一次脉冲的微秒时钟
      uint32_t pps_micros = pps_comp_->get_last_pps_micros();
      uint32_t now_micros = micros();
      
      // 计算从最近PPS脉冲到现在的微秒偏移（处理计数器回绕）
      uint32_t offset_since_pps = (now_micros - pps_micros) & 0xFFFFFFFFUL;
      
      // 计算NTP时间戳
      // 秒部分 = UTC基准秒 + NTP偏移 + 本秒内偏移的秒数部分
      ntp_seconds = epoch_secs + SEVENTY_YEARS + (offset_since_pps / 1000000UL);
      
      // 分数秒部分 = 微秒剩余部分 * (2^32 / 1,000,000)
      ntp_fraction = ((offset_since_pps % 1000000UL) * 4294967296UL) / 1000000UL;
      
      ESP_LOGD("NTP", "High-precision mode: %us + %u fraction", ntp_seconds, ntp_fraction);
      
      return true; // 成功使用高精度源
    }
  }
  
  // 回退：使用系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ntp_seconds = tv.tv_sec + SEVENTY_YEARS;
  ntp_fraction = (tv.tv_usec * 4294967296UL) / 1000000UL;
  
  ESP_LOGD("NTP", "Fallback to system time");
  
  return false; // 使用回退方案
}

void NTP_Server::processNTP() {
  // 检查是否有NTP请求到达
  int packetSize = udp_.parsePacket();
  if (packetSize) {
    // 读取请求数据包
    udp_.read(packet_buffer_, NTP_PACKET_SIZE);
    IPAddress remote = udp_.remoteIP();
    int portNum = udp_.remotePort();

    ESP_LOGI("NTP", "Request from %s:%d", remote.toString().c_str(), portNum);

    uint32_t tempval;
    uint32_t ntp_seconds, ntp_fraction;
    
    // === 核心：获取时间戳（高精度或回退）===
    bool use_high_precision = get_high_precision_timestamp(ntp_seconds, ntp_fraction);
    
    // 设置timestamp为计算出的NTP秒数
    time_t timestamp = ntp_seconds;

    // === 构建NTP响应包 ===
    
    // LI, Version, Mode (0b00=无警告, 0b100=版本4, 0b11=服务器模式)
    packet_buffer_[0] = 0b00100100;
    
    // Stratum (时间层级)
    if (timestamp < SEVENTY_YEARS * 2) {
      packet_buffer_[1] = 16; // 无效时间，强制客户端不同步
      ESP_LOGW("NTP", "Bad time detected, stratum=16");
    } else {
      packet_buffer_[1] = use_high_precision ? 1 : 4; // 高精度用1，普通用4
    }
    
    // Poll Interval (轮询间隔)
    packet_buffer_[2] = 6; // 2^6 = 64秒
    
    // Precision (精度)
    packet_buffer_[3] = 0xFA; // 2^-6 ≈ 15.6毫秒
    
    // Root Delay (根延迟)
    packet_buffer_[4] = 0;
    packet_buffer_[5] = 0;
    packet_buffer_[6] = 8;
    packet_buffer_[7] = 0;
    
    // Root Dispersion (根分散)
    packet_buffer_[8] = 0;
    packet_buffer_[9] = 0;
    packet_buffer_[10] = 0xC;
    packet_buffer_[11] = 0;
    
    tempval = timestamp;
    
    // 设置RefID为本机IP地址
    IPAddress myIP = network::get_ip_addresses()[0];
    packet_buffer_[12] = myIP[0];
    packet_buffer_[13] = myIP[1];
    packet_buffer_[14] = myIP[2];
    packet_buffer_[15] = myIP[3];
    
    // Reference Timestamp (参考时间戳)
    packet_buffer_[16] = (tempval >> 24) & 0xFF;
    packet_buffer_[17] = (tempval >> 16) & 0xFF;
    packet_buffer_[18] = (tempval >> 8) & 0xFF;
    packet_buffer_[19] = (tempval) & 0xFF;
    
    packet_buffer_[20] = 0;
    packet_buffer_[21] = 0;
    packet_buffer_[22] = 0;
    packet_buffer_[23] = 0;
    
    // Originate Timestamp (复制客户端的时间戳)
    packet_buffer_[24] = packet_buffer_[40];
    packet_buffer_[25] = packet_buffer_[41];
    packet_buffer_[26] = packet_buffer_[42];
    packet_buffer_[27] = packet_buffer_[43];
    packet_buffer_[28] = packet_buffer_[44];
    packet_buffer_[29] = packet_buffer_[45];
    packet_buffer_[30] = packet_buffer_[46];
    packet_buffer_[31] = packet_buffer_[47];
    
    // Receive Timestamp (接收时间戳)
    packet_buffer_[32] = (tempval >> 24) & 0xFF;
    packet_buffer_[33] = (tempval >> 16) & 0xFF;
    packet_buffer_[34] = (tempval >> 8) & 0xFF;
    packet_buffer_[35] = (tempval) & 0xFF;
    
    packet_buffer_[36] = 0;
    packet_buffer_[37] = 0;
    packet_buffer_[38] = 0;
    packet_buffer_[39] = 0;
    
    // === 关键：Transmit Timestamp (发送时间戳) ===
    // 使用我们计算的高精度时间戳
    packet_buffer_[40] = (ntp_seconds >> 24) & 0xFF;
    packet_buffer_[41] = (ntp_seconds >> 16) & 0xFF;
    packet_buffer_[42] = (ntp_seconds >> 8) & 0xFF;
    packet_buffer_[43] = (ntp_seconds) & 0xFF;
    
    packet_buffer_[44] = (ntp_fraction >> 24) & 0xFF;
    packet_buffer_[45] = (ntp_fraction >> 16) & 0xFF;
    packet_buffer_[46] = (ntp_fraction >> 8) & 0xFF;
    packet_buffer_[47] = (ntp_fraction) & 0xFF;
    
    // 发送响应包
    udp_.beginPacket(remote, portNum);
    udp_.write(packet_buffer_, NTP_PACKET_SIZE);
    udp_.endPacket();
    
    ESP_LOGD("NTP", "Response sent to %s", remote.toString().c_str());
  }
}

void NTP_Server::loop() {
  // 确保NTP服务已启动
  startNTPIfNeeded();
  
  // 处理NTP请求
  processNTP();
}

} // namespace ntp_server
} // namespace esphome
