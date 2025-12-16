#include "esphome.h"
#include "esphome/components/network/util.h"
#include <WiFiUdp.h>

WiFiUDP Udp;

#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];

const unsigned long seventyYears = 2208988800UL; // Unix时间(1970)转NTP时间(1900)的秒数差

namespace esphome {
namespace ntp_server {

// 声明全局变量（需要在其他组件的.cpp文件中定义）
// 例如在 custom_gps_time.cpp 中: custom_gps_time::CustomGPSTime* global_custom_gps_time = nullptr;
// 在 pps_sensor.cpp 中: pps_sensor::PPSSensor* global_pps_sensor = nullptr;
extern custom_gps_time::CustomGPSTime* global_custom_gps_time;
extern pps_sensor::PPSSensor* global_pps_sensor;

void startNTP() {
  Udp.begin(NTP_PORT);
}

/**
 * 获取高精度NTP时间戳
 * 成功返回true，使用高精度源
 * 失败返回false，使用系统时间回退
 */
bool get_high_precision_timestamp(uint32_t &ntp_seconds, uint32_t &ntp_fraction) {
  // 检查全局组件指针是否有效
  if (global_custom_gps_time != nullptr && global_pps_sensor != nullptr) {
    uint32_t epoch_secs, epoch_micros;
    
    // 从自定义GPS时间组件获取基准时间
    if (global_custom_gps_time->get_precise_time(epoch_secs, epoch_micros)) {
      // 从PPS组件获取最近一次脉冲的微秒时钟
      uint32_t pps_micros = global_pps_sensor->get_last_pps_micros();
      uint32_t now_micros = micros();
      
      // 计算从最近PPS脉冲到现在的微秒偏移（处理计数器回绕）
      uint32_t offset_since_pps = (now_micros - pps_micros) & 0xFFFFFFFFUL;
      
      // 计算NTP时间戳
      // 秒部分 = UTC基准秒 + NTP偏移 + 本秒内偏移的秒数部分
      ntp_seconds = epoch_secs + seventyYears + (offset_since_pps / 1000000UL);
      
      // 分数秒部分 = 微秒剩余部分 * (2^32 / 1,000,000)
      ntp_fraction = ((offset_since_pps % 1000000UL) * 4294967296UL) / 1000000UL;
      
      #ifdef DEBUG
      Serial.print("[NTP] High-precision mode: ");
      Serial.print(ntp_seconds);
      Serial.print("s + ");
      Serial.println(ntp_fraction);
      #endif
      
      return true; // 成功使用高精度源
    }
  }
  
  // 回退：使用系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ntp_seconds = tv.tv_sec + seventyYears;
  ntp_fraction = (tv.tv_usec * 4294967296UL) / 1000000UL;
  
  #ifdef DEBUG
  Serial.println("[NTP] Fallback to system time");
  #endif
  
  return false; // 使用回退方案
}

void processNTP() {
  // 检查是否有NTP请求到达
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // 读取请求数据包
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

    Serial.print("[NTP] Request from ");
    Serial.println(Remote.toString());

    #ifdef DEBUG
    Serial.print("[NTP] Packet size: ");
    Serial.println(packetSize);
    Serial.print("[NTP] From ");
    Serial.print(Remote.toString());
    Serial.print(", port ");
    Serial.println(PortNum);
    #endif

    uint32_t tempval;
    uint32_t ntp_seconds, ntp_fraction;
    
    // === 核心：获取时间戳（高精度或回退）===
    bool use_high_precision = get_high_precision_timestamp(ntp_seconds, ntp_fraction);
    
    // 设置timestamp为计算出的NTP秒数
    time_t timestamp = ntp_seconds;

    // === 构建NTP响应包 ===
    
    // LI, Version, Mode (0b00=无警告, 0b100=版本4, 0b11=服务器模式)
    packetBuffer[0] = 0b00100100;
    
    // Stratum (时间层级)
    if (timestamp < seventyYears * 2) {
      packetBuffer[1] = 16; // 无效时间，强制客户端不同步
      Serial.println("[NTP] Bad time detected, stratum=16");
    } else {
      packetBuffer[1] = use_high_precision ? 1 : 4; // 高精度用1，普通用4
    }
    
    // Poll Interval (轮询间隔)
    packetBuffer[2] = 6; // 2^6 = 64秒
    
    // Precision (精度)
    packetBuffer[3] = 0xFA; // 2^-6 ≈ 15.6毫秒
    
    // Root Delay (根延迟)
    packetBuffer[4] = 0;
    packetBuffer[5] = 0;
    packetBuffer[6] = 8;
    packetBuffer[7] = 0;
    
    // Root Dispersion (根分散)
    packetBuffer[8] = 0;
    packetBuffer[9] = 0;
    packetBuffer[10] = 0xC;
    packetBuffer[11] = 0;
    
    #ifdef DEBUG
    Serial.print("[NTP] Timestamp: ");
    Serial.println(timestamp);
    #endif
    
    tempval = timestamp;
    
    // 设置RefID为本机IP地址
    IPAddress myIP = network::get_ip_addresses()[0];
    packetBuffer[12] = myIP[0];
    packetBuffer[13] = myIP[1];
    packetBuffer[14] = myIP[2];
    packetBuffer[15] = myIP[3];
    
    // Reference Timestamp (参考时间戳)
    packetBuffer[16] = (tempval >> 24) & 0xFF;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    packetBuffer[19] = (tempval) & 0xFF;
    
    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;
    
    // Originate Timestamp (复制客户端的时间戳)
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];
    
    // Receive Timestamp (接收时间戳)
    packetBuffer[32] = (tempval >> 24) & 0xFF;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    packetBuffer[35] = (tempval) & 0xFF;
    
    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;
    
    // === 关键：Transmit Timestamp (发送时间戳) ===
    // 使用我们计算的高精度时间戳
    packetBuffer[40] = (ntp_seconds >> 24) & 0xFF;
    packetBuffer[41] = (ntp_seconds >> 16) & 0xFF;
    packetBuffer[42] = (ntp_seconds >> 8) & 0xFF;
    packetBuffer[43] = (ntp_seconds) & 0xFF;
    
    packetBuffer[44] = (ntp_fraction >> 24) & 0xFF;
    packetBuffer[45] = (ntp_fraction >> 16) & 0xFF;
    packetBuffer[46] = (ntp_fraction >> 8) & 0xFF;
    packetBuffer[47] = (ntp_fraction) & 0xFF;
    
    // 发送响应包
    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
    
    #ifdef DEBUG
    Serial.print("[NTP] Response sent to ");
    Serial.println(Remote.toString());
    #endif
  }
}

// 全局标志，确保只初始化一次
bool first_loop_flag = true;

void startNTPIfNeeded() {
  if (first_loop_flag) {
    first_loop_flag = false;
    startNTP();
    Serial.println("[NTP] Server started on port 123");
  }
}

void NTP_Server::setup() {
  // 组件初始化
  Serial.println("[NTP] NTP Server component initialized");
}

void NTP_Server::loop() {
  // 确保NTP服务已启动
  startNTPIfNeeded();
  
  // 处理NTP请求
  processNTP();
}

} // namespace ntp_server
} // namespace esphome
