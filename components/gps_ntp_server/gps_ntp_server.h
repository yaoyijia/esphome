#pragma once

#include "esphome.h"
#include <WiFiUdp.h>

namespace esphome {
namespace gps_ntp_server {

class GPSNTPServer : public Component {
 public:
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  void set_gps(gps::GPS *gps) { gps_ = gps; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // 状态查询
  bool is_pps_active() const { return pps_active_; }
  uint32_t get_pps_count() const { return pps_count_; }
  uint32_t get_ntp_requests() const { return ntp_requests_; }
  float get_time_error() const { return time_discipline_.error_us / 1000.0f; }
  
 protected:
  static void IRAM_ATTR pps_interrupt_handler();
  
 private:
  void send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                        byte *clientTransmit);
  void handle_ntp_request();
  void handle_pps();
  void discipline_time();
  void precise_discipline();  // 精密驯服函数
  
  // PPS相关
  uint8_t pps_pin_ = 0;
  volatile uint32_t pps_last_edge_us_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_triggered_ = false;
  bool pps_active_ = false;
  uint32_t pps_last_stable_ = 0;
  
  // NTP服务器
  WiFiUDP udp_;
  uint32_t ntp_requests_ = 0;
  
  // GPS（可选，为了兼容性保留）
  gps::GPS *gps_ = nullptr;
  
  // 精密时间驯服相关
  struct {
    float error_us = 0.0f;            // 当前时间误差（微秒）
    float accumulated_error_us = 0.0f; // 累积误差（微秒）
    float last_error_us = 0.0f;        // 上次误差（微秒）
    float error_rate_us = 0.0f;        // 误差变化率（微秒/秒）
    uint32_t last_discipline = 0;      // 上次驯服时间
    bool disciplining = false;         // 是否正在驯服
    uint32_t discipline_count = 0;     // 驯服次数
    uint32_t stable_count = 0;         // 稳定计数
    float clock_drift_ppm = 0.0f;      // 时钟漂移（PPM）
  } time_discipline_;
  
  // 循环控制
  uint32_t last_loop_ = 0;
  
  // 实例指针
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome
