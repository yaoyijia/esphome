#pragma once

#include "esphome.h"
#include <WiFiUdp.h>

namespace esphome {
namespace gps_ntp_server {

class GPSNTPServer : public Component {
 public:
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  void set_gps(gps::GPS *gps) { gps_ = gps; }  // 可选，仅用于显示状态
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // 状态查询
  bool is_pps_active() const { return pps_active_; }
  uint32_t get_pps_count() const { return pps_count_; }
  uint32_t get_ntp_requests() const { return ntp_requests_; }
  
 protected:
  static void IRAM_ATTR pps_interrupt_handler();
  
 private:
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
  
  // GPS（可选）
  gps::GPS *gps_ = nullptr;
  
  // 循环控制
  uint32_t last_loop_ = 0;
  
  // 实例指针
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome
