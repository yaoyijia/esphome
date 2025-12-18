#pragma once

#include "esphome.h"
#include "esphome/components/gps/gps.h"
#include <WiFiUdp.h>

namespace esphome {
namespace gps_ntp_server {

class GPSNTPServer : public Component, public gps::GPSListener {
 public:
  void set_gps(gps::GPS *gps) { 
    gps_ = gps;
    if (gps_) {
      gps_->register_listener(this);
    }
  }
  
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // GPSListener接口 - 可选，仅用于状态显示
  void on_update(TinyGPSPlus &tiny_gps) override;
  
  // 状态查询
  bool is_pps_active() const { return pps_active_; }
  uint32_t get_pps_count() const { return pps_count_; }
  uint8_t get_time_quality() const;
  
  enum TimeQuality {
    QUALITY_NO_SYNC = 0,
    QUALITY_SYSTEM = 1,
    QUALITY_PPS = 2,
    QUALITY_GPS_PPS = 3
  };

 protected:
  void process_ntp();
  void handle_pps();
  static void IRAM_ATTR pps_interrupt_handler();
  
  // 简单的NTP响应函数
  void send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                         byte *clientTransmit);
  
 private:
  gps::GPS *gps_ = nullptr;
  uint8_t pps_pin_ = 0;
  
  // PPS相关
  volatile uint32_t pps_last_edge_us_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_triggered_ = false;
  bool pps_active_ = false;
  uint32_t pps_last_stable_ = 0;
  
  // NTP服务器
  WiFiUDP udp_;
  bool ntp_started_ = false;
  
  // 状态
  uint32_t last_loop_ = 0;
  uint32_t ntp_requests_ = 0;
  
  // PPS校准状态
  struct {
    bool calibrated = false;
    uint32_t last_calibration_us = 0;
    int32_t drift_accumulated_us = 0;  // 漂移累计
  } pps_calibration_;
  
  // 实例指针
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome
