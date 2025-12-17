
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
  
  // GPSListener接口
  void on_update(TinyGPSPlus &tiny_gps) override;
  
  // 状态查询
  bool is_gps_valid() const { return gps_time_.valid; }
  bool is_pps_active() const { return pps_active_; }
  uint32_t get_pps_count() const { return pps_count_; }
  uint8_t get_time_quality() const;
  
  enum TimeQuality {
    QUALITY_NO_SYNC = 0,
    QUALITY_SYSTEM = 1,
    QUALITY_GPS = 2,
    QUALITY_PPS = 3,
    QUALITY_GPS_PPS = 4
  };

 protected:
  void process_ntp();
  void handle_pps();
  static void IRAM_ATTR pps_interrupt_handler();
  
private:
  gps::GPS *gps_ = nullptr;
  uint8_t pps_pin_ = 0;
  
  // GPS时间数据 - 命名结构体
  struct GPSTimeData {
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    bool valid = false;
    uint32_t last_update = 0;
  } gps_time_;
  
  // PPS数据
  volatile uint32_t pps_last_edge_us_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_triggered_ = false;
  bool pps_active_ = false;
  uint32_t pps_last_stable_ = 0;
  
  // 同步状态
  bool wait_for_pps_ = false;
  bool gps_updated_since_pps_ = false;
  bool time_valid_ = false;
  uint32_t last_sync_time_ = 0;
  uint32_t pps_at_last_sync_ = 0;
  
  // 同步时间数据 - 使用已命名的结构体
  GPSTimeData sync_gps_time_;
  
  // NTP服务器
  WiFiUDP udp_;
  bool ntp_started_ = false;
  
  // 统计
  uint32_t last_loop_ = 0;
  uint32_t ntp_requests_ = 0;
  
  // 实例指针
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome

