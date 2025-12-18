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
  
  // GPSListener接口 - 可选，用于状态显示
  void on_update(TinyGPSPlus &tiny_gps) override;
  
  // 状态查询
  bool is_pps_active() const { return pps_active_; }
  uint32_t get_pps_count() const { return pps_count_; }
  uint8_t get_time_quality() const;
  uint32_t get_ntp_requests() const { return ntp_requests_; }
  
  enum TimeQuality {
    QUALITY_NO_SYNC = 0,
    QUALITY_SYSTEM = 1,
    QUALITY_PPS = 2,
    QUALITY_GPS_PPS = 3
  };

 protected:
  void process_ntp_requests();
  void handle_pps_signal();
  static void IRAM_ATTR pps_interrupt_handler();
  
  // 精确时间获取和响应
  struct PreciseTimestamp {
    struct timeval system_time;  // 系统时间
    uint32_t micros_counter;     // micros()计数器
    uint32_t pps_edge_us;        // 最近的PPS边缘时间
    uint32_t pps_count;          // PPS计数
  };
  
  PreciseTimestamp get_precise_timestamp();
  void send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                         byte *client_transmit, const PreciseTimestamp &ts);
  
 private:
  gps::GPS *gps_ = nullptr;
  uint8_t pps_pin_ = 0;
  
  // PPS相关 - volatile用于中断
  volatile uint32_t pps_last_edge_us_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_triggered_ = false;
  bool pps_active_ = false;
  uint32_t pps_last_stable_ = 0;
  
  // PPS校准状态
  struct {
    bool calibrated = false;
    uint32_t last_calibration_us = 0;
    int32_t accumulated_offset_us = 0;  // 累计偏移
  } pps_calibration_;
  
  // NTP服务器
  WiFiUDP udp_;
  bool ntp_started_ = false;
  
  // 状态和统计
  uint32_t last_loop_ = 0;
  uint32_t ntp_requests_ = 0;
  uint32_t ntp_errors_ = 0;
  
  // 时间状态
  struct {
    bool system_time_valid = false;
    uint32_t last_system_check = 0;
    int32_t system_pps_offset_us = 0;  // 系统时间与PPS的偏移
  } time_state_;
  
  // GPS状态缓存
  struct {
    bool valid = false;
    uint32_t year = 0;
    uint32_t month = 0;
    uint32_t day = 0;
    uint32_t hour = 0;
    uint32_t minute = 0;
    uint32_t second = 0;
    uint32_t last_update = 0;
  } gps_status_;
  
  // 实例指针
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome
