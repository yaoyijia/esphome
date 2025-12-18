#pragma once

#include "esphome.h"
#include "esphome/components/gps/gps.h"
#include <WiFiUdp.h>

namespace esphome {
namespace gps_ntp_server {

class GPSNTPServer : public Component, public gps::GPSListener {
 public:
  void set_gps(gps::GPS *gps);
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  void on_update(TinyGPSPlus &tiny_gps) override;
  
  bool is_gps_valid() const { return gps_valid_; }
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
  void sync_system_time_with_gps_and_pps();
  
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
  bool gps_valid_ = false;
  uint32_t last_gps_update_ = 0;
  uint32_t last_loop_ = 0;
  uint32_t ntp_requests_ = 0;
  
  // GPS时间缓存（用于PPS同步）
  struct {
    bool valid = false;
    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;  // GPS的整秒部分
    time_t epoch;     // Unix时间戳
  } gps_time_cache_;
  
  // PPS同步状态
  struct {
    bool awaiting_sync = false;      // 等待PPS同步
    bool synced_once = false;        // 至少成功同步过一次
    uint32_t last_sync_millis = 0;   // 上次同步时间
    int64_t accumulated_offset_us = 0; // 累计的微秒偏移
  } pps_sync_;
  
  // 实例指针
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome
