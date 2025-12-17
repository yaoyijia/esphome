
#pragma once

#include "esphome.h"
#include <WiFiUdp.h>

namespace esphome {
namespace gps_ntp_server {

class GPSNTPServer : public Component {
 public:
  void set_uart(uart::UARTComponent *uart) { uart_ = uart; }
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // 状态查询方法
  bool is_gps_valid() const { return gps_valid_; }
  bool is_pps_active() const { return pps_active_; }
  uint32_t get_pps_count() const { return pps_count_; }
  uint8_t get_gps_hour() const { return gps_hour_; }
  uint8_t get_gps_minute() const { return gps_minute_; }
  uint8_t get_gps_second() const { return gps_second_; }
  int get_time_source() const;
  
  // 时间源质量枚举
  enum {
    TIME_SOURCE_NONE = 0,
    TIME_SOURCE_SYSTEM = 1,
    TIME_SOURCE_GPS = 2,
    TIME_SOURCE_PPS = 3,
    TIME_SOURCE_GPS_PPS = 4
  };

 protected:
  void parse_nmea();
  void handle_pps();
  void handle_ntp();
  bool get_ntp_time(uint32_t &seconds, uint32_t &fraction);
  
  // NMEA解析函数
  bool parse_gprmc(const char *data);
  bool parse_gpgga(const char *data);
  
  static void IRAM_ATTR pps_interrupt_handler();
  
 private:
  uart::UARTComponent *uart_ = nullptr;
  uint8_t pps_pin_ = 0;
  
  // GPS数据缓冲区
  char nmea_buffer_[128];
  uint8_t nmea_index_ = 0;
  
  // GPS时间数据
  uint8_t gps_hour_ = 0;
  uint8_t gps_minute_ = 0;
  uint8_t gps_second_ = 0;
  uint8_t gps_day_ = 0;
  uint8_t gps_month_ = 0;
  uint16_t gps_year_ = 0;
  bool gps_valid_ = false;
  uint32_t last_gps_update_ = 0;
  
  // PPS数据
  volatile uint32_t pps_last_edge_us_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_triggered_ = false;
  bool pps_active_ = false;
  uint32_t pps_last_stable_ = 0;
  
  // NTP服务器
  WiFiUDP udp_;
  bool ntp_running_ = false;
  
  // 状态
  uint32_t last_loop_ = 0;
  uint32_t last_status_log_ = 0;
  uint32_t ntp_requests_ = 0;
  
  // 实例指针（用于中断）
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome

