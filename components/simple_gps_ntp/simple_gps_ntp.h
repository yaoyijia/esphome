#pragma once

#include "esphome.h"
#include <WiFiUdp.h>

namespace esphome {
namespace simple_gps_ntp {

class SimpleGPSNTPServer : public Component {
 public:
  void set_uart(esphome::uart::UARTComponent *uart) { uart_ = uart; }
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  void set_debug_level(uint8_t level) { debug_level_ = level; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;

  // 公共方法，供YAML配置使用
  bool is_gps_valid() { return gps_valid_; }
  bool is_pps_active() { return pps_active_; }
  uint32_t get_pps_count() { return pps_count_; }
  uint8_t get_gps_hour() { return gps_hour_; }
  uint8_t get_gps_minute() { return gps_minute_; }
  uint8_t get_gps_second() { return gps_second_; }
  
 protected:
  void parse_gps();
  void handle_pps();
  void handle_ntp();
  bool get_ntp_time(uint32_t &seconds, uint32_t &fraction);
  
  static void IRAM_ATTR pps_isr();
  
 private:
  esphome::uart::UARTComponent *uart_ = nullptr;
  uint8_t pps_pin_ = 0;
  uint8_t debug_level_ = 1;
  
  char gps_buffer_[256];
  uint8_t gps_idx_ = 0;
  
  uint8_t gps_hour_ = 0;
  uint8_t gps_minute_ = 0;
  uint8_t gps_second_ = 0;
  uint8_t gps_day_ = 0;
  uint8_t gps_month_ = 0;
  uint16_t gps_year_ = 0;
  bool gps_valid_ = false;
  
  // 添加缺失的变量
  float time_calibration_ = 0.0f;
  
  volatile uint32_t last_pps_us_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_active_ = false;
  
  WiFiUDP udp_;
  bool ntp_started_ = false;
  
  uint32_t last_loop_ = 0;
  uint32_t last_ntp_log_ = 0;
  
  static SimpleGPSNTPServer *instance_;
};

}  // namespace simple_gps_ntp
}  // namespace esphome
