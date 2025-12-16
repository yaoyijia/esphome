#pragma once

#include "esphome.h"
#include <WiFiUdp.h>

namespace esphome {
namespace simple_gps_ntp {

class SimpleGPSNTPServer : public Component {
 public:
  // 修改方法签名，使用UARTComponent
  void set_uart(esphome::uart::UARTComponent *uart) { uart_ = uart; }
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  void parse_gps();
  void handle_pps();
  void handle_ntp();
  bool get_ntp_time(uint32_t &seconds, uint32_t &fraction);
  
  static void IRAM_ATTR pps_isr();
  
 private:
  esphome::uart::UARTComponent *uart_ = nullptr;  // 使用 UARTComponent
  uint8_t pps_pin_ = 0;
  
  // GPS数据
  char gps_buffer_[128];  // 减小缓冲区大小
  uint8_t gps_idx_ = 0;
  
  // 时间数据
  uint32_t gps_seconds_ = 0;
  uint32_t last_pps_us_ = 0;
  uint32_t pps_count_ = 0;
  bool gps_valid_ = false;
  bool pps_active_ = false;
  
  // NTP
  WiFiUDP udp_;
  bool ntp_started_ = false;
  
  // 状态
  uint32_t last_loop_ = 0;
  
  static SimpleGPSNTPServer *instance_;
};

}  // namespace simple_gps_ntp
}  // namespace esphome
