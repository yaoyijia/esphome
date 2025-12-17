#pragma once

#include "esphome.h"
#include <WiFiUdp.h>
#include <time.h>  // 添加time.h头文件

namespace esphome {
namespace simple_gps_ntp {

class SimpleGPSNTPServer : public Component {
 public:
  void set_uart(esphome::uart::UARTComponent *uart) { uart_ = uart; }
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  void parse_gps();
  void parse_rmc_sentence(char* sentence);  // 新增：解析RMC语句
  void calculate_unix_timestamp();          // 新增：计算Unix时间戳
  void handle_pps();
  void handle_ntp();
  bool get_ntp_time(uint32_t &seconds, uint32_t &fraction);
  
  static void IRAM_ATTR pps_isr();
  
 private:
  esphome::uart::UARTComponent *uart_ = nullptr;
  uint8_t pps_pin_ = 0;
  
  // GPS数据
  char gps_buffer_[256];  // 增大缓冲区
  uint8_t gps_idx_ = 0;
  
  // 日期和时间数据
  uint16_t gps_year_ = 0;      // 新增：年份
  uint8_t gps_month_ = 0;      // 新增：月份
  uint8_t gps_day_ = 0;        // 新增：日
  uint8_t gps_hour_ = 0;       // 新增：时
  uint8_t gps_minute_ = 0;     // 新增：分
  uint8_t gps_second_ = 0;     // 新增：秒
  
  uint32_t gps_seconds_ = 0;   // 当天从0:00:00开始的秒数
  uint32_t gps_timestamp_ = 0; // Unix时间戳（从1970-01-01开始的秒数）
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
