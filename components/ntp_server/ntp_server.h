#pragma once

#include "esphome.h"

// 前向声明，避免循环包含
namespace esphome {
namespace custom_gps_time {
class CustomGPSTime;
}
namespace pps_sensor {
class PPSSensor;
}
}  // namespace esphome

namespace esphome {
namespace ntp_server {

class NTP_Server : public Component {
 public:
  void set_time_component(custom_gps_time::CustomGPSTime *comp) { time_comp_ = comp; }
  void set_pps_component(pps_sensor::PPSSensor *comp) { pps_comp_ = comp; }
  
  void setup() override;
  void loop() override;

 protected:
  // 获取高精度时间戳的内部方法
  bool get_high_precision_timestamp(uint32_t &ntp_seconds, uint32_t &ntp_fraction);
  
  // 启动NTP服务器
  void startNTP();
  
  // 处理NTP请求
  void processNTP();
  
  // 确保NTP服务已启动
  void startNTPIfNeeded();

 private:
  custom_gps_time::CustomGPSTime *time_comp_{nullptr};
  pps_sensor::PPSSensor *pps_comp_{nullptr};
  
  bool first_loop_flag_{true};
  
  // UDP服务器
  WiFiUDP udp_;
  
  // 包缓冲区
  static constexpr int NTP_PORT = 123;
  static constexpr int NTP_PACKET_SIZE = 48;
  byte packet_buffer_[NTP_PACKET_SIZE];
  
  // Unix时间(1970)转NTP时间(1900)的秒数差
  static constexpr unsigned long SEVENTY_YEARS = 2208988800UL;
};

}  // namespace ntp_server
}  // namespace esphome
