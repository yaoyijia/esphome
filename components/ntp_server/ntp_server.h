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
  custom_gps_time::CustomGPSTime *time_comp_{nullptr};
  pps_sensor::PPSSensor *pps_comp_{nullptr};
  
  friend void processNTP(); // 让processNTP函数可以访问私有成员
};

}  // namespace ntp_server
}  // namespace esphome
