#ifndef NTP_SERVER_H
#define NTP_SERVER_H

#include "esphome/core/component.h"

namespace esphome {
namespace ntp_server {

class NTP_Server : public Component {
public:
  void set_time_component(CustomGPSTime *comp) { time_comp_ = comp; }
  void set_pps_component(PPSSensor *comp) { pps_comp_ = comp; }
  void setup() override; // called once
  void loop() override;  // called frequently
 protected:
  CustomGPSTime *time_comp_{nullptr};
  PPSSensor *pps_comp_{nullptr};
};

} // namespace ntp_server
} // namespace esphome

#endif
