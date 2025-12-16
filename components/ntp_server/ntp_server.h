#pragma once

namespace esphome {
namespace ntp_server {

class NTP_Server : public Component {
 public:
  void setup() override;
  void loop() override;
};

}  // namespace ntp_server
}  // namespace esphome
