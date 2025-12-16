#pragma once

#include "esphome/components/gps/gps.h"
#include "esphome/core/component.h"
#include "TinyGPS++.h"

namespace esphome {
namespace custom_gps_time {

class CustomGPSTime : public Component, public gps::GPSListener {
 public:
  void set_gps_parent(gps::GPS *parent) { this->gps_parent_ = parent; }
  
  // 供NTP服务器调用的核心方法
  bool get_precise_time(uint32_t &epoch_seconds, uint32_t &epoch_micros) const {
    if (last_epoch_ > 0) {
      epoch_seconds = last_epoch_;
      epoch_micros = last_epoch_micros_;
      return true;
    }
    return false;
  }

  void setup() override;
  void dump_config() override;
  void on_update(TinyGPSPlus &tiny_gps) override;

 protected:
  void sync_from_tiny_gps_(TinyGPSPlus &tiny_gps);
  
  gps::GPS *gps_parent_{nullptr};
  
  // 存储的精确时间基准
  uint32_t last_epoch_{0};
  uint32_t last_epoch_micros_{0};
};

// 声明全局变量（用于其他组件访问）
extern CustomGPSTime *global_custom_gps_time;

}  // namespace custom_gps_time
}  // namespace esphome
