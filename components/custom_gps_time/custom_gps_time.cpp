#include "custom_gps_time.h"
#include "esphome/core/log.h"
#include "esphome/components/time/real_time_clock.h"  // 添加此头文件以使用ESPTime

namespace esphome {
namespace custom_gps_time {

static const char *const TAG = "custom_gps_time";

// 定义全局变量（必须在函数定义之前）
CustomGPSTime *global_custom_gps_time = nullptr;

void CustomGPSTime::setup() {
  if (this->gps_parent_ != nullptr) {
    this->gps_parent_->register_listener(this);
    ESP_LOGI(TAG, "Registered listener to GPS");
  } else {
    ESP_LOGE(TAG, "GPS parent not set!");
  }
  
  // 设置全局变量引用
  global_custom_gps_time = this;
}

void CustomGPSTime::dump_config() { 
  // 使用正确的日志宏
  ESP_LOGCONFIG(TAG, "Custom GPS Time");
}

void CustomGPSTime::on_update(TinyGPSPlus &tiny_gps) {
  // 移除 has_time_ 检查，直接尝试同步
  this->sync_from_tiny_gps_(tiny_gps);
}

void CustomGPSTime::sync_from_tiny_gps_(TinyGPSPlus &tiny_gps) {
  if (!tiny_gps.time.isValid() || !tiny_gps.date.isValid() || 
      !tiny_gps.time.isUpdated() || !tiny_gps.date.isUpdated() || 
      tiny_gps.date.year() < 2025) {
    return;
  }

  // 使用ESPTime结构体
  ESPTime val{};
  val.year = tiny_gps.date.year();
  val.month = tiny_gps.date.month();
  val.day_of_month = tiny_gps.date.day();
  val.day_of_week = 1;   // 占位
  val.day_of_year = 1;   // 占位
  val.hour = tiny_gps.time.hour();
  val.minute = tiny_gps.time.minute();
  val.second = tiny_gps.time.second();
  
  val.recalc_timestamp_utc(false);

  // 记录精确时刻（添加校验）
  if (val.timestamp > 1609459200) {  // 确保时间在2021年之后
    this->last_epoch_ = val.timestamp;          // 存储UTC秒
    this->last_epoch_micros_ = micros();        // 存储此刻的微秒时钟
    
  ESP_LOGD(TAG, "Precise time recorded: Epoch=%lu, Micros=%u", 
           (unsigned long)val.timestamp, this->last_epoch_micros_);
}

}  // namespace custom_gps_time
}  // namespace esphome
