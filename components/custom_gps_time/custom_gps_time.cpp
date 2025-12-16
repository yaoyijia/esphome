#include "custom_gps_time.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custom_gps_time {

static const char *const TAG = "custom_gps_time";

void CustomGPSTime::setup() {
  // 向父GPS组件注册自己为监听器，以接收数据更新
  if (this->gps_parent_ != nullptr) {
    this->gps_parent_->register_listener(this);
    ESP_LOGI(TAG, "Registered listener to GPS");
  } else {
    ESP_LOGE(TAG, "GPS parent not set!");
  }
}

void CustomGPSTime::dump_config() { 
    LOG_TIME("Custom GPS Time", "Custom GPS Time", this); 
}

void CustomGPSTime::on_update(TinyGPSPlus &tiny_gps) {
  if (!this->has_time_) {
    this->sync_from_tiny_gps_(tiny_gps);
  }
}

void CustomGPSTime::sync_from_tiny_gps_(TinyGPSPlus &tiny_gps) {
  if (!tiny_gps.time.isValid() || !tiny_gps.date.isValid() || 
      !tiny_gps.time.isUpdated() || !tiny_gps.date.isUpdated() || 
      tiny_gps.date.year() < 2025) {
    return;
  }

  ESPTime val{};
  val.year = tiny_gps.date.year();
  val.month = tiny_gps.date.month();
  val.day_of_month = tiny_gps.date.day();
  val.day_of_week = 1;   // 占位，不用于计算
  val.day_of_year = 1;   // 占位，不用于计算
  val.hour = tiny_gps.time.hour();
  val.minute = tiny_gps.time.minute();
  val.second = tiny_gps.time.second();
  
  val.recalc_timestamp_utc(false);

  // === 核心操作：记录精确时刻 ===
  this->last_epoch_ = val.timestamp;          // 存储UTC秒
  this->last_epoch_micros_ = micros();        // 存储此刻的微秒时钟
  // =============================
  
  // 同步ESPhome系统时间（保持原有功能）
  this->synchronize_epoch_(val.timestamp);
  this->has_time_ = true;
  
  ESP_LOGD(TAG, "Time synced & recorded: Epoch=%lu, Micros=%u", 
           (unsigned long)val.timestamp, this->last_epoch_micros_);
}

}  // namespace custom_gps_time
}  // namespace esphome
