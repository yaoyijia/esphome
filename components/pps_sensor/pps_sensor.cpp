#include "pps_sensor.h"

namespace esphome {
namespace pps_sensor {

PPSSensor *PPSSensor::instance_ = nullptr;

// 中断服务程序（必须简洁！）
void IRAM_ATTR PPSSensor::pps_interrupt_handler() {
  if (PPSSensor::instance_ != nullptr) {
    uint32_t now = micros();
    PPSSensor::instance_->pps_interval_us_ = now - PPSSensor::instance_->last_pps_micros_;
    PPSSensor::instance_->last_pps_micros_ = now;
    PPSSensor::instance_->pulse_count_++;
    PPSSensor::instance_->pps_updated_ = true;
  }
}

void PPSSensor::setup() {
  // 设置静态实例，以便中断访问
  PPSSensor::instance_ = this;
  
  // 配置引脚和中断
  pinMode(this->pps_pin_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(this->pps_pin_), 
                  &PPSSensor::pps_interrupt_handler, 
                  FALLING); // 根据你的实际连接使用 RISING 或 FALLING
  
  ESP_LOGI("PPS", "PPS sensor initialized on GPIO %d", this->pps_pin_);
}

void PPSSensor::update() {
  // 检查是否有新的PPS脉冲
  if (this->pps_updated_ && this->interval_sensor_ != nullptr) {
    this->pps_updated_ = false;
    
    // 计算间隔（秒）
    float interval_s = this->pps_interval_us_ / 1000000.0f;
    
    // 通过传感器对象发布状态
    this->interval_sensor_->publish_state(interval_s);
    
    // 可选：调试日志
    static uint32_t last_log = 0;
    if (millis() - last_log > 5000) {
      ESP_LOGI("PPS", "Interval: %.6fs (Error: %.3f ms)", 
               interval_s, 
               (interval_s - 1.0) * 1000.0);
      last_log = millis();
    }
  }
}

}  // namespace pps_sensor
}  // namespace esphome
