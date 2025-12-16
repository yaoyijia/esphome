#include "pps_sensor.h"

namespace esphome {
namespace pps_sensor {

PPSSensor *PPSSensor::instance_ = nullptr;

// **中断服务程序 (ISR)**：必须极其精简！
void IRAM_ATTR PPSSensor::pps_interrupt_handler() {
  if (instance_ != nullptr) {
    uint32_t now = micros(); // 获取当前微秒时钟
    instance_->pps_interval_us_ = now - instance_->last_pps_micros_;
    instance_->last_pps_micros_ = now;
    instance_->pulse_count_++; // 增加脉冲计数
    instance_->pps_updated_ = true; // 设置更新标志
  }
}

void PPSSensor::setup() {
  instance_ = this; // 设置静态实例
  pinMode(this->pps_pin_, INPUT_PULLUP); // 配置为上拉输入
  // 绑定中断：在PPS引脚上升沿触发，调用中断处理函数
  attachInterrupt(digitalPinToInterrupt(this->pps_pin_), &PPSSensor::pps_interrupt_handler, FALLING);
  ESP_LOGI("PPS", "PPS sensor initialized on GPIO %d", this->pps_pin_);
}

void PPSSensor::update() {
  // 检查是否有新的PPS脉冲
  if (this->pps_updated_) {
    this->pps_updated_ = false;
    
    // 计算并发布脉冲间隔（单位：秒），理想值应为1.0
    float interval_s = this->pps_interval_us_ / 1000000.0f;
    this->publish_state(interval_s); // 作为传感器状态发布
    
    // 记录日志（可调低频率以免刷屏）
    static uint32_t last_log = 0;
    if (millis() - last_log > 5000) { // 每5秒日志一次
      ESP_LOGI("PPS", "Interval: %.6fs (Error: %.3f ms)", 
               interval_s, 
               (interval_s - 1.0) * 1000.0);
      last_log = millis();
    }
  }
}

}  // namespace pps_sensor
}  // namespace esphome
