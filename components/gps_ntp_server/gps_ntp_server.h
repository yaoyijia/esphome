#pragma once

#include "esphome.h"
#include <WiFiUdp.h>

namespace esphome {
namespace gps_ntp_server {

class GPSNTPServer : public Component {
 public:
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  void set_gps(gps::GPS *gps) { gps_ = gps; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // 状态查询
  bool is_pps_active() const { return pps_active_; }
  uint32_t get_pps_count() const { return pps_count_; }
  uint32_t get_ntp_requests() const { return ntp_requests_; }
  float get_time_error() const { return time_discipline_.error_us / 1000.0f; }
  bool is_disciplining() const { return time_discipline_.disciplining; }
  float get_frequency_error() const { return frequency_error_ppm_; }
  
 protected:
  static void IRAM_ATTR pps_interrupt_handler();
  
 private:
  void send_ntp_response(WiFiUDP &udp, IPAddress remote, int remotePort, 
                        byte *clientTransmit);
  void handle_ntp_request();
  void handle_pps();
  void discipline_time();
  void update_system_time();
  void calibrate_microsecond_counter();
  uint64_t get_precise_time_us();
  
  // 安全的临界区保护函数
  void enter_critical();
  void exit_critical();
  
  // PPS相关
  uint8_t pps_pin_ = 0;
  volatile uint32_t pps_last_edge_us_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_triggered_ = false;
  bool pps_active_ = false;
  uint32_t pps_last_stable_ = 0;
  
  // 时间基准（基于PPS计数）
  volatile uint64_t pps_base_seconds_ = 0;  // PPS脉冲计数的秒部分
  volatile uint32_t last_pps_micros_ = 0;   // 最后一次PPS时的微秒计数器值
  volatile uint32_t last_sync_us_ = 0;      // 最后一次同步时的微秒时间
  
  // 微秒计数器校准
  float micros_calibration_factor_ = 1.0f;  // 微秒计数器校准因子
  float frequency_error_ppm_ = 0.0f;        // 频率误差（ppm）
  uint32_t last_calibration_us_ = 0;
  uint32_t calibration_interval_us_ = 10000000;  // 10秒校准一次
  
  // NTP服务器
  WiFiUDP udp_;
  uint32_t ntp_requests_ = 0;
  
  // GPS（可选，为了兼容性保留）
  gps::GPS *gps_ = nullptr;
  
  // 时间驯服相关
  struct {
    float error_us = 0.0f;           // 当前时间误差（微秒）
    float last_error = 0.0f;         // 上次误差
    uint32_t last_discipline = 0;    // 上次驯服时间
    bool disciplining = false;       // 是否正在驯服
    uint32_t discipline_count = 0;   // 驯服次数
    uint32_t skip_count = 0;         // 跳过次数
  } time_discipline_;
  
  // 循环控制
  uint32_t last_loop_ = 0;
  
  // 实例指针
  static GPSNTPServer *instance_;
  
  // 临界区状态
  volatile bool in_critical_ = false;
};

}  // namespace gps_ntp_server
}  // namespace esphome
