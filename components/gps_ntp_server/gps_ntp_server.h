
#pragma once

#include "esphome.h"
#include <WiFiUdp.h>
#include <queue>

namespace esphome {
namespace gps_ntp_server {

class GPSNTPServer : public Component {
 public:
  void set_uart(esphome::uart::UARTComponent *uart) { uart_ = uart; }
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // 状态查询
  bool is_gps_valid() const { return gps_valid_; }
  bool is_pps_locked() const { return pps_locked_; }
  uint32_t get_pps_count() const { return pps_count_; }
  float get_time_accuracy() const { return time_accuracy_; }
  
  // 时间源质量
  enum TimeSource {
    TIME_SOURCE_NONE = 0,
    TIME_SOURCE_SYSTEM = 1,
    TIME_SOURCE_GPS = 2,
    TIME_SOURCE_PPS = 3,
    TIME_SOURCE_GPS_PPS = 4
  };
  
  TimeSource get_time_source() const;

 protected:
  void parse_nmea();
  void process_pps();
  void handle_ntp_request();
  void update_system_time();
  bool get_ntp_timestamp(uint32_t &seconds, uint32_t &fraction);
  
  // NMEA解析辅助函数
  bool parse_rmc(const char *data);
  bool parse_gga(const char *data);
  bool parse_zda(const char *data);
  
  // 时间计算函数
  uint32_t calculate_ntp_timestamp(uint32_t unix_seconds, uint32_t microseconds);
  void apply_pps_correction(uint32_t &seconds, uint32_t &fraction);
  
  // 中断服务例程
  static void IRAM_ATTR pps_interrupt_handler();
  
 private:
  // 硬件接口
  esphome::uart::UARTComponent *uart_ = nullptr;
  uint8_t pps_pin_ = 0;
  
  // GPS数据
  char nmea_buffer_[128];
  uint8_t nmea_index_ = 0;
  
  // 时间数据
  struct {
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint32_t microsecond = 0;
    bool valid = false;
    uint32_t last_update = 0;
  } gps_time_;
  
  // PPS数据
  struct {
    volatile uint32_t last_edge_us = 0;
    volatile uint32_t count = 0;
    volatile bool triggered = false;
    uint32_t last_stable_count = 0;
    float interval_avg = 1000000.0f;  // 平均间隔(us)
    float interval_std = 0.0f;        // 标准差
    bool locked = false;
  } pps_;
  
  // 系统时间校准
  struct {
    int64_t offset_ns = 0;           // 系统时间相对于真实时间的偏移(ns)
    float drift_ppm = 0.0f;          // 时钟漂移(ppm)
    uint32_t last_calibration = 0;
    float accuracy_ns = 1e9f;        // 时间精度(ns)
  } calibration_;
  
  // NTP服务器
  WiFiUDP ntp_socket_;
  bool ntp_running_ = false;
  
  // 状态
  bool gps_valid_ = false;
  bool pps_locked_ = false;
  float time_accuracy_ = 1.0f;  // 时间精度(秒)
  
  // 缓冲区
  std::queue<uint8_t> uart_queue_;
  
  // 统计
  uint32_t stats_ntp_requests_ = 0;
  uint32_t stats_gps_sentences_ = 0;
  
  // 实例指针（用于中断）
  static GPSNTPServer *instance_;
};

}  // namespace gps_ntp_server
}  // namespace esphome
