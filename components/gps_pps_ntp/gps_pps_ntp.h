#pragma once

#include "esphome.h"
#include <WiFiUdp.h>
#include "TinyGPS++.h"

namespace esphome {
namespace gps_pps_ntp {

class GPSPPSNTPServer : public Component, public uart::UARTDevice {
 public:
  GPSPPSNTPServer() = default;
  
  // 配置方法
  void set_pps_pin(uint8_t pin) { pps_pin_ = pin; }
  void set_gps_baud_rate(uint32_t baud) { gps_baud_rate_ = baud; }
  
  // 组件生命周期方法
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  // 获取当前状态
  bool is_gps_locked() const { return gps_locked_; }
  bool is_pps_active() const { return pps_active_; }
  
  // 修改：去掉const限定符，或者直接访问成员变量
  uint8_t get_satellites() { 
    // 直接从TinyGPSPlus对象获取卫星数量
    if (gps_.satellites.isValid()) {
      return gps_.satellites.value();
    }
    return 0;
  }
  
  float get_hdop() { 
    // 直接从TinyGPSPlus对象获取HDOP
    if (gps_.hdop.isValid()) {
      return gps_.hdop.hdop();
    }
    return 99.99f; // 无效值
  }
  
  // 时间质量指标
  float get_time_uncertainty() const { return time_uncertainty_; }

 protected:
  // PPS中断处理
  static void IRAM_ATTR pps_interrupt_handler();
  void handle_pps_interrupt();
  
  // GPS数据处理
  void parse_gps_data();
  void update_gps_time();
  
  // 时间计算
  void calculate_precise_time();
  bool get_ntp_timestamp(uint32_t &seconds, uint32_t &fraction);
  
  // NTP服务器
  void start_ntp_server();
  void handle_ntp_request();
  
  // 卡尔曼滤波器
  class KalmanFilter {
   public:
    KalmanFilter(float process_noise = 1e-5, float measurement_noise = 1e-4) 
      : Q(process_noise), R(measurement_noise) {}
    
    float update(float measurement) {
      // 预测
      x = x;
      P = P + Q;
      
      // 更新
      float K = P / (P + R);
      x = x + K * (measurement - x);
      P = (1 - K) * P;
      
      return x;
    }
    
    float get_value() const { return x; }
    
   private:
    float x = 0.0f;  // 估计值
    float P = 1.0f;  // 估计误差协方差
    float Q;         // 过程噪声协方差
    float R;         // 测量噪声协方差
  };
  
 private:
  // 硬件配置
  uint8_t pps_pin_;
  uint32_t gps_baud_rate_ = 9600;
  
  // GPS解析
  TinyGPSPlus gps_;
  WiFiUDP ntp_udp_;
  static constexpr int NTP_PORT = 123;
  static constexpr int NTP_PACKET_SIZE = 48;
  byte ntp_packet_buffer_[NTP_PACKET_SIZE];
  
  // 时间基准（volatile用于中断访问）
  volatile uint32_t last_pps_micros_ = 0;
  volatile uint32_t pps_count_ = 0;
  volatile bool pps_triggered_ = false;
  
  // GPS时间数据
  uint32_t gps_epoch_seconds_ = 0;      // GPS时间的UTC秒数
  uint32_t gps_epoch_micros_ = 0;       // GPS时间的微秒部分
  uint32_t last_gps_update_micros_ = 0; // 上次GPS更新时间
  
  // 状态标志
  bool gps_locked_ = false;
  bool pps_active_ = false;
  bool ntp_running_ = false;
  
  // 时间质量控制
  KalmanFilter offset_filter_;
  float time_uncertainty_ = 1.0f;  // 时间不确定性（秒）
  float accumulated_error_ = 0.0f; // 累积误差
  
  // 统计信息
  uint32_t ntp_requests_ = 0;
  uint32_t ntp_responses_ = 0;
  uint32_t gps_updates_ = 0;
  uint32_t pps_pulses_ = 0;
  
  // 静态实例指针（用于中断处理）
  static GPSPPSNTPServer* instance_;
  
  // NTP常数
  static constexpr uint32_t NTP_OFFSET = 2208988800UL;  // Unix到NTP的秒数差
  static constexpr uint32_t US_PER_SECOND = 1000000UL;
};

}  // namespace gps_pps_ntp
}  // namespace esphome
