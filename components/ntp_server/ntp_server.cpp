#include "esphome.h"
#include "esphome/components/network/util.h"
#include <WiFiUdp.h>

WiFiUDP Udp;

#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];

const unsigned long seventyYears = 2208988800UL;

namespace esphome {
namespace ntp_server {

void startNTP() {
  Udp.begin(NTP_PORT);
}

void processNTP() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

    Serial.print("NTP request from ");
    Serial.println(Remote.toString());

#ifdef DEBUG
    // 调试代码保持不变
#endif

    uint32_t tempval;
    uint32_t ntp_seconds, ntp_fraction;
    
    // === 核心修改开始：直接获取高精度时间 ===
    // 方法1：尝试使用自定义高精度时间源
    bool use_high_precision = false;
    
    // 注意：这里假设你的组件ID是 "my_precise_time" 和 "my_pps_driver"
    // 如果不同，请修改为你的实际ID
    auto *time_comp = reinterpret_cast<esphome::custom_gps_time::CustomGPSTime*>(id(my_precise_time));
    auto *pps_comp = reinterpret_cast<esphome::pps_sensor::PPSSensor*>(id(my_pps_driver));
    
    if (time_comp != nullptr && pps_comp != nullptr) {
        uint32_t epoch_secs, epoch_micros;
        if (time_comp->get_precise_time(epoch_secs, epoch_micros)) {
            uint32_t pps_micros = pps_comp->get_last_pps_micros();
            uint32_t now_micros = micros();
            uint32_t offset_since_pps = (now_micros - pps_micros) & 0xFFFFFFFFUL;
            
            ntp_seconds = epoch_secs + seventyYears + (offset_since_pps / 1000000UL);
            ntp_fraction = ((offset_since_pps % 1000000UL) * 4294967296UL) / 1000000UL;
            use_high_precision = true;
            
            #ifdef DEBUG
            Serial.print("High-precision NTP: ");
            Serial.print(ntp_seconds);
            Serial.print("s + ");
            Serial.print(ntp_fraction);
            Serial.println(" fraction");
            #endif
        }
    }
    
    // 方法2：回退到系统时间
    if (!use_high_precision) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        ntp_seconds = tv.tv_sec + seventyYears;
        ntp_fraction = (tv.tv_usec * 4294967296UL) / 1000000UL;
        
        #ifdef DEBUG
        Serial.println("Falling back to system time.");
        #endif
    }
    // === 核心修改结束 ===
    
    // 使用计算出的ntp_seconds作为timestamp
    time_t timestamp = ntp_seconds;

    packetBuffer[0] = 0b00100100;

    if (timestamp < seventyYears * 2) {
      packetBuffer[1] = 16;
      Serial.println("NTP Server likely has bad time - setting stratum to 16 to block sync.");
    } else {
      packetBuffer[1] = 4;
    }

    packetBuffer[2] = 6;
    packetBuffer[3] = 0xFA;

    packetBuffer[4] = 0;
    packetBuffer[5] = 0;
    packetBuffer[6] = 8;
    packetBuffer[7] = 0;

    packetBuffer[8] = 0;
    packetBuffer[9] = 0;
    packetBuffer[10] = 0xC;
    packetBuffer[11] = 0;

#ifdef DEBUG
    Serial.println(timestamp);
#endif

    tempval = timestamp;

    IPAddress myIP = network::get_ip_addresses()[0];
    packetBuffer[12] = myIP[0];
    packetBuffer[13] = myIP[1];
    packetBuffer[14] = myIP[2];
    packetBuffer[15] = myIP[3];

    // reference timestamp
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    packetBuffer[19] = (tempval)&0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    // copy originate timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // receive timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    packetBuffer[35] = (tempval)&0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // === 关键：传输时间戳使用高精度计算的结果 ===
    packetBuffer[40] = (ntp_seconds >> 24) & 0XFF;
    packetBuffer[41] = (ntp_seconds >> 16) & 0xFF;
    packetBuffer[42] = (ntp_seconds >> 8) & 0xFF;
    packetBuffer[43] = (ntp_seconds)&0xFF;

    packetBuffer[44] = (ntp_fraction >> 24) & 0XFF;
    packetBuffer[45] = (ntp_fraction >> 16) & 0xFF;
    packetBuffer[46] = (ntp_fraction >> 8) & 0xFF;
    packetBuffer[47] = (ntp_fraction)&0xFF;

    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}

void NTP_Server::setup() {
}

bool first_loop_flag = true;

void NTP_Server::loop() {
  if (first_loop_flag) {
    first_loop_flag = false;
    startNTP();
  }

  processNTP();
}

} // namespace ntp_server
} // namespace esphome
