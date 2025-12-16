#include "esphome.h"
#include "esphome/components/network/util.h"
#include <WiFiUdp.h>

// === 新增：包含自定义组件头文件 ===
// 确保这些头文件路径正确，或在编译环境中可用
// #include "esphome/components/custom_gps_time/custom_gps_time.h"
// #include "esphome/components/pps_sensor/pps_sensor.h"

WiFiUDP Udp;

#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];

const unsigned long seventyYears = 2208988800UL; // to convert unix time to epoch

namespace esphome {
namespace ntp_server {

void startNTP() {
  Udp.begin(NTP_PORT);
}

// === 新增：高精度时间计算辅助函数 ===
uint32_t get_precise_ntp_timestamp(uint32_t &ntp_seconds, uint32_t &ntp_fraction) {
  // 尝试从高精度时间源获取基准
  // 请确保这里的ID与您YAML配置中设置的完全一致
  auto *time_comp = id(my_precise_time); // custom_gps_time 组件ID
  auto *pps_comp = id(my_pps_driver);    // pps_sensor 组件ID
  
  if (time_comp != nullptr && pps_comp != nullptr) {
    // 注意：需要确保您的custom_gps_time组件有get_precise_time方法
    // 且返回bool和两个uint32_t参数（秒和微秒）
    uint32_t epoch_secs, epoch_micros;
    if (time_comp->get_precise_time(epoch_secs, epoch_micros)) {
      uint32_t pps_micros = pps_comp->get_last_pps_micros();
      uint32_t now_micros = micros();
      
      // 计算从最近PPS脉冲到现在的微秒偏移（处理计数器回绕）
      uint32_t offset_since_pps = (now_micros - pps_micros) & 0xFFFFFFFFUL;
      
      // 计算NTP时间戳
      ntp_seconds = epoch_secs + seventyYears + (offset_since_pps / 1000000UL);
      ntp_fraction = ((offset_since_pps % 1000000UL) * 4294967296UL) / 1000000UL;
      
      // 调试信息
      #ifdef DEBUG
      Serial.print("High-precision NTP: ");
      Serial.print(ntp_seconds);
      Serial.print("s + ");
      Serial.print(ntp_fraction);
      Serial.println(" fraction");
      #endif
      
      return 1; // 成功使用高精度源
    }
  }
  
  // 回退到系统时间
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ntp_seconds = tv.tv_sec + seventyYears;
  ntp_fraction = (tv.tv_usec * 4294967296UL) / 1000000UL;
  
  #ifdef DEBUG
  Serial.println("Falling back to system time.");
  #endif
  
  return 0; // 使用回退方案
}

void processNTP() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

    Serial.print("NTP request from ");
    Serial.println(Remote.toString());

#ifdef DEBUG
    Serial.println();
    Serial.print("Received UDP packet size ");
    Serial.println(packetSize);
    Serial.print("From ");

    for (int i = 0; i < 4; i++) {
      Serial.print(Remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.print(PortNum);

    byte LIVNMODE = packetBuffer[0];
    Serial.print("  LI, Vers, Mode :");
    Serial.print(LIVNMODE, HEX);

    byte STRATUM = packetBuffer[1];
    Serial.print("  Stratum :");
    Serial.print(STRATUM, HEX);

    byte POLLING = packetBuffer[2];
    Serial.print("  Polling :");
    Serial.print(POLLING, HEX);

    byte PRECISION = packetBuffer[3];
    Serial.print("  Precision :");
    Serial.println(PRECISION, HEX);

    for (int z = 0; z < NTP_PACKET_SIZE; z++) {
      Serial.print(packetBuffer[z], HEX);
      if (((z + 1) % 4) == 0) {
        Serial.println();
      }
    }
    Serial.println();
#endif

    uint32_t tempval;
    // === 修改点：替换原有的时间戳获取方式 ===
    uint32_t ntp_seconds, ntp_fraction;
    
    // 使用高精度时间源（或回退到系统时间）
    get_precise_ntp_timestamp(ntp_seconds, ntp_fraction);
    
    // 设置timestamp为计算出的NTP秒数（用于后续部分代码）
    time_t timestamp = ntp_seconds; // 注意：这里timestamp已经是NTP时间格式
    
    // ===== 原有代码保持不变，但使用新的timestamp值 =====
    packetBuffer[0] = 0b00100100; // LI, Version, Mode

    // 检查时间有效性（简化检查）
    if (timestamp < seventyYears * 2) { // 如果时间早于1970年*2
      packetBuffer[1] = 16; // 强制不同步
      Serial.println("NTP Server likely has bad time - setting stratum to 16 to block sync.");
    } else {
      packetBuffer[1] = 4; // 推荐值，因为精度有限
    }

    packetBuffer[2] = 6;    // polling minimum
    packetBuffer[3] = 0xFA; // precision

    packetBuffer[4] = 0; // root delay
    packetBuffer[5] = 0;
    packetBuffer[6] = 8;
    packetBuffer[7] = 0;

    packetBuffer[8] = 0; // root dispersion
    packetBuffer[9] = 0;
    packetBuffer[10] = 0xC;
    packetBuffer[11] = 0;

#ifdef DEBUG
    Serial.println(timestamp);
    // print_date(gps);
#endif

    // === 修改点：使用新的时间戳变量 ===
    tempval = timestamp;

    // Set refid to IP address if not locked
    IPAddress myIP = network::get_ip_addresses()[0];
    packetBuffer[12] = myIP[0];
    packetBuffer[13] = myIP[1];
    packetBuffer[14] = myIP[2];
    packetBuffer[15] = myIP[3];

    // reference timestamp (使用新的时间戳)
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    packetBuffer[19] = (tempval) & 0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    // copy originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // receive timestamp (使用新的时间戳)
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    packetBuffer[35] = (tempval) & 0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // === 关键修改点：传输时间戳使用高精度计算的结果 ===
    // 使用独立计算的ntp_seconds和ntp_fraction，而不是重复使用tempval
    packetBuffer[40] = (ntp_seconds >> 24) & 0XFF;
    packetBuffer[41] = (ntp_seconds >> 16) & 0xFF;
    packetBuffer[42] = (ntp_seconds >> 8) & 0xFF;
    packetBuffer[43] = (ntp_seconds) & 0xFF;

    packetBuffer[44] = (ntp_fraction >> 24) & 0XFF;
    packetBuffer[45] = (ntp_fraction >> 16) & 0xFF;
    packetBuffer[46] = (ntp_fraction >> 8) & 0xFF;
    packetBuffer[47] = (ntp_fraction) & 0xFF;

    // Reply to the IP address and port that sent the NTP request
    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}

void NTP_Server::setup() {
}

bool first_loop_flag = true;

void NTP_Server::loop() {
  if (first_loop_flag) { // wifi must init first...
    first_loop_flag = false;
    startNTP();
  }

  processNTP();
}

} // namespace ntp_server
} // namespace esphome
