#include "XRobot.hpp"

XROBOT_IMU::XROBOT_IMU() {
     
}

XROBOT_IMU::~XROBOT_IMU() {}

uint8_t XROBOT_IMU::CalculateCRC8(const uint8_t *buf, size_t len, uint8_t crc)
{
    while (len-- > 0) 
    {
        crc = CRC8_TAB[(crc ^ *buf++) & 0xff];
    }
    return crc;
}

bool XROBOT_IMU::VerifyData(const uint8_t *buf, size_t len)
{
    if (len < 2) 
    {
        return false;
    }
    uint8_t expected = CalculateCRC8(buf, len - sizeof(uint8_t), 0xff);
    return expected == buf[len - sizeof(uint8_t)];
}

#define DATA_IMU_LENGTH sizeof(Data)

void XROBOT_IMU::ProcessPacket(uint8_t* data, uint16_t len)
{
    uint8_t prefix = data[0];
    if(prefix == 0xA5)
    {
      Data* imu_data = (Data*)data;
      if (VerifyData(data, DATA_IMU_LENGTH))
      {
        memcpy(&xrobot_data, imu_data, sizeof(Data));
      }
    }
    
}

