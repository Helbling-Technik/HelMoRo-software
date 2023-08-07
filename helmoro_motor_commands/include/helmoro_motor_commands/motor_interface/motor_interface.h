#pragma once

#include "helmoro_motor_commands/serial_port/serial_port.h"

class MotorInterface {
  public:
    // constructor
    explicit MotorInterface(const std::string port, const int baud);
    ~MotorInterface(void){};

    // public write functions
    bool SendDutyM1M2Command(uint8_t address, int16_t duty_m1, int16_t duty_m2);
    bool SendSpeedM1M2Command(uint8_t address, double speed_m1, double speed_m2);

    // public read functions
    bool GetMainBatteryVoltage(uint8_t address, double &battery_voltage);
    bool GetM1SpeedFiltered(uint8_t address, double &speed);
    bool GetM2SpeedFiltered(uint8_t address, double &speed);
    bool GetM1Speed(uint8_t address, double &speed);
    bool GetM2Speed(uint8_t address, double &speed);
    bool GetM1M2Speed(uint8_t address, double &speed1, double &speed2);
    bool GetM1Pos(uint8_t address, double &pos);
    bool GetM2Pos(uint8_t address, double &pos);
    bool GetM1M2Pos(uint8_t address, double &pos1, double &pos2);


  private:

    // serial port
    SerialPort serial_;
    uint8_t buffer_[256];
    uint8_t buffer_length_;
    void ClearBuffer(void);
    void AddToBuffer(uint8_t data);
    void AddToBuffer(uint16_t data);
    void AddToBuffer(int16_t data);
    void AddToBuffer(int32_t data);
    void AddCRCToBuffer(void);
    bool SendBuffer(void);
    bool SendBufferAndReceive(void* buffer_read, const uint8_t buffer_length, uint8_t &bytes_received);
    bool Receive(void *buffer_read, const uint8_t buffer_length, uint8_t &bytes_received);

    // crc
    uint16_t crc_;
    void UpdateCRC(uint8_t data);
    void ClearCRC(void);
};