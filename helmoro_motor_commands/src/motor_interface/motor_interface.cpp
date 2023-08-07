#include "helmoro_motor_commands/motor_interface/motor_interface.h"

#include "helmoro_motor_commands/motor_interface/motor_command_types.h"

// STL
#include <poll.h>

// constructor
MotorInterface::MotorInterface(const std::string port, const int baud) {
  // serial port
  if (!serial_.Initialize(port, baud)) {
    std::printf("[%s] Failed to initialize serial port.\n", __func__);
  }
}

// public write functions
bool MotorInterface::SendDutyM1M2Command(uint8_t address, int16_t duty_m1, int16_t duty_m2) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kMixedDuty);
  AddToBuffer(duty_m1);
  AddToBuffer(duty_m2);
  AddCRCToBuffer();

  const uint8_t buffer_length = 1;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) {
    if (bytes_received > 0 && buffer_read[0] == 0xFF) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }
}

bool MotorInterface::SendSpeedM1M2Command(uint8_t address, double speed_m1, double speed_m2) {

  int32_t speed_int_m1 = (int32_t) (speed_m1);
  int32_t speed_int_m2 = (int32_t) (speed_m2);

  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kMixedSpeed);
  AddToBuffer(speed_int_m1);
  AddToBuffer(speed_int_m2);
  AddCRCToBuffer();

  const uint8_t buffer_length = 1;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) {
    if (bytes_received > 0 && buffer_read[0] == 0xFF) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }
}

// public read functions
bool MotorInterface::GetMainBatteryVoltage(uint8_t address, double &battery_voltage) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kGetMBatt);


  const uint8_t buffer_length = 4;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (!SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) return false;

  // check crc
  UpdateCRC(buffer_read[0]);
  UpdateCRC(buffer_read[1]);
  uint16_t crc = buffer_read[2] << 8 | buffer_read[3];
  if (crc_ != crc) {
    std::printf("[%s] Invalid crc.\n", __func__);
    return false;
  }

  uint16_t voltage_int = buffer_read[0] << 8 | buffer_read[1];
  battery_voltage = ((double) voltage_int)/10.0;

  return true;
}

bool MotorInterface::GetM1SpeedFiltered(uint8_t address, double &speed) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kGetM1SpeedFiltered);

  const uint8_t buffer_length = 7;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (!SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) return false;

  // check crc
  UpdateCRC(buffer_read[0]);
  UpdateCRC(buffer_read[1]);
  UpdateCRC(buffer_read[2]);
  UpdateCRC(buffer_read[3]);
  UpdateCRC(buffer_read[4]);
  uint16_t crc = buffer_read[5] << 8 | buffer_read[6];
  if (crc_ != crc) {
    std::printf("[%s] Invalid crc (rec: 0x%x 0x%x, exp: 0x%x 0x%x).\n", __func__, buffer_read[5], buffer_read[6], crc_>> 8, crc_ & 0xff);
    std::printf("data: 0x%x 0x%x 0x%x 0x%x 0x%x\n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3], buffer_read[4]);

    return false;
  }

  int32_t speed_int = buffer_read[0] << 24 | buffer_read[1] << 16 | buffer_read[2] << 8 | buffer_read[3];
  speed = (double) speed_int;

  return true;
}

bool MotorInterface::GetM2SpeedFiltered(uint8_t address, double &speed) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kGetM2SpeedFiltered);

  const uint8_t buffer_length = 7;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (!SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) return false;

  // check crc
  UpdateCRC(buffer_read[0]);
  UpdateCRC(buffer_read[1]);
  UpdateCRC(buffer_read[2]);
  UpdateCRC(buffer_read[3]);
  UpdateCRC(buffer_read[4]);
  uint16_t crc = buffer_read[5] << 8 | buffer_read[6];
  if (crc_ != crc) {
    std::printf("[%s] Invalid crc (rec: 0x%x 0x%x, exp: 0x%x 0x%x).\n", __func__, buffer_read[5], buffer_read[6], crc_>> 8, crc_ & 0xff);
    std::printf("data: 0x%x 0x%x 0x%x 0x%x 0x%x\n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3], buffer_read[4]);

    return false;
  }

  int32_t speed_int = buffer_read[0] << 24 | buffer_read[1] << 16 | buffer_read[2] << 8 | buffer_read[3];
  speed = (double) speed_int;

  return true;
}

bool MotorInterface::GetM1Speed(uint8_t address, double &speed) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kGetM1Speed);

  const uint8_t buffer_length = 7;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (!SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) return false;

  // check crc
  UpdateCRC(buffer_read[0]);
  UpdateCRC(buffer_read[1]);
  UpdateCRC(buffer_read[2]);
  UpdateCRC(buffer_read[3]);
  UpdateCRC(buffer_read[4]);
  uint16_t crc = buffer_read[5] << 8 | buffer_read[6];
  if (crc_ != crc) {
    std::printf("[%s] Invalid crc (rec: 0x%x 0x%x, exp: 0x%x 0x%x).\n", __func__, buffer_read[5], buffer_read[6], crc_>> 8, crc_ & 0xff);
    std::printf("data: 0x%x 0x%x 0x%x 0x%x 0x%x\n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3], buffer_read[4]);

    return false;
  }

  int32_t speed_int = buffer_read[0] << 24 | buffer_read[1] << 16 | buffer_read[2] << 8 | buffer_read[3];
  speed = (double) speed_int;

  return true;
}

bool MotorInterface::GetM2Speed(uint8_t address, double &speed) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kGetM2Speed);

  const uint8_t buffer_length = 7;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (!SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) return false;

  // check crc
  UpdateCRC(buffer_read[0]);
  UpdateCRC(buffer_read[1]);
  UpdateCRC(buffer_read[2]);
  UpdateCRC(buffer_read[3]);
  UpdateCRC(buffer_read[4]);
  uint16_t crc = buffer_read[5] << 8 | buffer_read[6];
  if (crc_ != crc) {
    std::printf("[%s] Invalid crc.\n", __func__);
    return false;
  }

  int32_t speed_int = buffer_read[0] << 24 | buffer_read[1] << 16 | buffer_read[2] << 8 | buffer_read[3];
  speed = (double) speed_int;
  
  return true;
}

bool MotorInterface::GetM1M2Speed(uint8_t address, double &speed1, double &speed2) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kGetM1M2Speed);

  const uint8_t buffer_length = 10;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (!SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) return false;

  // check crc
  for (int i = 0; i < 8; ++i) UpdateCRC(buffer_read[i]);
  uint16_t crc = buffer_read[8] << 8 | buffer_read[9];
  if (crc_ != crc) {
    std::printf("[%s] Invalid crc (rec: 0x%x 0x%x, exp: 0x%x 0x%x).\n", __func__, buffer_read[8], buffer_read[9], crc_>> 8, crc_ & 0xff);
    std::printf("data: 0x%x 0x%x 0x%x 0x%x\n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3]);
    std::printf("data: 0x%x 0x%x 0x%x 0x%x\n", buffer_read[4], buffer_read[5], buffer_read[6], buffer_read[7]);

    return false;
  }

  int32_t speed_int = buffer_read[0] << 24 | buffer_read[1] << 16 | buffer_read[2] << 8 | buffer_read[3];
  speed1 = (double) speed_int;

  speed_int = buffer_read[4] << 24 | buffer_read[5] << 16 | buffer_read[6] << 8 | buffer_read[7];
  speed2 = (double) speed_int;

  return true;
}

bool MotorInterface::GetM1M2Pos(uint8_t address, double &pos1, double &pos2) {
  ClearBuffer();
  AddToBuffer(address);
  AddToBuffer((uint8_t) CommandType::kGetM1M2Enc);

  const uint8_t buffer_length = 10;
  uint8_t buffer_read[buffer_length];
  uint8_t bytes_received;
  if (!SendBufferAndReceive(buffer_read, buffer_length, bytes_received)) return false;

  // check crc
  for (int i = 0; i < 8; ++i) UpdateCRC(buffer_read[i]);
  uint16_t crc = buffer_read[8] << 8 | buffer_read[9];
  if (crc_ != crc) {
    std::printf("[%s] Invalid crc (rec: 0x%x 0x%x, exp: 0x%x 0x%x).\n", __func__, buffer_read[8], buffer_read[9], crc_>> 8, crc_ & 0xff);
    std::printf("data: 0x%x 0x%x 0x%x 0x%x\n", buffer_read[0], buffer_read[1], buffer_read[2], buffer_read[3]);
    std::printf("data: 0x%x 0x%x 0x%x 0x%x\n", buffer_read[4], buffer_read[5], buffer_read[6], buffer_read[7]);

    return false;
  }

  int32_t pos_int = buffer_read[0] << 24 | buffer_read[1] << 16 | buffer_read[2] << 8 | buffer_read[3];
  pos1 = (double) pos_int;

  pos_int = buffer_read[4] << 24 | buffer_read[5] << 16 | buffer_read[6] << 8 | buffer_read[7];
  pos2 = (double) pos_int;

  return true;
}

// serial port
void MotorInterface::ClearBuffer(void) {
  //serial_.Flush(1);
  buffer_length_ = 0;
  ClearCRC();

  return;
}

void MotorInterface::AddToBuffer(uint8_t data) {
  buffer_[buffer_length_++] = data;
  UpdateCRC(data);

  return;
}

void MotorInterface::AddToBuffer(uint16_t data) {
  uint8_t* data_ptr = (uint8_t *) &data;

  buffer_[buffer_length_++] = data_ptr[1];
  UpdateCRC(data_ptr[1]);
  buffer_[buffer_length_++] = data_ptr[0];
  UpdateCRC(data_ptr[0]);

  return;
}

void MotorInterface::AddToBuffer(int16_t data) {
  uint8_t * data_ptr = (uint8_t *) &data;

  buffer_[buffer_length_++] = data_ptr[1];
  UpdateCRC(data_ptr[1]);
  buffer_[buffer_length_++] = data_ptr[0];
  UpdateCRC(data_ptr[0]);

  return;
}

void MotorInterface::AddToBuffer(int32_t data) {
  uint8_t * data_ptr = (uint8_t *) &data;

  buffer_[buffer_length_++] = data_ptr[3];
  UpdateCRC(data_ptr[3]);
  buffer_[buffer_length_++] = data_ptr[2];
  UpdateCRC(data_ptr[2]);
  buffer_[buffer_length_++] = data_ptr[1];
  UpdateCRC(data_ptr[1]);
  buffer_[buffer_length_++] = data_ptr[0];
  UpdateCRC(data_ptr[0]);

  return;
}

void MotorInterface::AddCRCToBuffer(void) {
  uint8_t* data_ptr = (uint8_t *) &crc_;

  buffer_[buffer_length_++] = data_ptr[1];
  buffer_[buffer_length_++] = data_ptr[0];

  return;
}

bool MotorInterface::SendBuffer(void) {
  if (serial_.Write(buffer_, buffer_length_) != buffer_length_) {
    std::printf("[%s] Error sending buffer.\n", __func__);
    return false;
  }

  return true;
}

bool MotorInterface::SendBufferAndReceive(void* buffer_read, const uint8_t buffer_length, uint8_t &bytes_received) {
  // set up reception
  struct pollfd fds;
  fds.fd = serial_.GetFileDescriptor();
  fds.events = POLLIN;

  // send data
  if (serial_.Write(buffer_, buffer_length_) != buffer_length_) return false;

  // receive data
  bytes_received = 0;
  int ret = poll(&fds, 1, 100); // timeout 100ms
  if (fds.revents & POLLIN) {
    int length_read = serial_.Read(buffer_read, buffer_length);
    
    if (length_read == 0) {
      std::printf("[%s] Received 0 bytes.\n", __func__);
      return false;
    }
  
    bytes_received = length_read;
    length_read = serial_.Read(buffer_read, buffer_length);
    if (length_read > 0) {
      std::printf("[%s] Read buffer too small.\n", __func__);
      return false;
    }

    return true;
  }

  std::printf("[%s] Timeout on waiting for data.\n", __func__);
  return false;
}

bool MotorInterface::Receive(void *buffer_read, const uint8_t buffer_length, uint8_t &bytes_received) {
 // set up reception
  struct pollfd fds;
  fds.fd = serial_.GetFileDescriptor();
  fds.events = POLLIN;

  // receive data
  int ret = poll(&fds, 1, 100); // timeout 100ms
  if (fds.revents & POLLIN) {
    int length_read = serial_.Read(buffer_read, buffer_length);
    
    if (length_read == 0) {
      std::printf("[%s] Received 0 bytes.\n", __func__);
      return false;
    }
  
    bytes_received = length_read;
    length_read = serial_.Read(buffer_read, buffer_length);
    if (length_read > 0) {
      std::printf("[%s] Read buffer too small.\n", __func__);
      return false;
    }

    return true;
  }

  std::printf("[%s] Timeout on waiting for data.\n", __func__);
  return false;
}

// crc
void MotorInterface::UpdateCRC(uint8_t data) {
  crc_^= data << 8;
  for (int i = 0; i < 8; ++i) {
    if ((crc_ & 0x8000) == 0x8000) {
      crc_ = (crc_ << 1)^0x1021;
    }
    else {
      crc_ <<= 1;
    }
  }

  return;
}

void MotorInterface::ClearCRC(void) {
  crc_ = 0;

  return;
}