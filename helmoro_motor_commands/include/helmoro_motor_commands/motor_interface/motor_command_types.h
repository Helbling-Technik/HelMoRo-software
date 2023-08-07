#pragma once

#include <cstdint>

enum class CommandType : uint8_t {
  kM1Forward = 0,
  kM1Backward = 1,
  kSetMinMB = 2,
  kSetMaxMB = 3,
  kM2Forward = 4,
  kM2Backward = 5,
  kGetM1Enc = 16,
  kGetM2Enc = 17,
  kGetM1SpeedFiltered = 18,
  kGetM2SpeedFiltered = 19,
  kGetMBatt = 24,
  kGetM1Speed = 30,
  kGetM2Speed = 31,
  kM1Duty = 32,
  kM2Duty = 33,
  kMixedDuty = 34,
  kMixedSpeed = 37,
  kGetM1M2Enc = 78,
  kGetM1M2Speed = 79,
};