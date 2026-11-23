#pragma once

#include <stdint.h>

enum class ControlMode : uint8_t {
  TANK,
  MIXED
};

struct ButtonState {
  uint8_t pin;
  bool rawPressed = false;
  bool pressed = false;
  bool justPressed = false;
  uint32_t rawChangedAt = 0;
};

struct CalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  int32_t leftXMin, leftXCenter, leftXMax;
  int32_t leftYMin, leftYCenter, leftYMax;
  int32_t rightXMin, rightXCenter, rightXMax;
  int32_t rightYMin, rightYCenter, rightYMax;
};
