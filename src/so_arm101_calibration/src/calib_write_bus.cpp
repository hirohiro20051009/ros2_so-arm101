// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_calibration/calib_write_bus.hpp"

namespace so_arm101_calibration
{
using so_arm101_hardware::Register;
using so_arm101_hardware::Status;

Register register_of(CalibReg r)
{
  switch (r) {
    case CalibReg::HomingOffset: return so_arm101_hardware::reg::kHomingOffset;
    case CalibReg::MinPositionLimit: return so_arm101_hardware::reg::kMinPositionLimit;
    case CalibReg::MaxPositionLimit: return so_arm101_hardware::reg::kMaxPositionLimit;
  }
  return so_arm101_hardware::reg::kHomingOffset;
}

const char * to_string(CalibReg r)
{
  switch (r) {
    case CalibReg::HomingOffset: return "Homing_Offset";
    case CalibReg::MinPositionLimit: return "Min_Position_Limit";
    case CalibReg::MaxPositionLimit: return "Max_Position_Limit";
  }
  return "?";
}

int max_raw_of(CalibReg r) {return r == CalibReg::HomingOffset ? 0x0FFF : 4095;}

Status CalibWriteBus::write_calib_raw(uint8_t id, CalibReg r, uint16_t raw)
{
  return write_bytes(id, register_of(r), raw, 0, max_raw_of(r));
}

}  // namespace so_arm101_calibration
