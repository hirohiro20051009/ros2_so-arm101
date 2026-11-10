// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_hardware/sts3215_write_bus.hpp"

namespace so_arm101_hardware
{

namespace
{
constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint8_t kMaxId = 0xFC;
}  // namespace

Register register_of(ServoSetting s)
{
  switch (s) {
    case ServoSetting::P: return reg::kPCoefficient;
    case ServoSetting::I: return reg::kICoefficient;
    case ServoSetting::D: return reg::kDCoefficient;
    case ServoSetting::ReturnDelayTime: return reg::kReturnDelayTime;
    case ServoSetting::MaxTorqueLimit: return reg::kMaxTorqueLimit;
    case ServoSetting::ProtectionCurrent: return reg::kProtectionCurrent;
    case ServoSetting::OverloadTorque: return reg::kOverloadTorque;
  }
  return reg::kPCoefficient;
}

const char * to_string(ServoSetting s)
{
  switch (s) {
    case ServoSetting::P: return "P_Coefficient";
    case ServoSetting::I: return "I_Coefficient";
    case ServoSetting::D: return "D_Coefficient";
    case ServoSetting::ReturnDelayTime: return "Return_Delay_Time";
    case ServoSetting::MaxTorqueLimit: return "Max_Torque_Limit";
    case ServoSetting::ProtectionCurrent: return "Protection_Current";
    case ServoSetting::OverloadTorque: return "Overload_Torque";
  }
  return "?";
}

std::vector<uint8_t> Sts3215WriteBus::build_write(
  uint8_t id, uint8_t address, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> params{address};
  params.insert(params.end(), data.begin(), data.end());
  return build_packet(id, static_cast<uint8_t>(WriteInstruction::Write), params);
}

std::vector<uint8_t> Sts3215WriteBus::build_sync_write(
  uint8_t address, uint8_t data_len, const std::vector<uint8_t> & ids,
  const std::vector<std::vector<uint8_t>> & data)
{
  // FF FF FE LEN 83 addr len [id d0 d1 ..]...  LEN = (data_len + 1)·N + 4
  std::vector<uint8_t> params{address, data_len};
  for (size_t i = 0; i < ids.size(); ++i) {
    params.push_back(ids[i]);
    params.insert(params.end(), data[i].begin(), data[i].end());
  }
  return build_packet(kBroadcastId, static_cast<uint8_t>(WriteInstruction::SyncWrite), params);
}

Status Sts3215WriteBus::write_bytes(uint8_t id, Register r, int value, int min_value, int max_value)
{
  SerialPort::Transaction tx(port_);
  if (!port_.is_open()) {return fail(Status::PortError, "port not open");}
  if (id > kMaxId) {return fail(Status::BadFrame, "invalid id");}
  if (value < min_value || value > max_value) {
    return fail(Status::BadFrame, "value " + std::to_string(value) + " out of range for register " +
           std::to_string(r.address));
  }
  std::vector<uint8_t> data;
  if (r.length == 2) {
    data = {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
  } else {
    data = {static_cast<uint8_t>(value & 0xFF)};
  }
  const auto req = build_write(id, r.address, data);
  port_.flush_input();
  if (!port_.write_all(req.data(), req.size())) {return fail(Status::PortError, "write failed");}
  std::vector<uint8_t> ack;
  return receive_status(id, 0, &ack);
}

Status Sts3215WriteBus::write_torque_enable(uint8_t id, bool on)
{
  return write_bytes(id, reg::kTorqueEnable, on ? 1 : 0, 0, 1);
}

Status Sts3215WriteBus::write_lock(uint8_t id, bool locked)
{
  return write_bytes(id, reg::kLock, locked ? 1 : 0, 0, 1);
}

Status Sts3215WriteBus::write_acceleration(uint8_t id, int value)
{
  return write_bytes(id, reg::kAcceleration, value, 0, kMaxAccelerationValue);
}

Status Sts3215WriteBus::write_torque_limit(uint8_t id, int value)
{
  return write_bytes(id, reg::kTorqueLimit, value, 0, kMaxTorqueLimitValue);
}

Status Sts3215WriteBus::write_goal_position(uint8_t id, int raw)
{
  uint16_t enc = 0;
  if (!encode_sign_magnitude(raw, kPositionSignBit, &enc)) {
    return fail(Status::BadFrame, "goal position " + std::to_string(raw) + " out of range");
  }
  return write_bytes(id, reg::kGoalPosition, enc, 0, 0xFFFF);
}

Status Sts3215WriteBus::write_servo_setting(uint8_t id, ServoSetting s, int value)
{
  const Register r = register_of(s);
  return write_bytes(id, r, value, 0, r.length == 2 ? 0xFFFF : 0xFF);
}

Status Sts3215WriteBus::sync_write_goal_position(
  const std::vector<uint8_t> & ids, const std::vector<int> & raws)
{
  SerialPort::Transaction tx(port_);
  if (!port_.is_open()) {return fail(Status::PortError, "port not open");}
  if (ids.empty() || ids.size() > 32 || ids.size() != raws.size()) {
    return fail(Status::BadFrame, "invalid id/value list");
  }
  std::vector<std::vector<uint8_t>> data;
  for (size_t i = 0; i < ids.size(); ++i) {
    uint16_t enc = 0;
    if (!encode_sign_magnitude(raws[i], kPositionSignBit, &enc)) {
      return fail(Status::BadFrame, "goal position " + std::to_string(raws[i]) + " out of range");
    }
    data.push_back({static_cast<uint8_t>(enc & 0xFF), static_cast<uint8_t>(enc >> 8)});
  }
  const auto req = build_sync_write(reg::kGoalPosition.address, reg::kGoalPosition.length, ids, data);
  if (!port_.write_all(req.data(), req.size())) {return fail(Status::PortError, "write failed");}
  return Status::Ok;  // SYNC_WRITE に応答は無い
}

}  // namespace so_arm101_hardware
