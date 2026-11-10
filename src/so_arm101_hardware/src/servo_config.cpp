// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_hardware/servo_config.hpp"

#include <sstream>

namespace so_arm101_hardware
{

ExpectedServoConfig expected_servo_config(const JointCalibration & joint)
{
  ExpectedServoConfig e;
  e.gripper_limits = (joint.kind == JointKind::LinearPercent);
  return e;
}

std::vector<ConfigDiff> compare_servo_config(const ExpectedServoConfig & e, const ServoSettings & a)
{
  std::vector<ConfigDiff> d;
  auto check = [&](const char * name, int servo, int expected) {
      if (servo != expected) {d.push_back({name, servo, expected});}
    };
  check("Operating_Mode", a.operating_mode, e.operating_mode);
  check("P_Coefficient", a.p, e.p);
  check("I_Coefficient", a.i, e.i);
  check("D_Coefficient", a.d, e.d);
  check("Return_Delay_Time", a.return_delay, e.return_delay);
  if (e.gripper_limits) {
    check("Max_Torque_Limit", a.max_torque_limit, e.max_torque_limit);
    check("Protection_Current", a.protection_current, e.protection_current);
    check("Overload_Torque", a.overload_torque, e.overload_torque);
  }
  return d;
}

std::string to_string(const std::vector<ConfigDiff> & diffs)
{
  std::ostringstream os;
  for (const auto & x : diffs) {
    os << " " << x.name << "(servo " << x.servo << ", expected " << x.expected << ")";
  }
  return os.str();
}

bool has_operating_mode_diff(const std::vector<ConfigDiff> & diffs)
{
  for (const auto & x : diffs) {
    if (x.name == "Operating_Mode") {return true;}
  }
  return false;
}

ConfigAction config_action(const std::vector<ConfigDiff> & diffs, bool read_only, bool apply_servo_config)
{
  if (diffs.empty()) {return ConfigAction::Ok;}
  if (read_only) {return ConfigAction::Warn;}
  if (apply_servo_config && !has_operating_mode_diff(diffs)) {return ConfigAction::Apply;}
  return ConfigAction::Abort;
}

}  // namespace so_arm101_hardware
