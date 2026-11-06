// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_hardware/sts3215_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace so_arm101_hardware
{

namespace
{
std::string param(const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & def)
{
  auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? def : it->second;
}
bool is_true(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s == "true" || s == "1";
}
}  // namespace

hardware_interface::CallbackReturn Sts3215System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  serial_port_ = param(info_, "serial_port", "");
  calibration_file_ = param(info_, "calibration_file", "");
  if (serial_port_.empty() || calibration_file_.empty()) {
    RCLCPP_ERROR(logger(), "hardware params 'serial_port' and 'calibration_file' are required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!is_true(param(info_, "read_only", "true"))) {
    RCLCPP_ERROR(
      logger(), "read_only:=false is not implemented (write mode is stage 2b-3). Refusing to start.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    timeout_ms_ = std::stoi(param(info_, "timeout_ms", "10"));
    max_consecutive_failures_ = std::stoi(param(info_, "max_consecutive_failures", "10"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "invalid numeric hardware param: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (timeout_ms_ < 1 || max_consecutive_failures_ < 1) {
    RCLCPP_ERROR(logger(), "timeout_ms and max_consecutive_failures must be >= 1");
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    calibration_ = load_calibration(calibration_file_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    const JointCalibration * c = calibration_.find(joint.name);
    if (!c) {
      RCLCPP_ERROR(logger(), "joint '%s' is not in the calibration file", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    cal_.push_back(c);
    ids_.push_back(c->id);
  }
  const size_t n = info_.joints.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  hw_commands_.assign(n, nan);
  hw_positions_.assign(n, nan);
  hw_velocities_.assign(n, nan);
  RCLCPP_INFO(
    logger(), "initialized: port=%s calibration=%s [%s] timeout=%dms max_consecutive_failures=%d",
    serial_port_.c_str(), calibration_file_.c_str(), calibration_.source.c_str(), timeout_ms_,
    max_consecutive_failures_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Sts3215System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return v;
}

std::vector<hardware_interface::CommandInterface> Sts3215System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return v;
}

bool Sts3215System::read_raw_positions(std::vector<int> * raw, std::string * why)
{
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed_id = 0;
  const Status s = bus_->sync_read(ids_, reg::kPresentPosition, &out, &failed_id);
  if (s != Status::Ok) {
    std::ostringstream os;
    os << to_string(s) << ": " << bus_->last_error();
    if (failed_id) {os << " [id " << int(failed_id) << "]";}
    *why = os.str();
    return false;
  }
  raw->resize(out.size());
  for (size_t i = 0; i < out.size(); ++i) {
    const uint16_t u = static_cast<uint16_t>(out[i][0] | (out[i][1] << 8));
    (*raw)[i] = decode_sign_magnitude(u, kPositionSignBit);
  }
  return true;
}

bool Sts3215System::run_diagnostics()
{
  int matched = 0;
  bool fatal = false;
  for (size_t i = 0; i < cal_.size(); ++i) {
    const JointCalibration & c = *cal_[i];
    uint16_t model = 0, homing_u = 0, mn = 0, mx = 0;
    std::vector<uint8_t> rd;
    Status s = bus_->read_u16(c.id, reg::kModelNumber, &model);
    if (s == Status::Ok) {s = bus_->read(c.id, reg::kReturnDelayTime, &rd);}
    if (s == Status::Ok) {s = bus_->read_u16(c.id, reg::kHomingOffset, &homing_u);}
    if (s == Status::Ok) {s = bus_->read_u16(c.id, reg::kMinPositionLimit, &mn);}
    if (s == Status::Ok) {s = bus_->read_u16(c.id, reg::kMaxPositionLimit, &mx);}
    if (s != Status::Ok) {
      RCLCPP_ERROR(
        logger(), "[diag] %s id=%d: cannot read servo (%s: %s). Check power, cable and id.",
        c.name.c_str(), int(c.id), to_string(s), bus_->last_error().c_str());
      fatal = true;
      continue;
    }
    if (model != kModelNumberSts3215) {
      RCLCPP_ERROR(
        logger(), "[diag] %s id=%d: model number %d, expected %d (STS3215)", c.name.c_str(), int(c.id),
        int(model), int(kModelNumberSts3215));
      fatal = true;
      continue;
    }
    const int homing = decode_sign_magnitude(homing_u, kHomingOffsetSignBit);
    std::ostringstream diff;
    if (homing != c.eeprom.homing_offset) {
      diff << " homing_offset(servo " << homing << ", yaml " << c.eeprom.homing_offset << ")";
    }
    if (int(mn) != c.eeprom.min_limit) {
      diff << " min_limit(servo " << mn << ", yaml " << c.eeprom.min_limit << ")";
    }
    if (int(mx) != c.eeprom.max_limit) {
      diff << " max_limit(servo " << mx << ", yaml " << c.eeprom.max_limit << ")";
    }
    std::ostringstream line;
    line << "[diag] " << c.name << " id=" << int(c.id) << " model=" << model
         << " return_delay=" << int(rd.empty() ? 0 : rd[0]) << " homing_offset=" << homing
         << " (yaml " << c.eeprom.homing_offset << ") min_limit=" << mn << " (yaml " << c.eeprom.min_limit
         << ") max_limit=" << mx << " (yaml " << c.eeprom.max_limit << ")";
    if (diff.str().empty()) {
      ++matched;
      RCLCPP_INFO(logger(), "%s: MATCH", line.str().c_str());
    } else {
      RCLCPP_WARN(
        logger(), "%s: MISMATCH%s -> 変換値が実機とずれる可能性。キャリブレーションを確認すること",
        line.str().c_str(), diff.str().c_str());
    }
  }
  RCLCPP_INFO(
    logger(), "[diag] EEPROM vs yaml: %d/%zu joints match", matched, cal_.size());
  return !fatal;
}

hardware_interface::CallbackReturn Sts3215System::on_configure(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!port_.open(serial_port_, calibration_.baudrate, &err)) {
    RCLCPP_ERROR(logger(), "cannot open serial port: %s", err.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  bus_ = std::make_unique<Sts3215Bus>(port_, std::chrono::milliseconds(timeout_ms_));
  if (!run_diagnostics()) {
    port_.close();
    bus_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Sts3215System::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(
    logger(),
    "READ-ONLY mode: servo registers (including torque) are never written; commands are DISCARDED. "
    "The arm will not move.");
  std::vector<int> raw;
  std::string why;
  if (!read_raw_positions(&raw, &why)) {
    RCLCPP_ERROR(logger(), "initial read failed: %s", why.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < raw.size(); ++i) {
    hw_positions_[i] = raw_to_rad(*cal_[i], calibration_.angle_ticks, raw[i]);
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = hw_positions_[i];
  }
  have_last_ = false;
  consecutive_failures_ = 0;
  reads_ = retried_ = failed_cycles_ = 0;
  warned_discard_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

void Sts3215System::invalidate_state()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::fill(hw_positions_.begin(), hw_positions_.end(), nan);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), nan);
  std::fill(hw_commands_.begin(), hw_commands_.end(), nan);
  have_last_ = false;
}

hardware_interface::CallbackReturn Sts3215System::on_deactivate(const rclcpp_lifecycle::State &)
{
  invalidate_state();
  RCLCPP_INFO(
    logger(), "deactivated (no register was written). reads=%lu retried=%lu failed_cycles=%lu", reads_,
    retried_, failed_cycles_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Sts3215System::on_cleanup(const rclcpp_lifecycle::State &)
{
  invalidate_state();
  bus_.reset();
  port_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Sts3215System::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  ++reads_;
  std::vector<int> raw;
  std::string why;
  bool ok = read_raw_positions(&raw, &why);
  if (!ok) {
    // 1 回だけ再試行（受信バッファは fail() で捨て済み）。再試行で成功しても数えて報告する。
    ++retried_;
    std::string why2;
    ok = read_raw_positions(&raw, &why2);
    if (!ok) {why = why2;}
  }
  if (!ok) {
    ++consecutive_failures_;
    ++failed_cycles_;
    // 50 Hz で埋め尽くさないよう、最初と 10 回ごとに警告する
    if (consecutive_failures_ == 1 || consecutive_failures_ % 10 == 0) {
      RCLCPP_WARN(
        logger(), "read failed (%s); consecutive=%d/%d; holding previous values",
        why.c_str(), consecutive_failures_, max_consecutive_failures_);
    }
    if (consecutive_failures_ >= max_consecutive_failures_) {
      RCLCPP_ERROR(
        logger(), "read failed %d times in a row (%s): giving up. State is set to NaN so that stale "
        "values are never published as valid", consecutive_failures_, why.c_str());
      invalidate_state();
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;  // 前回値を保持（警告済み）
  }
  if (consecutive_failures_ > 0) {
    RCLCPP_INFO(logger(), "read recovered after %d consecutive failures", consecutive_failures_);
    consecutive_failures_ = 0;
  }
  double dt = have_last_ ? (time.seconds() - last_time_s_) : 0.0;
  if (have_last_ && dt <= 1e-6) {dt = period.seconds();}
  for (size_t i = 0; i < raw.size(); ++i) {
    const double pos = raw_to_rad(*cal_[i], calibration_.angle_ticks, raw[i]);
    // velocity: 位置の有限差分（フィルタなし）。最初の 1 回は 0.0
    hw_velocities_[i] = (have_last_ && dt > 1e-6) ? (pos - hw_positions_[i]) / dt : 0.0;
    hw_positions_[i] = pos;
    hw_commands_[i] = pos;  // command は state に追従させる（書き込み経路が無いので破棄と同義）
  }
  have_last_ = true;
  last_time_s_ = time.seconds();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Sts3215System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 読み取り専用: command は送らない。最初に実際の指令差を見たときだけ 1 回警告する。
  if (!warned_discard_) {
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
      if (std::isfinite(hw_commands_[i]) && std::fabs(hw_commands_[i] - hw_positions_[i]) > 0.02) {
        RCLCPP_WARN(
          logger(), "READ-ONLY: command for '%s' differs from the measured position by %.3f rad and is "
          "discarded (no motion)", info_.joints[i].name.c_str(), hw_commands_[i] - hw_positions_[i]);
        warned_discard_ = true;
        break;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace so_arm101_hardware

PLUGINLIB_EXPORT_CLASS(so_arm101_hardware::Sts3215System, hardware_interface::SystemInterface)
