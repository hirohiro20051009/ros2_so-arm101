// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_hardware/sts3215_system.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>
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
/// "true"/"false"（大文字小文字は問わない）だけを受け付ける。それ以外は false を返す（呼び出し側が起動を拒否する）。
/// read_only の打ち間違いが書き込みモードにならないようにするため、「true 以外は false」とはしない。
bool parse_bool(std::string s, bool * out)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  if (s == "true") {*out = true; return true;}
  if (s == "false") {*out = false; return true;}
  return false;
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
  if (!parse_bool(param(info_, "read_only", "true"), &read_only_) ||
    !parse_bool(param(info_, "apply_servo_config", "false"), &apply_servo_config_) ||
    !parse_bool(param(info_, "probe_lock_sram", "false"), &probe_lock_sram_))
  {
    RCLCPP_ERROR(
      logger(), "read_only / apply_servo_config / probe_lock_sram must be 'true' or 'false' (read_only='%s'). "
      "Refusing to start", param(info_, "read_only", "true").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    timeout_ms_ = std::stoi(param(info_, "timeout_ms", "10"));
    // 通信エラーの後に「バスが静か」と見なす時間（ms）。コードの既定値 2。0 で従来どおり（捨てるだけ）
    drain_quiet_ms_ = std::stoi(param(info_, "drain_quiet_ms", "2"));
    max_consecutive_failures_ = std::stoi(param(info_, "max_consecutive_failures", "10"));
    acceleration_ = std::stoi(param(info_, "acceleration", std::to_string(kDefaultAcceleration)));
    torque_limit_ = std::stoi(param(info_, "torque_limit", std::to_string(kDefaultTorqueLimit)));
    max_joint_speed_ = std::stod(param(info_, "max_joint_speed", std::to_string(kDefaultMaxJointSpeed)));
    limit_window_s_ = std::stod(param(info_, "limit_window_s", std::to_string(kDefaultLimitWindow)));
    monitor_period_s_ = std::stod(param(info_, "monitor_period_s", std::to_string(kDefaultMonitorPeriod)));
    anomaly_cycles_ = std::stoi(param(info_, "anomaly_cycles", std::to_string(kDefaultAnomalyCycles)));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "invalid numeric hardware param: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (drain_quiet_ms_ < 0 || drain_quiet_ms_ > 10 || timeout_ms_ < 1 || max_consecutive_failures_ < 1) {
    RCLCPP_ERROR(logger(), "timeout_ms and max_consecutive_failures must be >= 1, drain_quiet_ms 0..10");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (acceleration_ < 1 || acceleration_ > kMaxAccelerationValue) {
    RCLCPP_ERROR(
      logger(), "acceleration must be 1..%d (got %d). 0 is refused: its meaning (possibly 'no limit') is "
      "unconfirmed", kMaxAccelerationValue, acceleration_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (torque_limit_ < 1 || torque_limit_ > kMaxTorqueLimitValue) {
    RCLCPP_ERROR(logger(), "torque_limit must be 1..%d (got %d)", kMaxTorqueLimitValue, torque_limit_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (max_joint_speed_ <= 0.0 || limit_window_s_ <= 0.0 || monitor_period_s_ <= 0.0 || anomaly_cycles_ < 1) {
    RCLCPP_ERROR(logger(), "max_joint_speed, limit_window_s, monitor_period_s, anomaly_cycles must be > 0");
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

bool Sts3215System::read_raw_positions(std::vector<int> * raw, std::string * why, Status * status)
{
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed_id = 0;
  const Status s = bus_->sync_read(ids_, reg::kPresentPosition, &out, &failed_id);
  if (status) {*status = s;}
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
    } else if (!read_only_) {
      // 書き込みモード: 変換がずれたままトルクを ON にしない（途中で失敗したキャリブの残りなど）。
      // 解除のスイッチは設けない。直すには restore で EEPROM を戻すか、yaml を実機に合わせる。
      RCLCPP_ERROR(
        logger(), "%s: MISMATCH%s -> refusing to start in write mode (torque is not enabled). "
        "Restore the EEPROM from the backup (restore) or update the calibration yaml; "
        "read-only mode (read_only:=true) still starts and only warns",
        line.str().c_str(), diff.str().c_str());
      fatal = true;
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

bool Sts3215System::run_servo_config_diagnostics()
{
  int matched = 0;
  bool fatal = false;
  diag_.assign(cal_.size(), Diag{});
  for (size_t i = 0; i < cal_.size(); ++i) {
    const JointCalibration & c = *cal_[i];
    // 読み取りだけ。値は符号なしのまま（Goal_Velocity だけ sign-magnitude をデコード）
    auto rd1 = [&](Register r, int * v, Status * st) {
        std::vector<uint8_t> b;
        if (*st == Status::Ok) {*st = bus_->read(c.id, r, &b);}
        if (*st == Status::Ok && !b.empty()) {
          *v = (r.length == 2) ? int(b[0] | (b[1] << 8)) : int(b[0]);
        }
      };
    ServoSettings a;
    int torque_enable = -1, lock = -1, accel = -1, torque_limit = -1, max_accel = -1, status = -1, volt = -1,
      temp = -1, goal_vel_raw = 0;
    Status st = Status::Ok;
    rd1(reg::kOperatingMode, &a.operating_mode, &st);
    rd1(reg::kPCoefficient, &a.p, &st);
    rd1(reg::kICoefficient, &a.i, &st);
    rd1(reg::kDCoefficient, &a.d, &st);
    rd1(reg::kReturnDelayTime, &a.return_delay, &st);
    rd1(reg::kMaxTorqueLimit, &a.max_torque_limit, &st);
    rd1(reg::kProtectionCurrent, &a.protection_current, &st);
    rd1(reg::kOverloadTorque, &a.overload_torque, &st);
    rd1(reg::kTorqueEnable, &torque_enable, &st);
    rd1(reg::kLock, &lock, &st);
    rd1(reg::kAcceleration, &accel, &st);
    rd1(reg::kTorqueLimit, &torque_limit, &st);
    rd1(reg::kGoalVelocity, &goal_vel_raw, &st);
    rd1(reg::kMaximumAcceleration, &max_accel, &st);
    rd1(reg::kStatus, &status, &st);
    rd1(reg::kPresentVoltage, &volt, &st);
    rd1(reg::kPresentTemperature, &temp, &st);
    if (st != Status::Ok) {
      RCLCPP_ERROR(
        logger(), "[diag] %s id=%d: cannot read servo settings (%s: %s)", c.name.c_str(), int(c.id), to_string(st),
        bus_->last_error().c_str());
      fatal = true;
      continue;
    }
    const int goal_vel = decode_sign_magnitude(static_cast<uint16_t>(goal_vel_raw), kGoalVelocitySignBit);
    const auto diffs = compare_servo_config(expected_servo_config(c), a);
    diag_[i].torque_enable = torque_enable;
    diag_[i].goal_velocity = goal_vel;
    diag_[i].max_acceleration = max_accel;
    diag_[i].diffs = diffs;
    if (!read_only_ && goal_vel != 0) {
      RCLCPP_ERROR(
        logger(), "[diag] %s id=%d: Goal_Velocity=%d (expected 0, the state LeRobot ran with). Refusing to start; "
        "the driver does not rewrite it", c.name.c_str(), int(c.id), goal_vel);
      fatal = true;
    }
    std::ostringstream line;
    line << "[diag] " << c.name << " id=" << int(c.id) << " EEPROM: operating_mode=" << a.operating_mode
         << " P=" << a.p << " I=" << a.i << " D=" << a.d << " return_delay=" << a.return_delay
         << " max_torque_limit=" << a.max_torque_limit << " protection_current=" << a.protection_current
         << " overload_torque=" << a.overload_torque << " | SRAM: torque_enable=" << torque_enable
         << " lock=" << lock << " acceleration=" << accel << " torque_limit=" << torque_limit
         << " goal_velocity=" << goal_vel << " | other: maximum_acceleration=" << max_accel
         << " status=0x" << std::hex << status << std::dec << " voltage=" << volt / 10.0 << "V temperature="
         << temp << "C";
    switch (config_action(diffs, read_only_, apply_servo_config_)) {
      case ConfigAction::Apply:
        RCLCPP_WARN(
          logger(), "%s: DIFFERS from LeRobot values:%s -> apply_servo_config:=true: will be written at activation "
          "(torque OFF, Lock=0)", line.str().c_str(), to_string(diffs).c_str());
        break;
      case ConfigAction::Ok:
        ++matched;
        RCLCPP_INFO(logger(), "%s: MATCH (LeRobot)", line.str().c_str());
        break;
      case ConfigAction::Warn:
        RCLCPP_WARN(
          logger(), "%s: DIFFERS from LeRobot values:%s%s", line.str().c_str(), to_string(diffs).c_str(),
          has_operating_mode_diff(diffs) ?
          " -> Operating_Mode is not 0 (position control): write mode would refuse to enable torque" :
          " -> read-only mode continues; write mode (read_only:=false) would refuse to start");
        break;
      case ConfigAction::Abort:
        RCLCPP_ERROR(
          logger(), "%s: DIFFERS from LeRobot values:%s -> refusing to start (torque is not enabled)",
          line.str().c_str(), to_string(diffs).c_str());
        fatal = true;
        break;
    }
  }
  RCLCPP_INFO(logger(), "[diag] servo settings vs LeRobot: %d/%zu joints match", matched, cal_.size());
  return !fatal;
}

hardware_interface::CallbackReturn Sts3215System::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mu_);
  std::string err;
  if (!port_.open(serial_port_, calibration_.baudrate, &err)) {
    RCLCPP_ERROR(logger(), "cannot open serial port: %s", err.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // read_only のときは書き込み用のクラスを生成しない（書き込み命令がコードに存在しない状態になる）
  write_bus_ = nullptr;
  if (read_only_) {
    bus_ = std::make_unique<Sts3215Bus>(port_, std::chrono::milliseconds(timeout_ms_));
  } else {
    auto wb = std::make_unique<Sts3215WriteBus>(port_, std::chrono::milliseconds(timeout_ms_));
    write_bus_ = wb.get();
    bus_ = std::move(wb);
  }
  bus_->set_drain_quiet(std::chrono::milliseconds(drain_quiet_ms_));
  if (!run_diagnostics() || !run_servo_config_diagnostics()) {
    port_.close();
    bus_.reset();
    write_bus_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Sts3215System::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mu_);
  have_last_ = false;
  consecutive_failures_ = 0;
  reads_ = retried_ = failed_cycles_ = 0;
  warned_discard_ = false;
  if (!read_only_) {
    if (err_latched_) {
      RCLCPP_ERROR(
        logger(), "refusing to activate: a servo protection error (ERR byte) was read earlier (%s). "
        "Restart the launch (the process) after checking the servo", fault_reason_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    fault_ = false;
    fault_reason_.clear();
    return activate_write_mode() ? hardware_interface::CallbackReturn::SUCCESS :
           hardware_interface::CallbackReturn::ERROR;
  }
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
  std::lock_guard<std::mutex> lock(mu_);
  if (!read_only_) {shutdown_servos("deactivate");}
  invalidate_state();
  if (read_only_) {
    RCLCPP_INFO(
      logger(), "deactivated (no register was written). reads=%lu retried=%lu failed_cycles=%lu %s", reads_,
      retried_, failed_cycles_, bus_ ? bus_->counters().to_string().c_str() : "bus: n/a");
  } else {
    unsigned long limited = 0;
    for (auto v : limited_count_) {limited += v;}
    RCLCPP_INFO(
      logger(), "deactivated. reads=%lu retried=%lu failed_cycles=%lu goal_writes=%lu monitors=%lu "
      "rate_limited=%lu port_contention=%lu startup_read_retries=%lu %s", reads_, retried_, failed_cycles_,
      goal_writes_, monitors_, limited, port_.contention_count(), startup_retries_,
      bus_ ? bus_->counters().to_string().c_str() : "bus: n/a");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Sts3215System::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!read_only_) {shutdown_servos("cleanup");}
  invalidate_state();
  bus_.reset();
  write_bus_ = nullptr;
  port_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

Sts3215System::~Sts3215System()
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!read_only_) {shutdown_servos("destructor");}
}

bool Sts3215System::fault_latched() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return fault_;
}

std::string Sts3215System::fault_reason() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return fault_reason_;
}

unsigned long Sts3215System::limited_total() const
{
  std::lock_guard<std::mutex> lock(mu_);
  unsigned long n = 0;
  for (auto v : limited_count_) {n += v;}
  return n;
}

unsigned long Sts3215System::retried_total() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return retried_;
}

unsigned long Sts3215System::failed_cycles_total() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return failed_cycles_;
}

hardware_interface::return_type Sts3215System::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(mu_);
  ++reads_;
  std::vector<int> raw;
  std::string why;
  Status st = Status::Ok;
  bool ok = read_raw_positions(&raw, &why, &st);
  if (!ok) {
    // 1 回だけ再試行（受信バッファは fail() で捨て済み）。再試行で成功しても数えて報告する。
    ++retried_;
    std::string why2;
    ok = read_raw_positions(&raw, &why2, &st);
    if (!ok) {why = why2;}
  }
  if (!ok && !read_only_ && st == Status::ServoError && armed_) {
    // ケース D: サーボ側の保護。送信を止める。再開はプロセスの再起動だけ
    set_fault("servo protection error while reading: " + why, true);
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
      if (!read_only_ && armed_) {
        RCLCPP_ERROR(
          logger(), "COMMUNICATION LOST: the servos keep holding their LAST goal with torque ON; this driver "
          "cannot turn torque OFF over a dead bus. Power off the servos or reconnect, then restart the launch");
      }
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
    if (read_only_) {
      hw_commands_[i] = pos;  // command は state に追従させる（書き込み経路が無いので破棄と同義）
    }
  }
  have_last_ = true;
  last_time_s_ = time.seconds();
  if (!read_only_) {monitor_servos(time);}  // 1 Hz の監視は同じ周期処理の中（別スレッドにしない）
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Sts3215System::write(const rclcpp::Time & time, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!read_only_) {return write_goal_positions(time);}
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

// ===================== 書き込みモード（段階2b-3b）=====================

bool Sts3215System::read_reg(uint8_t id, Register r, int * value, Status * status)
{
  std::vector<uint8_t> b;
  const Status s = bus_->read(id, r, &b);
  if (status) {*status = s;}
  if (s != Status::Ok || b.size() != r.length) {return false;}
  *value = (r.length == 2) ? int(b[0] | (b[1] << 8)) : int(b[0]);
  return true;
}

bool Sts3215System::read_reg_retry(const JointCalibration & c, Register r, const char * name, int * value)
{
  constexpr int kAttempts = 3;
  for (int attempt = 1; attempt <= kAttempts; ++attempt) {
    Status s = Status::Ok;
    if (read_reg(c.id, r, value, &s)) {return true;}
    if (!Sts3215Bus::is_comm_error(s)) {return false;}  // サーボの ERR、ID 違い等は再試行しない
    if (attempt == kAttempts) {return false;}            // 最後の失敗は呼び出し側が ERROR にする
    ++startup_retries_;
    RCLCPP_WARN(
      logger(), "%s id=%d: read of %s failed (%s: %s); retrying %d/%d", c.name.c_str(), int(c.id), name,
      to_string(s), bus_->last_error().c_str(), attempt, kAttempts - 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

bool Sts3215System::read_settings(const JointCalibration & c, ServoSettings * a)
{
  return read_reg(c.id, reg::kOperatingMode, &a->operating_mode) && read_reg(c.id, reg::kPCoefficient, &a->p) &&
         read_reg(c.id, reg::kICoefficient, &a->i) && read_reg(c.id, reg::kDCoefficient, &a->d) &&
         read_reg(c.id, reg::kReturnDelayTime, &a->return_delay) &&
         read_reg(c.id, reg::kMaxTorqueLimit, &a->max_torque_limit) &&
         read_reg(c.id, reg::kProtectionCurrent, &a->protection_current) &&
         read_reg(c.id, reg::kOverloadTorque, &a->overload_torque);
}

bool Sts3215System::write_check(const char * what, const JointCalibration & c, Status s)
{
  if (s == Status::Ok) {return true;}
  RCLCPP_ERROR(
    logger(), "%s id=%d: cannot write %s (%s: %s)", c.name.c_str(), int(c.id), what, to_string(s),
    bus_->last_error().c_str());
  return false;
}

bool Sts3215System::verify_reg(const JointCalibration & c, Register r, int expected, const char * name)
{
  int v = -1;
  if (!read_reg_retry(c, r, name, &v)) {
    RCLCPP_ERROR(
      logger(), "%s id=%d: cannot read back %s (%s)", c.name.c_str(), int(c.id), name, bus_->last_error().c_str());
    return false;
  }
  if (r.address == reg::kGoalPosition.address) {
    v = decode_sign_magnitude(static_cast<uint16_t>(v), kPositionSignBit);
  }
  if (v != expected) {
    RCLCPP_ERROR(
      logger(), "%s id=%d: %s read back as %d, expected %d", c.name.c_str(), int(c.id), name, v, expected);
    return false;
  }
  return true;
}

static bool setting_from_name(const std::string & name, ServoSetting * out)
{
  static const struct {const char * n; ServoSetting s;} table[] = {
    {"P_Coefficient", ServoSetting::P}, {"I_Coefficient", ServoSetting::I}, {"D_Coefficient", ServoSetting::D},
    {"Return_Delay_Time", ServoSetting::ReturnDelayTime}, {"Max_Torque_Limit", ServoSetting::MaxTorqueLimit},
    {"Protection_Current", ServoSetting::ProtectionCurrent}, {"Overload_Torque", ServoSetting::OverloadTorque}};
  for (const auto & t : table) {
    if (name == t.n) {*out = t.s; return true;}
  }
  return false;  // Operating_Mode はここに無い＝書けない
}

bool Sts3215System::apply_servo_settings(const JointCalibration & c, const Diag & d)
{
  for (const auto & diff : d.diffs) {
    ServoSetting s;
    if (!setting_from_name(diff.name, &s)) {
      RCLCPP_ERROR(logger(), "%s: %s cannot be written by this driver", c.name.c_str(), diff.name.c_str());
      return false;
    }
    if (!write_check(to_string(s), c, write_bus_->write_servo_setting(c.id, s, diff.expected))) {return false;}
    if (!verify_reg(c, register_of(s), diff.expected, to_string(s))) {return false;}
    RCLCPP_WARN(
      logger(), "[apply] %s id=%d: %s %d -> %d (EEPROM)", c.name.c_str(), int(c.id), to_string(s), diff.servo,
      diff.expected);
  }
  return true;
}

bool Sts3215System::probe_lock_sram()
{
  // 確認 1: トルク ON・Lock=1 の保持状態で、Acceleration を 1 だけずらして書けるか。腕は動かず、EEPROM に触れない。
  const int probe_value = acceleration_ > 1 ? acceleration_ - 1 : acceleration_ + 1;
  bool accepted = true;
  for (size_t i = 0; i < cal_.size(); ++i) {
    const JointCalibration & c = *cal_[i];
    if (!write_check("Acceleration (probe)", c, write_bus_->write_acceleration(c.id, probe_value))) {return false;}
    int v = -1;
    if (!read_reg(c.id, reg::kAcceleration, &v)) {return false;}
    if (v != probe_value) {accepted = false;}
    if (!write_check("Acceleration (restore)", c, write_bus_->write_acceleration(c.id, acceleration_))) {
      return false;
    }
    if (!verify_reg(c, reg::kAcceleration, acceleration_, "Acceleration (after probe)")) {return false;}
  }
  if (accepted) {
    RCLCPP_INFO(logger(), "[probe] with Lock=1 and torque ON, a write to SRAM (Acceleration) was ACCEPTED on all joints");
  } else {
    RCLCPP_WARN(
      logger(), "[probe] with Lock=1, a write to SRAM (Acceleration) was NOT accepted on some joint. Startup does "
      "not depend on this (SRAM settings are written with Lock=0), but report it");
  }
  return true;
}

bool Sts3215System::activate_write_mode()
{
  const size_t n = cal_.size();
  RCLCPP_WARN(
    logger(), "WRITE mode: this driver will turn torque ON and move the arm. Acceleration=%d torque_limit=%d "
    "max_joint_speed=%.2f rad/s x window=%.3f s. Support the arm and stop with Ctrl-C only from the folded pose.",
    acceleration_, torque_limit_, max_joint_speed_, limit_window_s_);

  // 1. 読むだけ（まだ何も書かない）
  desired_torque_limit_.assign(n, 0);
  std::vector<ServoSettings> settings(n);
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    int goal_vel_raw = 0, torque_enable = 0, max_accel = 0;
    if (!read_settings(c, &settings[i]) || !read_reg(c.id, reg::kGoalVelocity, &goal_vel_raw) ||
      !read_reg(c.id, reg::kTorqueEnable, &torque_enable) || !read_reg(c.id, reg::kMaximumAcceleration, &max_accel))
    {
      RCLCPP_ERROR(logger(), "%s id=%d: cannot read servo state (%s)", c.name.c_str(), int(c.id), bus_->last_error().c_str());
      return false;
    }
    if (settings[i].operating_mode != 0) {
      RCLCPP_ERROR(
        logger(), "%s: Operating_Mode=%d (expected 0 = position control). Torque is not enabled; this driver "
        "never writes Operating_Mode", c.name.c_str(), settings[i].operating_mode);
      return false;
    }
    const int goal_vel = decode_sign_magnitude(static_cast<uint16_t>(goal_vel_raw), kGoalVelocitySignBit);
    if (goal_vel != 0) {
      RCLCPP_ERROR(
        logger(), "%s: Goal_Velocity=%d (expected 0). Torque is not enabled; the driver does not rewrite it",
        c.name.c_str(), goal_vel);
      return false;
    }
    if (torque_enable != 0) {
      RCLCPP_WARN(
        logger(), "%s: Torque_Enable is already 1 (the previous run may not have exited cleanly)", c.name.c_str());
    }
    if (acceleration_ > max_accel) {
      RCLCPP_ERROR(
        logger(), "%s: acceleration %d exceeds Maximum_Acceleration %d", c.name.c_str(), acceleration_, max_accel);
      return false;
    }
    // 重力を受ける 3 関節は常に 1000（下げるとトルク ON でも沈む）。torque_limit は pan / roll だけに効く。
    // gripper は常に 500 以下。
    const bool heavy = c.name == "shoulder_lift" || c.name == "elbow_flex" || c.name == "wrist_flex";
    if (c.kind == JointKind::LinearPercent) {
      desired_torque_limit_[i] = std::min(torque_limit_, kGripperTorqueLimit);
    } else {
      desired_torque_limit_[i] = heavy ? kDefaultTorqueLimit : torque_limit_;
    }
    if (desired_torque_limit_[i] > settings[i].max_torque_limit) {
      RCLCPP_ERROR(
        logger(), "%s: wanted Torque_Limit %d exceeds the servo's Max_Torque_Limit %d", c.name.c_str(),
        desired_torque_limit_[i], settings[i].max_torque_limit);
      return false;
    }
    diag_.resize(n);
    diag_[i].torque_enable = torque_enable;
    diag_[i].diffs = compare_servo_config(expected_servo_config(c), settings[i]);
    if (!diag_[i].diffs.empty()) {
      if (!apply_servo_config_ || has_operating_mode_diff(diag_[i].diffs)) {
        RCLCPP_ERROR(
          logger(), "%s: servo settings differ from LeRobot values:%s. Torque is not enabled (apply_servo_config:=true "
          "writes them; Operating_Mode is never written)", c.name.c_str(), to_string(diag_[i].diffs).c_str());
        return false;
      }
    }
  }

  // ここから書く。失敗したら終了の手順（トルク OFF → Lock=1）で戻す
  touched_ = true;
  shutdown_attempts_ = 0;
  startup_retries_ = 0;
  auto fail = [&](const char * msg) {
      RCLCPP_ERROR(logger(), "activation failed: %s (startup_read_retries=%lu)", msg, startup_retries_);
      shutdown_servos("activation failed");
      return false;
    };

  // 2. Lock=0
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    if (!write_check("Lock=0", c, write_bus_->write_lock(c.id, false)) ||
      !verify_reg(c, reg::kLock, 0, "Lock"))
    {
      return fail("Lock=0");
    }
  }
  // 3. EEPROM（apply_servo_config:=true で差があるときだけ。トルク OFF が前提）
  for (size_t i = 0; i < n; ++i) {
    if (diag_[i].diffs.empty()) {continue;}
    const JointCalibration & c = *cal_[i];
    if (diag_[i].torque_enable != 0) {
      return fail("EEPROM settings differ but torque is ON; cannot apply (power-cycle the servos)");
    }
    if (!apply_servo_settings(c, diag_[i])) {return fail("apply_servo_config");}
  }
  // 4. Acceleration（毎回書く。0 や不一致ならトルクを ON にしない）と Torque_Limit
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    if (!write_check("Acceleration", c, write_bus_->write_acceleration(c.id, acceleration_))) {
      return fail("Acceleration");
    }
    int a = -1;
    if (!read_reg_retry(c, reg::kAcceleration, "Acceleration", &a) || a == 0 || a != acceleration_) {
      RCLCPP_ERROR(
        logger(), "%s id=%d: Acceleration reads back as %d (wanted %d, must be non-zero). Torque is NOT enabled",
        c.name.c_str(), int(c.id), a, acceleration_);
      return fail("Acceleration readback");
    }
    int tl = -1;
    if (!read_reg_retry(c, reg::kTorqueLimit, "Torque_Limit", &tl)) {return fail("Torque_Limit read");}
    if (tl != desired_torque_limit_[i]) {
      if (!write_check("Torque_Limit", c, write_bus_->write_torque_limit(c.id, desired_torque_limit_[i])) ||
        !verify_reg(c, reg::kTorqueLimit, desired_torque_limit_[i], "Torque_Limit"))
      {
        return fail("Torque_Limit");
      }
    }
  }
  // 5. Goal_Position = 現在位置（電源投入直後の Goal_Position は不明なので、トルク ON の前に必ず上書きする）
  std::vector<int> raw;
  std::string why;
  if (!read_raw_positions(&raw, &why)) {
    RCLCPP_ERROR(logger(), "initial read failed: %s", why.c_str());
    return fail("initial position read");
  }
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    if (!write_check("Goal_Position", c, write_bus_->write_goal_position(c.id, raw[i])) ||
      !verify_reg(c, reg::kGoalPosition, raw[i], "Goal_Position"))
    {
      return fail("Goal_Position");
    }
  }
  // 6. Torque_Enable=1
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    if (!write_check("Torque_Enable=1", c, write_bus_->write_torque_enable(c.id, true)) ||
      !verify_reg(c, reg::kTorqueEnable, 1, "Torque_Enable"))
    {
      return fail("Torque_Enable=1");
    }
  }
  // 7. Lock=1
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    if (!write_check("Lock=1", c, write_bus_->write_lock(c.id, true)) || !verify_reg(c, reg::kLock, 1, "Lock")) {
      return fail("Lock=1");
    }
  }
  // 8. 全項目を読み直して検証
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    int gv = -1;
    if (!verify_reg(c, reg::kTorqueEnable, 1, "Torque_Enable") || !verify_reg(c, reg::kLock, 1, "Lock") ||
      !verify_reg(c, reg::kAcceleration, acceleration_, "Acceleration") ||
      !verify_reg(c, reg::kTorqueLimit, desired_torque_limit_[i], "Torque_Limit") ||
      !read_reg_retry(c, reg::kGoalVelocity, "Goal_Velocity", &gv) || gv != 0)
    {
      return fail("final verification");
    }
  }
  if (probe_lock_sram_ && !probe_lock_sram()) {return fail("probe_lock_sram");}

  // 状態の初期化（指令は現在位置から始める）
  target_raw_ = raw;
  max_step_raw_.assign(n, 1);
  for (size_t i = 0; i < n; ++i) {
    const double tick = tick_in_rad(*cal_[i], calibration_.angle_ticks);
    max_step_raw_[i] = std::max(1, static_cast<int>(std::floor(max_joint_speed_ * limit_window_s_ / tick)));
    hw_positions_[i] = raw_to_rad(*cal_[i], calibration_.angle_ticks, raw[i]);
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = hw_positions_[i];
  }
  limited_count_.assign(n, 0);
  last_limit_warn_s_.assign(n, -1e9);
  anomaly_run_ = 0;
  have_monitor_time_ = false;
  monitor_fail_run_ = 0;
  monitors_ = goal_writes_ = 0;
  armed_ = true;
  RCLCPP_INFO(
    logger(), "write mode active: torque ON, Lock=1, Acceleration=%d, Goal_Position = current position. "
    "Per-cycle step limit: %d ticks (angle joints). startup_read_retries=%lu", acceleration_, max_step_raw_[0],
    startup_retries_);
  return true;
}

bool Sts3215System::shutdown_servos(const char * why)
{
  armed_ = false;
  if (!touched_ || !write_bus_ || !port_.is_open()) {return true;}
  if (shutdown_attempts_ >= 2) {return false;}  // 通信が死んでいるとき、終了のたびに待たない
  ++shutdown_attempts_;
  bool all_ok = true;
  // 1. 全関節のトルク OFF（読み戻し、3 回まで）
  for (size_t i = 0; i < cal_.size(); ++i) {
    const JointCalibration & c = *cal_[i];
    bool done = false;
    for (int attempt = 0; attempt < 3 && !done; ++attempt) {
      int v = -1;
      done = write_bus_->write_torque_enable(c.id, false) == Status::Ok && read_reg(c.id, reg::kTorqueEnable, &v) &&
        v == 0;
    }
    if (!done) {
      all_ok = false;
      RCLCPP_ERROR(
        logger(), "[%s] %s id=%d: could NOT confirm Torque_Enable=0 (%s). The servo may still hold its last goal "
        "with torque ON: power it off", why, c.name.c_str(), int(c.id), bus_->last_error().c_str());
    }
  }
  // 2. 全関節の Lock=1 に戻す（読み戻し、3 回まで）。LeRobot は Lock=0 のまま残していた。EEPROM を保護する
  for (size_t i = 0; i < cal_.size(); ++i) {
    const JointCalibration & c = *cal_[i];
    bool done = false;
    for (int attempt = 0; attempt < 3 && !done; ++attempt) {
      int v = -1;
      done = write_bus_->write_lock(c.id, true) == Status::Ok && read_reg(c.id, reg::kLock, &v) && v == 1;
    }
    if (!done) {
      all_ok = false;
      RCLCPP_ERROR(
        logger(), "[%s] %s id=%d: could NOT confirm Lock=1 (%s)", why, c.name.c_str(), int(c.id),
        bus_->last_error().c_str());
    }
  }
  if (all_ok) {
    RCLCPP_INFO(logger(), "[%s] torque OFF and Lock=1 confirmed (read back) on all joints", why);
    touched_ = false;
    shutdown_attempts_ = 0;
  }
  return all_ok;
}

void Sts3215System::set_fault(const std::string & reason, bool latch_err)
{
  if (latch_err) {err_latched_ = true;}
  if (fault_) {return;}
  fault_ = true;
  fault_reason_ = reason;
  RCLCPP_ERROR(
    logger(), "FAULT: %s. Goal_Position is no longer sent. Torque stays as it is (a powered servo keeps holding its "
    "last goal). %s", reason.c_str(),
    latch_err ? "Restart the launch (process) to recover." : "Deactivate (torque OFF) and activate again to recover.");
}

void Sts3215System::monitor_servos(const rclcpp::Time & time)
{
  if (!armed_ || fault_) {return;}
  const double t = time.seconds();
  if (!have_monitor_time_) {
    last_monitor_s_ = t;
    have_monitor_time_ = true;
    return;
  }
  if (t - last_monitor_s_ < monitor_period_s_) {return;}
  last_monitor_s_ = t;
  ++monitors_;
  std::vector<std::vector<uint8_t>> out;
  uint8_t failed = 0;
  Status s = bus_->sync_read(ids_, reg::kTorqueEnableAcceleration, &out, &failed);
  if (s != Status::Ok && s != Status::ServoError) {
    s = bus_->sync_read(ids_, reg::kTorqueEnableAcceleration, &out, &failed);  // 1 回だけ再試行
  }
  if (s == Status::ServoError) {
    set_fault(
      "servo protection error in monitor read (" + bus_->last_error() + ")", true);
    return;
  }
  if (s != Status::Ok) {
    if (++monitor_fail_run_ >= 3) {
      set_fault("monitor read failed 3 times in a row (" + bus_->last_error() + "); cannot verify torque", false);
    } else {
      RCLCPP_WARN(logger(), "monitor read failed (%s); %d/3", bus_->last_error().c_str(), monitor_fail_run_);
    }
    return;
  }
  monitor_fail_run_ = 0;
  for (size_t i = 0; i < out.size(); ++i) {
    const int te = out[i][0], acc = out[i][1];
    if (te != 1 || acc != acceleration_) {
      std::ostringstream os;
      os << "joint " << cal_[i]->name << ": Torque_Enable=" << te << " Acceleration=" << acc << " (expected 1 / "
         << acceleration_ << "): the servo was powered off or reset (SRAM returned to its power-on values); the "
         << "arm is limp";
      set_fault(os.str(), false);
      return;
    }
  }
}

hardware_interface::return_type Sts3215System::write_goal_positions(const rclcpp::Time & time)
{
  if (!armed_ || fault_) {return hardware_interface::return_type::OK;}
  const size_t n = cal_.size();
  const double t = time.seconds();
  std::vector<int> goals(n);
  bool anomalous = false;
  for (size_t i = 0; i < n; ++i) {
    const JointCalibration & c = *cal_[i];
    const int prev = target_raw_[i];
    int want = prev;
    const double cmd = hw_commands_[i];
    if (!std::isfinite(cmd)) {
      anomalous = true;  // NaN / inf は破棄して直前の目標を保持
    } else {
      want = rad_to_raw(c, calibration_.angle_ticks, cmd);
      if (want < c.raw_min - kClampToleranceTicks || want > c.raw_max + kClampToleranceTicks) {anomalous = true;}
      want = std::min(std::max(want, c.raw_min), c.raw_max);
      const int d = want - prev;
      if (std::abs(d) > max_step_raw_[i]) {
        const int limited = prev + (d > 0 ? 1 : -1) * max_step_raw_[i];
        ++limited_count_[i];
        if (t - last_limit_warn_s_[i] >= 5.0) {
          last_limit_warn_s_[i] = t;
          RCLCPP_WARN(
            logger(), "step limit: '%s' requested %+d ticks in one cycle, limited to %d (%s; limited %lu times so "
            "far)", c.name.c_str(), d, max_step_raw_[i],
            c.kind == JointKind::LinearPercent ? "gripper goal is a step: ramping toward it" :
            "unexpected jump for an angle joint", limited_count_[i]);
        }
        want = limited;
      }
    }
    goals[i] = want;
  }
  if (anomalous) {
    if (++anomaly_run_ >= anomaly_cycles_) {
      set_fault("commands were invalid (NaN/inf or far outside the joint range) for " +
        std::to_string(anomaly_run_) + " cycles", false);
      return hardware_interface::return_type::OK;
    }
  } else {
    anomaly_run_ = 0;
  }
  const Status s = write_bus_->sync_write_goal_position(ids_, goals);
  if (s != Status::Ok) {
    set_fault(std::string("Goal_Position write failed: ") + to_string(s) + ": " + bus_->last_error(), false);
    return hardware_interface::return_type::OK;
  }
  target_raw_ = goals;
  ++goal_writes_;
  return hardware_interface::return_type::OK;
}

}  // namespace so_arm101_hardware

PLUGINLIB_EXPORT_CLASS(so_arm101_hardware::Sts3215System, hardware_interface::SystemInterface)
