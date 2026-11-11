// Copyright 2026 so_arm101 authors. MIT License.
#include "so_arm101_calibration/eeprom_ops.hpp"

#include <chrono>
#include <ostream>
#include <thread>

namespace so_arm101_calibration
{
using so_arm101_hardware::Register;
using so_arm101_hardware::Status;
using so_arm101_hardware::Sts3215Bus;

Status read_calib_reg(Sts3215Bus & bus, uint8_t id, CalibReg r, int retries, uint16_t * out)
{
  Status s = Status::Ok;
  for (int t = 0; t <= retries; ++t) {
    s = bus.read_u16(id, register_of(r), out);
    if (s == Status::Ok || !Sts3215Bus::is_comm_error(s)) {break;}
  }
  return s;
}

JointRegs read_joint_regs(Sts3215Bus & bus, uint8_t id, int retries)
{
  JointRegs j;
  for (int i = 0; i < kNumCalibRegs; ++i) {
    uint16_t v = 0;
    if (read_calib_reg(bus, id, static_cast<CalibReg>(i), retries, &v) == Status::Ok) {j.v[i] = v;}
  }
  return j;
}

JointRegs read_joint_regs_confirmed(Sts3215Bus & bus, uint8_t id, int retries)
{
  // 独立に 2 回読み、2 回とも読めて同じ値のものだけを返す（古い応答が 1 回混ざっても「一致」にしない）
  const JointRegs a = read_joint_regs(bus, id, retries);
  const JointRegs b = read_joint_regs(bus, id, retries);
  JointRegs j;
  for (int i = 0; i < kNumCalibRegs; ++i) {
    if (a.v[i] && b.v[i] && *a.v[i] == *b.v[i]) {j.v[i] = a.v[i];}
  }
  return j;
}

static Status read_u8_retry(Sts3215Bus & bus, uint8_t id, Register r, int retries, int * out)
{
  Status s = Status::Ok;
  std::vector<uint8_t> d;
  for (int t = 0; t <= retries; ++t) {
    s = bus.read(id, r, &d);
    if (s == Status::Ok || !Sts3215Bus::is_comm_error(s)) {break;}
  }
  if (s == Status::Ok && !d.empty()) {*out = d[0];}
  return s;
}

int read_lock(Sts3215Bus & bus, uint8_t id, int retries)
{
  int v = -1;
  if (read_u8_retry(bus, id, so_arm101_hardware::reg::kLock, retries, &v) != Status::Ok) {return -1;}
  return v;
}

static std::string describe(Sts3215Bus & bus, Status s)
{
  return std::string(so_arm101_hardware::to_string(s)) + ": " + bus.last_error();
}

StepResult write_calib_verified(CalibWriteBus & bus, uint8_t id, CalibReg r, uint16_t raw, const Options & opt)
{
  const std::string what = std::string(to_string(r)) + "=" + std::to_string(raw) + " (id " + std::to_string(id) + ")";
  Status s = Status::Ok;
  for (int t = 0; t <= opt.retries; ++t) {
    s = bus.write_calib_raw(id, r, raw);
    if (s == Status::Ok || !Sts3215Bus::is_comm_error(s)) {break;}  // 通信エラーのときだけ再送（同じ値）
  }
  if (s != Status::Ok) {return {false, "write failed " + what + " [" + describe(bus, s) + "]"};}
  if (opt.wait_ms > 0) {std::this_thread::sleep_for(std::chrono::milliseconds(opt.wait_ms));}
  // 読み戻しを独立に 2 回行い、2 回とも期待値のときだけ一致とする（古い応答が 1 回混ざっても、
  // 実際の値が違うのに「一致」にならない。片方だけ一致は失敗＝呼び出し側が戻す）
  uint16_t back = 0;
  s = read_calib_reg(bus, id, r, opt.retries, &back);
  if (s != Status::Ok) {return {false, "read-back failed " + what + " [" + describe(bus, s) + "]"};}
  if (back != raw) {
    return {false, "read-back mismatch " + what + ": read " + std::to_string(back) + " after " +
           std::to_string(opt.wait_ms) + " ms (not retried)"};
  }
  uint16_t back2 = 0;
  s = read_calib_reg(bus, id, r, opt.retries, &back2);
  if (s != Status::Ok) {return {false, "second read-back failed " + what + " [" + describe(bus, s) + "]"};}
  if (back2 != raw) {
    return {false, "read-back mismatch " + what + ": the first read was " + std::to_string(back) +
           " but the second read was " + std::to_string(back2) + " (the two reads must agree; not retried)"};
  }
  return {};
}

StepResult set_lock_verified(CalibWriteBus & bus, uint8_t id, bool locked, const Options & opt)
{
  const std::string what = std::string("Lock=") + (locked ? "1" : "0") + " (id " + std::to_string(id) + ")";
  Status s = Status::Ok;
  for (int t = 0; t <= opt.retries; ++t) {
    s = bus.write_lock(id, locked);
    if (s == Status::Ok || !Sts3215Bus::is_comm_error(s)) {break;}
  }
  if (s != Status::Ok) {return {false, "write failed " + what + " [" + describe(bus, s) + "]"};}
  const int v = read_lock(bus, id, opt.retries);
  if (v < 0) {return {false, "read-back failed " + what};}
  if (v != (locked ? 1 : 0)) {return {false, "read-back mismatch " + what + ": read " + std::to_string(v)};}
  return {};
}

JointWriteResult write_joint_sequence(
  CalibWriteBus & bus, uint8_t id, const std::optional<uint16_t> (&values)[kNumCalibRegs], const Options & opt,
  std::ostream * progress, const std::string & name)
{
  JointWriteResult r;
  bool any = false;
  for (int i = 0; i < kNumCalibRegs; ++i) {any = any || values[i].has_value();}
  if (!any) {return r;}
  if (progress) {*progress << "-- " << name << " (id " << int(id) << "): Lock=0\n";}
  r.touched = true;
  StepResult s = set_lock_verified(bus, id, false, opt);
  if (!s.ok) {
    r.problems.push_back(s.msg);
    r.ok = false;
  }
  for (int i = 0; i < kNumCalibRegs && r.ok; ++i) {
    if (!values[i]) {continue;}
    if (progress) {*progress << "   " << to_string(static_cast<CalibReg>(i)) << " <- " << *values[i] << "\n";}
    s = write_calib_verified(bus, id, static_cast<CalibReg>(i), *values[i], opt);
    if (!s.ok) {
      r.problems.push_back(s.msg);
      r.ok = false;
    }
  }
  s = set_lock_verified(bus, id, true, opt);
  if (!s.ok) {
    r.problems.push_back(s.msg);
    r.lock_ok = false;
    r.ok = false;
  }
  return r;
}

bool preflight(Sts3215Bus & bus, const std::vector<JointId> & all, int retries, std::ostream & out)
{
  bool ok = true;
  for (const auto & j : all) {
    uint16_t model = 0;
    Status s = Status::Ok;
    for (int t = 0; t <= retries; ++t) {
      s = bus.read_u16(j.id, so_arm101_hardware::reg::kModelNumber, &model);
      if (s == Status::Ok || !Sts3215Bus::is_comm_error(s)) {break;}
    }
    if (s != Status::Ok) {
      out << "NG  " << j.name << " (id " << int(j.id) << "): cannot read the servo [" << describe(bus, s)
          << "]. 電源・ケーブル・ID を確認\n";
      ok = false;
      continue;
    }
    if (model != so_arm101_hardware::kModelNumberSts3215) {
      out << "NG  " << j.name << " (id " << int(j.id) << "): model number " << model << " (expected 777)\n";
      ok = false;
      continue;
    }
    int torque = -1;
    s = read_u8_retry(bus, j.id, so_arm101_hardware::reg::kTorqueEnable, retries, &torque);
    if (s != Status::Ok) {
      out << "NG  " << j.name << ": cannot read Torque_Enable [" << describe(bus, s) << "]\n";
      ok = false;
    } else if (torque != 0) {
      out << "NG  " << j.name << " (id " << int(j.id) << "): Torque_Enable=" << torque
          << "（トルクが入っている。EEPROM は書かない。ros2_control を止めて、トルクを OFF にしてから）\n";
      ok = false;
    } else {
      out << "OK  " << j.name << " (id " << int(j.id) << "): model 777, Torque_Enable=0\n";
    }
  }
  return ok;
}

std::string classify(const std::optional<uint16_t> & cur, uint16_t before, uint16_t target)
{
  if (!cur) {return "unknown";}
  if (*cur == target && *cur == before) {return "same";}
  if (*cur == target) {return "target";}
  if (*cur == before) {return "before";}
  return "unknown";
}

std::string show_value(CalibReg r, const std::optional<uint16_t> & v)
{
  if (!v) {return "(read failed)";}
  std::string s = std::to_string(*v);
  if (r == CalibReg::HomingOffset) {
    s += " (" + std::to_string(so_arm101_hardware::decode_sign_magnitude(*v, so_arm101_hardware::kHomingOffsetSignBit)) + ")";
  }
  return s;
}

}  // namespace so_arm101_calibration
