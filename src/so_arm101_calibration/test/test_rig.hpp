// Copyright 2026 so_arm101 authors. MIT License.
//
// 試験用の共通部品: 偽サーボ（pty）と、yaml の値で初期化した EEPROM を持つ Rig。実機・実ポートは開かない。
#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "fake_servo.hpp"
#include "so_arm101_calibration/calib_write_bus.hpp"
#include "so_arm101_calibration/eeprom_backup.hpp"
#include "so_arm101_calibration/eeprom_ops.hpp"
#include "so_arm101_calibration/restore.hpp"
#include "so_arm101_hardware/calibration.hpp"
#include "so_arm101_hardware/serial_port.hpp"

namespace rig
{
using namespace so_arm101_calibration;  // NOLINT
struct Rig
{
  fake::FakeServos servos{{1, 2, 3, 4, 5, 6}, 6};
  so_arm101_hardware::Calibration cal{so_arm101_hardware::load_calibration(CALIBRATION_YAML)};
  so_arm101_hardware::SerialPort port;
  std::unique_ptr<CalibWriteBus> bus;
  std::vector<JointId> all;

  Rig()
  {
    for (const auto & j : cal.joints) {
      servos.set_eeprom(j.id, 31, j.eeprom.homing_offset, 2);
      servos.set_eeprom(j.id, 9, j.eeprom.min_limit, 2);
      servos.set_eeprom(j.id, 11, j.eeprom.max_limit, 2);
      all.push_back({j.id, j.name});
    }
    std::string err;
    EXPECT_TRUE(port.open(servos.path(), 1000000, &err)) << err;
    bus = std::make_unique<CalibWriteBus>(port, std::chrono::milliseconds(30));
  }
  Backup backup()
  {
    Backup b;
    b.created = "2026-11-11T00:00:00";
    b.port = "fake";
    b.tool = "test";
    for (const auto & j : all) {
      JointEeprom e;
      std::string err;
      EXPECT_TRUE(read_joint_eeprom(*bus, j.id, j.name, 3, &e, &err)) << err;
      b.joints.push_back(e);
    }
    return b;
  }
  RestoreArgs args(const Backup & b, bool write = true)
  {
    RestoreArgs a;
    a.backup = b;
    a.backup_path = "BACKUP.json";
    a.all = all;
    a.opt.wait_ms = 0;
    a.write = write;
    return a;
  }
  int run(const RestoreArgs & a, std::string * text = nullptr, bool confirm = true)
  {
    std::ostringstream o;
    const int rc = run_restore(*bus, a, [confirm]() {return confirm;}, o);
    if (text) {*text = o.str();}
    return rc;
  }
  /// 全関節の Homing/Min/Max を、バックアップと違う値にする（Lock=1 のまま。実機の EEPROM が書き換わった状況）。
  void scramble()
  {
    for (const auto & j : all) {
      servos.set_eeprom(j.id, 31, 500 + j.id, 2);
      servos.set_eeprom(j.id, 9, 100 + j.id, 2);
      servos.set_eeprom(j.id, 11, 4000 + j.id, 2);
    }
  }
  bool equals(const Backup & b)
  {
    for (const auto & j : b.joints) {
      if (servos.u16(j.id, 31) != j.raw(CalibReg::HomingOffset) || servos.u16(j.id, 9) != j.raw(CalibReg::MinPositionLimit) ||
        servos.u16(j.id, 11) != j.raw(CalibReg::MaxPositionLimit)) {return false;}
    }
    return true;
  }
};

int index_of(const std::vector<fake::WriteRecord> & w, int id, int addr, int from = 0)
{
  for (size_t i = from; i < w.size(); ++i) {
    if (w[i].id == id && w[i].addr == addr) {return static_cast<int>(i);}
  }
  return -1;
}
int count_writes(const std::vector<fake::WriteRecord> & w, int id, int addr)
{
  int n = 0;
  for (const auto & r : w) {n += (r.id == id && r.addr == addr);}
  return n;
}
bool contains(const std::string & s, const std::string & t) {return s.find(t) != std::string::npos;}
}  // namespace rig
