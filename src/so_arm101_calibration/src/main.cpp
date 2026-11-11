// Copyright 2026 so_arm101 authors. MIT License.
//
// so_arm101_calib: サーボ EEPROM のバックアップと復元（段階2c-1）。実行するのはユーザーだけ。
// ros2_control（launch）を止めてから使う（シリアルポートを占有する）。
//
//   so_arm101_calib backup  [--port P] [--calibration Y] [--out FILE]            読み取り専用
//   so_arm101_calib restore --backup FILE [--joints a,b] [--write] [--rewrite-same]
//                           [--eeprom-wait-ms N]                                  既定は dry-run
//   so_arm101_calib measure [--out FILE] [--backup FILE]    測定（EEPROM には書かない。トルク OFF だけ書く）
//   so_arm101_calib compare --a FILE --b FILE [--calibration yaml]   オフライン（ポートを開かない）
//   so_arm101_calib apply   --measure FILE --backup FILE [--joints a,b] [--write]  既定は dry-run
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>

#include "so_arm101_calibration/apply.hpp"
#include "so_arm101_calibration/eeprom_backup.hpp"
#include "so_arm101_calibration/measure.hpp"
#include "so_arm101_calibration/restore.hpp"
#include "so_arm101_hardware/calibration.hpp"
#include "so_arm101_hardware/serial_port.hpp"

using namespace so_arm101_calibration;  // NOLINT

namespace
{
struct Args
{
  std::string cmd;
  std::map<std::string, std::string> opt;
  bool has(const std::string & k) const {return opt.count(k) > 0;}
  std::string get(const std::string & k, const std::string & d = "") const
  {
    auto it = opt.find(k);
    return it == opt.end() ? d : it->second;
  }
};

bool parse_args(int argc, char ** argv, Args * a, std::string * err)
{
  if (argc < 2) {
    *err = "no command";
    return false;
  }
  a->cmd = argv[1];
  const char * flags[] = {"--write", "--rewrite-same", "--help"};
  const char * values[] = {"--port", "--calibration", "--out", "--backup", "--joints", "--eeprom-wait-ms", "--timeout-ms", "--measure", "--a", "--b"};
  for (int i = 2; i < argc; ++i) {
    const std::string k = argv[i];
    bool found = false;
    for (auto f : flags) {
      if (k == f) {a->opt[k] = "1"; found = true;}
    }
    for (auto v : values) {
      if (k == v) {
        if (i + 1 >= argc) {*err = k + " needs a value"; return false;}
        a->opt[k] = argv[++i];
        found = true;
      }
    }
    if (!found) {*err = "unknown option " + k; return false;}
  }
  return true;
}

void usage()
{
  std::cout <<
    "so_arm101_calib backup  [--port /dev/ttyACM0] [--calibration yaml] [--out FILE] [--timeout-ms 100]\n"
    "    全関節の EEPROM(0..39) を読んで保存する。読み取り専用（何も書かない）。\n"
    "so_arm101_calib restore --backup FILE [--joints name,name|id,id] [--write] [--rewrite-same]\n"
    "                        [--eeprom-wait-ms 50] [--port ...] [--calibration yaml] [--timeout-ms 100]\n"
    "    バックアップから Homing_Offset と Min/Max_Position_Limit を戻す。既定は dry-run。\n"
    "    書くには --write と、確認文字列 WRITE-EEPROM の入力が必要。\n"
    "so_arm101_calib measure [--out FILE] [--backup FILE] [--port ...] [--calibration yaml]\n"
    "    脱力 → 中央 → MIN/MAX を測って測定ファイルに保存する。EEPROM には書かない（Lock にも触れない）。\n"
    "    書くのは Torque_Enable=0 だけ（トルクが入っているときだけ）。\n"
    "so_arm101_calib compare --a FILE --b FILE [--calibration yaml]\n"
    "    2 つの測定ファイルの再現性と、今の yaml との差を表示する。ポートは開かない。\n"
    "so_arm101_calib apply --measure FILE --backup FILE [--joints name,name] [--write] [--eeprom-wait-ms 50]\n"
    "    測定ファイルの値を EEPROM へ書く。既定は dry-run。--write と WRITE-EEPROM の入力が必要。\n"
    "    全関節を 1 回で書いたときだけ、--calibration の yaml を 1 回更新する。失敗したら全部バックアップへ戻す。\n"
    "ros2_control（launch）を止めてから実行する。実行するのはユーザーだけ。\n";
}

bool to_int(const std::string & s, int * v)
{
  try {
    size_t n = 0;
    *v = std::stoi(s, &n);
    return n == s.size();
  } catch (...) {return false;}
}

/// 記録中の表示を、制御端末（/dev/tty）に直接書く。開けなければ使わない（ログの行だけになる）。
class TtyLive
{
public:
  ~TtyLive() {if (fd_ >= 0) {::close(fd_);}}
  bool open()
  {
    fd_ = ::open("/dev/tty", O_WRONLY | O_NOCTTY | O_NONBLOCK);
    return fd_ >= 0;
  }
  /// 前のフレームの行数だけ上へ戻り、各行を消して書き直す。1 回の write で出し、書けなければ（EAGAIN）そのフレームを捨てる。
  void draw(const std::vector<std::string> & rows)
  {
    std::string buf;
    if (lines_ > 0) {buf += "\033[" + std::to_string(lines_) + "A";}
    for (const auto & r : rows) {buf += "\r\033[K" + r + "\n";}
    const ssize_t n = ::write(fd_, buf.data(), buf.size());
    if (n == static_cast<ssize_t>(buf.size())) {lines_ = static_cast<int>(rows.size());}
  }

private:
  int fd_{-1};
  int lines_{0};
};
}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  std::string err;
  if (!parse_args(argc, argv, &a, &err) || a.has("--help") || (a.cmd != "backup" && a.cmd != "restore" && a.cmd != "measure" && a.cmd != "compare" && a.cmd != "apply")) {
    if (!err.empty()) {std::cerr << "ERROR: " << err << "\n";}
    usage();
    return (a.has("--help") && err.empty()) ? 0 : kExitInput;
  }
  const std::string cal_path = a.get(
    "--calibration",
    ament_index_cpp::get_package_share_directory("so_arm101_hardware") + "/config/follower_calibration.yaml");
  so_arm101_hardware::Calibration cal;
  try {
    cal = so_arm101_hardware::load_calibration(cal_path);
  } catch (const std::exception & e) {
    std::cerr << "ERROR: cannot read the calibration yaml " << cal_path << ": " << e.what() << "\n";
    return kExitInput;
  }
  std::vector<JointId> all;
  for (const auto & j : cal.joints) {all.push_back({j.id, j.name});}
  int timeout_ms = 100;
  if (a.has("--timeout-ms") && (!to_int(a.get("--timeout-ms"), &timeout_ms) || timeout_ms < 1 || timeout_ms > 2000)) {
    std::cerr << "ERROR: --timeout-ms は 1〜2000\n";
    return kExitInput;
  }
  Options opt;
  if (a.has("--eeprom-wait-ms") && !to_int(a.get("--eeprom-wait-ms"), &opt.wait_ms)) {
    std::cerr << "ERROR: --eeprom-wait-ms は整数（0〜" << kMaxWaitMs << "）\n";
    return kExitInput;
  }
  opt.rewrite_same = a.has("--rewrite-same");

  // compare: オフライン（ポートを開かない）
  if (a.cmd == "compare") {
    MeasureFile ma, mb;
    if (!load_measure_file(a.get("--a"), &ma, &err) || !load_measure_file(a.get("--b"), &mb, &err)) {
      std::cerr << "ERROR: 測定ファイルを読めない/壊れている: " << err << "\n";
      return kExitInput;
    }
    std::vector<std::pair<std::string, YamlJoint>> yj;
    for (const auto & j : cal.joints) {
      YamlJoint y;
      y.kind = j.kind == so_arm101_hardware::JointKind::Angle ? "angle" : "linear_percent";
      y.center = j.center;
      y.min = j.raw_min;
      y.max = j.raw_max;
      y.homing_offset = j.eeprom.homing_offset;
      yj.push_back({j.name, y});
    }
    compare_measures(ma, mb, yj, cal.angle_ticks, std::cout);
    return kExitOk;
  }

  // restore / apply: 先にバックアップ（と測定）ファイルを検査する（ポートを開く前）
  Backup backup;
  MeasureFile measure;
  std::string backup_path = a.get("--backup");
  if (a.cmd == "apply") {
    if (a.get("--measure").empty()) {
      std::cerr << "ERROR: --measure が必要\n";
      return kExitInput;
    }
    if (!load_measure_file(a.get("--measure"), &measure, &err)) {
      std::cerr << "ERROR: 測定ファイルを読めない/壊れている: " << err << "。何も書かない（ポートも開かない）\n";
      return kExitInput;
    }
    if (const std::string why = measure_not_applicable_reason(measure); !why.empty()) {
      std::cerr << "ERROR: " << why << "。何も書かない（ポートも開かない）\n";
      return kExitInput;
    }
  }
  if (a.cmd == "restore" || a.cmd == "apply") {
    if (backup_path.empty()) {
      std::cerr << "ERROR: --backup が必要\n";
      return kExitInput;
    }
    if (!load_backup_file(backup_path, &backup, &err)) {
      std::cerr << "ERROR: バックアップを読めない/壊れている: " << err << "。何も書かない（ポートも開かない）\n";
      return kExitInput;
    }
  }
  std::vector<uint8_t> selected;
  if (a.has("--joints")) {
    std::stringstream ss(a.get("--joints"));
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      int id = -1;
      for (const auto & j : all) {
        if (j.name == tok || (to_int(tok, &id) && j.id == id)) {id = j.id; break;}
        id = -1;
      }
      if (id < 0) {
        std::cerr << "ERROR: 不明な関節 '" << tok << "'\n";
        return kExitInput;
      }
      selected.push_back(static_cast<uint8_t>(id));
    }
  }

  so_arm101_hardware::SerialPort port;
  const std::string path = a.get("--port", "/dev/ttyACM0");
  if (!port.open(path, cal.baudrate, &err)) {
    std::cerr << "ERROR: cannot open " << path << ": " << err << "（ros2_control が動いていないか確認）\n";
    return kExitPrecondition;
  }
  CalibWriteBus bus(port, std::chrono::milliseconds(timeout_ms));

  if (a.cmd == "backup") {
    // 読み取り専用。書き込み関数は呼ばない
    std::cout << "backup: 読み取り専用（何も書かない）。ポート " << path << "\n";
    Backup b;
    b.created = now_string(false);
    b.port = path;
    b.tool = "so_arm101_calib 0.1.0";
    for (const auto & j : all) {
      JointEeprom e, e2;
      if (!read_joint_eeprom(bus, j.id, j.name, 3, &e, &err) || !read_joint_eeprom(bus, j.id, j.name, 3, &e2, &err)) {
        std::cerr << "ERROR: " << err << "\nバックアップを保存しない\n";
        return kExitPrecondition;
      }
      if (e.bytes != e2.bytes) {
        std::cerr << "ERROR: " << j.name << ": 2 回読んだ EEPROM が一致しない。保存しない\n";
        return kExitPrecondition;
      }
      if (e.model != so_arm101_hardware::kModelNumberSts3215) {
        std::cerr << "ERROR: " << j.name << ": model number " << e.model << " (expected 777)。保存しない\n";
        return kExitPrecondition;
      }
      std::cout << "  " << j.name << " (id " << int(j.id) << "): Homing_Offset=" << e.homing_offset()
                << " Min=" << e.raw(CalibReg::MinPositionLimit) << " Max=" << e.raw(CalibReg::MaxPositionLimit)
                << "（2 回読んで一致）\n";
      b.joints.push_back(e);
    }
    std::string out_path = a.get("--out");
    if (out_path.empty()) {
      const char * home = std::getenv("HOME");
      out_path = std::string(home ? home : ".") + "/so_arm101_check/eeprom_backup_" + now_string(true) + ".json";
    }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(out_path).parent_path(), ec);
    if (!save_backup_file(out_path, b, &err)) {
      std::cerr << "ERROR: " << err << "\nバックアップを保存できなかった。先へ進まない\n";
      return kExitPrecondition;
    }
    std::cout << "保存して読み直し、一致を確認した: " << out_path << "\n";
    return kExitOk;
  }

  auto confirm_cli = []() {
      std::cout << "EEPROM に書き込む。続けるなら " << kConfirmText << " と入力して Enter: " << std::flush;
      std::string line;
      std::getline(std::cin, line);
      return line == kConfirmText;
    };

  if (a.cmd == "measure") {
    std::vector<std::string> kinds;
    for (const auto & j : cal.joints) {
      kinds.push_back(j.kind == so_arm101_hardware::JointKind::Angle ? "angle" : "linear_percent");
    }
    MeasureIO io;
    io.wait_enter = [](const std::string & prompt) {
        std::cout << "\n>>> " << prompt << " " << std::flush;
        std::string line;
        return static_cast<bool>(std::getline(std::cin, line));
      };
    // 中央の測定中（約 0.3 秒）に押された Enter が溜まっていると、記録がすぐ終わる。記録を始める直前に捨てる
    io.on_recording_start = []() {
        if (::isatty(STDIN_FILENO)) {::tcflush(STDIN_FILENO, TCIFLUSH);}
      };
    io.should_stop = []() {
        pollfd p{0, POLLIN, 0};
        if (::poll(&p, 1, 0) > 0 && (p.revents & POLLIN)) {
          std::string line;
          std::getline(std::cin, line);
          return true;
        }
        return false;
      };
    // 今の yaml の生の範囲（actual 座標 = min_limit/max_limit + Homing_Offset。wrist_roll は 4095 をまたぐ）
    for (const auto & j : cal.joints) {
      io.yaml_ref.push_back({j.raw_min + j.eeprom.homing_offset, j.raw_max + j.eeprom.homing_offset});
    }
    // 記録中の表示。ログ（stdout は tee につながるので ANSI は出さない）には 1 秒ごとの行が run_measure から出る。
    // 端末には /dev/tty を直接開いて、7 行を書き換える。O_NONBLOCK なので端末が詰まっても読み取りを止めない（そのフレームは捨てる）。
    TtyLive tty;
    if (tty.open()) {io.on_live = [&tty](const std::vector<LiveJoint> & v) {tty.draw(format_live_table(v));};}
    std::cout << "measure: EEPROM には書かない。書くのはトルクが入っている関節の Torque_Enable=0 だけ。ポート " << path << "\n";
    MeasureFile result;
    const int rc = run_measure(bus, all, kinds, opt, io, std::cout, &result);
    if (rc == kExitOk || rc == kExitInput) {
      if (result.joints.empty()) {return rc;}  // 中止などで結果が無い
      result.port = path;
      result.backup_file = backup_path;
      std::string out_path = a.get("--out");
      if (out_path.empty()) {
        const char * home = std::getenv("HOME");
        out_path = std::string(home ? home : ".") + "/so_arm101_check/measure_" + now_string(true) + ".json";
      }
      std::error_code ec;
      std::filesystem::create_directories(std::filesystem::path(out_path).parent_path(), ec);
      if (!save_measure_file(out_path, result, &err)) {
        std::cerr << "ERROR: " << err << "\n";
        return kExitPrecondition;
      }
      std::cout << "測定ファイルを保存して読み直し、一致を確認した: " << out_path << "\n";
    }
    return rc;
  }

  if (a.cmd == "apply") {
    ApplyArgs aa;
    aa.measure = measure;
    aa.measure_path = a.get("--measure");
    aa.backup = backup;
    aa.backup_path = backup_path;
    aa.all = all;
    aa.selected = selected;
    aa.opt = opt;
    aa.write = a.has("--write");
    aa.yaml_path = cal_path;
    const char * home = std::getenv("HOME");
    aa.fallback_yaml_path = std::string(home ? home : ".") + "/so_arm101_check/follower_calibration_new_" + now_string(true) + ".yaml";
    return run_apply(bus, aa, confirm_cli, std::cout);
  }

  RestoreArgs ra;
  ra.backup = backup;
  ra.backup_path = backup_path;
  ra.all = all;
  ra.selected = selected;
  ra.opt = opt;
  ra.write = a.has("--write");
  return run_restore(bus, ra, confirm_cli, std::cout);
}
