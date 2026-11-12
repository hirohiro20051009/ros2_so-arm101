// Copyright 2026 so_arm101 authors. MIT License.
//
// Feetech STS3215 を直接制御する ros2_control SystemInterface。
//   read_only:=true（既定。段階2b-2/2b-3a）: Present_Position と設定レジスタの読み取りのみ。トルクを含め、
//     サーボには一切書き込まない。command は破棄する。書き込み用のクラスは生成しない。
//   read_only:=false（段階2b-3b）: トルク ON と Goal_Position の送信。手順と異常時の扱いは
//     docs/inventory.md「段階2b-3 の計画 v3」「段階2b-3b の計画 v4」。
//   - velocity は位置の有限差分。
//   - スレッド・タイマー・非同期は使わない。1 Hz の監視も read() の中で行う。read/write/on_*/デストラクタは
//     1 本の mutex で守り、バスの 1 往復は SerialPort::Transaction で守る（シリアルポートの同時使用を防ぐ）。
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so_arm101_hardware/calibration.hpp"
#include "so_arm101_hardware/serial_port.hpp"
#include "so_arm101_hardware/servo_config.hpp"
#include "so_arm101_hardware/sts3215_bus.hpp"
#include "so_arm101_hardware/sts3215_write_bus.hpp"

namespace so_arm101_hardware
{

// 既定値。試験が joint_limits.yaml / controllers.yaml との整合を確認する。
constexpr double kDefaultMaxJointSpeed = 6.0;  // rad/s
constexpr double kDefaultLimitWindow = 0.08;   // s
constexpr int kDefaultAcceleration = 50;       // 単位は未確認。電源投入時は 0（意味は未確認）なので 0 は許さない
constexpr int kDefaultTorqueLimit = 1000;
constexpr int kGripperTorqueLimit = 500;
constexpr double kDefaultMonitorPeriod = 1.0;  // s
constexpr int kDefaultAnomalyCycles = 50;      // 異常な指令が続いたらフォルトにする周期数（50 Hz で 1 秒）
constexpr int kClampToleranceTicks = 50;       // 限界外の指令がこの tick 数以内なら黙ってクランプし、異常と数えない

class Sts3215System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Sts3215System)

  ~Sts3215System() override;

  /// 起動手順（書き込みモード）の読み戻しで行った再試行の回数。
  unsigned long startup_read_retries() const {return startup_retries_;}

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // --- 試験用の参照（mutex の外から呼んでよい読み取りだけ）---
  bool fault_latched() const;
  std::string fault_reason() const;
  unsigned long port_contention() const {return port_.contention_count();}
  unsigned long limited_total() const;
  unsigned long retried_total() const;        // 周期の読み取りを再試行した回数（試験・測定用）
  unsigned long failed_cycles_total() const;  // 再試行しても失敗した周期の数（同上）

private:
  /// 診断で読んだ、関節ごとの値（起動で使う）。
  struct Diag
  {
    int torque_enable{-1}, goal_velocity{0}, max_acceleration{-1};
    std::vector<ConfigDiff> diffs;
  };

  bool read_raw_positions(std::vector<int> * raw, std::string * why, Status * status = nullptr);
  bool run_diagnostics();
  bool run_servo_config_diagnostics();
  void invalidate_state();
  rclcpp::Logger logger() const {return rclcpp::get_logger("Sts3215System");}

  // --- 書き込みモード（read_only:=false）---
  bool activate_write_mode();                        ///< 起動の手順 1〜8（失敗したら後始末して false）
  bool read_reg(uint8_t id, Register r, int * value, Status * status = nullptr);
  /// 起動手順の読み戻し用。通信エラー（タイムアウト・フレーム不良・チェックサム不良）のときだけ、最大 3 回まで
  /// 読み直す。正しく受信できた値は、期待と違っても再試行しない（呼び出し側が判定する）。
  bool read_reg_retry(const JointCalibration & c, Register r, const char * name, int * value);
  bool read_settings(const JointCalibration & c, ServoSettings * out);
  bool write_check(const char * what, const JointCalibration & c, Status s);
  bool verify_reg(const JointCalibration & c, Register r, int expected, const char * name);
  bool apply_servo_settings(const JointCalibration & c, const Diag & d);
  bool probe_lock_sram();
  bool shutdown_servos(const char * why);            ///< 終了の手順: トルク OFF → Lock=1（読み戻し、3 回まで）
  void set_fault(const std::string & reason, bool latch_err);
  void monitor_servos(const rclcpp::Time & time);    ///< 1 Hz の監視（read() の中から呼ぶ）
  hardware_interface::return_type write_goal_positions(const rclcpp::Time & time);

  // パラメータ
  std::string serial_port_;
  std::string calibration_file_;
  int timeout_ms_{10};
  int drain_quiet_ms_{2};  // ハードウェアパラメータ drain_quiet_ms（既定 2。0 で従来どおり）
  int max_consecutive_failures_{10};
  int acceleration_{kDefaultAcceleration};
  int torque_limit_{kDefaultTorqueLimit};
  double max_joint_speed_{kDefaultMaxJointSpeed};
  double limit_window_s_{kDefaultLimitWindow};
  double monitor_period_s_{kDefaultMonitorPeriod};
  int anomaly_cycles_{kDefaultAnomalyCycles};
  bool apply_servo_config_{false};
  bool probe_lock_sram_{false};

  Calibration calibration_;
  std::vector<const JointCalibration *> cal_;  // info_.joints と同じ順
  std::vector<uint8_t> ids_;

  SerialPort port_;
  std::unique_ptr<Sts3215Bus> bus_;  // read_only のときは Sts3215Bus、書き込みモードでは Sts3215WriteBus
  Sts3215WriteBus * write_bus_{nullptr};  // 書き込みモードのときだけ非 null（bus_ が所有）
  std::vector<Diag> diag_;

  std::vector<double> hw_commands_, hw_positions_, hw_velocities_;
  bool have_last_{false};
  double last_time_s_{0.0};

  // 失敗の扱い
  int consecutive_failures_{0};
  unsigned long reads_{0}, retried_{0}, failed_cycles_{0};
  unsigned long startup_retries_{0};  ///< 起動手順の読み戻しで行った再試行の回数（W0 の報告用）
  bool warned_discard_{false};
  bool read_only_{true};

  // 書き込みモードの状態
  mutable std::mutex mu_;
  bool touched_{false};        ///< 何かを書いた（終了の手順で戻す必要がある）
  bool armed_{false};          ///< トルク ON で Goal_Position を送っている
  bool fault_{false};          ///< 送信停止（トルクは保持）
  std::string fault_reason_;
  bool err_latched_{false};    ///< サーボの ERR バイトを読んだ。プロセスの再起動までは再開しない
  int shutdown_attempts_{0};
  std::vector<int> target_raw_, max_step_raw_, desired_torque_limit_;
  int anomaly_run_{0};
  double last_monitor_s_{0.0};
  bool have_monitor_time_{false};
  int monitor_fail_run_{0};
  unsigned long monitors_{0}, goal_writes_{0};
  std::vector<unsigned long> limited_count_;
  std::vector<double> last_limit_warn_s_;
};

}  // namespace so_arm101_hardware
