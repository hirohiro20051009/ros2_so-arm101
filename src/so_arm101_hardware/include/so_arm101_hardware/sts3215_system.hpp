// Copyright 2026 so_arm101 authors. MIT License.
//
// Feetech STS3215 を直接読む ros2_control SystemInterface（段階2b-2: 読み取り専用）。
//   - Present_Position の読み取りのみ。トルクを含め、サーボのレジスタには一切書き込まない。
//   - command は破棄する（write() は何もしない）。
//   - velocity は位置の有限差分（So101HardwareBridge の bridge と同じ扱い）。
#pragma once

#include <memory>
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

namespace so_arm101_hardware
{

class Sts3215System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Sts3215System)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 全関節の Present_Position を 1 回 SYNC_READ して raw を得る（失敗は説明つきで false）。
  bool read_raw_positions(std::vector<int> * raw, std::string * why);
  /// 起動時診断（EEPROM と yaml の比較）。致命的な失敗なら false。
  bool run_diagnostics();
  /// 段階2b-3a: 設定レジスタを読んで LeRobot の値と比較し、結果を [diag] で表示する（読み取りだけ）。
  /// 読み取りに失敗、または read_only:=false で不一致なら false（起動を中止）。read_only:=true の不一致は WARN のみ。
  bool run_servo_config_diagnostics();
  /// state / command を NaN にする（古い値が有効値として流れ続けないように）。
  void invalidate_state();
  rclcpp::Logger logger() const {return rclcpp::get_logger("Sts3215System");}

  // パラメータ
  std::string serial_port_;
  std::string calibration_file_;
  int timeout_ms_{10};
  int max_consecutive_failures_{10};

  Calibration calibration_;
  std::vector<const JointCalibration *> cal_;  // info_.joints と同じ順
  std::vector<uint8_t> ids_;

  SerialPort port_;
  std::unique_ptr<Sts3215Bus> bus_;

  std::vector<double> hw_commands_, hw_positions_, hw_velocities_;
  bool have_last_{false};
  double last_time_s_{0.0};

  // 失敗の扱い
  int consecutive_failures_{0};
  unsigned long reads_{0}, retried_{0}, failed_cycles_{0};
  bool warned_discard_{false};
  bool read_only_{true};
};

}  // namespace so_arm101_hardware
