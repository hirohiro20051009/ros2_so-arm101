// Copyright 2026 so_arm101 authors. MIT License.
//
// ツールの版の表記（1 か所）。測定ファイルの `tool`、バックアップの `tool`、yaml の出所の `applied_by` が同じ値を使う。
// package.xml の <version> と同じにする（試験 `Version.MatchesPackageXml` が照合する）。
#pragma once

#include <string>

namespace so_arm101_calibration
{

constexpr const char * kToolName = "so_arm101_calib";
constexpr const char * kToolVersion = "0.2.0";

/// "so_arm101_calib 0.2.0"
inline std::string tool_id() {return std::string(kToolName) + " " + kToolVersion;}

}  // namespace so_arm101_calibration
