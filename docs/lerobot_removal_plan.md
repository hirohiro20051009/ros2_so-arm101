# LeRobot 経路の削除の計画（2026-11-12。第 2 版。L1 は 2026-11-12 に実施済み（下の「進捗」）。L2 以降は未実施）

ユーザーの判断（`inventory.md`「ユーザーの判断の記録」）: **LeRobot 経路は、このプロジェクトの ROS パッケージから削除する。**
この文書は洗い出し・分類・段階・確認の計画。**コード・launch・xacro・URDF・yaml・CMake・package.xml は変更していない。実機・シリアルポートにも触れていない。**

**方針（ユーザー判断）**: LeRobot の完全排除と、必要最小限のパッケージにすることを優先する。問題が出たらその都度対策する。
**削除の方法（ユーザー判断）**: `archive/` は作らず、**`git rm` で完全に削除する**（so_arm101_lerobot_bridge、`tools/gen_conversion_golden.py`、`tools/lerobot_readonly_state.py`、`tools/convert_lerobot_calibration.py`、nimiCurtis 由来のコード、`Gili.json`・`Tzili.json` を含む）。
**戻すときは、タグ `stage2c-done`（コミット 1a3628e）から取り出せる**（例: `git checkout stage2c-done -- src/so_arm101_lerobot_bridge`）。

> **注意（CLAUDE.md との関係）**: CLAUDE.md の規則は「ファイルの削除はしない。不要なものは archive/ へ移動し、理由を記録する」。今回のユーザー判断（git rm で削除、タグから戻す）はこの規則と異なる。**CLAUDE.md は変更していない**。実施の承認と合わせて、CLAUDE.md の規則を直すかをユーザーが決める（直さない場合、実施のたびに例外の承認が要る）。

凡例: **[事実]** = grep・コード読みで確認した。**[推測]** = 未確認。

## 進捗と、ユーザーの追加判断（2026-11-12）
- **L1 実施済み**（コミットは `git log` を参照。結果は `inventory.md` の「L1 実施」）。**L2 以降は未実施**。
- **判断**: `hardware` 引数の検査は `mock:=true` のときも行う（L1 で実装済み）。plugin xml の改名は **L3** で行う（L2 では `So101HardwareBridge` のブロックを消すだけで、ファイル名は変えない）。**候補 B（`camera.launch.py` とカメラ設定 yaml 3 つ）は L2 で `git rm` する**。`LICENSE.so101_ros2` は、description の urdf・meshes・controller の由来が確定するまで**残す**（ヘッダが無いことは、由来でない証拠にならないため）。package.xml の `maintainer` は変えてよいが、`license` 欄は変えない（L3 で）。CLAUDE.md の archive の規則は **L4 で**「削除は `git rm`。戻すためのタグかコミットを `inventory.md` に書く」に直す（文言の案は L4 の前に出す）。
- **LeRobot の削除（L1〜L4）の後に別のコミットで行う候補**: A（`apriltag_test`）・G（`tools/` の開発用スクリプト）・H（既存の `archive/`）と、C の `assets/step`・`assets/stl`。**保留**: C の `usd/`、D・E・F。
- 7 章の「推奨 (a)」「LICENSE の削除」は、上の判断で置き換わる（B は L2 で削除、LICENSE は由来が確定するまで残す）。

## 0. 前提

- **[事実]** 既定の経路は `hardware:=feetech`。`Sts3215System` は LeRobot に依存しない（import も実行時依存も無い）。
- **[事実]** LeRobot 経路は、`so_arm101_lerobot_bridge`（Python。`lerobot` を import してサーボを読み書きするノード）と、それとトピックで繋ぐ `So101HardwareBridge`（`so_arm101_hardware` 内の SystemInterface）の組。`hardware:=lerobot` のときだけ使われる。
- **[事実]** コード上の「LeRobot」の言及は 2 種類: (a) **経路そのもの**（bridge パッケージ・launch・xacro・プラグイン）→ **削除**。(b) **由来の記載・基準値**（変換式・レジスタ表・EEPROM の期待値の出典、黄金表）→ **残す／名前を変える**。(b) は依存ではなく、出典の記録と正しさの根拠（Apache-2.0 の帰属表示を含む）。

## 1. 洗い出し（ファイル・行・役割・分類）

行番号は 2026-11-12 時点（HEAD ef3cb03）。

### 1.1 `so_arm101_lerobot_bridge`（削除: `git rm -r`）
| 場所 | 役割 | 分類・理由 |
|---|---|---|
| `src/so_arm101_lerobot_bridge/` の全 34 ファイル（`bridge/*`、`follower_ros2_node.py`・`leader_ros2_node.py`・`policy_runner_ros2_node.py`、`policy/*`（SmolVLA）、`utils/*`、`launch/so101_ros2_bridge.launch.py`、`config/*.yaml`、`config/calibration/Gili.json`・`Tzili.json`、`config/policies/*`、`setup.py`・`setup.cfg`・`package.xml`・`resource/*`、`test/*`） | `lerobot` を import（`bridge.py:41-49`、`build.py:24-32`、`smolvla.py:25-36`、`utils/conversion.py:23-32`）。`ensure_conda_site_packages_from_env()`（`utils/core.py:26`）が環境変数 `LECONDA_SITE_PACKAGES` の conda の site-packages を足す。`calibration_dir` は `~/.cache/huggingface/lerobot/...`（リポジトリ外） | **削除**（ユーザー判断）。leader・policy_runner も消える。最終構成の leader arm（`so_arm101_hardware` を名前空間で複数起動）には使わない |

### 1.2 launch・xacro・URDF（**承認が要る変更**）
| 場所 | 役割 | 分類・理由 |
|---|---|---|
| `src/so_arm101_bringup/launch/bringup.launch.py:6-15`（docstring）、`:63-67`（`bridge_launch`）、`:79-83`（`hardware` の `choices=["lerobot","feetech"]` と説明）、`:162-173`（`hardware:=lerobot` のときだけ bridge を include） | 経路の選択 | **変更**。`hardware` 引数は残し、**`feetech` だけ受け付け、ほかの値は「`lerobot` は削除済み。戻すには `git checkout stage2c-done -- ...`」のようなはっきりしたエラーで止める**（ユーザー判断）。bridge の include と `bridge_launch` は削除 |
| `bringup.launch.py:119`（`apply_servo_config` の説明「LeRobot の値と違う EEPROM…」） | 説明文 | **文言だけ変更**（「基準値と違う」） |
| `src/so_arm101_bringup/launch/bringup_with_cameras.launch.py`（`:18` の既定 `hardware='lerobot'`） | カメラ付きの起動。**[事実]** 参照するファイルは docs 以外に無い。既定が `lerobot` のままで、`bringup.launch.py` の既定（feetech）と食い違っていた | **削除（`git rm`）**（ユーザー判断）。カメラと腕を一緒に起動する launch は、必要になったら新しく作る |
| `src/so_arm101_bringup/package.xml:16` | `<exec_depend>so_arm101_lerobot_bridge</exec_depend>` | **削除**（残すと bridge の削除後にビルドの依存が解決できない） |
| `src/so_arm101_moveit_config/config/so101_new_calib.urdf.xacro:7` | `<xacro:arg name="hardware" default="lerobot" />` | **変更**: 既定を `feetech` に |
| `src/so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro:3`（マクロ引数 `hardware:=lerobot`）、`:25-27`（`So101HardwareBridge` のプラグイン） | lerobot のときのプラグイン選択 | **変更**: 既定を `feetech`、`So101HardwareBridge` の分岐を削除。**`hardware` が `feetech` 以外のときの xacro の失敗のさせ方**は、L1 で xacro の標準の方法を確認して決める（**[推測]** `xacro` に明示的なエラーを出す機能があるが、この環境のバージョンでの書き方は未確認） |
| `src/so_arm101_description/urdf/so101_new_calib.ros2_control.xacro:23`（`So101HardwareBridge`）、`:13`（topic_based）、`:19`（Ignition） | **[事実]** description 側の別の ros2_control xacro。description 側の `so101_new_calib.urdf.xacro:16-17` が include している | **変更**: `So101HardwareBridge` の参照を外す（ユーザー判断）。topic_based・Ignition は LeRobot とは無関係なので、ここでは触らない（8 章の候補） |

### 1.3 `So101HardwareBridge`（ファイルごと。すべて削除の予定）
**コード読みの事実（通信の方法）**:
- **[事実]** `So101HardwareBridge`（`so101_hardware_interface::So101HardwareBridge`、`hardware_interface::SystemInterface`）は、**自分用の ROS 2 ノード `so101_hardware_bridge_node`** を `rclcpp::Node::make_shared` で作る（`so101_hardware_bridge.cpp:42`）。サーボには触れない。
- **[事実]** `on_activate`: パブリッシャ `joint_commands`（`std_msgs/Float64MultiArray`、キュー 10）と、サブスクライバ `joint_states_raw`（`sensor_msgs/JointState`、キュー 10）を作る（`:88-92`）。**トピック名は相対名で、ルート名前空間なら `/joint_commands`・`/joint_states_raw`**。
- **[事実]** `write()`: `hw_commands_`（関節ごとの位置指令。NaN のときは最後の位置）を `joint_commands` に**毎周期パブリッシュ**する。**確認応答・失敗の検出は無い**。
- **[事実]** `read()`: `rclcpp::spin_some(node_)` で `joint_states_raw` のコールバックを処理し、関節名で `hw_positions_`・`hw_velocities_` を更新する。**メッセージが来なくても `read()` は OK を返す**（古い値のまま。タイムアウトの検出が無い）。
- **[事実]** 相手は Python の `so_arm101_lerobot_bridge` の `bridge.py`（`joint_states_raw` を 30 Hz で出す。`:80-84`。`joint_commands` を購読して `lerobot` 経由でサーボへ書く）。**つまり、bridge が落ちると状態が止まる**（`docs/feetech_readonly_check.md` の「bridge が接続に失敗して落ちた」の記述と一致）。
- **[事実]** Sts3215System はこれらのトピックを使わない（直接シリアル）。

| ファイル | 内容 | 分類 |
|---|---|---|
| `src/so_arm101_hardware/src/so101_hardware_bridge.cpp`（169 行。ヘッダに `Copyright 2025 nimiCurtis` と MIT の許諾文） | 実装 | **削除（`git rm`）** |
| `src/so_arm101_hardware/include/so_arm101_hardware/so101_hardware_bridge.hpp`（同様に nimiCurtis の MIT のヘッダ） | 宣言 | **削除** |
| `src/so_arm101_hardware/so101_hardware_bridge_plugins.xml` | **[事実]** `So101HardwareBridge`（ライブラリ `so101_hardware_bridge`）と `Sts3215System`（ライブラリ `so_arm101_sts3215`）の**両方**を登録 | **ファイルとしては残す**。`So101HardwareBridge` の `<library>` ブロックだけを削除。名前を `so_arm101_hardware_plugins.xml` に直すなら、`package.xml:28` と `CMakeLists.txt:49` も直す |
| `src/so_arm101_hardware/CMakeLists.txt:19-22`（`add_library(so101_hardware_bridge ...)`・`ament_target_dependencies`）、`:38`（`install` の `TARGETS so101_hardware_bridge so_arm101_sts3215`）、`:48-49`（`pluginlib_export_plugin_description_file`）、`:86`（コメントの `so101_hardware_bridge.hpp`） | ビルド | **変更**: `so101_hardware_bridge` のライブラリと install の行を削除。`:48-49` は xml の名前変更をするときだけ変更 |
| `src/so_arm101_hardware/package.xml:28` | `<hardware_interface plugin_description_file="so101_hardware_bridge_plugins.xml"/>` | xml の名前を変えるときだけ変更 |
| `src/so_arm101_hardware/include/so_arm101_hardware/sts3215_system.hpp:8` | コメントで `So101HardwareBridge` に言及 | **文言だけ変更** |
| **試験** | **[事実]** `git grep` で `src/*/test` に `So101HardwareBridge`・`so101_hardware_bridge` を参照する試験は**無い**（偽サーボの試験は `Sts3215System` だけを対象にしている） | **削除する試験は無い** |

### 1.4 `[diag]`「servo settings vs LeRobot」と基準値（**残す。文言を reference に**）
| 場所 | 役割 | 分類・理由 |
|---|---|---|
| `servo_config.hpp:3-10`（コメント）、`:38-43`（`ExpectedServoConfig`: `operating_mode=0`、`p=16`、`i=0`、`d=32`、`return_delay=0`、gripper の `max_torque_limit=500`・`protection_current=250`・`overload_torque=25`） | 期待値。出所は LeRobot 0.4.2 の `SO101Follower.configure()`（149-171 行）と `FeetechMotorsBus.configure_motors()`（222-230 行） | **残す**（ユーザー判断 A）。**値は変えない**。文言は「reference（基準値）」にし、由来の LeRobot 0.4.2 は**コメントと `docs/third_party.md` に残す**（Apache-2.0 の帰属表示） |
| `servo_config.cpp` | `expected_servo_config`・`compare_servo_config` | 残す（変更なし） |
| `sts3215_system.cpp:273`（「the state LeRobot ran with」）、`:289`・`:294`・`:298`・`:305`（`DIFFERS from LeRobot values`・`MATCH (LeRobot)`）、`:311`（`[diag] servo settings vs LeRobot: %d/%zu joints match`）、`:724`、`:874`（コメント） | ログの文言 | **文言を変更**（L3）。新しい文言（案）: `[diag] servo settings vs reference: n/6 joints match`、`MATCH (reference)`、`DIFFERS from reference values`。**承認が要る**（ログの出力が変わる） |
| `test_servo_config.cpp`（13 箇所: `lerobot_arm()`・`lerobot_gripper()`・`ExpectedValuesAreLeRobotConfigure`） | 期待値の試験 | **名前だけ変更**（`reference_arm` 等）。**試験の件数は変わらない** |
| 手順書: `docs/feetech_servo_config_check.md:36,70`（`servo settings vs LeRobot: n/6 joints ma…`・`DIFFERS from LeRobot values`）、`inventory.md` の記録（過去の記録なので**書き換えない**） | grep の文言 | L3 で手順書の grep を新しい文言に直す。`inventory.md` の過去の記録は変えない（注記だけ） |
| **[事実]** 試験は `[diag]` のログ文言を検査していない | `grep` で見つからなかった（`test_conversion.cpp` の 1 件はコメント） | 試験への影響なし |

### 1.5 キャリブレーションの yaml・変換・試験データ
| 場所 | 役割 | 分類・理由 |
|---|---|---|
| `src/so_arm101_hardware/config/follower_calibration.yaml:12`（「LeRobot 等価（分母 4095）…」）、`:6-10`（`source: kind: ros_calibrated`） | 本番の yaml。`source.kind` は `ros_calibrated`（LeRobot の json からの変換ではない） | **残す**。**名前・書式（`schema_version: 1`）・値は変えない**。`:12` のコメントの文言だけ変更。`angle_ticks: 4095` は**値を変えない**（物理的には 4096 の可能性という既存の保留。LeRobot の削除とは別件） |
| `src/so_arm101_calibration/src/apply.cpp:71` | yaml を書き出すときのコメント行「LeRobot 等価（分母 4095）」 | **文言だけ変更**。**承認が要る**（生成される yaml の文言が変わる。`test_measure_apply.cpp`・`test_calibration.cpp` がこの行を検査しているかを L3 の前に確認する） |
| `measure.hpp:8`・`measure.cpp:401` | コメント・案内文で LeRobot の `set_half_turn_homings`・中央姿勢に言及 | 文言だけ変更（参照の記述は残す） |
| `calibration.cpp:56-65`（`source` の旧形式の読み込み） | **[事実]** 本番は新形式。旧形式は試験用の固定ファイルと `test_conversion.cpp:202-206` が使う | **残す**（ユーザー判断: yaml の互換は推奨どおり） |
| `calibration.hpp:6-7,21-22,61-73`、`sts3215_bus.hpp:10-12`、`servo_config.hpp:5-7`、`test_conversion.cpp:5` | LeRobot 0.4.2（Apache-2.0）の式・レジスタ表・期待値の出典と帰属表示 | **残す。帰属表示は消さない** |
| `src/so_arm101_hardware/test/data/follower_calibration_lerobot_json_20260907.yaml`（`kind: converted_from_lerobot_json`、`file: ~/.cache/huggingface/lerobot/...`）。参照: `so_arm101_hardware/CMakeLists.txt:58,80`、`so_arm101_calibration/CMakeLists.txt:42,49`、`test_conversion.cpp:104-107,202-206` | 試験専用の固定ファイル（旧校正）。LeRobot の json そのものは読まない | **名前だけ変更**（案: `follower_calibration_fixture_20260907.yaml`。`git mv`。**中身は変えない**。`source.kind` と `file:` は由来の記録として残す）。CMake の 4 箇所を同時に直す（承認） |
| `test/data/conversion_golden.csv`・`unnormalize_golden.csv`（ヘッダ「generated by tools/gen_conversion_golden.py from LeRobot 0.4.2 ...」、列名 `lerobot_value`・`lerobot_raw`）、`test_conversion.cpp`（25 箇所: `MatchesLeRobotGoldenTable`・`YamlMatchesLeRobotJsonSource`・`GripperIsClampedToRangeLikeLeRobot`・`AngleIsNotClampedLikeLeRobot`・`SignMagnitude.DecodesLikeLeRobot`・`InverseConversion.MatchesLeRobotUnnormalizeWithinOneTick`） | 変換が LeRobot 0.4.2 と一致することの正解表（凍結した数値） | **残す。名前を中立にする**（ユーザー判断 A）: 列名 `reference_value`・`reference_raw`、試験名 `MatchesReferenceGoldenTable` など。CSV のヘッダに「生成の記録: LeRobot 0.4.2 の `MotorsBus._normalize` / `_unnormalize`（Apache-2.0）。再生成スクリプトは削除済み（タグ `stage2c-done` の `tools/gen_conversion_golden.py`）」。**帰属表示（Apache-2.0）は残す**。**試験の件数は変わらない** |
| `tools/gen_conversion_golden.py`（19 箇所）、`tools/lerobot_readonly_state.py`（9 箇所）、`tools/convert_lerobot_calibration.py`（7 箇所） | `lerobot` を import する、または LeRobot の json を読む。**[事実]** ビルドにも実行時のドライバにも含まれない | **削除（`git rm`）**。再生成は、タグ `stage2c-done` から取り出して conda 環境で行う（`docs/third_party.md` に書く） |
| `tools/compare_joint_states.py`・`record_joint_states.py`（引数例に `lerobot_P1.csv`）、`tools/fake_sts3215.py:98`（コメント） | LeRobot の値との比較・記録 | **残す（文言だけ変更）**。2 つの CSV の比較として一般化。`record_joint_states.py` は `docs/feetech_readonly_check.md` が使う |

### 1.6 conda・環境変数・セットアップ
| 場所 | 役割 | 分類・理由 |
|---|---|---|
| `setup_so101.sh:8-9`（`# lerobot python path`、`export LECONDA_SITE_PACKAGES=...`） | bridge が conda の site-packages を見つけるための環境変数 | **その 2 行を削除**（ユーザー判断 8）。**[事実]** 残りは `source /opt/ros/humble/setup.bash`・`RMW_IMPLEMENTATION`・`RCUTILS_COLORIZED_OUTPUT`・workspace の `source`。**承認が要る**（セットアップスクリプト） |
| 手順書の `CONDA_PREFIX` の確認（`feetech_readonly_check.md:75,100`、`feetech_servo_config_check.md:21`、`feetech_write_check.md:18`、`so_arm101_calibration_check.md:15`） | conda を activate していない端末で実施する | **残す**（ユーザー判断 8: LeRobot と関係なく、conda の Python が混ざるのを防ぐため）。`LECONDA_SITE_PACKAGES` の確認の記述だけ、「LeRobot 経路は削除済みなので不要」と注記 |
| conda 環境 `~/miniforge3/envs/ros2_lerobot`、`~/.bashrc` の設定 | リポジトリの外 | **対象外**。ユーザーが判断・確認する |

### 1.7 docs
| 場所 | 分類・理由 |
|---|---|
| LeRobot を使う手順（`docs/feetech_readonly_check.md`（`lerobot_readonly_state.py`・`LECONDA_SITE_PACKAGES` を使う部分: `:49,65,96-122,178-180`）、`docs/feetech_servo_config_check.md:27`、`docs/stage2c_plan.md` ほか） | **削除せず、「歴史的（LeRobot 経路は削除済み。タグ `stage2c-done` で再現できる）」と注記**（ユーザー判断 7）。該当の節の先頭に 1 行 |
| `docs/third_party.md` | **残す・更新**: LeRobot 0.4.2（Apache-2.0）の帰属表示、削除した nimiCurtis 由来のコードの記録、削除したスクリプトをタグから取り出す方法 |
| `docs/inventory.md` | 過去の記録は書き換えない。削除の実施ごとに節を追記。**戻し方（タグ `stage2c-done` から取り出せる）の 1 行を書く** |
| `CLAUDE.md` | 変更なし（「LeRobotに依存しない」はそのまま）。archive の規則だけ、上の注意を参照 |
| `LICENSE.so101_ros2` | 7 章の条件で削除するか決める |
| `archive/stage2a/bringup/launch/include/follower.launch.py:33`（`so_arm101_lerobot_bridge` を参照） | 既存の archive（LeRobot の削除の対象ではない）。8 章の候補 H |

## 2. 事実（grep・コード読みで確認）と推測

**[事実]**
- `hardware:=lerobot` の経路は、`bringup.launch.py`、`so101_new_calib.urdf.xacro:7`、`so101_new_calib.ros2_control.xacro:3,25-27`（moveit_config 側）、`so_arm101_description/urdf/so101_new_calib.ros2_control.xacro:23` に閉じている。
- `lerobot` を import するのは `so_arm101_lerobot_bridge` と `tools/` の 2 本（`gen_conversion_golden.py`・`lerobot_readonly_state.py`）だけ。C++ のドライバ・キャリブレーションツールは依存していない。
- 試験は LeRobot を実行しない（黄金表は凍結した CSV）。`CALIBRATION_YAML` などはリポジトリ内のファイル。
- `So101HardwareBridge` を参照する試験は無い（1.3）。
- `so_arm101_bringup/package.xml` が `so_arm101_lerobot_bridge` に `exec_depend` している。
- `so101_hardware_bridge_plugins.xml` に `Sts3215System` も登録されている（1.3）。
- `bringup_with_cameras.launch.py` を参照するものは docs 以外に無い。
- `include/camera.launch.py` を参照するのは `bringup_with_cameras.launch.py` だけ（7 章）。

**[推測]（実施前に確認する）**
- xacro で `hardware` が `feetech` 以外のときに明示的なエラーを出す書き方（L1 で確認）。
- `apply.cpp:71` の yaml コメント行を、試験が検査しているか（L3 の前に確認）。
- `so_arm101_description` の urdf を、実行時にどこが読むか（`bringup.launch.py` は moveit_config 側を使っていると見られる）。
- 手順書・過去の記録の中で、`MATCH (LeRobot)` 等を合格基準として grep している箇所の全体（L3 の前に一覧化）。

## 3. ユーザーが決めた事項（2026-11-12）と、残る小さな判断

| # | 項目 | 決定 |
|---|---|---|
| 1 | `[diag]` の基準値 | **A**: コードに残し、文言は reference。由来の LeRobot 0.4.2 はコメントと `docs/third_party.md` に残す |
| 2 | yaml の互換 | 推奨どおり（旧形式の読み込みを残す。固定ファイルは名前だけ変える。`angle_ticks: 4095` は値を変えずコメントだけ変える。本番 yaml の名前・書式は変えない） |
| 3 | 黄金表 | **A**: 凍結データとして名前を中立にして残す。再生成スクリプトは削除。帰属表示は残す |
| 4 | `hardware` 引数 | 残して `feetech` だけ受け付ける。ほかの値ははっきりしたエラーで止める |
| 5 | `so_arm101_description` の xacro | `So101HardwareBridge` の参照を外す。`bringup_with_cameras.launch.py` は削除 |
| 6 | nimiCurtis 由来 | 削除。削除後に MIT の帰属が必要なコードが残らないか grep で確認し、残らなければ `LICENSE.so101_ros2` も削除。残るなら報告（7 章） |
| 7 | docs の LeRobot の手順 | 削除せず「歴史的」と注記 |
| 8 | `setup_so101.sh` | `LECONDA_SITE_PACKAGES` の行を削除。`CONDA_PREFIX` の確認は手順書に残す |
| 他 | archive | 作らない。`git rm` で削除し、戻すときはタグ `stage2c-done` から |

**残る小さな判断（実施の承認のときに決める）**:
- `hardware` 引数の検査を、`mock:=true` のときも行うか（現状 `mock:=true` では `hardware` は無視される）。**案（推奨）**: `mock` に関わらず検査する（`hardware:=lerobot` を黙って受け付けない）。
- `so101_hardware_bridge_plugins.xml` の名前を `so_arm101_hardware_plugins.xml` に直すか（直すと `package.xml:28`・`CMakeLists.txt:49` も変わる）。**案（推奨）**: 直す。
- CLAUDE.md の archive の規則（上の注意）。

## 4. 実施の段階（各段階: 承認 → `colcon build` → 関係パッケージの `colcon test` → 試験の件数の前後比較 → コミット）

**試験の件数の基準（実施前、HEAD ef3cb03）**: gtest の試験数 so_arm101_hardware **99**、so_arm101_calibration **71**。`colcon test-result` の集計は hardware **105**、calibration **73**（gtest に CTest の項目 6 件・2 件が加わる）。**各段階で、前後の件数を報告する**。

**件数の見通し（[事実] に基づく計画）**: 本計画のどの変更も、**hardware・calibration の gtest の試験を削除しない**（`So101HardwareBridge` を参照する試験は無い。名前の変更だけ）。したがって、**全段階で hardware 99（105）・calibration 71（73）のまま変わらない**見込み。**件数が減ったら、その段階で止めて、どの試験がなぜ減ったかを報告する**（想定外として扱う）。`so_arm101_lerobot_bridge` の試験（lint 3 件・pytest。`colcon test-result` では「結果ファイルなし」の失敗 1 件）は、パッケージごと消える（**この失敗が消えるのは削除による。解消ではない**。8 章）。

### L0（調査のみ・変更なし）
2 章の [推測] を grep・コード読みで確定し、`inventory.md` に記録する（xacro のエラーの書き方、`apply.cpp:71` を検査する試験の有無、合格基準の grep の一覧）。試験の件数: 変更なし（99/71）。

### L1（LeRobot の起動経路を外す。承認が要る: launch・xacro・package.xml）
**ビルドが通ったままにするため、bridge パッケージ自体はこの段階では消さない**（L2 で消す）。具体的な変更（ファイルと要点）:
1. `src/so_arm101_bringup/launch/bringup.launch.py`:
   - docstring（`:6-15`）から `hardware:=lerobot` の説明を削除。
   - `bridge_launch`（`:63-67`）と、bridge の `IncludeLaunchDescription`（`:162-173`）を削除。
   - `hardware` 引数（`:79-83`）の `choices=["lerobot", "feetech"]` を `feetech` だけにし、**`feetech` 以外が渡されたときは明示的なエラーで停止する**（メッセージ案: `hardware:=<値> is not supported: the LeRobot path was removed (tag stage2c-done has it). Use hardware:=feetech`。実装は launch 引数の検査（`OpaqueFunction` で `LaunchConfiguration` を評価）を第一とする）。
   - bridge の include だけが使っていた import があれば削除。
2. `src/so_arm101_bringup/launch/bringup_with_cameras.launch.py`: **`git rm`**。
3. `src/so_arm101_bringup/package.xml:16`: `<exec_depend>so_arm101_lerobot_bridge</exec_depend>` を削除。
4. `src/so_arm101_moveit_config/config/so101_new_calib.urdf.xacro:7`: `hardware` の既定を `feetech` に。
5. `src/so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro`: マクロ引数 `hardware:=lerobot` → `feetech`。`So101HardwareBridge` の分岐（`:25-27`）を削除。`feetech` 以外のときは、xacro 側でも明示的に失敗させる（書き方は L0 で確認）。
6. `src/so_arm101_description/urdf/so101_new_calib.ros2_control.xacro:23`: `So101HardwareBridge` の参照を外す（`:13`・`:19` は触らない）。
7. **変更しない**: C++・CMake・URDF 本体・関節リミット・`planning_frame`・IK solver・yaml・bridge パッケージ（まだ存在する）。
- **確認**: `colcon build`、`colcon test`（hardware・calibration・bringup・moveit_config）。件数: 99/71 のまま。**モックでの起動確認**（ユーザーが許可したとき）: `mock:=true` と `hardware:=lerobot` を渡したときのエラー表示。起動したものを止めた後は `ps` と `ros2 node list` の両方で残りが無いことを確認（CLAUDE.md）。
- **実機確認 1（ユーザーが行う。L1 の後）**: 読み取り専用の起動（`hardware:=feetech read_only:=true`）と、**W0**（書き込みモードの起動・停止）。**この時点では `[diag]` の文言は旧い**（`servo settings vs LeRobot: 6/6 joints match`）。見るもの: `startup_read_retries=0`・全カウンタ 0、停止後 `torque_enable=0 lock=1 acceleration=50`。

### L2（削除。承認が要る: CMake・package.xml・`setup_so101.sh`）
**名前を「archive へ移動」から「削除」に変更**（ユーザー判断）。すべて `git rm`:
1. `src/so_arm101_lerobot_bridge/`（34 ファイル）。
2. `src/so_arm101_hardware/src/so101_hardware_bridge.cpp`、`include/so_arm101_hardware/so101_hardware_bridge.hpp`。
3. `src/so_arm101_hardware/CMakeLists.txt`: `so101_hardware_bridge` のライブラリ（`:19-22`）と install の対象（`:38`）を削除。`so101_hardware_bridge_plugins.xml` から `So101HardwareBridge` のブロックを削除（名前を変えるなら `git mv` と、`package.xml:28`・`CMakeLists.txt:49` の更新）。`sts3215_system.hpp:8` のコメント。
4. `tools/gen_conversion_golden.py`、`tools/lerobot_readonly_state.py`、`tools/convert_lerobot_calibration.py`。
5. `setup_so101.sh` の `LECONDA_SITE_PACKAGES` の 2 行（`# lerobot python path` と `export ...`）。
6. （`LICENSE.so101_ros2` は、7 章の条件を満たしてから別の 1 手で削除。ここには含めない。）
7. `docs/third_party.md` と `inventory.md` に、削除の記録と**「戻すときはタグ `stage2c-done` から取り出せる」の 1 行**。
- **確認**: **クリーンビルド**（`build/`・`install/` に古い bridge の成果が残るため、別のディレクトリでの全体ビルド、または `install/so_arm101_lerobot_bridge` の確認）。`ros2 pkg list` に `so_arm101_lerobot_bridge` が無いこと。`colcon test`（hardware・calibration・bringup・moveit_config）。件数: 99/71（105/73）のまま。
- **実機確認はこの段階では行わない**（L3 の後にまとめて行う。ドライバ本体の挙動は変わらず、`So101HardwareBridge` は feetech の経路で使われていない）。hardware のライブラリは再ビルドされるので、L3 の実機確認で全体を見る。

### L3（名前・文言・試験の整理。承認が要る: ログの文言・yaml のコメント・試験データの名前）
1. `sts3215_system.cpp` のログの文言を reference に（1.4）。`servo_config.hpp` のコメント、`test_servo_config.cpp` の名前。
2. 試験データ・試験名の中立化（1.5）: `follower_calibration_lerobot_json_20260907.yaml` → `follower_calibration_fixture_20260907.yaml`（中身は変えない。CMake の 4 箇所、`test_conversion.cpp` のコメント）。黄金表の列名・ヘッダ・試験名。
3. `apply.cpp:71`・`follower_calibration.yaml:12`・`measure.*` の文言（`apply.cpp` の文言は、生成される yaml のコメントが変わるので、試験の有無を L0 で確認済みであること）。
4. **手順書の grep の文言を、新しい文言に直す**（`docs/feetech_servo_config_check.md:36,70` ほか。L0 で一覧化した箇所）。過去の記録（`inventory.md`）は書き換えない。
5. docs の LeRobot の手順に「歴史的」の注記（1.7）。
- **確認**: `colcon build`、`colcon test`。**件数: 99/71（105/73）のまま**（名前の変更のみ。減ったら止めて報告）。試験名の変更の前後で、`--gtest_list_tests` の**試験の一覧を比較**し、**名前の対応表を `inventory.md` に記録**する（中立化で消えた試験がないこと）。
- **実機確認 2（ユーザーが行う。L3 の後）**: 読み取り専用の起動と **W0**。見るもの: **`[diag] servo settings vs reference: 6/6 joints match`（新しい文言）と `MATCH (reference)` が 6 行**、`startup_read_retries=0`、全カウンタ 0、停止後 `torque_enable=0 lock=1 acceleration=50`、`CONDA_PREFIX` が空であること。**手順書の grep（直した文言）で、期待する行が拾えること**も確認する。

### L4（記録）
`inventory.md` に各段階の結果（件数の前後を含む）、`docs/third_party.md` を更新（Apache-2.0 の帰属表示は残す）。タグ `stage2c-done` はそのまま（戻すための基準点）。

## 5. 削除後に必要な確認（まとめ）
1. `colcon build`（クリーン）、`ros2 pkg list` に bridge が無い。
2. `colcon test`（関係パッケージ）: hardware 99（105）・calibration 71（73）。名前の対応表。他パッケージの古い失敗（bringup の lint・apriltag_test）は別件。
3. モックの起動（ユーザーが許可したときだけ）。起動後に止めたら `ps` と `ros2 node list` の両方で残りが無いこと。
4. **実機（ユーザーが行う）**: L1 の後と L3 の後（4 章）。読み取り専用の起動と W0。
5. `grep -r -i lerobot` の残りが、**由来の記載・帰属表示・歴史的な docs だけ**であること（コードの依存・launch・xacro・CMake・package.xml に残っていない）。

## 6. 承認が要る変更（一覧）
- **launch の起動引数**: `bringup.launch.py` の `hardware`（`feetech` だけ、ほかはエラー）と bridge の include、`bringup_with_cameras.launch.py` の削除。
- **xacro**: `so101_new_calib.urdf.xacro:7`、moveit_config の `so101_new_calib.ros2_control.xacro`、description の `so101_new_calib.ros2_control.xacro:23`。
- **CMake・package.xml**: `so_arm101_hardware`（`so101_hardware_bridge`・plugin xml）、`so_arm101_bringup/package.xml`、固定ファイルの改名に伴う hardware・calibration の CMake。
- **ログの文言**（`[diag]` ほか）と、それを grep する手順書。
- **`apply.cpp` の yaml コメント文言**。
- **`setup_so101.sh`**。
- **ファイルの削除（`git rm`）**: CLAUDE.md の規則（archive へ移動）の例外。
- MoveIt 本体の設定（URDF・関節リミット・planning_frame・IK solver）は**変えない**。

## 7. nimiCurtis 由来（MIT）の帰属 — 削除後に何が残るか（grep の事実）
対象: `LICENSE.so101_ros2`（Copyright (c) 2025 nimiCurtis、MIT）。ユーザー判断: **削除後に MIT の帰属が必要なコードが残らないことを grep で確認し、残らなければ `LICENSE.so101_ros2` も削除。残る場合は、その箇所を報告する**。

**[事実] 2026-11-12 の grep（`nimiCurtis`・`Copyright`・`MIT License` を、docs・archive 以外で検索）**:
- 本計画で削除するもの: `so_arm101_lerobot_bridge` 全体、`so101_hardware_bridge.cpp`・`.hpp`（どちらも `Copyright 2025 nimiCurtis` と MIT の許諾文を持つ）。→ これらは消える。
- **残るもの（L1〜L3 では消えない）**:
  1. **`src/so_arm101_bringup/launch/include/camera.launch.py`**: ヘッダに `Copyright 2025 nimiCurtis` と MIT の許諾文。**[事実]** 参照するのは `bringup_with_cameras.launch.py`（削除する）だけなので、削除後は**どこからも使われない孤児**になる。**このファイルを消さない限り、MIT の帰属表示（`LICENSE.so101_ros2`）は必要**。
  2. **`package.xml` の `<maintainer email="nimicu21@gmail.com">Nimrod</maintainer>`**: `so_arm101_bringup`・`so_arm101_controller`・`so_arm101_description`・`so_arm101_hardware`（と、削除する bridge）。**[事実]** 取り込み元の記録。**[推測]** `so_arm101_controller`・`so_arm101_description`・`so_arm101_bringup` のファイルの一部（`urdf`・`meshes`・`bringup.launch.py` の構造）は、so101_ros2 由来の可能性がある（ヘッダは無く、由来は `docs/inventory.md` の段階 1 の記録に依る。**今回は由来の範囲を確定できていない**）。description の package.xml の license は BSD。
  3. `docs/third_party.md:17,46-48`: `bridge.py` の `math.radians`（度→rad）の記述。コードは C++ で再実装済みで、`bridge.py` のコピーではない。
  4. `src/so_arm101_bringup/config/so101_cameras.yaml`・`so101_realsense2.yaml`・`so101_usb_cam.yaml`: ヘッダなし。**[推測]** so101_ros2 由来の設定ファイル。
- **結論（grep の範囲）**: bridge 系を消しても、**ヘッダに MIT の帰属表示を持つコードが 1 つ残る（`camera.launch.py`）**。したがって**今のままでは `LICENSE.so101_ros2` は消せない**。**報告する箇所は上の 1（と、由来が未確定の 2・4）**。
- **ユーザーへの選択肢**:
  - (a) `camera.launch.py` と、孤児になるカメラの設定 yaml 3 つ（8 章の候補 B）も `git rm` する → その後の grep でヘッダの MIT 表示が残らなければ、`LICENSE.so101_ros2` も削除（ただし 2・4 の由来が確定できないので、**「ヘッダに表示があるものが無い」ことと「由来が未確定のものがある」ことを分けて報告する**）。
  - (b) `camera.launch.py` を残す → `LICENSE.so101_ros2` も残す。
  - **推奨**: (a)。`bringup_with_cameras.launch.py` の削除と一緒にカメラ関連の孤児を外す（カメラの launch は必要になったら新しく作る、というユーザーの判断と整合）。**`LICENSE.so101_ros2` の削除は、(a) の実施と、2・4 の確認の報告の後の別の 1 手**。

## 8. LeRobot 以外で、必要最小限にするために外せそうなパッケージ・ファイルの候補（**一覧のみ。削除するかはユーザーが別に判断する。今回は何も削除・変更しない**）

既存の `colcon test` の失敗（他パッケージの古い結果。ユーザー指示で今は調べない）との関係も書く。2026-11-12 の `colcon test-result` の失敗の発生源: `so_arm101_bringup` の lint（copyright・flake8・pep257 と CTest の 3 件）、`apriltag_test`（結果ファイルなし 1）、`so_arm101_lerobot_bridge`（結果ファイルなし 1）。**[事実]** `so_arm101_bringup` には `test/` が無く、`ament_lint_auto` が launch の `.py` を検査している（`CMakeLists.txt:20-31`）。

| 候補 | 参照しているもの（参照元） | 外したときの影響 | 既存の失敗との関係 |
|---|---|---|---|
| **A. `src/apriltag_test`**（10 ファイル。`launch/apriltag_d415.launch.py`（`apriltag_ros` の `apriltag_node`）、`config/tags_36h11.yaml`、lint の試験 3 本。`package.xml` の license は「TODO」） | **[事実]** コードからの参照は無い（`git grep` で docs 以外）。`docs/inventory.md` に 6 件、CLAUDE.md の最終構成の「so_arm101_calibration: …＋AprilTag の Eye-to-Hand」が将来の用途 | ビルド・実行への影響なし（依存するパッケージが無い）。**将来の AprilTag の Eye-to-Hand の試作を失う**（タグ stage2c-done から戻せる） | **[事実]** 「`apriltag_test/pytest.xml`: 1 error（結果ファイルなし）」はこのパッケージ。外せば消える（解消ではなく消滅） |
| **B. カメラ関連の孤児（bringup_with_cameras の削除後）**: `src/so_arm101_bringup/launch/include/camera.launch.py`、`config/so101_cameras.yaml`・`so101_realsense2.yaml`・`so101_usb_cam.yaml` | **[事実]** `camera.launch.py` を参照するのは `bringup_with_cameras.launch.py` だけ。yaml は `camera.launch.py` 経由と見られる（**[推測]**。`camera.launch.py` を読んで確認が要る） | 影響なし（削除後は誰も使わない）。**7 章のとおり、`LICENSE.so101_ros2` を消せる条件に関わる** | `so_arm101_bringup` の lint（flake8・pep257）が `camera.launch.py` も検査している可能性（**[推測]**）。外すと失敗の内訳が変わりうる（未確認） |
| **C. `src/so_arm101_description` の未使用の資産**: `assets/step/**`（16 ファイル）、`assets/stl/**`（20 ファイル）、`usd/**`（5 ファイル。Isaac 用）、`joints_properties.xml`、`urdf/so101_new_calib.urdf`（plain URDF）・`so101_new_calib_with_transmission.xacro`、`config/joint_names_so_arm_urdf.yaml`、`.flake8` | **[事実]** `meshes/*.stl`（13 個）は `urdf/so101_new_calib.urdf` が参照している（残す）。`assets/` と `usd/` の参照は、`git grep` で見つからない（`CMakeLists.txt:8` のコメントだけ）。**[推測]** description 側の `so101_new_calib.urdf.xacro` が実機の起動で使われているかは未確認（moveit_config 側を使っている見込み） | **URDF 本体・関節リミット・meshes は MoveIt 本体の設定に関わるので、触らない前提**。`assets/` `usd/` は参照が無いので影響なし（**[推測]**。外す前に `git grep` と `colcon build` で確認）。リポジトリの容量（STEP・STL・USD）が減る | 無関係 |
| **D. `src/so_arm101_description` と moveit_config の重複**: `urdf/so101_new_calib.ros2_control.xacro` と moveit_config 側の同名の複製、`config/initial_positions.yaml`・`joint_limits.yaml`（moveit_config 側にも同名） | **[事実]** description 側の `so101_new_calib.urdf.xacro:16-17` が description 側の ros2_control.xacro を include。moveit_config 側は自前のものを include（`so101_new_calib.urdf.xacro:21`） | 統合すると、`So101HardwareBridge` の参照のような二重の修正が要らなくなる。**ただし URDF・関節リミットは MoveIt 本体の設定。承認が要る** | 無関係 |
| **E. `src/so_arm101_controller`**（4 ファイル: `config/controllers.yaml`、`launch/`（空）、`CMakeLists.txt`、`package.xml`） | **[事実]** `bringup.launch.py:62-64` と `so_arm101_hardware/CMakeLists.txt:82`（試験の `CONTROLLERS_YAML`）が `controllers.yaml` を使う。`so_arm101_bringup/package.xml:14` が exec_depend | **外せない**（参照あり）。統合案: `controllers.yaml` を `so_arm101_bringup/config` へ移してパッケージを消す（launch・hardware の CMake・package.xml の変更が要る。承認制） | 無関係 |
| **F. `src/so_arm101_moveit_config` の補助ファイル**: `launch/setup_assistant.launch.py`・`.setup_assistant`（`CMakeLists.txt:12,16`）、`launch/servo.launch.py`・`config/servo.yaml`（MoveIt Servo。**[事実]** `bringup.launch.py` は include していない）、`config/pilz_cartesian_limits.yaml`、`launch/warehouse_db.launch.py`（`bringup.launch.py:179` が `db` 引数で include）、`static_virtual_joint_tfs.launch.py`（`:139` が include） | **MoveIt の設定の一部。CLAUDE.md により、MoveIt 本体の設定（URDF・関節リミット・planning_frame・IK solver）は勝手に変えない**。servo・setup_assistant は、使われていないことの確認（実機・モックでの運用）が要る | 影響は確認してから | 無関係 |
| **G. `tools/` の開発用スクリプト**: `pty_measure_check.py`・`pty_measure_notty_check.py`（**[事実]** 参照は `inventory.md` だけ）、`fake_sts3215.py`（`docs/inventory.md`・`stage2c_plan.md` が参照。**[事実]** `CMakeLists.txt`・`test/` からの参照は無い）、`compare_joint_states.py`・`record_joint_states.py`（`feetech_readonly_check.md` が参照。LeRobot との比較用）。**`pose_margin.py`（`feetech_write_check.md`・`so_arm101_calibration_check.md`）と `send_joint_delta.py`（`feetech_write_check.md`）は手順書が使うので残す** | `pty_measure_*`・`fake_sts3215.py`・`compare_joint_states.py` は、実機なしの検証・過去の比較の記録のためのもの。外すと過去の手順を再現できなくなる（タグから戻せる） | 無関係 |
| **H. 既存の `archive/`**（`archive/stage1_build`、`archive/stage2a/**`（旧パッケージの launch・README））。**[事実]** `archive/stage2a/bringup/launch/include/follower.launch.py:33` が `so_arm101_lerobot_bridge` を参照 | `colcon` は `archive/` を見ない（`src/` の外）。参照は記録のみ | 影響なし。今回の方針（archive を作らず削除）と揃えるなら `git rm`。タグから戻せる | 無関係 |
| **I. `docs/`** | `stage2c_plan.md`（完了した計画。`stage2c-done` に対応）、`feetech_readonly_check.md`・`feetech_servo_config_check.md`（歴史的な手順） | 削除せず「歴史的」の注記で残す方針（判断 7）。**外す候補ではなく、残す** | 無関係 |
| **J. `so_arm101_bringup` の lint の失敗** | `ament_lint_auto`（`CMakeLists.txt:20-31`）が `bringup.launch.py` などの `.py` を検査（copyright・flake8・pep257） | 外す対象ではなく、**直すか無効にするかを別に判断**（今回は調べない）。L1 で `bringup.launch.py` を変更するので、**変更前後の flake8・pep257 の件数を見る**のは L1 の確認に含められる | **[事実]** 現在の失敗の主な発生源。L1・`bringup_with_cameras` の削除で内訳が変わりうる |

**一覧の読み方**: 「LeRobot とは別に、最小限にするための候補」。**今回は何も削除・変更しない**。A・B・G・H は参照が少なく外しやすい。C・D・F は MoveIt 本体の設定・URDF に関わるので、承認と確認が要る。E は外せない（統合案のみ）。

## 9. 判断・確認が要る項目（実施の承認のときに）
1. `hardware` 引数の検査を `mock:=true` でも行うか（推奨: 行う）。
2. `so101_hardware_bridge_plugins.xml` の改名（推奨: する）。
3. **`camera.launch.py` とカメラ設定 yaml（候補 B）を、`bringup_with_cameras.launch.py` と一緒に削除するか**（推奨: する。`LICENSE.so101_ros2` を消す条件に関わる。7 章）。
4. **`LICENSE.so101_ros2` の削除は、3 の後の grep の結果と、7 章の 2・4（由来が未確定のもの）の報告を見てから**。
5. CLAUDE.md の「archive へ移動」の規則を、`git rm` とタグでの復元に直すか。
6. 8 章の候補（A〜J）のうち、どれをいつ外すか（別途）。
7. 実機確認（L1 の後と L3 の後）の日程（ユーザー）。

## 10. 後の段階: URDF まわりの作り直し（方針の記録。**今は実施しない**）
**時期**: LeRobot の削除（L1〜L4）と、候補の削除（A・G・H・C の step/stl）の**後**。別の段階とする。
**方針（ユーザー）**: URDF まわりは、将来、SO-ARM101 のオリジナルの URDF（**TheRobotStudio/SO-ARM100 の `Simulation/SO101` にある `so101_new_calib.urdf` と思われる**。未確認）から**正しく引用する形**に作り直す。
- **案**: オリジナルを**変更せずに**置き、**薄い xacro で include** して、ros2_control と追加のフレームだけを足す。
- **取得元の URL・コミットとライセンスを記録する**（`docs/third_party.md`）。
- **進め方**: ① **調査**（今の xacro とオリジナルの差の表: 0 の位置・軸の向き・関節リミット・リンク名・メッシュのパス。SRDF・`controllers.yaml`・`moveit_controllers.yaml`・`joint_limits.yaml` などへの影響）→ ② **計画** → ③ **承認** → ④ **実施** → ⑤ **実機での確認のやり直し**（読み取り専用の起動・W0〜W4・MoveIt の Plan & Execute）。
- **注意**: URDF・関節リミット・`planning_frame`・IK solver は MoveIt 本体の設定（CLAUDE.md により勝手に変えない）。差の表で関節リミット・0 の位置・軸の向きが変わるなら、校正（`follower_calibration.yaml` の `direction`・`center`）や既存の記録（出発点の値、elbow の URDF の上限との差）にも影響する。

