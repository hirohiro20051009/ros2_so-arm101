# LeRobot 経路の削除の計画（2026-11-12。計画のみ。実施は未。承認の後）

ユーザーの判断（`inventory.md`「ユーザーの判断の記録」）: **LeRobot 経路は、このプロジェクトの ROS パッケージから将来すべて削除する**。
この文書は、洗い出し・分類・順序・確認の計画。**コード・launch・xacro・URDF・yaml は変更していない。実機・シリアルポートにも触れていない。**
CLAUDE.md のとおり、**ファイルは削除せず `archive/` へ移動し、理由を記録する**（`git mv`。履歴は残る）。

凡例: **[事実]** = grep・コード読みで確認した。**[推測]** = 未確認。

## 0. 前提と範囲

- **[事実]** 既定の経路は `hardware:=feetech`（`bringup.launch.py` の既定）。`Sts3215System` は LeRobot に依存しない（import も実行時依存も無い。`docs/third_party.md`、`docs/feetech_readonly_check.md` の手順 6 で確認済み）。
- **[事実]** LeRobot 経路は、`so_arm101_lerobot_bridge`（Python。`lerobot` を import し、サーボの I/O をするノード）と、それとトピックで繋ぐ `So101HardwareBridge`（`so_arm101_hardware` 内の SystemInterface）の組。`hardware:=lerobot` のときだけ使われる。
- **[事実]** `so_arm101_lerobot_bridge` は git 管理下で 34 ファイル（leader / policy_runner / smolvla の推論ノード、`Gili.json`・`Tzili.json` の校正、`io.yaml`・`smolvla.yaml` を含む）。
- **[事実]** コード上の「LeRobot」への言及は 2 種類ある。(a) **LeRobot 経路そのもの**（bridge パッケージ・launch・xacro・プラグイン）。(b) **由来の記載・比較の基準値**（変換式・レジスタ表・EEPROM の期待値の出所、試験の正解表）。(b) はコードが LeRobot に依存しているのではなく、出典の記録と、ドライバの正しさの根拠（Apache-2.0 の帰属表示を含む）。**(a) は削除、(b) は「残す／名前を変える／判断が要る」に分ける。**
- **[事実]** 他の言及: `setup_so101.sh` の `LECONDA_SITE_PACKAGES`、docs の conda 確認手順、`archive/stage2a/` の旧 launch（すでに archive。変更しない）。
- **[推測]** 削除後に `so_arm101_bridge`（最終構成の rosbridge 用）や leader arm を作るときは、LeRobot の bridge の leader / policy_runner は参考にならない（LeRobot を前提にした実装）。作り直しになる。**ユーザーの判断で、policy_runner（SmolVLA の推論）も含めて消える。**

## 1. 洗い出し（ファイル・行・役割）

行番号は 2026-11-12 時点（HEAD 1a3628e）。

### 1.1 `so_arm101_lerobot_bridge` パッケージ（LeRobot 経路の本体）
| 場所 | 役割 | 分類 |
|---|---|---|
| `src/so_arm101_lerobot_bridge/` の全 34 ファイル（`bridge/bridge.py`・`build.py`・`registry.py`、`follower_ros2_node.py`・`leader_ros2_node.py`・`policy_runner_ros2_node.py`、`policy/*`、`utils/*`、`launch/so101_ros2_bridge.launch.py`、`config/*.yaml`・`config/calibration/Gili.json`・`Tzili.json`・`config/policies/*`、`setup.py`・`setup.cfg`・`package.xml`・`resource/*`、`test/*`） | `lerobot` を import してサーボを読み書きし、`/joint_commands`・`/joint_states_raw` を出し入れする。leader・SmolVLA の推論も含む。`bridge.py:41-46`・`build.py:24-30`・`smolvla.py:25-36`・`utils/conversion.py:23-32` が `ensure_conda_site_packages_from_env()`（`utils/core.py:26`、環境変数 `LECONDA_SITE_PACKAGES`）で conda の site-packages を足して `import lerobot` する | **削除（`archive/` へ移動）**。理由: ユーザーの判断。ドライバ（`Sts3215System`）が直接サーボを制御するので不要 |
| `src/so_arm101_lerobot_bridge/config/so101_follower_params.yaml:6`・`so101_leader_params.yaml:6` | `calibration_dir: ~/.cache/huggingface/lerobot/calibration/...`（リポジトリ外の LeRobot のキャッシュを読む） | 上と同じ（パッケージごと移動） |
| `src/so_arm101_lerobot_bridge/config/calibration/Gili.json`・`Tzili.json` | nimiCurtis 由来の別個体の校正 json | 上と同じ。**判断が要る**: 出所（`LICENSE.so101_ros2`、MIT）の表示は `docs/third_party.md` に残す |

### 1.2 launch・xacro・URDF（起動引数と既定値。**承認が要る変更**）
| 場所 | 役割 | 分類 |
|---|---|---|
| `src/so_arm101_bringup/launch/bringup.launch.py:6-15`（docstring）、`:63-67`（`bridge_launch`）、`:79-83`（`hardware` の `choices=["lerobot","feetech"]` と説明）、`:162-173`（`hardware:=lerobot` のときだけ bridge の launch を include） | `hardware` 引数で経路を選ぶ。docstring に「段階2c の後に archive へ移す予定」とある | **削除（引数の整理）**。**承認が要る**: `hardware` 引数そのものを消すか、`feetech` だけの選択肢に残すかは **判断が要る**（案は 3 章） |
| `bringup.launch.py:119`（`apply_servo_config` の説明「LeRobot の値と違う EEPROM…」） | 引数の説明文 | **残す（文言だけ変える）**。基準値の扱い（3.1）に従う |
| `src/so_arm101_bringup/launch/bringup_with_cameras.launch.py:18` | `DeclareLaunchArgument('hardware', default_value='lerobot')`。**[事実]** このファイルを参照するものは、リポジトリ内（docs 以外）に無い。既定が lerobot のままで、既定の `bringup.launch.py`（feetech）と食い違っている | **削除または修正**。**承認が要る**。別ランチャーなので、`hardware` を渡さない・既定を `feetech` にする、のどちらか |
| `src/so_arm101_bringup/package.xml:16` | `<exec_depend>so_arm101_lerobot_bridge</exec_depend>` | **削除** |
| `src/so_arm101_moveit_config/config/so101_new_calib.urdf.xacro:7` | `<xacro:arg name="hardware" default="lerobot" />`（**[事実]** xacro の既定が lerobot。launch は常に値を渡すので、launch 経由では効かない） | **修正**。**承認が要る（xacro）**。MoveIt 本体の設定（URDF・関節リミット・planning_frame・IK solver）ではなく、ros2_control のプラグインの選択だが、承認制とする |
| `src/so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro:3`（マクロの `hardware:=lerobot`）、`:25-27`（`So101HardwareBridge` のプラグイン、「so_arm101_lerobot_bridge と接続する」のコメント） | lerobot のときのプラグイン選択 | **削除**。**承認が要る（xacro）** |
| `src/so_arm101_description/urdf/so101_new_calib.ros2_control.xacro:23`（`so_arm101_hardware/So101HardwareBridge`）、`:13`（topic_based）、`:19`（Ignition） | **[事実]** description パッケージ側の別の ros2_control xacro。`So101HardwareBridge` を参照している。**[事実]** この xacro は `so_arm101_description/urdf/so101_new_calib.urdf.xacro:16-17` が include している（`so_arm101_moveit_config` 側の xacro とは別のマクロの複製）。**[推測]** 実機の起動（`bringup.launch.py`）は `so_arm101_moveit_config` 側を使っており、description 側の urdf を実行時に読むのは `so_arm101_controller` などの別経路だけ（どこから読まれるかは未確認） | **判断が要る**。使われていなければ `So101HardwareBridge` の行を消す／ファイルごと archive。使われているなら移行（確認は 5 章の最初で grep） |

### 1.3 `so_arm101_hardware`（`So101HardwareBridge` と、由来・基準値の記載）
| 場所 | 役割 | 分類 |
|---|---|---|
| `src/so_arm101_hardware/src/so101_hardware_bridge.cpp`（169 行）、`include/so_arm101_hardware/so101_hardware_bridge.hpp`、`so101_hardware_bridge_plugins.xml`（`:8`「LeRobot bridge 経由。比較用に残す」）、`CMakeLists.txt:19-22`（ライブラリ `so101_hardware_bridge`）・`:38`（install）・`:48-49`（`pluginlib_export_plugin_description_file`）、`package.xml:28`（`<hardware_interface plugin_description_file="so101_hardware_bridge_plugins.xml"/>`） | LeRobot bridge とトピックで繋ぐ SystemInterface。**[事実]** `Sts3215System` のプラグイン登録は `sts3215_system.cpp` 末尾の `PLUGINLIB_EXPORT_CLASS`。**[事実]** 同じ plugin xml（`so101_hardware_bridge_plugins.xml`）に、`So101HardwareBridge`（ライブラリ `so101_hardware_bridge`）と `Sts3215System`（ライブラリ `so_arm101_sts3215`、`so_arm101_hardware/Sts3215System`）の**両方**が登録されている。`package.xml:28` と `CMakeLists.txt:48-49` はこの 1 つの xml を指す。 | **削除（archive）**。**ただし xml は削除せず、`So101HardwareBridge` の `<library>` ブロックだけを外し、`Sts3215System` のブロックを残す**（xml の名前を `so_arm101_hardware_plugins.xml` に直すなら、`package.xml:28` と `CMakeLists.txt:49` も直す）。**承認が要る**（CMake・package.xml の変更） |
| `include/so_arm101_hardware/sts3215_system.hpp:8` | コメントで `So101HardwareBridge` に言及 | **残す（文言だけ変える）** |
| `CMakeLists.txt:25`（コメント「LeRobot には依存しない」）・`:54`（コメント「LeRobot 0.4.2 の _normalize 由来」）・`:86` | 説明 | 文言だけ。3.3 の方針に従う |

### 1.4 `[diag]`「servo settings vs LeRobot」の比較と、基準値の出所
| 場所 | 役割 | 分類 |
|---|---|---|
| `src/so_arm101_hardware/include/so_arm101_hardware/servo_config.hpp:3-10`（コメント）、`:38-43`（`ExpectedServoConfig`: `operating_mode=0`、`p=16`、`i=0`、`d=32`、`return_delay=0`、gripper の `max_torque_limit=500`・`protection_current=250`・`overload_torque=25`） | 期待値。出所は LeRobot 0.4.2 の `SO101Follower.configure()`（149-171 行）と `FeetechMotorsBus.configure_motors()`（222-230 行）。**[事実]** 値そのもの（P=16 等）は、サーボの安全で滑らかな動作のためにユーザーの実機で既に使われている値で、書き込みモードの起動の条件（不一致なら起動拒否）になっている | **残す（名前と文言を変える）**。**判断が要る**: 3.1 |
| `src/so_arm101_hardware/src/servo_config.cpp` | `expected_servo_config`・`compare_servo_config` | **残す** |
| `src/so_arm101_hardware/src/sts3215_system.cpp:273`（「the state LeRobot ran with」）、`:289`・`:294`・`:298`・`:305`（`DIFFERS from LeRobot values`・`MATCH (LeRobot)`）、`:311`（`[diag] servo settings vs LeRobot: %d/%zu joints match`）、`:724`（`servo settings differ from LeRobot values`）、`:874`（Lock=0 の LeRobot との違いのコメント） | ログの文言（実機の手順書・合格基準が grep するかもしれない） | **文言を変える**。**承認が要る**（ログの出力が変わる）。**[事実]** 手順書（`docs/feetech_*`、`so_arm101_calibration_check.md`）が `MATCH (LeRobot)`・`servo settings vs LeRobot` を grep・合格基準にしている可能性がある。変更時は手順書の grep を同時に直す（5 章で grep して一覧にする） |
| `src/so_arm101_hardware/test/test_servo_config.cpp`（13 箇所。`lerobot_arm()`・`lerobot_gripper()`・`ExpectedValuesAreLeRobotConfigure`） | 期待値の試験 | **残す（名前を変える）**（`lerobot_arm` → `reference_arm` など。試験の数は変えない） |

### 1.5 キャリブレーションの yaml・変換・試験データ
| 場所 | 役割 | 分類 |
|---|---|---|
| `src/so_arm101_hardware/config/follower_calibration.yaml:12`（「LeRobot 等価（分母 4095）…」）、`:6-10` の `source: kind: ros_calibrated` | **[事実]** 本番の yaml。`source.kind` は `ros_calibrated`（C3 で `so_arm101_calib apply` が書いた。LeRobot の json からの変換ではない）。`conversion: {angle_ticks: 4095}` の根拠が「LeRobot 等価」 | **残す**。コメントだけ書き換える。**判断が要る**: `angle_ticks` の扱い（3.2） |
| `src/so_arm101_calibration/src/apply.cpp:71` | yaml を書き出すときのコメント行「LeRobot 等価（分母 4095）」を出力する | **残す（文言だけ変える）**。**承認が要る**（生成される yaml の文言が変わる。試験が見ているか要確認） |
| `src/so_arm101_calibration/include/so_arm101_calibration/measure.hpp:8`・`src/measure.cpp:401` | コメント・案内文で LeRobot の `set_half_turn_homings`・中央姿勢に言及 | **残す（文言だけ）** |
| `src/so_arm101_hardware/src/calibration.cpp:56-65`（`source` の旧形式: `kind, file, file_date`。LeRobot の json からの変換）、`:124-146`（`raw_to_rad` 等のコメントの出典） | `source` の旧形式の読み込み。**[事実]** 本番の yaml は新形式（`measure_file` 等）。旧形式は、試験用の固定ファイル（下）と `test_conversion.cpp:202-206` が使う | **残す（コード）**。旧形式の読み込みを消すかは **判断が要る**（3.2） |
| `src/so_arm101_hardware/include/so_arm101_hardware/calibration.hpp:6-7,21-22,61-73`（LeRobot の `_normalize`・`_unnormalize` との対応のコメント、Apache-2.0 の帰属表示） | 変換式の出典。**Apache-2.0 の帰属は残す必要がある**（式・数値を再実装したため。`docs/third_party.md`） | **残す**（帰属表示は消さない） |
| `src/so_arm101_hardware/include/so_arm101_hardware/sts3215_bus.hpp:10-12`・`servo_config.hpp:5-7` | レジスタ表・EEPROM 期待値の出典の帰属 | **残す** |
| `src/so_arm101_hardware/test/data/follower_calibration_lerobot_json_20260907.yaml`（`kind: converted_from_lerobot_json`、`file: ~/.cache/huggingface/lerobot/...`）、使うもの: `so_arm101_hardware/CMakeLists.txt:58,80`、`so_arm101_calibration/CMakeLists.txt:42,49`、`test_conversion.cpp:104-107,202-206` | 試験専用の固定ファイル（旧校正）。**[事実]** 偽サーボの EEPROM の初期値・書き込みモードの試験の前提・黄金表との一致に使う。実機とは無関係で、LeRobot の json そのものは読まない | **残す（名前を変える）**。**判断が要る**: ファイル名（例: `follower_calibration_fixture_20260907.yaml`）と `source.kind`（`converted_from_lerobot_json`）の扱い。3.2 |
| `src/so_arm101_hardware/test/data/conversion_golden.csv`・`unnormalize_golden.csv`（ヘッダの「generated by tools/gen_conversion_golden.py from LeRobot 0.4.2」、列名 `lerobot_value`・`lerobot_raw`）、`test_conversion.cpp`（25 箇所。`MatchesLeRobotGoldenTable`・`YamlMatchesLeRobotJsonSource`・`GripperIsClampedToRangeLikeLeRobot` ほか） | 変換が LeRobot 0.4.2 と一致することの正解表。ドライバの変換の正しさの根拠 | **残す（凍結データとして）**。**判断が要る**: 3.3 |
| `tools/gen_conversion_golden.py`（19 箇所）、`tools/lerobot_readonly_state.py`（9 箇所）、`tools/convert_lerobot_calibration.py`（7 箇所） | `lerobot` を import する（またはその json を読む）開発・比較用スクリプト。**[事実]** ビルドにも実行時のドライバにも含まれない | **削除（archive）**。ただし `gen_conversion_golden.py` は黄金表の生成の記録なので、archive に置き、`third_party.md` から参照を残す |
| `tools/compare_joint_states.py`・`tools/record_joint_states.py`・`tools/fake_sts3215.py:98` | 引数例・コメントに `lerobot_P1.csv` などの名前。LeRobot の値との比較用（`compare_joint_states.py`） | **残す（文言だけ）**。**判断が要る**: LeRobot 側の記録が無くなるので、`compare_joint_states.py` の用途（2 つの CSV の比較）は一般化して残す |

### 1.6 conda・環境変数・セットアップ
| 場所 | 役割 | 分類 |
|---|---|---|
| `setup_so101.sh:8-9`（`# lerobot python path`、`export LECONDA_SITE_PACKAGES=/home/hirok/miniforge3/envs/ros2_lerobot/...`） | LeRobot bridge が conda の site-packages を見つけるための環境変数 | **削除（その行だけ）**。**承認が要る**（ユーザーの環境のセットアップ）。**[推測]** ユーザーの `~/.bashrc` にも同じ設定があるかもしれない（リポジトリの外。ユーザーが確認する） |
| conda 環境 `~/miniforge3/envs/ros2_lerobot` | リポジトリの外 | **対象外**（削除しない。ユーザーが判断） |
| `docs/feetech_readonly_check.md:49,65,75,96-122,178-180`、`docs/feetech_servo_config_check.md:21-27`、`docs/feetech_write_check.md:18`、`docs/so_arm101_calibration_check.md:15` | 「conda を activate していない端末で」「`CONDA_PREFIX`・`LECONDA_SITE_PACKAGES` が空」「`lerobot`・`miniforge3` のプロセスなし」「`/proc/$P/maps` に `lerobot|miniforge3` が無い」「`ldd` に python・lerobot・conda が無い」の確認手順 | **残す（手順は有効）**。削除後は「LeRobot 経路が存在しない」ので、一部（LeRobot を比較用に起動する手順 `lerobot_readonly_state.py`）は**歴史的な手順**として「実施済み・再現不可」と注記する。**判断が要る**（5.4） |

### 1.7 docs
| 場所 | 役割 | 分類 |
|---|---|---|
| `docs/inventory.md`（LeRobot 123 行）、`docs/third_party.md`（24）、`docs/feetech_readonly_check.md`（48）、`docs/feetech_servo_config_check.md`（15）、`docs/stage2c_plan.md`（6）、`docs/feetech_write_check.md`（3）、`docs/so_arm101_calibration_check.md`（5）、`CLAUDE.md`（1） | 経緯・手順・出典の記録 | **残す（記録は消さない）**。`third_party.md` は**帰属表示のため必須**。削除後に、各文書の冒頭へ「LeRobot 経路は削除済み（日付・コミット）」の 1 行を足す。`CLAUDE.md` の「LeRobotに依存しない」は変更不要 |
| `LICENSE.so101_ros2`（リポジトリのルート） | nimiCurtis の so101_ros2（MIT）のライセンス | **残す**。bridge パッケージ（その由来のコード）を archive へ移したあとも、archive に由来のコードが残るので、LICENSE は残す。**判断が要る**: archive 内のコードの扱い（`third_party.md` に明記） |

### 1.8 archive（すでに移動済み。変更しない）
- `archive/stage2a/bringup/launch/include/follower.launch.py:33`（`so_arm101_lerobot_bridge` を参照）。**残す**（archive は参照だけ）。

## 2. 事実（grep・コード読みで確認）と推測の分離

**[事実]**
- `hardware:=lerobot` を選ぶ経路は、`bringup.launch.py`（引数と include）、`so101_new_calib.urdf.xacro:7`・`so101_new_calib.ros2_control.xacro:3,25-27`（xacro の既定とプラグイン）の 3 か所に閉じている。
- `So101HardwareBridge` を参照するのは、`so_arm101_hardware`（実体・CMake・plugin xml・package.xml）、`so_arm101_moveit_config` の xacro、`so_arm101_description` の xacro（`:23`）。
- `lerobot` を import するのは `so_arm101_lerobot_bridge`（`bridge.py`・`build.py`・`smolvla.py`・`utils/conversion.py`）と、`tools/` の 2 本（`gen_conversion_golden.py`・`lerobot_readonly_state.py`）だけ。**C++ のドライバ・キャリブレーションツールは import も実行時依存もしていない**。
- 試験は LeRobot を実行しない（黄金表は凍結した CSV）。`CALIBRATION_YAML` などはリポジトリ内のファイル。
- `so_arm101_bringup/package.xml` が `so_arm101_lerobot_bridge` に `exec_depend` している（これを消さないと、bridge を archive へ移したあとの `colcon build` で依存が解決できない）。

**[推測]（実施前に確認が必要）**
- `so_arm101_description` の urdf（`so101_new_calib.urdf.xacro`）が、実行時にどこから読まれるか（1.2）。
- 手順書・合格基準が grep しているログの文言の全体（**[事実]** 確認できた範囲では、`docs/feetech_servo_config_check.md:36,70` が `servo settings vs LeRobot: n/6 joints ma…`・`DIFFERS from LeRobot values` に言及し、`inventory.md` の記録にも `MATCH (LeRobot)` がある。試験（`so_arm101_hardware/test`・`so_arm101_calibration/test`）がこのログ文言を検査している箇所は、grep では見つからなかった（`test_conversion.cpp` の 1 件はコメント）。）
- ユーザーの `~/.bashrc` 等の `LECONDA_SITE_PACKAGES`。
- `bringup_with_cameras.launch.py` が今も使われているか（**[事実]** 参照するファイルは docs 以外に無い。使用の有無はユーザーに確認）。

## 3. 判断が要る項目と案

### 3.1 `[diag]` の基準値（P=16 など）をどこに残すか
基準値は、**「安全に動くことを実機で確かめた設定」**であり、書き込みモードの起動条件になっている。出所が LeRobot であることは由来の記録でしかない。
- **案 A（推奨）**: コードの値と比較はそのまま。名前・文言だけ「LeRobot」から「reference（基準値）」に変え、出典は `docs/third_party.md` とヘッダのコメントに残す（Apache-2.0 の帰属表示）。ログは `[diag] servo settings vs reference: n/6 joints match`。**利点**: 挙動が変わらない。値の根拠が残る。**欠点**: ログの文言が変わる（手順書の grep の修正が要る）。
- **案 B**: 基準値を yaml（`follower_calibration.yaml` の `servo_defaults:`）に移す。**利点**: コードに数値が無く、サーボ個体・用途（リーダーアーム）ごとに変えられる。**欠点**: yaml の書式（`schema_version`）と検査・試験の変更が大きい。EEPROM と yaml の一致検査が増える。リーダーアーム（将来の拡張）には都合がよいが、今は過剰。
- **案 C**: 比較をやめ、起動時の EEPROM の値の表示だけにする。**欠点**: 安全の確認（P・I・D・Max_Torque_Limit 等が想定と違う状態でトルクを ON にしない）を失う。**推奨しない**。

### 3.2 キャリブの yaml の互換
- **`source` の旧形式**（`kind, file, file_date`。LeRobot の json からの変換）の読み込み（`calibration.cpp:56-65`）: **案 A（推奨）**: 残す（試験が使う。本番は新形式）。**案 B**: 旧形式を消し、試験用の固定ファイルを新形式に直す。**欠点**: 凍結のはずの固定ファイルの変更になる（「変更しない」と明記したファイル）。
- **試験用の固定ファイル名**（`follower_calibration_lerobot_json_20260907.yaml`）: 名前だけ `..._fixture_20260907.yaml`（仮）に変える（`git mv`。CMake の 4 箇所と `test_conversion.cpp` のコメントを直す）。中身は**変えない**（`source.kind: converted_from_lerobot_json` と `file: ~/.cache/huggingface/...` は、由来の記録として残す）。**案 A（推奨）**。
- **`angle_ticks: 4095`（「LeRobot 等価」）**: 値は変えない（変えると全関節の変換が変わる。「物理的には 4096 の可能性、変える場合は別途決める」の既存の保留）。コメントの文言だけ「LeRobot 互換（旧運用との一貫性。物理的には 4096 の可能性）」に変える。値の再検討は LeRobot の削除とは別件。
- **本番 yaml の `source: kind: ros_calibrated`**: そのまま（LeRobot 由来ではない）。
- **ファイル名・書式**: 本番 `follower_calibration.yaml` の名前・`schema_version: 1` は**変えない**（`calibration_records/`、手順書、試験が参照している）。

### 3.3 変換の黄金表・LeRobot との一致の試験
- **案 A（推奨）**: CSV と `MatchesLeRobotGoldenTable` 等の**試験は残し**、名前を中立にする（`ReferenceTable`、列名 `reference_value`）。**ただし `tools/gen_conversion_golden.py`（lerobot を import）は archive へ移すので、CSV は再生成できなくなる**。ヘッダに「生成の記録: `archive/tools/gen_conversion_golden.py`、LeRobot 0.4.2」と書く。**利点**: 変換の正しさの根拠（実測の LeRobot の出力との 1e-12 の一致）を失わない。
- **案 B**: 試験を、数式による期待値（式を試験側で再実装）に置き換える。**欠点**: 実測の正解表という独立した根拠を失う。作業が大きい。
- **案 C**: 黄金表の試験を削除。**推奨しない**（変換の回帰試験が減る）。

### 3.4 `So101HardwareBridge` と description の xacro
- `so_arm101_description` 側の xacro（1.2）が使われていなければ、案 A: `So101HardwareBridge` の行を削除。使われているなら案 B: 先に `Sts3215System` へ移す。

### 3.5 `hardware` 引数
- **案 A（推奨）**: `hardware` 引数を、**`feetech` だけを受け付ける形で一旦残す**（`choices=["feetech"]`）。**利点**: 既存の手順書のコマンド（`hardware:=feetech`）が壊れない。将来、別のドライバ（リーダーアームなど）を足すときに使える。
- **案 B**: `hardware` 引数を消す（`mock` と `read_only` だけにする）。**欠点**: 手順書・過去のログのコマンドが通らなくなり、すべて直す必要がある。
- いずれも**承認が要る**（launch の起動引数・xacro）。

### 3.6 `bringup_with_cameras.launch.py`
- 既定 `hardware='lerobot'` が既定の `bringup.launch.py` と食い違っている（**[事実]**）。使われているなら `feetech` に直す。使われていないなら archive。**ユーザーに確認**。

## 4. 実施の段階（案。段階ごとに承認 → `colcon build` → 関係パッケージの `colcon test` → コミット）

1. **段階 L0（調査のみ・変更なし）**: 2 章の [推測] を grep・コード読みで確定する（plugin xml、description の xacro の使われ方、手順書の grep 文言、`bringup_with_cameras`）。結果を `inventory.md` に記録。
2. **段階 L1（LeRobot の起動経路を外す。承認が要る: launch・xacro）**: `bringup.launch.py` の include と `hardware` の `lerobot` を外す。xacro の既定・`lerobot` 分岐を外す。`bringup/package.xml` の `exec_depend` を外す。**まだパッケージは動かさない**。`colcon build`。
3. **段階 L2（パッケージと実体を archive へ。承認が要る: CMake・package.xml）**: `src/so_arm101_lerobot_bridge` を `archive/lerobot_bridge/` へ `git mv`。`So101HardwareBridge`（cpp・hpp・plugin xml・CMake・package.xml）を archive へ（plugin xml に `Sts3215System` の登録が有れば残して名前を直す）。`tools/` の 3 本を archive へ。`setup_so101.sh` の 2 行を削除（承認）。`archive/` に移動理由の記録（`archive/README` または `inventory.md`）。**`colcon build`（`build/` `install/` に古い bridge の成果が残る。クリーンビルドまたは該当の `install` の確認）**。
4. **段階 L3（名前・文言・試験の整理。承認が要る: ログの文言）**: 3.1〜3.3 の方針で、`servo_config`・ログ・試験名・固定ファイル名を中立に。手順書の grep 文言を同時に直す。`docs` の冒頭に注記。
5. **段階 L4（記録）**: `inventory.md` と `third_party.md` を更新（Apache-2.0 の帰属表示は残す）。

各段階は、`colcon build` が通り、関係パッケージ（`so_arm101_hardware`・`so_arm101_calibration`・`so_arm101_bringup`・`so_arm101_moveit_config`）の `colcon test` が通ってからコミット（CLAUDE.md の規則どおり）。

## 5. 削除後に必要な確認

1. **`colcon build`**: クリーン（`build/`・`install/`・`log/` を残したままでは、古い bridge の成果が残って依存の欠落に気づけないため、**別のディレクトリでの全体ビルド**または対象パッケージの `install` の確認）。`ros2 pkg list` に `so_arm101_lerobot_bridge` が無いこと。
2. **`colcon test`（関係パッケージ）**: `so_arm101_hardware`（105 件、うち gtest 99 件）・`so_arm101_calibration`（73 件、うち gtest 71 件）が、**件数・結果とも変わらない**こと（名前の変更で試験が消えていないこと）。`so_arm101_bringup`・`so_arm101_moveit_config` の既存の試験。他パッケージの古い失敗（bringup の lint・apriltag_test）は、今回の対象外（記録済み）。
3. **モックの起動**（Claude が行ってよいのは、ユーザーが許可したときだけ。今回の依頼では行わない）: `mock:=true` で move_group・controller が立つこと。起動したものを止めた後は、`ps` と `ros2 node list` の両方で残りが無いことを確認（CLAUDE.md）。
4. **実機での確認（ユーザーが行う）**:
   - **読み取り専用の起動**（`hardware:=feetech read_only:=true`）: `[diag]` の行（文言を変えた場合は新しい文言）で `6/6 joints match`、`deactivated (no register was written)`、`bus:` の全項目 0。
   - **W0**（書き込みモードの起動と停止。腕を支える）: 5 回、`startup_read_retries=0`・全カウンタ 0・停止後 `torque_enable=0 lock=1 acceleration=50`。
   - `feetech_readonly_check.md` の手順 6（LeRobot 不使用の確認: プロセス・`maps`・`ldd`）は、LeRobot が無いので自明になるが、**環境変数が空であること**は引き続き見る。
5. **手順書の整合**: `docs/feetech_*` の grep・合格基準の文言（ログの文言を変えた場合）、LeRobot を起動する手順（`lerobot_readonly_state.py`）の注記。

## 6. 承認が要る変更（一覧）
- **launch の起動引数**: `bringup.launch.py` の `hardware` 引数（`lerobot` の削除、または引数ごとの削除）、bridge の include、`bringup_with_cameras.launch.py` の `hardware`。
- **xacro**: `so101_new_calib.urdf.xacro:7`、`so101_new_calib.ros2_control.xacro`（`hardware` の既定・`lerobot` 分岐）、`so_arm101_description` の `so101_new_calib.ros2_control.xacro:23`。
- **CMake・package.xml**: `so_arm101_hardware`（`so101_hardware_bridge` のライブラリ・plugin xml）、`so_arm101_bringup/package.xml`、試験用の固定ファイルの名前変更に伴う CMake（hardware・calibration）。
- **ログの文言**（`[diag] servo settings vs LeRobot` ほか）と、それを grep する手順書。
- **`setup_so101.sh`**（環境変数の行）。
- **`apply.cpp` が書き出す yaml のコメント文言**（試験の有無を確認）。
- **ファイルの移動**（`git mv` で `archive/` へ。CLAUDE.md の規則どおり、削除はしない）。
- MoveIt 本体の設定（URDF・関節リミット・planning_frame・IK solver）は**変えない**。

## 7. 判断が要る項目の一覧（要点）
1. `[diag]` の基準値の置き場所（3.1: A=コードに残して名前だけ変える（推奨）／B=yaml へ移す／C=比較をやめる）。
2. キャリブ yaml の互換（3.2: 旧形式 `source` の読み込みを残すか、固定ファイルの名前を変えるか、`angle_ticks: 4095` のコメント）。
3. 変換の黄金表と試験の扱い（3.3: A=残して中立の名前に（推奨）／B=式で置き換え／C=削除）。
4. `hardware` 引数（3.5: `feetech` だけで残す（推奨）／消す）。
5. `so_arm101_description` の ros2_control xacro と `So101HardwareBridge`（3.4）、`bringup_with_cameras.launch.py`（3.6）の扱い。
6. `Gili.json`・`Tzili.json`・nimiCurtis 由来のコード（MIT）を archive に残す扱いと、`LICENSE.so101_ros2` の置き場所。
7. `docs` の LeRobot を使う手順（`lerobot_readonly_state.py` の比較など）を「歴史的」と注記するか。
8. `setup_so101.sh` の削除と、`~/.bashrc`（リポジトリ外）の確認はユーザーが行う。
