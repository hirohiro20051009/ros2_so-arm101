# E: `controllers.yaml` を `so_arm101_bringup` へ移し、`so_arm101_controller` を無くす計画

作成: 2026-11-12 / **調査と計画のみ（実装しない。承認待ち）** / 基準: HEAD `a5c8cbd`

- 調査の方法: 読み取り、`xacro`、Python での launch の読み込み、`colcon test-result`（既存の結果の集計）、`find`・`sha256sum`。**実機・シリアルポートには触れていない。モックも起動していない**（`ps` で `ros2_control_node`・`move_group` が 0 件）。`build/`・`install/` は変更していない。
- 凡例: **[事実]** = 実際に確かめた、**[推測]** = 確かめていない（実装の段階で確かめる方法を併記）。

## 1. 要点

1. `so_arm101_controller` は**コードを持たない設定だけのパッケージ**。中身は `config/controllers.yaml` 1 つ。これを使うのは **launch（実行時）と hardware の試験（ソースのパス）の 2 か所だけ**。パッケージとして依存しているのは `so_arm101_bringup/package.xml:15` の exec_depend のみ。
2. 移動先は **`so_arm101_bringup/config/controllers.yaml`**（名前は変えない）を推奨。`git mv` で履歴を残す。
3. **一番の注意は、`install/`・`build/` に古い `so_arm101_controller` が残ること**（isolated + symlink-install）。src から消すと `install/so_arm101_controller/.../controllers.yaml` はぶら下がったリンクになり、`ros2 pkg prefix so_arm101_controller` は成功し続ける見込み。実装では「そのディレクトリだけを消す」（承認制）を推奨。消す前に控えるものは 6 章。
4. 変更は小さい: `git mv` 1、`git rm` 2（+ 空ディレクトリの rmdir）、CMake 2 か所、package.xml 1 行、launch 1 か所（3 行→3 行）、docs。
5. ライセンス: `controllers.yaml` は nimiCurtis 由来（MIT）。**`LICENSE.so101_ros2` は残す**。`THIRD_PARTY_NOTICES.md` の 2 か所を直す。

## 2. 調査の結果

### 2.1 `so_arm101_controller` のファイルと役割（調査項目 1）

`git ls-files src/so_arm101_controller` は **3 ファイル**（[事実]）。

| ファイル | 役割 | 備考 |
|---|---|---|
| `config/controllers.yaml`（52 行） | ros2_control の設定。`controller_manager`（update_rate 50）、`arm_controller`（JTC。`update_rate: 25`、`open_loop_control: true`（43 行）、gains 全 0、`time_scaling 1.1`）、`gripper_controller`（`joint: gripper`）。mock・実機共通、ルート名前空間 | sha256 `8515d643d6c60175b3216da52cc66ee1a52982fc3e4af6187ed892bbccf20308`。`git log --follow` で `3bc04f8`・`b58df44` までたどれる。冒頭コメントに出所（旧 `so101_controllers.yaml` の /follower ブロック） |
| `CMakeLists.txt`（25 行） | `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`（12 行）だけ。コードなし | nimiCurtis の骨組み（`29ff6bf`） |
| `package.xml`（32 行） | MIT、`<author>Nimrod</author>`。depend: `rclcpp`・`rclpy`・`trajectory_msgs`・`sensor_msgs`。exec_depend: `ros2launch`・`robot_state_publisher`・`xacro`・`controller_manager`・`so_arm101_description`・`joint_state_publisher_gui`。buildtool_depend: `ament_cmake_python` | **[事実]** コードが無いので、これらの依存は何にも使われていない。bringup は `controller_manager`・`so_arm101_description` を自分で exec_depend に持つ |
| `launch/`（空ディレクトリ） | 何もない | git は空ディレクトリを追跡しない。`git rm` では消えず、rmdir が要る |

`.flake8` は 8c5734b で `git rm` 済み。git 管理外のファイル（`.gitignore` 対象）は `src/so_arm101_controller/` に**無い**（`git status --ignored` で確認）。

`install/so_arm101_controller/`（[事実]。`find` で全部見た）: **実ファイル 9 個**（`colcon-core/packages/so_arm101_controller`、`hook/cmake_prefix_path.{dsv,ps1,sh}`、`package.{bash,dsv,ps1,sh,zsh}`）と **シンボリックリンク 15 個**。リンクのうち **src を指すのは 2 つだけ**: `share/so_arm101_controller/config/controllers.yaml`、`share/so_arm101_controller/package.xml`。残り 13 個は `build/so_arm101_controller/…`（ament_index 3、cmake 2、environment の dsv 2、local_setup 4 など）か `/opt/ros/humble/…`（environment の sh 2）を指す。`build/so_arm101_controller/` は CMake の生成物（`CMakeCache.txt`、`Makefile`、`ament_cmake_*`、`Testing`、colcon の `*.rc` など）。

### 2.2 参照の全部（調査項目 2）

`grep -rn so_arm101_controller` と `grep -rn controllers.yaml`（`build/`・`install/`・`log/`・`.git` を除く）の結果。

**変える必要があるもの**

| 場所 | 内容 | 役割 |
|---|---|---|
| `src/so_arm101_bringup/launch/bringup.launch.py:97-99` | `get_package_share_directory("so_arm101_controller"), "config", "controllers.yaml"` | **実行時の唯一の読み手**。184 行で `ros2_control_node` の `parameters=[moveit_config.robot_description, controllers_yaml]` に渡す |
| `src/so_arm101_bringup/package.xml:15` | `<exec_depend>so_arm101_controller</exec_depend>` | 依存 |
| `src/so_arm101_hardware/CMakeLists.txt:75` | `CONTROLLERS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_controller/config/controllers.yaml"`（`test_sts3215_system` の `target_compile_definitions`） | **試験が読む**。`test/test_sts3215_system.cpp:631` の `DesignLimits.StepLimitCoversTheWorstCaseTrajectoryStep` が `arm_controller.ros__parameters.update_rate`（25）を読み、`joint_limits.yaml` の `max_velocity` と合わせて `max_joint_speed × limit_window_s ≥ 1.2 × max_velocity × 2 / update_rate` を確かめる。**ソースのパスを読む**（install の share は読まない） |
| `THIRD_PARTY_NOTICES.md:21` | controller の `controllers.yaml`・`package.xml`・`CMakeLists.txt` | 帰属の表 |
| `src/so_arm101_controller/package.xml:4`、`CMakeLists.txt:2` | 自分自身の名前 | 削除するファイル |

**変更の要らないもの**（[事実] すべて grep で 0 件を確認）: `src/so_arm101_bringup/CMakeLists.txt`（`so_arm101_controller` を**参照していない**。ここで要るのは参照の変更ではなく、`config/` の **install の追加**。4 章 b1）、`tools/`、`CLAUDE.md`（最終構成の一覧に controller は無く、この変更で目標に近づく）、`src/so_arm101_calibration/`、`src/so_arm101_moveit_config/`、手順書（`docs/so_arm101_calibration_check.md`・`docs/feetech_write_check.md`・`docs/feetech_readonly_check.md`・`docs/feetech_servo_config_check.md`）、`docs/stage2c_plan.md`・`docs/bus_layer_fix_plan.md`・`docs/third_party.md`。`sts3215_system.hpp:32` は「`controllers.yaml`」と名前をコメントに書くだけ。ワークスペース外の `~/so_arm101_check/*.sh` と `setup_so101.sh`（git 管理外）にも参照なし。

**docs の歴史的な記録（書き換えない。当時の事実の記録なので）**: `docs/inventory.md`（26 か所。168・1083・1102・1669 行など）、`docs/lerobot_removal_plan.md:232,251,274`、`docs/original_urdf_plan.md`（125・212・214・250・319・466・470・590・610・616・648 行）、`docs/hardware_arg_removal_plan.md:129,165`、`docs/license_plan.md`（21・25・40・106・125・126・164 行）。旧パスを探す人が新パスにたどり着けるよう、`inventory.md` の末尾に「旧→新」の記録を足す（D6）。

**テスト**: `so_arm101_controller` の試験は 0 件。ほかの試験が `so_arm101_controller` という名前を使うことは無い（hardware の試験は上のソースのパスだけ）。bringup の `test_launch_arg_check.py`（34 件）は launch を読むが、古い install が残る間は `get_package_share_directory("so_arm101_controller")` が成功してしまうため、**参照の取りこぼしを見つけられない**（[推測]。D4）。

### 2.3 実行時に `controllers.yaml` が読まれる経路（調査項目 3）

`bringup.launch.py` を Python で読み込み、`generate_launch_description()` の戻りを調べた（**ノードは起動していない。ポートも開かない**）。**[事実]**:

- `ros2_control_node` の `parameters` は 2 つ: ① `moveit_config.robot_description`（dict）、② **`ParameterFile`**（パス `/home/hirok/ros2_so-arm101/install/so_arm101_controller/share/so_arm101_controller/config/controllers.yaml`）。
- このパスは**シンボリックリンク**で、`os.path.realpath` は `/home/hirok/ros2_so-arm101/src/so_arm101_controller/config/controllers.yaml`。sha256 は src 側・install 側で同一（`8515d643…`）。
- `robot_state_publisher` は `robot_description` と `publish_frequency` だけで、`controllers.yaml` を読まない。spawner（`spawn_controllers.launch.py`）は `moveit_controllers.yaml` からコントローラ名を作るだけ。
- **[推測]** 実行時にこのパスを読むのは `ros2_control_node` だけ。`strace` で `openat` を見る案もあるが、ノードを起動する必要があるので今回はしない。**読まれた値**は、実装後のモックの起動（8 章 2）で `ros2 param get /arm_controller open_loop_control`（True）・`update_rate`（25）で確かめる。

### 2.4 moveit_config の `ros2_controllers.yaml` との関係（調査項目 4）

**[事実]** `src/so_arm101_moveit_config/config/ros2_controllers.yaml`（sha256 `00ffd314…`）は Setup Assistant の生成物で、**実行時にも試験でも読まれない**（`src/` の `*.py`・`*.txt`・`*.xml`・`*.cpp`・`*.hpp`・`*.yaml` に参照なし。`moveit_config/CMakeLists.txt:15` の `install(DIRECTORY config …)` でインストールされるだけ）。`controllers.yaml` との差（`diff` で確認）: `controllers.yaml` だけが `arm_controller` の `update_rate: 25`・`type`・`gains`・`open_loop_control`・`state_publish_rate`・`action_monitor_rate`・`time_scaling` と、`gripper_controller` の `type` を持つ。

統合しない前提で、**混同のおそれ**だけ書く:
- 移動後は `so_arm101_bringup/config/controllers.yaml` と `so_arm101_moveit_config/config/ros2_controllers.yaml` という**名前の似た 2 つ**が別のパッケージにある。動くのは前者だけで、後者を編集しても何も変わらない（今も同じ）。
- 対策案（D5）: `controllers.yaml` の冒頭コメント（4〜5 行目の「統合元」）に「実行時に読まれるのはこのファイルだけ。`ros2_controllers.yaml` は Setup Assistant 用で読まれない」を足す。コメントだけの変更だが sha256 が変わるので、`git mv` のコミットとは**分ける**。

### 2.5 isolated 構成の install から消したときの影響（調査項目 5）

構成（[事実]）: `install/` は isolated（パッケージごとのディレクトリ）、symlink-install（`config/`・`package.xml` が src へのリンク）、`install/COLCON_IGNORE` あり。

| 状況 | 影響 | 確度 |
|---|---|---|
| src から消して `colcon build` | `colcon list` には出なくなるが、`install/so_arm101_controller/` と `build/so_arm101_controller/` は**残る**見込み。`config/controllers.yaml` と `package.xml` は、消えた src を指す**ぶら下がりリンク**になる | リンクの種類は [事実]。colcon が消さないことは [推測]（実装後に `ls` で確かめる） |
| `source install/setup.bash` の後 | `ros2 pkg prefix so_arm101_controller` が成功し続け、`ros2 pkg list` にも出る（`ament_index/resource_index/packages/so_arm101_controller` が `build/` へのリンクで残るため） | 今の動作は [事実]、消した後は [推測] |
| launch が古い名前を参照したまま（実装の不備） | `get_package_share_directory` は成功するが、ファイルが無く `ros2_control_node` が起動に失敗する。**今は symlink-install なので、古い内容が読まれることは無い**。ただし symlink でないビルドに変えた場合は install にコピーが残り、古い内容が読まれうる | リンクは [事実]、コピー版は [推測] |
| launch を新パスに直した後 | 古い install が残っても動く。ただし残骸が `ros2 pkg list`・ament index に残る | [事実] |
| bringup の exec_depend を消した後 | `install/so_arm101_bringup/share/colcon-core/packages/so_arm101_bringup`（コロン区切り）と `…/package_run_dependencies/so_arm101_bringup`（セミコロン区切り）から `so_arm101_controller` が消える | 今の内容に入っていることは [事実] |

**片付けの案**（D3）:

| 案 | 内容 | 長所 | 短所 |
|---|---|---|---|
| **B（推奨）** | `install/so_arm101_controller` と `build/so_arm101_controller` の **2 つのディレクトリだけ**を消す（承認を得て、直前に報告） | 速い。他のパッケージの `build/`・`install/` が変わらず、前後の install の diff が読みやすい | CLAUDE.md の「`build/` と `install/` を消すのはクリーンビルドのときだけ」の**例外**なので承認が要る |
| A | クリーンビルド（`build/`・`install/` を全部消して `colcon build`。`log/` は消さない） | 規則どおり。取りこぼしが無い | hardware・calibration の C++ を全部ビルドし直すので時間がかかる。生成物が変わり、diff が大きくなる |
| C | 消さない | 手間なし | 残骸が `ros2 pkg list` に残り、古い名前の参照の不備を**隠す** |

### 2.6 ライセンス・帰属（調査項目 6）

**[事実]** `controllers.yaml` の値は nimiCurtis/so101_ros2（MIT）の `so101_controllers.yaml`（/follower ブロック）由来（冒頭コメント、`docs/license_plan.md:21`）。`LICENSE.so101_ros2`（Copyright (c) 2025 nimiCurtis）が許諾文を持つ。controller の `package.xml`・`CMakeLists.txt` も nimiCurtis の骨組み（`29ff6bf`）。

- `controllers.yaml` は**移動するだけで nimiCurtis 由来のまま** → **`LICENSE.so101_ros2` は残す**（変更なし）。`THIRD_PARTY_NOTICES.md:21` のパスを直す（4 章 c2）。
- 削除する controller の `package.xml`・`CMakeLists.txt` は、同じ行から外す。`<author>Nimrod</author>` の記録の文（「bringup, controller, description and hardware」）から controller を外す（c3）。bringup・description・hardware の `package.xml` は残るので、「`LICENSE.so101_ros2` は、これらが 1 つも残らなくなったときだけ削除できる」という文言は変えない。
- MIT の条件は「著作権表示と許諾文を**コピーに残すこと**」。`controllers.yaml` の冒頭に出所の 1 行（`値の出典: nimiCurtis/so101_ros2（MIT。LICENSE.so101_ros2）`）を足す案は `docs/license_plan.md:164`（5 番、別承認）にある。移動と**同じコミットにしない**（sha256 の確認を単純にするため。D5）。
- **履歴を残すために `git mv` を使う**（推奨）。内容が同一なら類似度 100% のリネームとして検出される（実装で `git show --stat -M` で確認）。`package.xml`・`CMakeLists.txt` は `git rm`（直前のコミットから取り出せる）。

## 3. 案の比較と推奨

### 3.1 移動先

| 案 | パス | 長所 | 短所 |
|---|---|---|---|
| **P1（推奨）** | `src/so_arm101_bringup/config/controllers.yaml` | launch が読む設定を、launch と同じパッケージに置ける。CLAUDE.md の最終構成（bringup = ランチャー一式）と合う。パッケージが 1 つ減る | hardware の試験が bringup の src を読む（`../so_arm101_bringup/config/…`）。ただし今も `../so_arm101_moveit_config/config/joint_limits.yaml` を読んでおり前例あり。package.xml の依存は増えない（bringup → hardware の向きのまま） |
| P2 | `src/so_arm101_hardware/config/controllers.yaml` | 試験は自パッケージ内。hardware に `config/` が既にある | コントローラの設定はドライバの設定ではない。将来リーダーアームを `/leader` で動かすとき、hardware の config に混ざる |
| P3 | `src/so_arm101_moveit_config/config/` | 試験は既に読んでいる | `ros2_controllers.yaml` と並び、**混同が最大**。Setup Assistant の再生成で壊れうる |

### 3.2 ファイル名

| 案 | 判断 |
|---|---|
| **N1（推奨）`controllers.yaml` のまま** | sha256 の比較が単純。100% のリネームになる。docs の記録（`controllers.yaml:43` など）と名前が合う。bringup に同名のファイルは無い |
| N2 `ros2_control_params.yaml` などに変える | `ros2_controllers.yaml` との混同は減るが、履歴の追跡が崩れ、docs 26 か所以上の記録の名前と合わなくなる |

## 4. 変更の一覧（行番号は HEAD `a5c8cbd` のもの）

### (a) `git mv` / `git rm` するファイル

| # | 操作 | 対象 | 理由 |
|---|---|---|---|
| a1 | `git mv` | `src/so_arm101_controller/config/controllers.yaml` → `src/so_arm101_bringup/config/controllers.yaml` | 本体の移動。内容は変えない（sha256 `8515d643…` が同一） |
| a2 | `git rm` | `src/so_arm101_controller/package.xml`（32 行） | パッケージを無くす。使われている依存が無い（2.1） |
| a3 | `git rm` | `src/so_arm101_controller/CMakeLists.txt`（25 行） | 同上 |
| a4 | `rmdir`（git 管理外。中身なし） | `src/so_arm101_controller/launch/`、`config/`（a1 の後に空）、`src/so_arm101_controller/` | a1〜a3 で空になるディレクトリ。ファイルが無いので控えは要らない（`find` で空を確かめてから）。承認の対象に含める |

CLAUDE.md の規則どおり、**承認を得てから実行**し、`docs/inventory.md` に「削除の直前のコミット `<hash>` から取り出せる」と書く（hash は実装のコミットの直前の `a5c8cbd`。実装のコミットのメッセージにも書く）。

### (b) CMake・package.xml の変更

| # | ファイル:行 | 前 | 後 |
|---|---|---|---|
| b1 | `src/so_arm101_bringup/CMakeLists.txt:14-16` の直後 | `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}` … `)` だけ | 続けて `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})` を追加 |
| b2 | `src/so_arm101_bringup/package.xml:15` | `  <exec_depend>so_arm101_controller</exec_depend>` | 行を削除（description・moveit_config・hardware・controller_manager は残す） |
| b3 | `src/so_arm101_hardware/CMakeLists.txt:75` | `CONTROLLERS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../so_arm101_controller/config/controllers.yaml")` | `…/../so_arm101_bringup/config/controllers.yaml")` |
| — | `src/so_arm101_hardware/package.xml` | — | 変更なし（試験がソースのパスを読むだけ。循環する依存は作らない） |

### (c) launch・docs・手順書の変更

| # | ファイル:行 | 前 | 後 |
|---|---|---|---|
| c1 | `src/so_arm101_bringup/launch/bringup.launch.py:98` | `get_package_share_directory("so_arm101_controller"), "config", "controllers.yaml"` | `get_package_share_directory("so_arm101_bringup"), "config", "controllers.yaml"`。**行数は変わらない**ので、bringup の lint の失敗（名前に行番号が入る `E501 …:115,136,193,194` など）の名前も変わらない。98 行は今 89 桁で、`so_arm101_controller` → `so_arm101_bringup` で 3 桁短くなる（86 桁）（E501 は増えない。実装で確認） |
| c2 | `THIRD_PARTY_NOTICES.md:21` | `src/so_arm101_controller/config/controllers.yaml` (values from upstream `so101_controllers.yaml`), `src/so_arm101_controller/package.xml`, `CMakeLists.txt` | `src/so_arm101_bringup/config/controllers.yaml` (values from upstream `so101_controllers.yaml`; moved from the removed `so_arm101_controller` with `git mv`)。controller の `package.xml`・`CMakeLists.txt` は外す |
| c3 | `THIRD_PARTY_NOTICES.md`（`<author>Nimrod</author>` の文） | `… of bringup, controller, description and hardware …` | `… of bringup, description and hardware …` |
| c4 | `docs/inventory.md`（末尾に追記） | — | 「E の実施」: 実装のコミット、削除の直前のコミット、旧→新のパス、6 章の控え、7 章の前後の表、モック・実機の確認の結果 |
| c5 | `docs/lerobot_removal_plan.md:251`・`docs/original_urdf_plan.md:610`・`docs/license_plan.md` の `so_arm101_controller` の行 | 「保留」「外せない」など | **書き換えない**（当時の記録）。c4 の追記から、この計画へ案内する |
| c6（別コミット。D5） | `src/so_arm101_bringup/config/controllers.yaml` の冒頭コメント | `#   - so_arm101_moveit_config/config/ros2_controllers.yaml （MoveIt Setup Assistant 生成。無変更・未使用）` ほか | 「実行時に読まれるのはこのファイルだけ。`ros2_controllers.yaml` は Setup Assistant 用」と、出所の 1 行（`license_plan.md` 5 番）を足す。値は変えない |
| — | 手順書 `docs/so_arm101_calibration_check.md`・`docs/feetech_write_check.md` ほか | 記載なし | **変更なし** |
| — | `CLAUDE.md`、`LICENSE`、`LICENSE.so101_ros2` | | 変更なし |

## 5. 戻し方

- **実装のコミット**: `git revert <実装のコミット>`、または `git checkout a5c8cbd -- src/so_arm101_controller src/so_arm101_bringup src/so_arm101_hardware/CMakeLists.txt THIRD_PARTY_NOTICES.md`（新しく足した `bringup/config/` は `git rm`）。その後 `colcon build`。空の `launch/` は git で戻らないが、中身が無く使われていないので不要。
- **消した `install/so_arm101_controller`・`build/so_arm101_controller`**: git では戻らない（管理外）が、パッケージを戻した後の `colcon build --packages-select so_arm101_controller` で**再生成できる**。元と同じかは 6 章の控えと比べる。
- 戻した後の確認: `ros2 pkg prefix so_arm101_controller` が成功、`--show-args` が 21 個、`colcon test` の件数が前と同じ。

## 6. 消す前に控えるもの（CLAUDE.md の規則）

- **git 管理下**（`git rm`）: a2・a3 は、実装のコミットの直前のコミット（`a5c8cbd`）から取り出せる。`inventory.md` に hash を書く。
- **git 管理外**（案 B で消す `install/so_arm101_controller`・`build/so_arm101_controller`）: 消す**前**に `docs/inventory.md` へ書く。
  1. `find install/so_arm101_controller -mindepth 1 \( -type l -o -type f \) -printf '%y %p -> %l\n' | sort`（実ファイル 9 + リンク 15。リンクの先も）。
  2. 実ファイル 9 個の `sha256sum`。`colcon-core/packages/so_arm101_controller` の中身: `controller_manager:joint_state_publisher_gui:rclcpp:rclpy:robot_state_publisher:ros2launch:sensor_msgs:so_arm101_description:trajectory_msgs:xacro`。
  3. `build/so_arm101_controller` はファイル数（`find | wc -l`）と、再現に要る `cmake_args.last`・`colcon_build.rc`・`colcon_test.rc` の内容、`CMakeCache.txt` の `CMAKE_INSTALL_PREFIX`。
  4. 消すものがこの 2 パスだけであること、ビルドで再生成できること。
- **`log/` は消さない。** クリーンビルド（案 A）を選んだときは、直前に報告する。

## 7. 前後で比べるもの

実装の**前**に基準を取り直して `docs/inventory.md` に記録する（下の「今の値」は、今回の調査で取れたもの）。いずれもポートを開かない。

| # | 比べるもの | 方法 | 今の値 | 期待（後） |
|---|---|---|---|---|
| 1 | `controllers.yaml` の sha256 | `sha256sum`（移動の前後） | `8515d643d6c60175b3216da52cc66ee1a52982fc3e4af6187ed892bbccf20308`（src・install で同一） | **同一**（c6 は別コミット） |
| 2 | `ros2_control_node` に渡るパラメータファイルのパス | `bringup.launch.py` を Python で読み込み、`Node` の `parameters` を見る（2.3 と同じ。ノードは起動しない）。`realpath` も | `…/install/so_arm101_controller/share/so_arm101_controller/config/controllers.yaml`（実体 `src/so_arm101_controller/config/…`）。parameters は 2 つ | `…/install/so_arm101_bringup/share/so_arm101_bringup/config/controllers.yaml`（実体 `src/so_arm101_bringup/config/…`）。個数・1 つ目（`robot_description`）は同一 |
| 3 | `install/` のファイル一覧の diff | `find install -mindepth 1 -printf '%y %p -> %l\n' \| sort`（前後）→ `diff` | controller 以外 286 件 + controller 分（実 9・リンク 15 ほか） | 下の表。**各行に理由を書く** |
| 4 | `--show-args` の 21 個 | `ros2 launch so_arm101_bringup bringup.launch.py --show-args`（名前と説明を前後で `diff`） | 21 | 同一 |
| 5 | `robot_description` の sha256（6 通り） | `~/so_arm101_check/gen_rd6.sh <出力先>` を前後で実行し、`~/so_arm101_check/urdf_after/` と `cmp` | 6 通りとも `urdf_after/` と**同一**（確認済み）: `1_mock_feetech`・`2_mock_default` = `d8eebdcd…`、`3_feetech_ro` = `9bfaceea…`、`4_feetech_rw` = `a5160ee8…`、`5_feetech_rw_apply` = `6bf7aa60…`、`6_feetech_rw_probe` = `7dc2c92c…` | 同一（xacro・URDF を変えない） |
| 6 | `MoveItConfigsBuilder` の pipelines | Python で `MoveItConfigsBuilder("so101_new_calib", package_name="so_arm101_moveit_config").to_moveit_configs()` の `planning_pipelines` と `to_dict()` のキー | `chomp`・`ompl`・`pilz_industrial_motion_planner`、default `ompl`。`to_dict()` のキーは 18 個（`chomp`、`default_planning_pipeline`、`moveit_controller_manager`、`moveit_manage_controllers`、`moveit_simple_controller_manager`、`ompl`、`pilz_industrial_motion_planner`、`planning_pipelines`、`publish_*` 6 個、`robot_description`、`robot_description_kinematics`、`robot_description_planning`、`robot_description_semantic`） | 同一 |
| 7 | `colcon test` の件数 | `colcon test-result --all`、パッケージごとは `colcon test-result --test-result-base build/<pkg>` | 全体 **231 件・失敗 10**。bringup 51（失敗 10。すべて `launch/bringup.launch.py` の lint: pep257 D400・D415、copyright、flake8 E501 ×4 ほか）、calibration 73、controller 0、description 2、hardware 105、moveit_config 0 | 全体 231・失敗 10。**各パッケージの件数も同一**（controller の 0 件だけ、パッケージごと無くなる）。失敗の名前も同一（c1 は行数を変えない）。実装後に `colcon test` を回して数える（今の値は前回の実行の結果） |

**7 の 3（install の diff）の期待**:

| 行 | 前 → 後 | 理由 |
|---|---|---|
| `install/so_arm101_bringup/share/so_arm101_bringup/config/controllers.yaml` | 無 → 有（src の `controllers.yaml` へのリンク） | b1。symlink-install でも、**bringup の `colcon build` を回さないとリンクされない** |
| `install/so_arm101_bringup/share/colcon-core/packages/so_arm101_bringup` | 内容から `so_arm101_controller` が消える | b2 |
| `…/ament_index/resource_index/package_run_dependencies/so_arm101_bringup`（実体は `build/` 側） | 同上 | b2 |
| `install/so_arm101_bringup/share/so_arm101_bringup/launch/__pycache__/…` | `.pyc` の更新の可能性。diff では無視 | c1 でソースが変わる |
| `install/so_arm101_controller/**`（実 9 + リンク 15）、`build/so_arm101_controller/**` | 有 → 無（案 B で消したとき。案 C なら**残る**） | a2・a3 でパッケージが無くなる |
| ほかのパッケージ（description・hardware・calibration・moveit_config） | **変更なし** | b3 は試験の定義だけで、install するファイルは変わらない |

## 8. 実装後の確認

1. **オフライン**: 7 章の 1〜7。`colcon build` が通る（5 パッケージ）。`colcon list` と `ros2 pkg list | grep so_arm101` が 5 個。案 B のときは `ros2 pkg prefix so_arm101_controller` が**失敗する**こと。
2. **モック**（**ユーザーが「実機を使っていない」と返事をした後**、`mock:=true` に限る）: 1 回起動し、`ros2 control list_controllers` で **3 つ（`joint_state_broadcaster`・`arm_controller`・`gripper_controller`）が active**。`ros2 param get /arm_controller open_loop_control` → True、`ros2 param get /arm_controller update_rate` → 25。止めるのは、自分で起動した PID への `kill -INT` だけ（`pkill`・`killall` は使わない）。`ps` と `ros2 node list` の**両方**で残りが無いことを確認（`ros2 node list` だけに残り `ps` が空なら `ros2 daemon stop; ros2 daemon start` の後にもう一度）。
3. **実機の読み取り専用の起動**: **ユーザーに依頼する**（私は実行しない。`read_only:=true` の既定のまま。手順は `docs/feetech_readonly_check.md`）。見るもの: 3 つのコントローラが active、`open_loop_control` が True。終了時の `move_group`・`rviz2` の Segmentation fault は、`inventory.md` の「無視してよい出力の一覧」にある。
4. **コミットの前**: `colcon build`、関係するパッケージ（bringup・hardware。念のため全体）の `colcon test`。失敗があればコミットの**前に**報告する。

## 9. 実施の段階（案）

| 段階 | 内容 | 確認・コミット |
|---|---|---|
| E0 | 基準の取得（7 章）と、6 章の控えを `inventory.md` に書く（案 B のとき） | コミット 1 に含める |
| E1 | a1〜a4、b1〜b3、c1〜c4 | `colcon build`、7 章の 1・2・4〜7、`colcon test`。**コミット 1**（`git show --stat -M` でリネームの検出を確認） |
| E2 | 古い `install/so_arm101_controller`・`build/so_arm101_controller` の片付け（案 B。承認と直前の報告）。7 章の 3、`ros2 pkg prefix` の失敗の確認 | `inventory.md` に追記（**コミット 2**。docs のみ） |
| E3 | モックの確認（ユーザーの返事の後）、実機の読み取り専用の起動（ユーザーに依頼） | 結果を追記（**コミット 3**。docs のみ） |
| E4（任意） | c6（冒頭コメント。出所の 1 行を含む） | `colcon build`・`colcon test`。別コミット（sha256 が変わることを記録） |

## 10. ユーザーに決めてほしいこと

| # | 項目 | 推奨 | 理由・ほかの案 |
|---|---|---|---|
| D1 | 移動先 | **P1: `so_arm101_bringup/config/controllers.yaml`** | launch と同じパッケージ。P2・P3 は 3.1 |
| D2 | ファイル名 | **`controllers.yaml` のまま** | sha256・履歴・docs の記録と合う。混同は D5 のコメントで防ぐ |
| D3 | 古い `install/`・`build/` の片付け | **B: 2 つのディレクトリだけを消す**（承認と直前の報告つき。先に 6 章の控え） | CLAUDE.md の「クリーンビルドのときだけ」の例外なので承認が要る。A・C は 2.5 |
| D4 | 参照の取りこぼしを試験で守るか | **今回は追加しない**。8 章 1 の `ros2 pkg prefix` の失敗で確かめる | 試験を足すと件数が変わる。必要なら `test_launch_arg_check.py` に「launch が `so_arm101_controller` を参照しない」の 1 件を足せる（別承認） |
| D5 | `controllers.yaml` の冒頭コメントの更新と出所の 1 行（`license_plan.md` 5 番） | **別コミット（E4）で、承認後に** | 値は変えないが sha256 が変わる。移動の前後の同一性の確認を単純にしたい |
| D6 | docs の歴史的な記録のパス | **書き換えず、`inventory.md` の末尾に「旧→新」を足す** | 当時の記録を変えない |
| D7 | コミットの分け方 | **E1・E2・E3・E4 の 4 つ** | 戻しやすい。E2・E3 は docs のみ |
| D8 | 各 `package.xml` の `<author>Nimrod</author>` | **今回は変えない** | nimiCurtis 由来のファイルが残る間は残す（`original_urdf_plan.md:470`） |
| D9 | モックの確認の時期 | **ユーザーが「実機を使っていない」と返事をした後** | 実機を使っているときは起動も停止もしない |

## 11. 不確かなこと（実装の段階で確かめる）

- **[推測]** colcon が古い `install/so_arm101_controller` を消さないこと、消えるまで `ros2 pkg prefix` が成功すること（実装後に `ls`・`ros2 pkg prefix` で確かめる）。
- **[推測]** `ros2_control_node` が bringup の share の `controllers.yaml` を読むこと（2.3 は、launch に渡るパスまで。値は 8 章 2 のモックで確かめる）。
- **[推測]** bringup の `colcon build` で `install/…/config/controllers.yaml` がリンクとして作られること（symlink-install の通常の動作）。
- **[推測]** `test_launch_arg_check.py` は、古い install が残っていても通ること（2.2）。
- **[推測]** c1 の後も bringup の lint の失敗の名前が変わらないこと（行数は変えない。実装で `colcon test-result` の名前を前後で比べる）。
- `usd/` の由来、`servo.yaml` の元は、今回の範囲外（`docs/license_plan.md`）。
