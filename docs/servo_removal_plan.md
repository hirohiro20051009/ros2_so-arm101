# Servo 一式の削除の計画

作成: 2026-11-13。**【実行済み（2026-11-13。削除の直前のコミット `c764829`。結果は `docs/inventory.md` の「Servo 一式の削除」の節）】** 以下は実行前の計画のまま。
（当初の記述: 計画だけ。削除・ファイルの変更・ビルド・モック・ノードの起動はしていない。） ユーザーの承認を得てから実行する（CLAUDE.md の削除の規則: 一覧と理由を示し、承認を得てから `git rm`）。
**承認と決定（ユーザー、2026-11-13）: 削除を承認。J1: `servo_math.py` は削除。J2: `moveit_config` 側の `servo.launch.py`・`servo.yaml` と、`moveit_config/package.xml` の `moveit_servo` の依存も削除（R6・R7・E7 を追加）。bringup のクリーンビルドを承認（実機は使っていない。ビルドの直前に `ps` で launch・`ros2_control_node` などが動いていないことを確認し、動いていれば自分では止めずに報告）。以前 moveit_config の Servo を残した理由は `docs/inventory.md` に記録として残す。削除の直前のコミットは `c764829`。コミットは削除だけで 1 つ。**
前提（ユーザーの決定、2026-11-13）: (d) 解析 IK の差分モードを採用する。**(d) が実現できる見込みがある**と判断できたら Servo 一式を削除する。**判断と根拠は `docs/analytic_diff_plan.md` の 5 章**（見込みあり・条件つき: 解析 IK の定数の更新、ラッパーの姿勢の対応、実機での摩擦の確認）。この計画はその続き。
区別: **[事実]** はリポジトリの確認、**[推測]** は未確認。

## 1. 戻すための印

- **削除の直前のコミット**: 実行の直前の `HEAD`（この計画のコミットの次。実行時に `git rev-parse HEAD` で確定し、`docs/inventory.md` に「削除の直前のコミット `<hash>` から取り出せる（`git checkout <hash> -- <path>`）」と書く）。
- **Servo の実装を足したコミット**（参照用）: `5b35507`（段 1: 設定・静的試験）、`d6d141c`（段 2: `use_servo`）、`5689f27`（段 3: 送信ツール）、`66a9acc`（exec_depend と試験の分離）、`2bb511f`（送信ツールの Ctrl-C）。**Servo の関係の物は、すべて git 管理下**（`git ls-files` で確認済み。管理外のものは無い）。`~/so_arm101_check/` のログ（リポジトリの外）は削除しない。
- 削除は `git rm`（`archive/` には移さない）。`build/`・`install/`・`log/` は、**クリーンビルドのときだけ**（直前に報告）。

## 2. 削除する物の一覧と理由

### 2.1 `git rm`（ファイルごと削除）

| # | ファイル | 行数 | 理由 |
|---|---|---|---|
| R1 | `src/so_arm101_bringup/config/servo.yaml` | 65 | Servo 専用の設定。使い手が `use_servo` だけ |
| R2 | `src/so_arm101_bringup/config/servo_joint_limits.yaml` | 24 | Servo 専用の関節速度の上限（`robot_description_planning`）。同上 |
| R3 | `tools/send_servo_twist.py` | 284 | Servo に Twist を送る実機の道具。Servo が無ければ使えない |
| R4 | `tools/servo_math.py` | 128 | Servo の Jacobian・drift の計算。使い手は R3 と R5 だけ（**判断 J1**） |
| R5 | `src/so_arm101_bringup/test/test_servo_integration.py` | 468 | Servo の静的試験（35 件）。CMake の登録（E5）も消す |
| **R6** | `src/so_arm101_moveit_config/launch/servo.launch.py` | 55 | Servo のノードを起動する launch。KDL を渡すので drift と両立せず（`servo_integration_plan.md` 1.2）、実行時に使われない。**J2 の決定で追加** |
| **R7** | `src/so_arm101_moveit_config/config/servo.yaml` | 64 | MoveIt Setup Assistant 由来の元の設定（bringup の `servo.yaml` とは別物。実行時に使われない）。**J2 の決定で追加** |

### 2.2 編集（ファイルは残す。Servo の部分だけ取り除く）

| # | ファイル | 取り除くもの（現在の行） |
|---|---|---|
| E1 | `src/so_arm101_bringup/launch/bringup.launch.py` | docstring の `use_servo` の説明（`:38-40`）、`use_servo = LaunchConfiguration("use_servo")` と `servo_params`・`servo_joint_limits` の読み込み（`:130-132`）、`DeclareBooleanLaunchArg("use_servo", …)`（`:194-198`）、TimerAction の中の forward の spawner（`:241-246`。`include("spawn_controllers.launch.py")` だけが残る）、`servo_node_main` の Node（`:253-270` 付近）、**未使用になる `_read_yaml()`（`:94-96`）と `import yaml`（`:57`）**（`flake8` の未使用の警告を避ける）。`IfCondition`・`os` は、ほかの条件（`use_rviz`・`db`）や `bringup_config` で使うので残る（**実行時に確認**） |
| E2 | `src/so_arm101_bringup/config/controllers.yaml` | `controller_manager` の `forward_position_controller:` の定義（`:27-29` とコメント）と、末尾の `forward_position_controller:` のブロック（`:65-` 以降。`joints`・`interface_name`） |
| E3 | `src/so_arm101_bringup/package.xml` | `<exec_depend>moveit_servo</exec_depend>`（`:18`）、`<exec_depend>forward_command_controller</exec_depend>`（`:19`） |
| E4 | `src/so_arm101_bringup/test/test_launch_arg_check.py` | `len(DECLARED) == 22` → 21、名前の列から `'use_servo'`（`:66-70`） |
| E5 | `src/so_arm101_bringup/CMakeLists.txt` | `ament_add_pytest_test(test_servo_integration …)` とそのコメント（`:37-39`） |
| **E7** | `src/so_arm101_moveit_config/package.xml` | `<exec_depend>moveit_servo</exec_depend>`（`:24`）。**J2 の決定で追加**（`moveit_servo` への依存がリポジトリから消える） |
| E6 | `src/so_arm101_description/…` / `so_arm101_moveit_config/config/so101_new_calib.urdf.xacro` のコメント | `docs/servo_integration_plan.md 3.3.2` を参照するコメント（L1 の説明）。**L1 の値・引数は残す**。コメントは「Servo の」の語を整える程度（**判断 J3**。コメントだけなので省略可） |

### 2.3 判断を仰ぐもの（別扱い）

| # | 物 | 得失 |
|---|---|---|
| **J1（決定: 削除）** | `tools/servo_math.py` の**一部（`chain_from_urdf`・`jacobian`）は Servo 専用ではない**（URDF から FK と幾何 Jacobian を作る numpy の関数）。(d) の試験（URDF の FK との一致）と、将来の Diff-IK で使えそう | **残す**: 試験に使える。「Servo だけが使うもの」の削除の方針と合わない部分が残る（drift・射影の関数は不要）。**削る**: Servo の語・不要な関数が残らない。(d) の試験が必要なら、`so_arm101_ik` の試験に**自前の URDF の FK（30 行程度）**を書く。**推奨: 削除**（(d) は `so_arm101_ik` に自前の FK を持つ。`servo_math` を残すと、「Servo のものが残る」ことになる） |
| **J2（決定: 消す）** | **`src/so_arm101_moveit_config/launch/servo.launch.py`（55 行）と `config/servo.yaml`（64 行）、`package.xml` の `<exec_depend>moveit_servo</exec_depend>`（`:24`）**（ユーザーが以前の判断で残した Setup Assistant 一式の一部） | **残す**: 「Setup Assistant 一式を残す」判断と一貫。コストはゼロ（実行時に使われない）。**失**: `servo.launch.py` は `kinematics`（KDL）を渡すので、**drift と両立しない**（`servo_integration_plan.md` 1.2 で確認）＝**そのまま起動すると誤った設定になる**。`moveit_servo` の依存が残る。「Servo を削除した」と矛盾する。**消す**: 一貫。誤用を防ぐ。`moveit_servo` への依存が全パッケージから消える。**失**: Setup Assistant を再実行すると `servo.yaml` が再生成されうる（再生成を選んだ場合のみ）。`git` から戻せる。**推奨: 消す**（`servo.launch.py`・`servo.yaml`・`moveit_servo` の依存の 3 点。**ユーザーの判断**） |
| J3 | E6 のコメント | 省略してよい（docs が参照先として残る） |

### 2.4 残すもの（削除しない）

- **L1（ドライバの 1 周期の上限）**: xacro の `max_joint_speed`・`limit_window_s`、`sts3215_system.cpp` の `max_joint_speed_`・`limit_window_s_`、ドライバの試験。**JTC・MoveIt・差分モードの JTC 直接送りにも効く**。
- `tools/send_joint_delta.py`（JTC 直接。`servo` の語は「サーボの電源」の意味）。`tools/pose_margin.py`。
- **JTC・MoveIt・`arm_controller`・`gripper_controller`・`joint_state_broadcaster`**（削除後のコントローラは 3 つ）。
- **docs 全部**（`servo_integration_plan.md`・`servo_real_check.md`・`servo_w3_investigation_plan.md`・`diffik_accumulate_plan.md`・`diffik_tool_plan.md`・`accuracy_measurement_plan.md`・`dependency_audit_plan.md`・`inventory.md`）。**各ファイルの冒頭に「Servo は削除済み（削除の直前のコミット `<hash>`）」の注記**を足す（過去の記録は書き換えない）。**`servo_real_check.md` は Servo 専用の手順書だが、W 段階の共通の約束（止め方・12 V を切る条件・停止後の確認）の出どころなので残す**（新しい手順書がそれを引き継ぐ）。
- **THIRD_PARTY_NOTICES.md**: 変更なし（Servo のコードを含めていない）。

## 3. 不要になるもの

- **D14**（Servo の WARN の扱い）: 不要（`servo_node` が無くなる）。`inventory.md` に「削除により不要」と記録。
- **`inventory.md` の Servo 関係の残課題**: 「**削除により解消**」と記録（Servo の WARN・終了時の `ros2_control_node` の −11 の `use_servo:=true` での頻度の調査・`status` の意味・特異点のしきい値・Servo のモックの確認項目の未了分など。**一覧は実行時に `grep` で作る**）。
- **`ros2_control_node` の終了時の −11**: `use_servo:=false` での既知の挙動（`move_group` の −11）は残る（Servo と無関係）。`use_servo:=true` だけの観察は不要になる。

## 4. 削除の前後で比べるもの（実行時に、前に取って、後と比べる）

| 項目 | 前 | 後（見込み） | 方法 |
|---|---|---|---|
| **robot_description の sha256（6 通り）** | mock 2 通り `d8eebdcd27762b2b`、ro `f8dc8da49a238a75`、rw `088bdfa09bdeb0ca`、rw_apply `c5a19e7a006e269c`、rw_probe `1461b070b550512d`（inventory の「Servo 統合 段 1」の記録） | **すべて不変**（L1 は残る。xacro の `<param>` は変更しない） | `~/so_arm101_check/gen_rd6.sh`（前 `rm_before/`・後 `rm_after/`）、`cmp` |
| **`--show-args` の引数の数** | 22（`use_servo` を含む） | **21**。差は `use_servo` の 1 個だけ | `ros2 launch so_arm101_bringup bringup.launch.py --show-args`（ノードを起動しない） |
| **`colcon test` の件数** | 266（bringup 86、calibration 73、description 2、hardware 105、moveit_config 0） | **227**（bringup **47**）。**内訳（−39）**: `test_servo_integration.py` の pytest **−35**、`test_launch_arg_check.py` の引数ごとの試験（`DECLARED` 22 → 21）**−1**、ファイルごとの `copyright`・`pep257` が各 **−1**（−2）、ctest の登録（`test_servo_integration`）**−1**。（**[事実]** 266 − 39 = 227 は、Servo の試験を足す前の件数 227（inventory の「Servo 統合 段 1」の「244」の前）と一致） | `source /opt/ros/humble/setup.bash && source install/setup.bash` してから `colcon test` と `colcon test-result` |
| **`install/` の diff** | `install/so_arm101_bringup/share/so_arm101_bringup/config/` の `servo.yaml`・`servo_joint_limits.yaml`（`--symlink-install` のリンク）、`install/.../test` | **2 つの config ファイルが消える**（`ament_cmake` の再ビルドは、ソースから消えたファイルのリンクを**自動では消さない**。**ぶら下がったリンクが残る**ので、**bringup だけのクリーンビルド**（`build/so_arm101_bringup`・`install/so_arm101_bringup` を削除してビルド。**直前に報告**）を提案） | `find install -type f \| sort` の前後を `diff` |
| `controllers.yaml` | `arm_controller`・`gripper_controller`・`joint_state_broadcaster`・`forward_position_controller` | **forward を除く 3 つ**（Servo の段 1 の前と同一になる見込み。**`git diff 5b35507^ -- controllers.yaml` と比べて、forward の行以外が同一か**） | `diff` |
| launch の記述（`bringup.launch.py`） | 条件つきの追加 2 つ（servo_node、forward の spawner） | **条件つきの追加なし**（`use_rviz`・`db` のものを除く）。TimerAction の中は `include` だけ（Servo の段 2 の前と同一の見込み） | launch の記述の比較（段 2 で行った方法） |
| `ament` のリソース・パッケージの依存 | `moveit_servo`・`forward_command_controller` の exec_depend | 消える（J2 を消す場合は moveit_config の依存も） | `package.xml` の `grep` |

## 5. 実行の手順（承認後。1 段階ずつ。各段の後に `colcon build` が通ることと `git commit`）

| 段 | 内容 | ビルド | コミット |
|---|---|---|---|
| X0 | **前の記録を取る**（sha256 の 6 通り、`--show-args`、`find install`、`colcon test` の件数）。`git rev-parse HEAD` を控える | なし | なし |
| X1 | **`git rm` R1〜R5**（承認済みの分。J1・J2 は決定に従う）、**編集 E1〜E5**（J2 を消す場合は moveit_config の 3 点も） | **bringup のクリーンビルド**（直前に `build/so_arm101_bringup`・`install/so_arm101_bringup` の削除を報告）、`colcon build`（`--symlink-install` が通る）。**ほかのパッケージはビルド不要**（moveit_config を変えたら moveit_config も） | **コミット 1**（`colcon test` の件数を前後で比べ、**失敗があれば先に報告**） |
| X2 | **後の記録**を取って比べる（4 章）。結果を `docs/inventory.md` に（**戻すコミットを書く**。Servo 関係の残課題を「削除により解消」。D14 は不要） | なし | （コミット 2 にまとめる） |
| X3 | **docs の冒頭の注記**（2.4）と inventory | なし | **コミット 2**（docs。`colcon test` の件数は X1 と同一） |

- 実行中、**実機は使わない・モックも起動しない**（削除と静的な確認だけ。launch の `--show-args` と試験は、ノードを起動しない）。`--show-args` は launch のファイルを読むだけで、ノードは起動しない（これまでの記録と同じ）。
- **リスク（事実と推測）**: ① `bringup.launch.py` の編集で `IfCondition`・`os`・`yaml` の import の整理を誤ると `flake8` が落ちる（**試験で検知される**）。② `controllers.yaml` の編集で、`arm_controller` の設定を壊す（**`git diff` で forward 以外が不変**を確認）。③ ぶら下がった `install/` のリンク（クリーンビルドで解消）。④ **forward controller が無くなると、連続の VR（20〜50 Hz）の送り先は JTC のトピック経路になる**（`analytic_diff_plan.md` 2.7）。**必要になれば、削除の直前のコミットから戻せる**（`git checkout <hash> -- src/so_arm101_bringup/config/controllers.yaml` など）。

## 6. 決めてほしいこと

1. **削除の承認**（R1〜R5、E1〜E5）。
2. **J1**: `tools/servo_math.py` を削除するか（推奨: 削除。(d) の試験は `so_arm101_ik` に自前の FK）。
3. **J2**: `moveit_config` の `servo.launch.py`・`servo.yaml`・`moveit_servo` の依存を消すか（推奨: 消す。残す場合は、README に「使わない」と注記）。
4. bringup の**クリーンビルド**（`build/so_arm101_bringup`・`install/so_arm101_bringup` の削除）の承認（実行の直前に報告する）。
5. docs の冒頭の注記の文面（「Servo は削除済み。削除の直前のコミット `<hash>`。以下は削除前の記録」）でよいか。
