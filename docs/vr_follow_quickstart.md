# VR 追従（vr_follow）簡易説明書 P6d

対象: 別のマシンで、VR 機器から SO-ARM101 を動かして確認・修正する設計者。最低限の説明だけ。詳細は計画 `docs/p6_plan.md`、モックの結果 `docs/p6_mock_check.md`、実機の結果 `docs/p6_real_check.md`（9 章）。

> **状態**: このノード（`so_arm101_ik` の `vr_follow`）は、**偽入力（Joy と PoseStamped を作るスクリプト）でだけ**、モックと実機（2026-11-17、全段合格）で確かめた。**実際の VR 機器では、まだ何も確かめていない。** 値はすべて暫定（8 章）。
>
> **実機の安全**: 腕を動かすときは、腕を折りたたみ姿勢から手で支え、片手を 12 V の電源の近くに置く。Ctrl-C で停止するとトルクが OFF になり、腕は落ちる（止める前に、折りたたみ姿勢で支える）。非常停止は 12 V を切る。

## 1. 前提

- Ubuntu 22.04、ROS 2 Humble。MoveIt 2 は bringup が使う（vr_follow 自体は MoveIt を使わず、解析 IK の結果を `arm_controller`（JTC）へ直接送る）。
- 依存パッケージは、各 `package.xml` の `depend` と `exec_depend`（`rosdep install --from-paths src --ignore-src -r -y`）。Python は numpy。
- ビルドと source:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

- 端末ごとに、2 つの `source` が要る（上のとおり）。`ROS_DOMAIN_ID` は、VR 側のマシンと同じにする（複数マシンの接続は、この説明書の範囲外）。
- 実機のシリアルポートは、既定 `/dev/ttyACM0`（`serial_port:=` で変える）。ポートを開く権限（dialout グループなど）が要る。

## 2. 腕の校正（センターキャリブレーション）

校正の結果は `src/so_arm101_hardware/config/follower_calibration.yaml` に入る（bringup の `calibration_file:=` で別のファイルも指定できる）。リポジトリにある yaml は、**この開発用の腕のもの**（起動時に、EEPROM と yaml が違うと、書き込みモードは起動を拒否する）。場合を分ける。

- **同じこの腕を、別のマシンにつなぐ場合**: リポジトリの `follower_calibration.yaml` を**そのまま使う**。腕の EEPROM と一致しているので、**校正のやり直しは不要（EEPROM は書き直さない）**。変えるのは `zero_offset_deg` だけ: 0 にする（行ごと消してもよい。下の項）。
- **別の腕の場合**: リポジトリの yaml は使えない。**その腕で校正をやり直す**（下の手順）。`zero_offset_deg` は 0。
  - `src` の yaml を書き換えず、コピーで校正する方法（遠隔の作業者向けのコマンド一覧）: `docs/calibration_new_arm_remote.md`。

別の腕の校正の手順は `docs/so_arm101_calibration_check.md`（C0〜C3）と `docs/stage2c_plan.md`。要点（ツールは `ros2 run so_arm101_calibration so_arm101_calib`）:

1. **ros2_control（launch）を止めてから使う**（シリアルポートを占有する）。腕はトルク OFF。
2. `backup`: 全関節の EEPROM を保存する（読み取りだけ。`restore` の元になるので消さない）。
3. `measure`: 脱力 → 中央の姿勢 → 各関節を MIN・MAX まで動かす、を測って測定ファイルに保存する（EEPROM には書かない）。2 回行い、`compare` で再現性を見る。
4. `apply`: 測定ファイルの値を EEPROM（Homing_Offset と Min・Max の 3 項目）へ書く。既定は dry-run。`--write` と確認文字列 `WRITE-EEPROM` の入力が要る。**全関節を 1 回で書いたときだけ、yaml が更新される。**

### zero_offset_deg（0 のずれの補正）

- yaml の関節ごとの項目 `zero_offset_deg`。意味: **関節ごとの、角度の 0 のずれの補正。真の角度 = 測定した角度 + δ**（δ が zero_offset_deg）。単位は**度**。書かない関節は 0。
- **別のマシンでは、すべて 0 から始める**（同じ腕でも別の腕でも）。リポジトリの yaml に `zero_offset_deg` の行があれば、0 にするか、行ごと消す（この開発用の腕の暫定の値なので、そのまま使わない）。`apply` は、yaml にある `zero_offset_deg` を持ち越すので、別の腕で `apply` した後にも確認する。
- 値の測り方と決め方は**後で対応**（8 章）。0 のままでも動く（精度が下がる）。

## 3. 起動の手順

端末 A（腕。これが先）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
# 実機（書き込みモード。腕が動く。トルク ON）
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false torque_limit:=500 calibration_file:=<その腕の yaml の絶対パス（$HOME か /home/… で書く。~ は使わない）>
# モック（腕なし。まず、これで VR 側の動きを確かめてよい）
ros2 launch so_arm101_bringup bringup.launch.py mock:=true
```

- `calibration_file:=` は、**別の腕では `docs/calibration_new_arm_remote.md` で作ったコピー（`$HOME/so_arm101_check/follower_calibration_new_arm.yaml`）を指定する。開発用の腕では省略できる**（省略するとリポジトリの yaml が使われる）。別の腕で省略すると、リポジトリの yaml（開発用の腕のもの）が使われ、書き込みモードは起動を拒否する。
- **引数のパスに `~` を使わない（`$HOME` か絶対パスを使う。`:=` の後ろの `~` は展開されない）。** `calibration_file:=~/…` と書くと、ドライバに `~` のまま渡り、`cannot read calibration '~/…'; bad file` で起動に失敗する。
- `read_only:=true`（既定）の実機は、サーボに書かない（腕は動かない。指令は進むが測定は動かない）。
- bringup は、**俯瞰カメラを起動しない**（`so_arm101_camera` は別。ここでは使わない）。
- 実機は、起動前に、腕を折りたたみ姿勢にして支える。起動で `arm_controller`・`gripper_controller`・`joint_state_broadcaster` が active になる（`ros2 control list_controllers`）。

端末 B（追従ノード。何も操作しない）:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch so_arm101_ik vr_follow.launch.py scale:=0.5
```

- 起動直後は `IDLE START_WAIT`。**クラッチを押すまで何も送らない。**
- launch の引数（空なら `config/vr_follow.yaml` の値）: `scale`、`rate_hz`、`point_time_s`、`clutch_mode`、`max_tracking_error`、`gripper_open_rad`、`gripper_closed_rad`、`floor_z`、`button_clutch`、`button_open`、`button_close`、`params_file`。ほかの値は yaml を直して起動し直す。実行中に変えられるのは `scale` だけ（`ros2 param set /vr_follow scale 0.5`。範囲 (0, 3]。変えた瞬間に基準を取り直す）。
- **`floor_z` は、既定の 0.045 m のまま使う。** 下げられる（0.01 m 以上）のは、腕が動かない `read_only:=true` の起動のときだけ（ノードが `robot_description` で確かめ、違えば起動しない）。

### 作業姿勢への移り方

折りたたみ姿勢のままクラッチを押すと、先端の高さが床（0.045 m）に近く、**クラッチの前の検査が `REFUSED FLOOR` で拒否しうる**（F6 の読みからの手計算で余裕は約 4 mm。たたみ方で変わる）。作業姿勢（例: T = 関節 0, −22, 30, 83, 0 度。先端が約 x 0.177 m、z 0.059 m）へ移してから追従を始める。

**モックも同じ。** モックの初期姿勢は、先端 z が約 0.03 m で、クラッチが `REFUSED FLOOR` になる（`docs/p6_mock_check.md` の準備）。モックでも、RViz か ik_move で作業姿勢 T へ移してから始める。

- **ik_move の 2 段**（折りたたみから T への 1 回は、各関節 1.0 rad の上限で拒否されるため、中間の姿勢 M を経由）。各段の前に `--dry-run`（何も送らない）を見る。腕を支える:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -u tools/ik_move_run.py --dry-run --joints-deg 0 -55 60 40 0
python3 -u tools/ik_move_run.py --joints-deg 0 -55 60 40 0
python3 -u tools/ik_move_run.py --dry-run --joints-deg 0 -22 30 83 0
python3 -u tools/ik_move_run.py --joints-deg 0 -22 30 83 0
```

  結果は `REACHED`（終了コード 0）。実行のたびに、軌道の確認の Enter を聞かれる。**dry-run が拒否したら（折りたたみの出発点の z が 0.045 m 未満の `START_FLOOR_MEASURED` など）、実行しない。**
- **RViz**（bringup が起動する RViz の Joints タブで、同じ関節値へ動かす。実績あり。手順は `docs/zero_offset_apply_real_check.md` の段 c）。**elbow は +91° より上にしない。**
- 折りたたみへ戻すときは、ik_move が `GOAL_FLOOR`（先端 z が 0.048 m 未満）で拒否することがある。そのときは RViz で戻す。

## 4. VR 側が出すもの、ノードが出すもの

VR 側が出す（トピック名は `config/vr_follow.yaml` の `joy_topic`・`pose_topic` で変えるか、launch で remap する）:

| トピック（既定） | 型 | 内容 |
|---|---|---|
| `/vr/right/joy` | `sensor_msgs/Joy` | ボタン。クラッチとグリッパー。購読は既定の QoS（RELIABLE、深さ 10） |
| `/vr/right/pose` | `geometry_msgs/PoseStamped` | 手の位置と姿勢（右手系、単位 m）。`frame_id` が空か `base_link` ならそのまま。それ以外は TF で `base_link` へ変換する（変換できなければ捨て、STALE になる）。購読は既定の QoS（RELIABLE、深さ 100）。**メッセージごとに判定する**（1 周期に複数来てよい。上限 100 まで解く） |

- **QoS（注意）**: `/vr/right/pose` と `/vr/right/joy` の購読は **RELIABLE**。VR のブリッジが **BEST_EFFORT** で出すと、**届かない**（ROS 2 の QoS の仕様。RELIABLE の購読は BEST_EFFORT の出版と接続しない）。そのときは、ブリッジ側を RELIABLE にするか、ノードの QoS を変える（コードの変更。**未対応**）。確かめ方: `ros2 topic info --verbose /vr/right/pose`（`/vr/right/joy` も）で、出版側と `vr_follow` の購読側の Reliability を見る。
- 「新しさ」は、メッセージの `header.stamp` ではなく、**受け取った時刻**で見る。
- ボタンの割り当て（パラメータ、`Joy.buttons` の番号）:

| パラメータ | 既定 | 働き |
|---|---|---|
| `button_clutch` | 0 | クラッチ（追従の入り切り） |
| `button_open` | 1 | グリッパーを開く |
| `button_close` | 2 | グリッパーを閉じる |
| `clutch_mode` | `hold` | `hold`: 押している間だけ追従。`toggle`: 押すたびに切り替え |
| `gripper_requires_clutch` | false | true なら、グリッパーもクラッチを押している間だけ |

- **ボタンは 0.2 s 以上押す。** 1 周期（既定 100 ms）より短い押しは取りこぼす（クラッチもグリッパーも）。
- 開始は「離している状態から押した変化」だけ。起動時にすでに押されているボタンは、押し直すまで開始しない。Joy が途切れたあとに戻ったとき、押されたままのボタンも立ち上がりとみなさない。

ノードが出す:

| トピック | 型 | 内容 |
|---|---|---|
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 5 関節、位置のみ、1 点（`point_time_s` 0.08 s）。追従中だけ、周期 `rate_hz`（10 Hz）で |
| `/vr_follow/status` | `std_msgs/String` | 状態。`STATE REASON key=value …`（例: `TRACKING OK windup=0.03`、`HOLD_JUMP TARGET_JUMP … jump_mm=60`）。20 Hz と、状態が変わったとき。transient_local |
| `/vr_follow/target_pose` | `geometry_msgs/PoseStamped` | 倍率と投影の後の目標（`base_link`） |
| `/vr_follow/gripper` | `std_msgs/Float64` | ボタンの変化があったときだけ。0 = 全開、1 = 全閉。transient_local |

グリッパーは `/gripper_controller/gripper_cmd`（GripperCommand のアクション）へ送る。量 0〜1 を、`gripper_open_rad`（1.0）と `gripper_closed_rad`（0.05）の間に直線で対応させる（実機で、どちらも確かめた値）。グリッパーの目標は階段で、ドライバが 1 周期の上限で追う（ログに `step limit: 'gripper' … ramping toward it` の WARN が出てよい）。

状態は、端末 D で見る:

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo --no-daemon /vr_follow/status
```

## 5. VR の基準の取り方

- **クラッチで基準を取る（相対の動作）。** クラッチを押した瞬間の、腕の先端（指令）の位置・姿勢と、手の位置・姿勢を基準にする。以降、手の動き（基準からの差）が、腕の目標の動きになる。**押し直すたびに基準を取り直す**（ラチェット操作ができる）。
- 式（`base_link` の軸で）: 目標の位置 = 腕の基準 + scale × R_fix × (手の位置 − 手の基準)。姿勢は、手の基準からの相対の回転（scale をかけない。R_fix で軸を合わせる）を、腕の基準の姿勢にかける。
- **倍率 `scale`**: 手の動きに対する腕の動きの倍率（範囲 (0, 3]）。0.5 なら、手 20 mm で腕 10 mm。偽入力の実機確認は 0.5 で行った。
- **向きの合わせ込み `input_rotation_rpy_deg`**（[roll, pitch, yaw]、度。既定 [0, 0, 0]）: VR の座標軸を `base_link` の軸（x 前、y 左、z 上）に合わせる回転 R_fix。VR の座標系が違うとき、または `frame_id` を `base_link` にできないときに使う。**合わせ込みは未確認**（8 章）。
- **投影**: 腕は 5 関節で、pan が yaw を決めるため、**姿勢は pitch と roll だけを作れる。手の yaw は腕に伝わらない**（pan は位置から決まる）。捨てた角度は状態の `proj_deg` に出る。
- 判定（飛び・再開・遅れ）は、すべて**倍率をかけた後の腕の目標の空間（`base_link`）**で行う（VR の手の空間ではない）。

## 6. 状態と止まる条件、再開

腕は、止まっている間は何も受け取らない。JTC が最後の点を保持する（腕はそこで止まる）。

| 状態 | 理由 | 条件 | 再開 |
|---|---|---|---|
| IDLE | START_WAIT, RELEASED, JOY_TIMEOUT | 起動直後、クラッチを離した、Joy が 1 s（`joy_timeout_s`）途切れた（C4） | クラッチを押し直す |
| IDLE | REFUSED …（理由が並ぶ） | クラッチの前の検査に落ちた。WINDUP（指令と測定の差が 0.06 rad 超）、FLOOR（先端 z が 0.045 m 未満）、RANGE（指令が関節の範囲外）、POSE・JOINT_STATE・CONTROLLER_STATE（入力が古い）、BUSY（ほかの goal が動いている） | 原因を直し、押し直す |
| TRACKING | OK, SPEED_LIMITED | 追従中。1 周期に 1 点送る。SPEED_LIMITED は速さの上限（0.5 rad/s）で縮めた | |
| HOLD_JUMP | TARGET_JUMP | 入力の 1 メッセージで、目標が前のメッセージより 50 mm または 30° 以上動いた | 目標が、最後に受け付けた目標から 20 mm かつ 10° 以内に戻ると自動で再開 |
| HOLD_RANGE | UNREACHABLE, JOINT_RANGE, ELBOW_LIMIT, FLOOR, ORIENT, IK_JUMP | 目標が作れない（届かない、関節範囲外、elbow が +91° 超、先端 z が床未満、姿勢が決まらない）。IK_JUMP（C1）は、IK の解が前のメッセージの解から 0.35 rad 以上動いた（枝の入れ替わり） | HOLD_JUMP と同じ |
| HOLD_LAG | LAG | 目標が、指令の先端から 0.15 m 以上先（C2） | 遅れが半分以下に戻ると自動 |
| STALE | POSE など | pose か joint_states が 0.5 s 途切れた | HOLD_JUMP と同じ（遠くへ戻ったら、押し直し） |
| BUSY | | ほかの goal（MoveIt・ik_move など）が動いている | **クラッチの押し直し** |
| BLOCKED | TRACKING_ERROR | 測定と指令の差が 0.2 rad 超（C3）。腕が指令に追いつかない | **クラッチの押し直し**（下の注意） |

### BLOCKED の後（注意）

- BLOCKED の後も、**JTC は最後の点（測定から最大約 0.2 rad 先）を指令として持ち続ける。** そのため、次のクラッチは、指令と測定の差が `clutch_windup`（0.06 rad）を超えるあいだ `IDLE REFUSED WINDUP` になる。実機の読み取り専用で確かめた（`docs/p6_real_check.md` 9 章の c0-2。腕は指令に追いつかないので、差は 0.203 rad のまま残った）。
- **書き込みモードの見込み（実機では未確認）**: BLOCKED が出るのは、腕が何かに当たって指令に追いつけないとき。追いつけないままなら、次のクラッチも `REFUSED WINDUP` になる見込み。**当たっていた物が外れると、腕は JTC が持つ最後の点へ、クラッチなしで動く見込み。BLOCKED の後は、障害物を外す前に腕を支えること。**
- **戻し方**: bringup（端末 A）の起動し直し（支えてから Ctrl-C）、または、RViz か `ik_move_run.py` で**今の位置への goal** を送る（指令が測定に戻る）。そのあと、クラッチを押し直す。
- HOLD_JUMP・HOLD_RANGE・STALE の再開では、再開の周期で、解が最後に受け付けた解から 0.35 rad 以上離れていないことも見る。HOLD 中は、各メッセージを最後に受け付けた解から解き、それと比べる（f34a6c2。実機では未確認）。

## 7. 停止の手順

1. クラッチを離す（追従が止まる）。偽入力・VR を止めても、Joy が 1 s 途切れて IDLE になる。
2. 端末 B（vr_follow）を **Ctrl-C**。
3. 実機は、腕を折りたたみ姿勢へ戻し（3 章。ik_move が拒否したら RViz で）、**腕を支えてから**、端末 A（bringup）を **Ctrl-C を 1 回**。トルクが OFF になる。ログに `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints` が出て、`exit=0`。`move_group` と `rviz2` の終了時の exit code -11 は既知（無視してよい）。
4. 残りがないことを確認する（`ros2 daemon` には触らない）:

```bash
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|ik_move_run|vr_follow" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
```

どちらも空であること。`--no-daemon` にだけ残り、`ps` が空なら、数秒待ってもう一度確認する。プロセスは、**自分で起動したものだけ**を Ctrl-C（SIGINT）で止める。`pkill`・`killall`・`kill -9` は使わない。理由: 自分が起動していないプロセス（ほかの人の launch や ros2 daemon）まで止めてしまうため。`kill -9` は、終了時の処理（トルク OFF の確認）が走らないため。

## 8. 暫定の値と、次の担当者に残す課題

暫定の値（`config/vr_follow.yaml`。偽入力の実機確認で動いた値。実際の VR の手の動きでは未確認）:

| 値 | 暫定 | 備考 |
|---|---|---|
| 速さの上限 `max_joint_speed` | 0.5 rad/s | 全関節を同じ比率で縮める |
| 飛び `jump_pos`・`jump_ang_deg` | 50 mm・30° | 入力メッセージごと、倍率後の空間 |
| 再開 `resume_pos`・`resume_ang_deg` | 20 mm・10° | |
| C1 `ik_jump` | 0.35 rad | 枝の入れ替わりを見る |
| C2 `max_lag` | 0.15 m | |
| C3 `max_tracking_error` | 0.2 rad | 実機の偽入力の最大は 0.0423 rad（余裕は約 5 倍） |
| C4 `joy_timeout_s` | 1.0 s | |
| `stale_s` | 0.5 s | |
| `clutch_windup` | 0.06 rad | 追従中の残り（lift 約 0.027 rad、elbow 約 0.034 rad）より大きい |
| `floor_z`・`elbow_max_deg` | 0.045 m・+91° | |
| グリッパー | 閉 0.05、開 1.0 rad | 実機で確かめて確定 |

残課題:

1. **VR の座標の合わせ込み**（`input_rotation_rpy_deg`、`frame_id`、VR の原点・向き）。実際の VR の機器で、手の動きと腕の動きの向きを合わせる。未確認。
2. **閾値の見直し**（0.5 rad/s、50 mm・30°、C1〜C4）。実際の手の動きの記録（rosbag2 で `/vr/right/pose`、`/vr_follow/status` を記録）から決める。
3. **`update_rate` 25 Hz と、10 Hz 送信**。JTC は 20 Hz だと reference が 0.1 rad ずつ飛ぶため、10 Hz 送信・`point_time_s` 0.08 s にした（`docs/p6_plan.md` 7 章）。`update_rate` を上げる案は、`controllers.yaml` とドライバの試験に関わるので、行っていない。
4. **1 周期（100 ms）より短いボタンの押しの取りこぼし**（クラッチもグリッパーも）。VR のボタンの出し方（保持する、長めに出す）か、ノードの受け方（立ち上がりの保持）の改善。
5. **lift・elbow の追従の残り**: 追従中、lift と elbow は指令に少し追いつかない（`windup` に出る。T で 0.032 rad 程度）。補正の要否を見る。
6. **`zero_offset_deg` の測り方と決め方**: **後で対応。** 計画は `docs/zero_offset_calib_plan.md`（床当ての方法。この腕の実機では不合格で保留）。別の方法（カメラの当てはめ、`docs/camera_stage_plan.md`）もある。決まるまでは 0 のままにする。
7. **書き込みモードの BLOCKED の後の動き**（腕が追いつけないまま・障害物が外れたときの動き）は、実機で未確認（6 章）。
8. **HOLD 中の IK の比べ方の修正**（f34a6c2）は、実機・モックで未確認。次に jump を行うときに、再開の瞬間の `ik_jump` が小さいことを見る。
9. **手の yaw は腕に伝わらない**（5 関節。設計上の制約）。
10. **入力の QoS**: Joy・PoseStamped の購読は RELIABLE。VR のブリッジが BEST_EFFORT だと届かない（`ros2 topic info --verbose` で確かめる）。ブリッジ側を RELIABLE にするか、ノードの QoS を変える（コードの変更。未対応）。
