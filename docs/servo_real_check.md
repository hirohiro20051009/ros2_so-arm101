# 実機確認手順: MoveIt Servo（Diff-IK。use_servo）

対象: `use_servo:=true` の実機。段階 R（読み取り専用。腕は動かない）と段階 W（書き込みモード。**腕が動く。サーボにトルクが入る**）。**実行するのはユーザーだけ**（Claude は実機を動かさない）。
計画と根拠: `docs/servo_integration_plan.md`（6.2 がこの手順書の目次。14 章がモックの結果）。書き込みモードの基本は `docs/feetech_write_check.md`（W0〜W4。この手順書は、その共通の約束と止め方を引き継ぐ）。校正の状態は `docs/so_arm101_calibration_check.md`。

**範囲**: 「出発点から MoveIt（OMPL）で中間の姿勢に出る → 止まっていることを確かめて Servo（forward controller）に切り替える → Python から単発の小さい補正（`tools/send_servo_twist.py`）→ Servo を止めて JTC に戻す → MoveIt で出発点へ戻す」まで。**VR・共通の層（ユーザーのブリッジ）・モード管理ノード・解析 IK は、この手順書の範囲外**（末尾の「後の段階」に目次だけ）。

> **【注（2026-11-13）W3 以降は、Servo の見直しが決まるまで実行できない。** 実機の W 段階（W0・W1・W2・W5・W6 は合格、W4 は省略）で、**W3 は腕が動かなかった**（Servo の status は全周期 0）。結果と調査の計画は `docs/servo_w3_investigation_plan.md`。この手順書の W3・W4 と Q1〜Q6 は、見直しの結論が出るまで保留（直しは見直しのあとに 1 回）。**以下の実測値は、2026-11-13 の W 段階の実測で訂正した。**

段階は R0 → R1 → R2 → R3 → W0 → W1 → W2 → W3 → W4 → W5 → W6。**前の段階が合格するまで次へ進まない。** 各ステップに「実行するコマンド」「確認すること」「合格の条件」がある。

---

## 共通の約束（全段階）

1. **実行するのはユーザーだけ。** 新しい端末（conda を activate していない端末）を使う。端末は、launch 用（ログを流す。**操作しない**）と、操作用（コマンドを打つ）の 2 つ以上。
2. **止めるのは Ctrl-C（SIGINT）だけ。** `kill -9`、`pkill`・`killall`（名前での kill）、端末を閉じる、PC の電源断はしない（トルクが ON のまま残る）。ほかのプロセスを止めるのは、**自分で起動したものを、控えた PID で `kill -INT`**（S4）だけ。
3. **書き込みモード（W 段階）の前に、腕を折りたたみ姿勢にして、手で支える。** 止める前にも、支える。Ctrl-C でドライバがトルクを OFF にする（腕は脱力して落ちる）。
4. **腕の周りに物を置かない。** ケーブルが引っかからないようにする。**床に近づく姿勢・指令にしない。床は腕の取り付け面と同じ高さ（`base_link` の z ≈ −0.0024 m。2026-11-13 の実測）。** 床までの距離は **TCP（指先の間）で測る**（グリッパーの見た目の最下面は、姿勢の影響が大きいので使わない。参考: 見た目の最下面から床まで、出発点で 55 mm、T で 65 mm）。指は、真下の接近で先端のフレームより約 6〜7 mm 下に出る。この手順書の動きは、先端が**床から十分高い**（中間の姿勢 T で先端 z ≈ 0.055 m（実測）、出発点で 0.033 m（実測））範囲だけで、**下げる向き（−z）は 1 回 5 mm 以内、T からの正味の下がりが 15 mm 以内（先端 z ≥ 0.045 m を優先）**にする。
5. **非常停止はサーボの外部電源（12 V）を切る**（USB だけではない）。常に片手を電源の近くに置き、もう片手で腕を支える。切ると腕が落ちる（下の「12 V を切る条件」）。
6. ログの `FAULT:` または `COMMUNICATION LOST` が出たら、その段階を中止して、支えながら Ctrl-C。原因を報告してから再開する。
7. **前の段階のログの末尾を確認してから次へ**（下の「停止したあとの確認」）。
8. 後始末の確認（毎回、止めたあと）: `ps aux | grep -E "ros2_control_node|move_group|servo_node_main|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"` が空。新しいシェルの `ros2 node list` に名前が残っても、`ps` が空なら **ros2 daemon の古い一覧（既知）**として報告するだけでよい（`ros2 daemon` には触らない）。
9. 非常時にサーボの外部電源（12 V）をすぐ切れる位置に立つ。

### 止め方の表（全段階で共通。上から順に試す）

| 順 | 操作 | 効果 | コマンド・手順 |
|---|---|---|---|
| S1 | Twist の送信を止める | 0.1 秒後に Servo の出力が止まる。腕は最後の目標で止まる | 送信ツールの端末で **Ctrl-C**（ツールは、ゼロの Twist を 0.3 秒送ってから、終了コード 130 で終わる。確認の画面での Ctrl-C は `cancelled (nothing was sent)` で静かに終わる） |
| S2 | `pause_servo` | Servo が何も出さなくなる（`/forward_position_controller/commands` が止まる。**`/servo_node/status` は止まらない**）。**止めておく鍵ではない**: `send_servo_twist.py` は、送る前に必ず `unpause_servo` を呼ぶので、ツールを実行すると再開する（S2 のあと、別の送信を止めておきたいときは、S3 で forward controller を外す） | `ros2 service call /servo_node/pause_servo std_srvs/srv/Trigger` |
| S3 | **手動で JTC に戻す** | forward controller が外れ、`arm_controller` が直前の指令を保持する | 先に「止まっていることの確認」（コマンド A）→ `ros2 control switch_controllers --strict --deactivate forward_position_controller --activate arm_controller` → `ros2 control list_controllers` で `arm_controller` が `active`、`forward_position_controller` が `inactive`。**両方 `inactive` なら `ros2 control switch_controllers --strict --activate arm_controller`**（保持のまま） |
| S4 | Servo のノードだけを止める | 出力が止まる（forward controller は最後の目標を保持。腕は止まる） | `ps aux \| grep servo_node_main \| grep -v grep` で **自分の launch の子の PID** を控え、`kill -INT <PID>`。launch は動き続け、forward controller は active のまま（R3 で確認済み） |
| S5 | ハードウェアの launch を止める | トルク OFF。**腕が自重で落ちる** | 腕を折りたたみ姿勢で支えて、launch の端末で **Ctrl-C を 1 回** |
| S6 | 12 V を切る | トルクが切れて腕が落ちる（最後の手段） | 下の条件のときだけ |

**12 V を切る条件**: 次のどれかで、S1〜S4 が効かないとき。
- (a) 停止の操作の後も、腕が 1 秒以上動き続ける、または床・自分の体・物に向かって動いている
- (b) 異音（ガリガリ、うなり）、焦げた匂い、煙、サーボの異常な発熱（触れない熱さ）
- (c) ドライバが `COMMUNICATION LOST` を出し、腕が保持せずに動く
- (d) 手が届かない、または S5 の操作ができない状況

切る前に、腕の下から手と物をどける。切ると腕は落ちる（姿勢によっては床に当たる）。**事前に、落ちる姿勢の下に障害物が無いことを確認する。**

### 止まっていることの確認（切り替えの直前と、戻す直前に、毎回）【コマンド A】

位置を 0.6 秒あけて 2 回読み、5 関節の差がすべて 0.003 rad（約 2 tick）以内であること。`/joint_states` の velocity は、1 tick/周期 ≒ 0.077 rad/s の有限差分のノイズがあるので、判定に使わない。あわせて、MoveIt の実行が終わっている（RViz に実行中の表示が無い）こと。

```bash
python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
rclpy.init(); n = Node('stopped_check')
def read():
    st = {}
    sub = n.create_subscription(JointState, '/joint_states', lambda m: st.update(zip(m.name, m.position)), 10)
    end = time.time() + 3
    while time.time() < end and not all(j in st for j in ARM):
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_subscription(sub)
    return [st[j] for j in ARM]
a = read(); time.sleep(0.6); b = read()
d = max(abs(x - y) for x, y in zip(a, b))
print('pos1', [round(x, 5) for x in a]); print('pos2', [round(x, 5) for x in b])
print('max diff %.5f rad -> %s' % (d, 'STOPPED' if d <= 0.003 else 'MOVING (do not switch)'))
EOF
```

### 停止したあとの確認（W 段階。**毎回**）— 終了時の Segmentation fault について

**事実（モックの調査。計画 14 章）**: `use_servo:=true` の終了時に、`ros2_control_node` が exit −11（Segmentation fault）で終わることが、5 回に 1 回あった（`use_servo:=false` は 2 回とも無し。`move_group` の −11 は毎回で既知）。**落ちたのは、ハードウェアの `deactivate`・`shutdown` が成功した後**（ログ順: `'deactivate' hardware … Successful` → `'shutdown' hardware … Successful` → `Shutting down the controller manager.` → その後に落ちる。`rclcpp::Executor::~Executor()`、main の後始末）。**実機の書き込みモードで、トルク OFF・Lock=1 の確認（`on_deactivate`）より前に落ちた例は、観察していない。** ただし、実機のドライバでは未確認（モックにはトルクが無い）。

そこで、**停止のたびに、次を確認する**:
1. ログに次の 2 行があること（**無ければ、トルクが ON のままの可能性がある。腕を支えて 12 V を切る（S6）**）:
   `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`
   `deactivated. reads=… goal_writes=… monitors=… rate_limited=… port_contention=…`（末尾の `bus:` のカウンタも）
2. `ros2_control_node` が `process has died … exit code -11` になった場合は、**上の 2 行が、その前にあるか**を確認し、ログの全文を報告する（落ちた位置のスタックも）。
3. 停止後に**読み取り専用で起動**して、SRAM が `torque_enable=0 lock=1` であること（`[diag]`）を確認する（W0 と同じ。トルクが OFF になっている最終確認）。

**避け方（運用）**: 書き込みモードを止める前に、(1) Twist の送信を止める（S1）、(2) `pause_servo`（S2）、(3) 止まっていることを確認して、forward controller を外して `arm_controller` に戻す（S3）、(4) 腕を折りたたみ姿勢で支える、(5) **launch の端末で Ctrl-C を 1 回**（S5）。Servo を先に止め、forward controller を外した状態で止めると、終了時に `ros2_control_node` が閉じるコントローラ・トピックが減る（**効果は未確認。5 回に 1 回の頻度の原因は特定できていない**）。**ログの 2 行の確認は、この運用をしても必ず行う。**

---

## 0. 準備（共通）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
mkdir -p ~/so_arm101_check
# ビルド済みなら不要（実行すると install/ が作り直され、端末を開き直す必要がある）。ソースを更新したときだけ:
# cd ~/ros2_so-arm101 && colcon build --symlink-install
ls -l /dev/ttyACM0                                          # ポートがあること
git log --oneline | head -3                                 # Servo の統合のコミット（5b35507 以降）があること
```

- `colcon test` が全件通っていること（Claude が確認済み。全体 265 件・失敗 0。bringup 85）。
- **ほかに実機を使う launch が動いていない**こと（`ps aux | grep -E "ros2_control_node|move_group" | grep -v -E "grep|gvfsd"` が空）。
- 校正ファイルが今の状態であること（`ros2_so-arm101/src/so_arm101_hardware/config/follower_calibration.yaml`。EEPROM との一致は R0 で見る）。
- **記録するもの（W 段階の前に控える）**: 起動直後の実測（`ros2 topic echo /joint_states --once`）＝**出発点**。W6 で戻す先。

---

## R0a. 読み取り専用で起動する（`use_servo:=false`。今までと同じ）

目的: 今までと同じ起動が、L1 の param を含む `robot_description` で問題なく動くこと。**腕は触らない。トルク OFF のまま。**

```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false use_rviz:=false 2>&1 | tee -i ~/so_arm101_check/servo_R0a.log; echo "exit=${PIPESTATUS[0]}"
```

10 秒ほど待って（`ros2 control list_controllers` で 3 つ `active` を確認）、**Ctrl-C を 1 回**。

確認すること（ログ）:
- `initialized: port=/dev/ttyACM0 …` があり、起動が止まらない（`robot_description` に `max_joint_speed`・`limit_window_s` の `<param>` が入っていても、ドライバが受け付ける）。
- `[diag] EEPROM vs yaml: 6/6 joints match`、`[diag] servo settings vs reference: 6/6 joints match`（**`MISMATCH`・`DIFFERS` が無い**）。
- `READ-ONLY mode` の WARN があり、`write mode active` が無い。
- 停止時: `deactivated (no register was written)`、`reads=… retried=0 failed_cycles=0`、`bus:` のカウンタがすべて 0（`timeouts=0 bad_checksum=0` まで）。
- 終了コード `exit=0`。気になるログ（`FAULT`・`COMMUNICATION LOST`・`read failed`）が 0 行。
- 止めた後、`ps`（上の 8）が空。

合格: すべて満たす。**`MISMATCH`・`DIFFERS`・`FAULT`、`exit` が 0 でない、のどれかなら中止して報告。**

## R0b. 読み取り専用で起動する（`use_servo:=true`）

目的: `use_servo:=true` で、`servo_node` が起動し、forward controller が `inactive` で読み込まれ、Servo は**起動しても何も出さない**こと。L1 の param が `robot_description` に入っていること。**腕は触らない。**

```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false use_servo:=true use_rviz:=false 2>&1 | tee -i ~/so_arm101_check/servo_R0b.log; echo "exit=${PIPESTATUS[0]}"
```

操作用の端末で（起動して 15 秒後）:

```bash
ros2 control list_controllers                     # arm・gripper・joint_state_broadcaster が active、forward_position_controller が inactive
ros2 node list | grep -E "servo_node|controller_manager"
ros2 param get /robot_state_publisher robot_description | grep -o '<param name="\(max_joint_speed\|limit_window_s\)">[0-9.]*</param>'
ros2 param get /servo_node moveit_servo.command_out_topic        # /forward_position_controller/commands
ros2 param get /servo_node robot_description_planning.joint_limits.elbow_flex.max_velocity   # 0.3
timeout 3 ros2 topic echo /servo_node/status --once              # 何も出ない（start_servo を呼ぶまで Servo は出さない）。タイムアウトで終わる
```

確認すること:
- コントローラ: 3 つ `active`、`forward_position_controller` が `inactive`。
- `robot_description` に `<param name="max_joint_speed">0.75</param>` と `<param name="limit_window_s">0.08</param>` がある（**L1 の値が渡されている。読み取り専用の起動ログには、この値は出ない**。効いていることの確認は W0）。
- ログに Servo の WARN（内容を報告。**既知のもの**: `Intra-process communication is disabled…`、`No kinematics plugins defined…`、`No kinematics solver instantiated for group 'arm'. Will use inverse Jacobian…`）。それ以外の WARN・ERROR は報告。`[diag]` は 6/6 ×2（R0a と同じ）。
- Ctrl-C のあと: `deactivated (no register was written)`、`reads=… retried=0 failed_cycles=0`、`bus:` がすべて 0、`exit=0`。**`ros2_control_node` が exit −11 になったら、ログの全文（落ちた位置のスタック）と、`deactivated` の行の有無を報告**（腕には書いていないので安全。頻度の記録が目的）。

**この R0b の起動と停止を、あと 2 回繰り返す**（ログは `servo_R0b_2.log`、`servo_R0b_3.log`）。**3 回のうち `ros2_control_node` の −11 が何回出たか**を報告する。

合格: 3 回とも起動し、上の確認がすべて満たされる。

## R1. Servo の開始と、出発点での動き（読み取り専用）

目的: `start_servo`・drift の設定が通ること。**出発点では Servo は動き出せない**（自己衝突までの距離が約 3.9 mm で、Servo の速度倍率が約 0.015）ことを、`collision_velocity_scale` で確認する。**腕は動かない（トルク OFF）。**

起動は R0b と同じ（`servo_R1.log`）。操作用の端末で:

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger            # success: true（これは信用しない。次の status を見る）
timeout 5 ros2 topic echo /servo_node/status --once                        # data: 0
timeout 5 ros2 topic echo /servo_node/collision_velocity_scale --once      # data: 0.0 台（出発点の姿勢のとき。0.015 前後の見込み）
```

確認すること: `status` が来る。**`status` は、動かす指令を処理している周期だけ値が付き、指令が無い（止まっている）周期は 0**（`servo_calcs.cpp:322-326`、計画 9.1）。この R1 では Twist を送らないので、0 でよい。減速・停止の値（1〜6）は、**送信中**に見る（R3）。**出発点（elbow ≈ +1.69）では `collision_velocity_scale` が 1 より十分小さい**（見込み 0.015。0.1 未満）。値を報告する。1.0 だった場合は、姿勢が出発点と違う（elbow が開いている）ので、実測の姿勢と合わせて報告。
`pause_servo` を呼ぶ（S2）。**`/servo_node/status` は pause の後も約 29.4 Hz で出続ける**（止まらない）ので、pause の効き目は `status` では見ない。**効き目は、Twist を送っている間に `/forward_position_controller/commands` が止まること**で確認する（R3。Twist を送っていないときは、pause の前でも commands は出ない）。Ctrl-C で止める。停止時の確認は R0b と同じ。

合格: `start_servo` 後に `status` が来る、出発点で `collision_velocity_scale` が小さい。

## R2. 手で中間の姿勢に置いて、Servo の指令の計算を確認する（読み取り専用）

目的: 実機の `/joint_states` を入力に、Servo の出力が**期待どおり**であること（向き・大きさ）。**サーボのトルクは OFF なので、腕は手で動く**。出力は forward controller に渡るが、ドライバは**読み取り専用で破棄**する（`READ-ONLY` の WARN は、指令と測定の差が 0.02 rad を超えたときだけ。Servo の 1 周期の差は約 0.001 rad なので、出ない見込み）。

準備:
1. R1 と同じ起動（`servo_R2.log`）。
2. **腕を手で、中間の姿勢 T に置く**（pan 0°、lift −22°、elbow +30°、wrist_flex +83°、roll 0°。先端 z ≈ 0.055 m（実測）。接近軸はほぼ真下）。**きっちり合わせなくてよい**（±3° で可）。実測を控える: `ros2 topic echo /joint_states --once --field position`。
3. 切り替え（読み取り専用でも行う。手順は W2 と同じ）: コマンド A（止まっていること。**手を放して 1 秒静止**）→ `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger` → `ros2 service call /servo_node/pause_servo std_srvs/srv/Trigger` → `ros2 control switch_controllers --strict --deactivate arm_controller --activate forward_position_controller` → `ros2 control list_controllers`（forward が `active`、arm が `inactive`）。

送信（腕は**手で静止させておく**。トルク OFF なので、手を放すと重力で動く。姿勢を保つため、**手で支えたまま**送る）:

```bash
python3 ~/ros2_so-arm101/tools/send_servo_twist.py --linear 0 0 -0.5 --seconds 2 --dry-run   # 計画だけ（何も送らない）
python3 ~/ros2_so-arm101/tools/send_servo_twist.py --linear 0 0 -0.5 --seconds 2             # 目標が表示される。Enter で送る
```

確認すること:
- ツールの表示: `drift: rx`（pan ≈ 0 のとき）、`5x5 condition number` が 40 未満（見込み 20 台）、`dropped by the projection` が 0 % 付近。
- 送信中に別の端末で `ros2 topic echo /forward_position_controller/commands`: **5 要素。各要素が `/joint_states` の実測との差 0.002 rad 以内**（1 周期の差。向きは、z を下げる Twist で lift・elbow・wrist_flex が動く向き）。**順序は pan、lift、elbow、wrist_flex、roll**。
- 送信中の `/servo_node/status`（`ros2 topic echo /servo_node/status`）が 0、`/servo_node/collision_velocity_scale`: 1.0（姿勢 T は自己衝突まで約 97 mm）。
- **腕が（手で止めている以外に）勝手に動かない**。`READ-ONLY: command … is discarded` の WARN が出ていれば、内容を報告。
- 先端が実際には動かないので、ツールの最後の表示（`tip moved`）は 0 付近でよい。

（任意）**Servo の速度上限（0.3 rad/s）の強制の確認**: 通常の入力（unitless 1.0 × scale 0.05 rad/s）では上限に届かない。`ros2 param set /servo_node moveit_servo.scale.joint 2.0` が**動的に反映されるかは未確認**。反映されるなら、`JointJog`（1 関節、速度 1.0、1 秒）を送り、`/forward_position_controller/commands` の 1 周期の差が **0.3 rad/s × 0.034 s ≈ 0.0102 rad 以下**であることを確認し（上限なしなら 2.0 × 0.034 = 0.068 rad）、**終わったら必ず `scale.joint` を 0.05 に戻す**。反映されない、または不安なら、**この項目は飛ばして報告**（モックでも未確認の項目として残す）。

終了: `pause_servo`（S2）→ コマンド A → 手動で JTC に戻す（S3）→ Ctrl-C（S5）。

合格: 出力の向き・大きさが期待どおり、`status` 0、勝手に動かない。

## R3. 停止の確認（読み取り専用）

R2 の状態（forward が active、Servo が動作中）で、**送信を何度かやり直して**、次のそれぞれで出力が止まることを確認する（`ros2 topic hz /forward_position_controller/commands` で、止まったら出力が無くなる）:
- S1: 送信中にツールの Ctrl-C → 0.1 秒後に出力が止まる。
- S2: **送信中に**別の端末で `pause_servo` → `/forward_position_controller/commands` が止まる（ツールは送り続けるが、Servo は出さない）。`/servo_node/status` は止まらない（約 29.4 Hz で出続ける）。`unpause_servo` で再開。**注意: `send_servo_twist.py` は送る前に必ず `unpause_servo` を呼ぶ**（pause は止めておく鍵にならない）。Twist を送っていないときは、pause の前でも commands は出ない。
- S4: `ps aux | grep servo_node_main | grep -v grep` の PID に `kill -INT` → 出力が止まる。**launch が動き続けるか（他のノードが生きているか）を `ros2 node list` と `list_controllers` で確認して報告**。確認後、launch の端末で Ctrl-C。

合格: それぞれで出力が止まる。

---

## W0. 書き込みモードで起動し、L1 の値のログを確認する。保持のみ（10 秒）

目的: L1（ドライバの 1 周期の上限 = 0.75 rad/s × 0.08 s = 0.06 rad、39 tick）が**効いている**こと。トルクが入っても腕が動かないこと。**ここで初めて腕にトルクが入る。**

- **腕の支え方**: 腕を折りたたみ姿勢（出発点）にして、**片手で前腕、もう片手で上腕を軽く支える**。トルクが入る瞬間に、腕がわずかに引き締まる感触はあってよい。大きく動くなら止める。
- **何もしない**: RViz は使わない。action も topic も送らない。**Servo は起動するが、`start_servo` は呼ばない**。

```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false use_servo:=true use_rviz:=false \
  probe_lock_sram:=true 2>&1 | tee -i ~/so_arm101_check/servo_W0.log; echo "exit=${PIPESTATUS[0]}"
```

操作用の端末で（起動して 15 秒後。ログの確認）:

```bash
grep -E "WRITE mode:|write mode active:" ~/so_arm101_check/servo_W0.log
```

確認すること:
- **`WRITE mode: …` の WARN に `max_joint_speed=0.75 rad/s x window=0.080 s`**。
- **`write mode active: … Per-cycle step limit: 39 ticks (angle joints). startup_read_retries=0`**（INFO）。
- `[probe] … ACCEPTED on all joints`（NOT accepted でも起動は続く。報告）。
- 10 秒待つ。腕が動いていないこと。`FAULT` が出ていないこと。`ros2 control list_controllers` で arm が `active`、forward が `inactive`。
- **止め方**: 腕を支えたまま **Ctrl-C を 1 回**（S5）。**「停止したあとの確認」の 2 行**（`[deactivate] torque OFF and Lock=1 confirmed` と `deactivated. …`）を確認。`rate_limited=0`、`port_contention=0`、`monitors` は約 1 秒に 1 回（W0 の実測は 59 秒で 58 回。W6 は約 87 分で 5153 回。起動時間に比例する）。
- 続けて、**読み取り専用で起動**して（R0a のコマンド。`servo_W0_after.log`）、`[diag]` の SRAM が `torque_enable=0 lock=1`、`acceleration=50`、`6/6 joints match`。

合格: L1 の 2 行が上の値で出る／腕が動かない／`rate_limited=0`・`port_contention=0`／停止後に `torque_enable=0 lock=1`。**L1 の 2 行が出ない（または値が違う）なら、先へ進まず報告。**

## W1. MoveIt（OMPL、Scaling 0.1）で出発点から中間の姿勢 T へ

目的: Servo を使える姿勢（自己衝突まで 10 mm 以上、床から十分高い）へ、JTC（MoveIt）で出る。**出発点では Servo が動かない（R1）ので、必ずここを通る。**

- **腕の支え方**: 折りたたみ姿勢から開くので、**前腕の下を支える**。開く向き（自重に逆らう向き）は、目標の約 0.04 rad 手前で止まる（既知）。
- 起動（RViz あり。`feetech_write_check.md` の W3.5・W4 と同じ `torque_limit:=500`）:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false use_servo:=true \
    torque_limit:=500 2>&1 | tee -i ~/so_arm101_check/servo_W1.log; echo "exit=${PIPESTATUS[0]}"
  ```
  L1 の 2 行（W0 と同じ）が出ていること。`ros2 topic echo /joint_states --once` で**出発点を控える**（W6 で戻す先）。
- RViz の MotionPlanning: **Planner は OMPL（Pilz は使わない。出発点の elbow が上限を超えるため）**。**Velocity Scaling と Acceleration Scaling は 0.1 のまま**（0.125 を超えると、ドライバの上限で削られる）。Planning タブで **Goal State を `<current>` に選び直し**、Joints タブで次の**整数の度**に設定する（スライダーは整数の度）:
  **shoulder_pan 0、shoulder_lift −22、elbow_flex +30、wrist_flex +83、wrist_roll 0**（gripper は触らない）。これが姿勢 T（先端は計画値 (0.177, 0, 0.059) m、**実測は (0.172, 0.000, 0.055) m**、接近軸はほぼ真下、自己衝突まで約 97 mm、Servo の 5×5 の条件数 約 23（実測 22.3））。
- **Plan してから、Execute の前に確認する**: アニメーションで**動かしていない関節（gripper を含む）が動かない**こと、動きが滑らかで、**先端が下がらない**こと（出発点の先端 z は実測 0.033 m（計画値は 0.022 m）。**Show Trail で先端の軌跡が出発点より下に行かない**こと。下がるなら中止）。
- Execute。**1 回ごとに動きが止まってから次へ。** 見ること: 滑らかに動く／終点で止まる／**`step limit` の WARN が arm 関節で出ない**（Scaling 0.1 では削られない設計。出たら報告）／`FAULT` なし。
- 終点で、実測を読む: `ros2 topic echo /joint_states --once --field position`。**T から約 0.05 rad 以内**（開く向きの手前止まり 約 0.04 rad を含む）。先端の位置: `ros2 run tf2_ros tf2_echo base_link gripper_frame_link`（数秒見て Ctrl-C）。**先端 z が 0.04 m 以上**であること（床から十分高い）。

合格: 計画どおりに動く／`step limit` が arm で出ない／先端 z ≥ 0.04 m／`FAULT` なし。**中間の姿勢に出られなければ、ここで中止し、止め方（S5 の手順。出発点へ戻す）に進む。**

## W2. 止まっていることを確かめて、Servo（forward controller）に切り替える

目的: 腕が止まっているときだけ、1 回の `switch_controllers` で `arm_controller` を `forward_position_controller` に入れ替える。切り替えで腕が飛ばないこと。

操作用の端末で（W1 の起動のまま）:

1. **W2a. 止まっていることの確認**: **コマンド A**。差が 0.003 rad 以内、MoveIt の実行が終わっている。**MOVING なら、切り替えない。**
2. **W2b. Servo の準備**: `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger`（初回だけ）→ `ros2 service call /servo_node/pause_servo std_srvs/srv/Trigger`（**切り替えの間は一時停止**）。
3. **W2c. 切り替え**（1 回。strict）:
   ```bash
   ros2 control switch_controllers --strict --deactivate arm_controller --activate forward_position_controller
   ```
4. **W2d. 確認**: `ros2 control list_controllers`（**forward が `active`、arm が `inactive`**。**両方 `inactive` なら、`ros2 control switch_controllers --strict --activate arm_controller` で戻して中止**）。**コマンド A**（切り替えの前後で位置の変化が 0.003 rad 以内。**飛ばない**）。
5. **W2e. 再開**: `ros2 service call /servo_node/unpause_servo std_srvs/srv/Trigger`。`ros2 topic echo /servo_node/status --once` が 0。`ros2 topic echo /servo_node/collision_velocity_scale --once` が 1.0（T は 97 mm）。

合格: forward が active・arm が inactive、切り替えの前後で腕が動かない（0.003 rad 以内）、`status` 0・`collision_velocity_scale` 1.0。**腕が動いたら、S3（戻す）→ 報告。**

## W3. Python から単発の小さい補正

目的: Servo で、先端が指令の向きに動くこと。**実速度の比**（期待に対する実測。モックでは約 0.47）と、**動き出す最小の速度**、**方向ごとの止まる位置**を測る（計画 11.1 の Q1〜Q6。**記録だけで、合否にしない**）。

- **床に近づかない**: 姿勢 T の先端 z は 実測 0.055 m。**下げる向き（−z）は 1 回 5 mm 以内、T からの正味の下がりが 15 mm 以内**。**上げる向き（+z）・前後（x）・横（y）は 1 回 10 mm 以内**。1 回ごとに、先端 z を `tf2_echo` で確認し、**0.045 m を下回りそうなら −z は送らない**。
- ツールの上限（超える指定は拒否される）: 成分 0.5（`--allow-fast` で 1.0）、時間 3 秒、先端の総移動量 2 cm、frame は `base_link` か `gripper_frame_link`。**作れない向きの回転（rx。pan ≈ 0 のとき）は、ツールが拒否する**。
- 各回の手順（**毎回**）: コマンド A（止まっている）→ `--dry-run` → 本番（Enter）→ 終了後に先端の位置（`tf2_echo`）と関節の実測を記録。

**W3a. 最初の 1 回は、床から離れる向き（+z）に 5 mm**（実機で向きを確かめる）:
```bash
python3 ~/ros2_so-arm101/tools/send_servo_twist.py --linear 0 0 0.5 --seconds 1 --dry-run
python3 ~/ros2_so-arm101/tools/send_servo_twist.py --linear 0 0 0.5 --seconds 1
```
期待（`scale` × 時間）: +5 mm。見ること: **先端が上に動く**（`tf2_echo base_link gripper_frame_link` の z が増える。動かなくてもよい。Q1 の不感帯の可能性。記録）。**向きが逆（下に動く）なら、直ちに中止して報告**。**腕が急に動く、異音、`FAULT` なら S1→S2→S3**。**上に動くことを確かめてから、ほかの向きに進む。**

**W3b. 下げる向き（床に近づく。1 回 3 mm、T からの正味の下がりが 15 mm 以内）**: `--linear 0 0 -0.3 --seconds 1`（−3 mm）。先端 z が 0.045 m を下回りそうなら送らない。**W3c. 前後・横**: `--linear 0.5 0 0 --seconds 2`（+10 mm）、`--linear 0 0.5 0 --seconds 2`（+10 mm。pan と roll が動く）。**W3d. ピッチ**: `--angular 0 0.5 0 --seconds 2`。

見ること（各回）: **送信中の `/servo_node/status`**（0 以外なら、減速・停止の理由。値は 1 近づく特異点で減速、2 特異点で停止、3 衝突で減速、4 衝突で停止、5 関節の端、6 離れる特異点で減速。**ツールの最後の行に、送信中の値と回数が出る**。status は動かす指令を処理している周期だけ値が付く）／先端が**指令の向き**に動く／**ほかの軸のずれ**が小さい（モックでは 0.01 mm 以内。実機では記録）／向き（姿勢）が大きく変わらない／`status` が 0／`READ-ONLY` の WARN は出ない（書き込みモード）／**`step limit` の WARN が arm で出ない**／`FAULT` なし。

**Q1〜Q6（記録。計画 11.1）**: 次を、W3 の送信で測る。各回の「期待」はツールが表示する。

| 測る項目 | 測り方 | 記録 |
|---|---|---|
| **Q1 動き出す最小の速度** | z の 1 方向に、入力を 0.1 → 0.2 → 0.3 → 0.5（各 2 秒。同じ入力を 3 回） | 各回の関節の変化と、期待との比。**比が 0.5 を超える最小の入力**（0.5 tick 未満の指令は動かない見込み＝約 0.023 rad/s） |
| **Q2 方向ごとの手前止まり** | 開く向き・閉じる向きのそれぞれで、期待 5 mm → 10 mm → 20 mm（入力と時間で調整。**−z は T からの正味の下がりが 15 mm まで**） | 止まった後の先端の変位と、期待との差 [mm]（開く向きで約 0.04 rad ≒ 先端で 4〜10 mm の手前止まり、という推測と比べる） |
| **Q3 小さな補正の繰り返し** | 同じ方向に 2 mm を 5 回（毎回止まるのを待つ） | 1 回ごとの変位と累積。**積み上がるか、動かないままか** |
| **Q4 方向の反転** | +5 mm → −5 mm → +5 mm（z。−z は 5 mm 以内に収める） | 各回の変位。反転で失われる量 |
| **Q5 戻りの再現性** | 出発の位置 → +10 mm → −10 mm → 出発の位置からの差。3 回 | 往復の再現性 [mm] |
| **Q6 静止後の揺れ** | 補正後 5 秒間の `/joint_states` と先端の位置 | 揺れ [tick、mm] |

実速度の比（期待に対する実測）も記録する（**モックでは約 0.47**。Servo の「測定位置 + delta」と Butterworth 平滑化による）。

合格: W3a〜W3d で、腕が暴走せず、先端が指令の向きに動く（動かない回があれば、Q1 の記録として報告。不合格にしない）。**記録は、判定の基準にしない（初回）。**

## W4. 停止の試験（実機）

目的: S1・S2・S4 のそれぞれで腕が止まること。**止まる距離を記録**する。**ドライバ・forward controller は止めない。**

それぞれ、`--linear 0.5 0 0 --seconds 3`（+x。最大 3 秒）を送り、**送信の途中で**止める。止めた直後の関節の実測と、1 秒後の実測の差（**止まる距離**）を記録する:
- **W4a. S1**: ツールの端末で Ctrl-C。
- **W4b. S2**: 別の端末で `pause_servo`（その後 `unpause_servo`）。
- **W4c. S4**: `ps aux | grep servo_node_main | grep -v grep` の PID に `kill -INT`（**自分の launch の子**）。**この後 Servo は無くなる**（W5 以降は、必要なら launch を止めて再起動）。**launch が動き続けるか**を `list_controllers` で確認して報告。W4c は**最後**に行う。

合格: それぞれで 0.1 秒以内に出力が止まり、腕が止まる（止まる距離が数 tick 以内。数値を報告）。

## W5. Servo を止めて JTC に戻す

1. `ros2 service call /servo_node/pause_servo std_srvs/srv/Trigger`（S2。W4c で Servo を止めた場合は不要）
2. **コマンド A**（止まっている）。
3. S3: `ros2 control switch_controllers --strict --deactivate forward_position_controller --activate arm_controller`。
4. `ros2 control list_controllers`（**arm が `active`、forward が `inactive`**。両方 inactive なら arm を activate）。**コマンド A**（切り替えの前後で 0.003 rad 以内。飛ばない）。
5. **W5b. gripper の開閉（L1 の確認）**: ドライバの上限（0.06 rad/周期）で、gripper の開閉が完了すること。
   ```bash
   time python3 ~/ros2_so-arm101/tools/send_joint_delta.py gripper 0.5 --seconds 5 --yes
   time python3 ~/ros2_so-arm101/tools/send_joint_delta.py gripper -0.5 --seconds 5 --yes
   ```
   見ること: action の `result status: 4`（SUCCEEDED）、`gripper` の `step limit … ramping toward it` の WARN は出てよい（gripper の目標は階段状。**arm 関節で出てはいけない**）。**開閉にかかった時間**（全開閉 約 0.6 秒の見込み）を報告。**完了しない、または中断するなら報告**。
6. **L1 の効き（MoveIt の実行）**: W6 の MoveIt の実行で `step limit` の WARN が arm で出ないこと（Scaling 0.1）。

合格: arm が active・forward が inactive、切り替えで腕が飛ばない、gripper の開閉が完了する。

## W6. MoveIt で出発点へ戻す

- **戻し先は W1 で控えた出発点の実測**。`feetech_write_check.md` の **W4 の「止め方」の手順 1〜6** に従う（動きが止まっているのを確認 → 実測を読む → 動かした関節ごとに「出発点 − 実測」を計算 → OMPL（Scaling 0.1）で戻すか、`send_joint_delta.py` で 1 関節ずつ。**elbow_flex は +1.69009 を超える値には戻さない**）。**MoveIt の実行の開始位置と実測の差が 0.01 rad（`allowed_start_tolerance`）以内**であること（超えると実行が拒否される。拒否されたら、実測を読み直して、現在位置から計画し直す）。
- **前腕を支える**（戻せないときも手で支える）→ **launch の端末で Ctrl-C を 1 回**（S5）→ **「停止したあとの確認」の 2 行**、`ps`、`ros2 node list`（空でなくても `ps` が空なら daemon の古い一覧。既知）。
- 続けて**読み取り専用で起動**して `torque_enable=0 lock=1` を確認（W0 と同じ）。

合格: 出発点から約 0.02 rad 以内／`step limit`（arm）が無い／停止後に `torque_enable=0 lock=1`。

---

## 合格基準（全体）

- R0a・R0b: 起動できる、`[diag]` 6/6 ×2、停止時の `deactivated` の行と `bus:` が 0、`exit=0`。L1 の param が `robot_description` にある。
- R1〜R3: Servo の出力が期待どおり（向き・大きさ・順序）。出発点では動かない（倍率が小さい）。S1・S2・S4 で止まる。
- W0: L1 の 2 行（`max_joint_speed=0.75 … window=0.080`、`Per-cycle step limit: 39 ticks`）。
- W1〜W2: 中間の姿勢 T に MoveIt（Scaling 0.1）で出られる。切り替えで飛ばない。
- W3〜W4: 先端が指令の向きに動く。止め方が効く。Q1〜Q6 の記録。
- W5〜W6: JTC に戻せる。gripper の開閉が完了。出発点に戻せる。**毎回、停止後に `[deactivate]` と `deactivated.` の行がある。**

## 報告してほしいもの（段階ごと）

- ログのパス（`~/so_arm101_check/servo_<段階>.log`）と、`exit=`。
- 各ステップの合否と、数値（先端の変位 [mm]、関節の変位 [rad]、止まる距離、`collision_velocity_scale`、`status`）。
- **R0b の 3 回の起動で、`ros2_control_node` が exit −11 になった回数**。−11 が出たときは、ログの全文と、`deactivated` の行の有無。
- **W0 の L1 の 2 行**の全文。
- Servo の新しい WARN・ERROR の内容。
- Q1〜Q6 の記録（表）と、実速度の比。
- gripper の開閉時間。
- 腕の挙動の気づき（異音、引っかかり、揺れ）。

## ログの読み方

| ログ | 意味 | 対応 |
|---|---|---|
| `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints` | 終了でトルク OFF・Lock=1 を確認 | **毎回、停止後に必須**。無ければ S6 |
| `[deactivate] … could NOT confirm …` | 終了でトルク OFF／Lock=1 を確認できなかった | 腕を支えて 12 V を切る（S6） |
| `ros2_control_node … process has died … exit code -11`（`use_servo:=true`） | 終了時の異常。モックでは、ハードウェアの `deactivate`・`shutdown` の**後**に落ちた（5 回に 1 回） | **直前に `[deactivate]` と `deactivated.` の行があるか確認**して、ログの全文を報告 |
| `move_group`・`rviz2` が exit code -11 | 終了時の既知の挙動 | 無視してよい（`deactivated.` の後） |
| `FAULT: …` / `COMMUNICATION LOST` | feetech_write_check.md の「ログの読み方」のとおり | 中止。支えて Ctrl-C。12 V を切る条件に注意 |
| `step limit: 'X' requested +N ticks … limited`（arm 関節） | 1 周期の上限（0.06 rad = 39 tick）を超える指令を削った。MoveIt の Scaling が高い、または Servo・変換の不具合 | **arm で出たら報告**。gripper の `ramping` は正常 |
| `Servo status` 1・2・3・4・5・6 | 1 特異点の減速、2 特異点で停止、3 衝突の減速、4 衝突で停止、5 関節の端、6 特異点から離れる向きの減速 | 値と姿勢を報告。出発点付近では 3・4 が出うる |
| 送信ツール `refused: …` | 上限を超える指定、または作れない向き（射影で 50 % 以上を捨てる）、controller の状態が違う | 指定を直す。controller の状態を `list_controllers` で確認 |
| `change_drift_dimensions failed` | drift が設定できなかった。ツールは Twist を送らない | Servo が起動しているか確認 |

---

## 後の段階（目次のみ。この手順書では実機で行わない）

- **M. モード管理ノード（案 M2）**: ユーザーの共通の層とつなぐときに作る（計画 9.2）。上の手動の手順（止まっていることの確認 → `pause_servo` → 切り替え → 確認 → `unpause_servo`）をそのまま自動化する。
- **V. VR（解析 IK の連続出力。Servo は使わない）**: V0 streaming への切り替えと保持（必要なら制限つき forward controller を作る）／V1 小さな振幅の手の動き（デッドマン、入力のタイムアウト、目標の飛びの検知）／V2 下の層の上限（L1 の 0.06 rad、または制限つき forward controller の `max_step`）が効くことの確認（モックで）／V3 停止の確認。
