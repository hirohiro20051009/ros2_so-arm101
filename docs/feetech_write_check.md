# 実機確認手順: 書き込みモード（トルク ON、Goal_Position）（段階2b-3c）

対象: `hardware:=feetech read_only:=false`。**腕が動く。サーボにトルクが入る。** 実行するのはユーザーだけ（Claude は実機を動かさない）。
実装の仕様は `docs/inventory.md` の「段階2b-3 の計画 v3」と「段階2b-3b の計画 v4」。

段階は W0 → W1 → W2 → W3 → W4（W5 は任意）。**前の段階が合格するまで次へ進まない。** 各段階に「腕の支え方」と「止め方」がある。必ず読んでから始める。

## 共通の約束（全段階）

1. **止める前に、腕を折りたたみ姿勢にして、手で支える。** Ctrl-C でドライバがトルクを OFF にする（腕は脱力して落ちる）。
2. 止めるのは **Ctrl-C だけ**。`kill -9`、端末を閉じる、PC の電源断はしない（トルクが ON のまま残る）。
3. 止めた後、ログに次の行があることを確認する。無ければサーボの電源を切る（腕が落ちるので支える）。
   `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`
4. **非常停止はサーボの外部電源を切る**（USB だけではない）。切ると腕が落ちる。常に片手を電源の近くに置き、もう片手で腕を支える。
5. ログの `FAULT:` または `COMMUNICATION LOST` が出たら、その段階を中止して Ctrl-C（支えながら）。原因を報告してから再開する。
6. LeRobot 経路と同時に起動しない。conda を activate していない新しい端末を使う。
7. 腕の周りに物を置かない。ケーブルが引っかからないようにする。

## 0. 準備（共通）
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
mkdir -p ~/so_arm101_check
cd ~/ros2_so-arm101 && colcon build --symlink-install     # 最新のビルドであること
ls -l /dev/ttyACM0                                          # ポートがあること
```
基本の起動（段階ごとに引数だけ変える）:
```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech read_only:=false use_rviz:=false \
  2>&1 | tee -i ~/so_arm101_check/write_<段階>.log
```
起動で出るべきログ: `WRITE mode: this driver will turn torque ON ...`（WARN）、`write mode active: torque ON, Lock=1, Acceleration=50, ...`。
**起動の途中で ERROR（`activation failed`、`Acceleration reads back as 0`、`Goal_Velocity=...` など）が出た場合、トルクは ON になっていない**（ドライバが Lock=1・トルク OFF に戻したことをログで確認する）。

---

## W0. 保持のみ（10 秒）。指令は一切送らない

目的: トルクが入っても腕が動かないこと（Goal_Position=現在位置）、Acceleration が毎回書かれること、Lock=1 のまま SRAM（Acceleration）へ書けるか（確認 1）。

- **腕の支え方**: 腕を折りたたみ姿勢にして、**片手で前腕、もう片手で上腕を軽く支える**。トルクが入る瞬間に、腕がわずかに引き締まる感触はあってよい。大きく動くなら止める。
- **何もしない**: RViz は使わない（`use_rviz:=false`）。action も topic も送らない。
- 起動（`acceleration:=50 torque_limit:=1000` は既定値。確認 1 を有効にする）:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech read_only:=false use_rviz:=false \
    probe_lock_sram:=true 2>&1 | tee -i ~/so_arm101_check/write_W0.log
  ```
- 見るログ: `write mode active`、`[probe] with Lock=1 and torque ON, a write to SRAM (Acceleration) was ACCEPTED on all joints`（NOT accepted の WARN でも起動は続く。報告する）。
- 10 秒待つ。腕が動いていないこと。`FAULT` が出ていないこと。
- **止め方**: 腕を支えたまま **Ctrl-C を 1 回**。腕が脱力する。`[deactivate] torque OFF and Lock=1 confirmed` を確認。ログ末尾の `deactivated. reads=… goal_writes=… monitors=… rate_limited=… port_contention=…` も見る。
- 続けて、読み取り専用で状態を確認する（数秒後に Ctrl-C。トルクは OFF のまま）:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech use_rviz:=false \
    2>&1 | tee -i ~/so_arm101_check/write_W0_after.log
  ```
  `[diag]` の SRAM で **`torque_enable=0 lock=1`**（LeRobot では lock=0 のまま残っていた）、`acceleration=50` であること。`6/6 joints match`。

合格: 起動でトルクが ON になっても腕が動かない / `rate_limited=0` / `port_contention=0` / `monitors` が約 10（10 秒ぶん）/ 停止後に `torque_enable=0 lock=1`。
- 起動の読み戻しの再試行: `write mode active` の行と、停止時の `deactivated. ...` の行の末尾に `startup_read_retries=N` が出る。**0 が望ましい**。N>0 のときは、直前の WARN `read of ... failed (...: ... [tx=… rx=… flushed_before_request=…]); retrying` の行を**全部**報告する（原因の切り分けに使う）。再試行は通信エラー（checksum mismatch・フレーム不良・タイムアウト）のときだけ、読み戻し 1 件につき最大 3 回。読み戻した値が期待と違うときは再試行せずに起動を中止する。3 回失敗したらトルクは入らない。

## W1. wrist_roll と gripper（軽い関節）

目的: 実際の指令でサーボが動くこと。`torque_limit:=500`（pan と roll だけに効く。lift / elbow / wrist_flex は常に 1000、gripper は常に 500 以下）。

- **腕の支え方**: 折りたたみ姿勢。**片手で前腕（wrist より手前）を持って重さを受ける**。動かす wrist_roll と gripper には触れない（指を挟まない）。
- 起動:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech read_only:=false use_rviz:=false \
    torque_limit:=500 2>&1 | tee -i ~/so_arm101_check/write_W1.log
  ```
- 別の端末（同じ `source` をして）で、少しずつ動かす。**1 回ごとに、動きが止まってから次へ**:
  ```bash
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py wrist_roll 0.2 --seconds 4     # 目標を表示し、Enter で送る
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py wrist_roll -0.4 --seconds 6
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py wrist_roll 0.2 --seconds 4     # 元へ
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py gripper 0.3 --seconds 3
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py gripper -0.3 --seconds 3
  ```
  （このツールは |delta|≤0.5 rad、速度≤0.1 rad/s を超える指定を拒否する。gripper は GripperActionController が目標へ一気に飛ぶ指令（階段状）を出す。ただし gripper の 1 周期の上限は約 408 tick（≈ 0.48 rad。校正範囲 1485 tick が 100 ≙ 1.745 rad なので 1 tick ≈ 0.001175 rad）で、0.3 rad は約 255 tick なので**上限に掛からず、`step limit: 'gripper' ... ramping` の WARN は出ない**（W1 の実測: 出なかった）。出るのは約 0.48 rad より大きな一歩のときだけ。出ても正常で、`rate_limited` に数える）
- 見ること: 指令どおりの向きに動く／止まる位置が指令の位置と合う（目視と `ros2 topic echo /joint_states --once`）／動作中に異音・振動が無い／`FAULT` が無い。
- **止め方**: 動きが止まっていることを確認 → 折りたたみ姿勢 → 前腕を支える → **Ctrl-C**。
- 合格: 上記 / ログ末尾の `rate_limited=0`（0.3 rad の gripper は上限に掛からないので全体で 0 になる。`step limit` の WARN が出たら、関節名と大きさを報告）/ `port_contention=0`。

## W2. shoulder_pan

- **腕の支え方**: 折りたたみ姿勢。**上腕の付け根と前腕を持って重さを受ける**が、pan で腕全体が回る向き（±0.3 rad ≈ ±17°）には手を置かない。回転する側から体と物をどける。
- 起動は W1 と同じ（`torque_limit:=500`）。
  ```bash
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py shoulder_pan 0.2 --seconds 4
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py shoulder_pan -0.4 --seconds 6
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py shoulder_pan 0.2 --seconds 4
  ```
- **止め方**: 動きが止まっていることを確認 → 元の位置（折りたたみ・正面）に戻っている → 腕を支える → **Ctrl-C**。
- 合格: W1 と同じ基準。ログに `step limit: ... unexpected jump for an angle joint` が出ていないこと（出たら報告）。

## W3. shoulder_lift / elbow_flex / wrist_flex（重力を受ける関節）

目的: 重力を受ける 3 関節で、トルク ON のまま動かせること。Torque_Limit は常に 1000（下げない）。

- **腕の支え方（最重要）**: 折りたたみ姿勢。**前腕の下に手のひらを構え、触れずに数 cm あけておく**（W3 の実測は、手で支えると遊びの分だけ腕が動いて測定に影響するため、動かすあいだは自重だけを受けた状態で行った）。沈む／弱いと感じたらすぐ手で受ける。**止めるとき（Ctrl-C の前）は前腕に触れて重さを受ける**。もう片手はサーボ電源の近く。
- 起動（`torque_limit` は不要。指定しても lift / elbow / wrist_flex には効かない）:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech read_only:=false use_rviz:=false \
    2>&1 | tee -i ~/so_arm101_check/write_W3.log
  ```
- 1 関節ずつ、**小さく（0.1 rad）、ゆっくり（4 秒）**。**開始時の実測値（`ros2 topic echo /joint_states --once`）を控えておく**。向きは実測（W3、2026-11-11）: **wrist_flex の − と elbow_flex の − が「開く向き」**（手先／前腕が離れる）。**shoulder_lift の + は腕全体が前へ倒れ、先端が下がる向き**で、0.1 rad で gripper の先端が台に接触した（強い押し付け・異音は無し）。lift は台に近いので + は小さく、まず 0.05 rad から。
  ```bash
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py wrist_flex -0.1 --seconds 4
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py elbow_flex  -0.1 --seconds 4
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py shoulder_lift 0.05 --seconds 4   # 台に近い。先端が当たらないか見ながら
  ```
- **戻し方（重要）**: `send_joint_delta.py` は**今の実測値から**目標を計算する。開く向き（自重に逆らう向き）では、実測が目標の手前で止まる（W3 の実測: elbow_flex で約 27 tick ≈ 0.04 rad、wrist_flex で約 8 tick）。**戻すときに +0.1／−0.1 の固定量を送ると、開始位置を越える**（elbow_flex では校正範囲の端 約 +1.71 を越える目標になった）。そこで、**戻す量 = 開始時の値 − 今の実測値** を計算して送る。端に近い関節（elbow_flex、shoulder_lift）は、開始時の値の**少し（0.02 rad ほど）手前まで**にする。
  ```bash
  ros2 topic echo /joint_states --once        # 今の実測値
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py elbow_flex <開始時の値 - 今の値 - 0.02> --seconds 4
  ```
  ※ 折りたたみ姿勢の shoulder_lift / elbow_flex は関節リミットの端に近い（`inventory.md` の問題2）。**リミットの外へ向かう向きは動かさない**（ドライバは校正範囲でクランプする）。
- **止め方**: 元の折りたたみ姿勢（開始時の値）に戻っている → **前腕に触れて重さを受ける** → **Ctrl-C**（脱力で腕が沈む）。
- 合格: 沈まない／指令の向きに動く／`FAULT` なし／`rate_limited` が 0。

## W3.5. W4 の前に: 折りたたみ姿勢から少し開いた姿勢にする

理由: home の折りたたみ姿勢は衝突判定で Plan & Execute が中止される既知の問題がある（`inventory.md` の問題2）。RViz を使う前に、腕を少し開いておく。
**W3 に合格してから行う。**

- **どの関節をどの向きにどれだけ動かすかは、W3 で確かめた向き（開く／閉じる）の結果を見てから決める。** ここでは決め打ちしない。目安は、W3 で「開く向き」と確認できた関節（例: `elbow_flex`）を 0.1 rad ずつ、1 回ごとに止まってから、数回まで。
- **腕の支え方**: W3 と同じ（前腕の下を支える）。
- 起動は W4 と同じ（RViz あり。ここでは RViz を操作しない）。**W3.5 と W4 は同じ起動のまま続けて行う**（開いた姿勢で Ctrl-C しないため）。ログは `write_W4.log`:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech read_only:=false use_rviz:=true \
    2>&1 | tee -i ~/so_arm101_check/write_W4.log
  ```
- 別の端末で（W3 と同じ `send_joint_delta.py`。向きは W3 の結果に合わせる）:
  ```bash
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py elbow_flex -0.1 --seconds 4    # 向きは W3 の結果で決める
  ```
- 開いた姿勢で動きが止まっていること、`FAULT` が無いことを確認してから W4 へ。

## W4. MoveIt（RViz）から。速度スケーリング 0.1

- **腕の支え方**: W3 と同じ（前腕の下を支える）。最初の動作は**必ず W3.5 で作った少し開いた姿勢から**。
- 起動: W3.5 の起動のまま（RViz あり）。
- RViz の MotionPlanning で、**Velocity Scaling と Acceleration Scaling を 0.1**（既定）のまま、小さな目標（現在位置から 0.1〜0.2 rad 程度）を Plan & Execute。1 回ごとに動きが止まってから次へ。
- 見ること: 計画どおりに滑らかに動く／終点で止まる／`step limit` の WARN が arm 関節で出ない／`FAULT` なし。
- **止め方**: **開いた姿勢のまま Ctrl-C すると腕が落ちる。** 順に: ① 動きが止まっていることを確認 → ② RViz で、W3.5 で作った開いた姿勢へ戻す（W4 で動かした後は W3.5 の姿勢からずれているため）→ ③ `send_joint_delta.py` で W3.5 の逆（0.1 rad ずつ）を送って折りたたみ姿勢へ戻す → ④ 前腕を支える（戻せないときも手で支える）→ ⑤ **launch の端末で Ctrl-C**。
- 合格: W3 と同じ基準＋ログ末尾 `rate_limited=0`（arm）。

## W5（任意・最後に 1 回）. 通信断
**やるかどうかはユーザーが決める。** サーボは通信が切れても最後の目標で保持し続ける、という前提（実機で未確認）を確かめる。
- 腕を折りたたみ姿勢で支え、W0 の起動のまま、**USB ケーブルを抜く**（サーボの外部電源は入れたまま）。
- 期待: ドライバが `read failed ... consecutive=10/10` → `COMMUNICATION LOST` を出し、`/joint_states` が NaN になる。**腕は最後の目標で保持され続ける**（脱力しない）はず。保持されずに脱力するなら、その事実を報告する。
- 終了: **腕を支えたまま、サーボの外部電源を切る**（USB を抜いたので Ctrl-C ではトルクを OFF にできない）。その後 Ctrl-C。USB を挿し直す。

---

## ログの読み方
| ログ | 意味 | 対応 |
|---|---|---|
| `FAULT: joint X: Torque_Enable=0 Acceleration=0 ... powered off or reset` | サーボの電源が一瞬落ちた／リセットされた。腕は脱力。**送信は止まった** | 腕を支える。Ctrl-C して原因（電源・配線）を確認。サーボの電源が戻っていれば再起動できる |
| `FAULT: ... protection error ...`（ERR バイト） | サーボ側の保護（過負荷・温度・電圧など）。**プロセスの再起動まで再開しない** | 支えて Ctrl-C。サーボが冷えるまで待つ。無理な姿勢・負荷が無いか確認 |
| `FAULT: commands were invalid ... for 50 cycles` | NaN／範囲外の指令が 1 秒続いた。送信停止（トルクは保持） | 支えて Ctrl-C（脱力で腕が落ちる）。指令の出元を確認 |
| `COMMUNICATION LOST` | 通信断。**サーボは最後の目標でトルク ON のまま。ドライバから OFF にできない** | 腕を支えて、サーボの電源を切る |
| `step limit: 'X' requested +N ticks ... limited` | 1 周期の変化量の上限（arm: 312 tick ≈ 0.48 rad、gripper: 約 408 tick ≈ 0.48 rad）を超える指令を削った。gripper の `ramping` は、大きな一歩のときだけ出る（正常） | arm 関節で出たら報告 |
| `activation failed: ...` | 起動の手順のどこかで失敗。トルクは ON にならない（または OFF に戻した） | ログを報告 |
| `read of X failed (...); retrying n/2`（WARN） | 起動の読み戻しの通信エラー（checksum mismatch・フレーム不良・タイムアウト）。`[tx=… rx=… flushed_before_request=…]` に送った要求と受信したバイトが出る。再試行で直れば起動は続く | 行を全部報告。`startup_read_retries` の数も見る |
| `cannot read back X (...)`、直前に `retrying` が 2 回 | 3 回とも通信エラー。トルクは ON にならない | 報告。`[activation failed] torque OFF and Lock=1 confirmed` を確認 |
| `ros2_control_node` が exit code -6、直前に `terminate called after throwing an instance of 'std::runtime_error'` / `Failed to set the initial state of the component ... to active` | 起動に失敗したとき、controller_manager（Humble）が例外で終了する。Humble の仕様 | `[activation failed] torque OFF and Lock=1 confirmed (read back) on all joints` の行があれば安全面の問題はない。その行が無ければサーボの電源を切る |
| `[deactivate] ... could NOT confirm ...` | 終了でトルク OFF／Lock=1 を確認できなかった | 腕を支えてサーボの電源を切る |

## 報告してほしいもの（段階ごと）
- ログのパス（`~/so_arm101_check/write_<段階>.log`）
- 段階ごとの抜き出しの出力（W3.5 は W4 のログに含まれる）:
  ```bash
  P='WRITE mode\|write mode active\|\[probe\]\|FAULT\|COMMUNICATION\|step limit\|activation failed\|\[deactivate\]\|deactivated\|ERROR\|retrying\|startup_read_retries'
  grep -a "$P" ~/so_arm101_check/write_W0.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-300
  grep -a "\[diag\]\|ERROR" ~/so_arm101_check/write_W0_after.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-300   # 停止後の読み取り専用
  grep -a "$P" ~/so_arm101_check/write_W1.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-300
  grep -a "$P" ~/so_arm101_check/write_W2.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-300
  grep -a "$P" ~/so_arm101_check/write_W3.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-300
  grep -a "$P" ~/so_arm101_check/write_W4.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-300
  ```
  （該当する段階の行だけ実行する。W5 は W0 と同じ形式で `write_W5.log`）
- 想定と違う挙動（トルクが入った瞬間に動いた、沈んだ、異音、止まらなかった、など）
