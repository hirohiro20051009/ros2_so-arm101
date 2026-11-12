# 実機確認手順: 書き込みモード（トルク ON、Goal_Position）（段階2b-3c）

対象: `hardware:=feetech read_only:=false`。**腕が動く。サーボにトルクが入る。** 実行するのはユーザーだけ（Claude は実機を動かさない）。
実装の仕様は `docs/inventory.md` の「段階2b-3 の計画 v3」と「段階2b-3b の計画 v4」。

起動の既定（段階2b-4 以降）: `hardware` は `feetech`、`read_only` は `true`。**書き込みモード（トルク ON）は `read_only:=false` を明示したときだけ**。LeRobot 経路は `hardware:=lerobot` を明示したときだけ。この手順書の起動コマンドは両方を明示してあるので、既定の変更で意味は変わらない。

段階は W0 → W1 → W2 → W3 → W4（W5 は任意）。**前の段階が合格するまで次へ進まない。** 各段階に「腕の支え方」と「止め方」がある。必ず読んでから始める。

## 共通の約束（全段階）

1. **止める前に、腕を折りたたみ姿勢にして、手で支える。** Ctrl-C でドライバがトルクを OFF にする（腕は脱力して落ちる）。
2. 止めるのは **Ctrl-C だけ**。`kill -9`、端末を閉じる、PC の電源断はしない（トルクが ON のまま残る）。
3. 止めた後、ログに次の行があることを確認する。無ければサーボの電源を切る（腕が落ちるので支える）。
   `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`
4. **非常停止はサーボの外部電源を切る**（USB だけではない）。切ると腕が落ちる。常に片手を電源の近くに置き、もう片手で腕を支える。
5. ログの `FAULT:` または `COMMUNICATION LOST` が出たら、その段階を中止して Ctrl-C（支えながら）。原因を報告してから再開する。
6. LeRobot 経路と同時に起動しない（LeRobot 経路は削除済みなので、今は該当しない。conda の確認は残す）。conda を activate していない新しい端末を使う。
7. 腕の周りに物を置かない。ケーブルが引っかからないようにする。
8. 後始末の確認: `ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"` と `ros2 node list` の両方で残りがないことを確認する。**`ros2 node list` に残っていても `ps` が空なら、`ros2 daemon stop; ros2 daemon start` の後にもう一度確認する**（move_group が Segmentation fault で終わると、daemon に古い一覧が残ることがある。推測）。それでも残るなら、ps に出ない原因を調べる。

## C4 の全体の順序（C3 合格後。バス層の修正の確認を含む）

**実行するのはユーザーだけ。** 各ステップは前のステップが合格してから。ログは `~/so_arm101_check/` に取る。止めるのは **Ctrl-C だけ**（`kill -9`・名前での kill はしない）。各ステップの後に、共通の約束 8 の後始末の確認（`ps` と `ros2 node list` の両方）を行う。
前提: `git log` に、バス層の修正（段階 1〜4）のコミットがある／`colcon build --symlink-install` 済み／`colcon test`（`so_arm101_hardware`・`so_arm101_calibration`）が全件通っている（Claude が確認済み）。

| 順 | ステップ | 腕・トルク | 何をするか | 報告してほしいもの |
|---|---|---|---|---|
| 1 | **C0.6**（案 C の確認。`docs/so_arm101_calibration_check.md`） | トルク OFF。EEPROM に書く（gripper、同じ値） | 手順 0（新しいバックアップ `eeprom_backup_before_C0.6.json` を取り、`eeprom_after_C3.json` と全関節が一致することを確かめる）→ 1（dry-run で gripper が 547/1454/2944 を確かめる）→ 2・3（書く・状態確認）→ 4（待ち 0 ms）。戻す方法と終了コードごとの対応は C0.6 の節。**実行前に Claude に伝えて確認を得る** | 手順 0 の `MATCH`/`DIFF`、dry-run の gripper の 3 値、各ログと `exit=`、`time` の出力、失敗時は `bus:` の行 |
| 2 | **読み取り専用の運転 10 分**（バス層の修正の健全性） | トルク OFF。腕は触らない | 下の「2. 読み取り専用 10 分」 | `deactivated (no register was written)` の行の全文（`reads`・`retried`・`failed_cycles`・`bus:`）、経過時間 |
| 3 | **読み値のばらつきの確認**（出発点） | トルク OFF | 腕をたたみ切った姿勢に作り直して、読み取り専用の起動を 3 回（各 5 秒）。`[diag]` と `/joint_states` の elbow_flex を比べる | 3 回の elbow_flex の読み値（出発点 +1.69009 との差） |
| 4 | **W0 を 5 回**（起動と停止の繰り返し） | トルク ON（保持のみ）。腕を両手で支える | 下の「4. W0 を 5 回」 | 各回の `startup_read_retries`・`deactivated.` の行（`bus:` を含む）、`checksum mismatch` が出たら `rx=` と `flushed_before_request=` の行を全部 |
| 5 | **W1 → W2 → W3 → W3.5 → W4**（本文の各段階どおり。**W4 は 10 分**） | 各段階の支え方どおり | 各段階の合格基準を満たしてから次へ。W4 は OMPL、目標は elbow_flex を + に動かさない | 各段階の抜き出し（報告してほしいもの）＋ `deactivated.` の `retried`・`failed_cycles`・`monitors`・`rate_limited`・`port_contention`・`bus:` |
| 6 | **RViz の重なり・問題2・`allowed_start_tolerance` の確認、範囲をわずかに超えた出発点からの計画の確認**（`inventory.md` の「未確認として残すもの」） | W4 と同じ起動で | 下の「6. 範囲を超えた出発点からの計画」（W3.5 の前、elbow が開いていない状態で行う）。その後、W4 の最初の Execute の前に、MotionPlanning の現在の状態で upper_arm_link と wrist_link の重なりを見る。Plan & Execute が実機で通るか | 下の「6」の報告項目、重なりの有無、Plan & Execute の結果（成功／中止の表示） |
| 7 | **`measure` の記録**（任意。EEPROM には書かない。トルク OFF、ros2_control を止める） | トルク OFF | C1c と同じ手順（`docs/so_arm101_calibration_check.md`） | `記録:` の行と `bus:` の行（失敗 0、間隔 約 21.7 ms、`drained_bytes=0 id_mismatch=0 answered_first=0`） |
| 8 | **`drain_quiet_ms:=0` との比較**（問題が出たときだけ） | 問題の出た段階と同じ | **今は実施できない**: 起動引数 `drain_quiet_ms` が launch・xacro に無い（ドライバはコードの既定値 2 で動く）。引数を足すには `bringup.launch.py`・`so101_new_calib.urdf.xacro`・`so101_new_calib.ros2_control.xacro` の差分が要り、**ユーザーの承認が要る**（差分は Claude が示す）。承認されたら、`drain_quiet_ms:=0` を足した同じ起動で、同じ運転の `retried`・`failed_cycles`・`bus:` を比べる | 比較の表（0 と 2 の `retried`・`failed_cycles`・`bus:`） |

### 2. 読み取り専用 10 分
```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech use_rviz:=false \
  2>&1 | tee -i ~/so_arm101_check/c4_readonly_10min.log
```
- 10 分ほど放置（腕は触らない。トルク OFF なので、腕を自重で落ちない姿勢にしておく）して、**Ctrl-C を 1 回**。
- 合格: `retried=0`、`failed_cycles=0`、`bus:` の **`drained_bytes=0`、`id_mismatch=0`、`answered_first=0`**（健全なバスでは、捨てるものが無い＝A・B は何もしない）。`flushed_requests`・`timeouts`・`bad_checksum` も 0 が望ましい。
- **正常な経路が遅くなっていないこと**: 同じ時間での `reads` の数が、修正前の記録（約 50 Hz。10 分で約 3 万回）と同じ。
- 0 でない項目があれば、その行と、直前の `read failed` の WARN（`[tx=… rx=… flushed_before_request=…]`）を全部報告する。次へ進む前に Claude と相談する。
- 抜き出し: `grep -a "deactivated\|read failed\|WARN\|ERROR" ~/so_arm101_check/c4_readonly_10min.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-400`

### 4. W0 を 5 回
W0 の手順（本文の「W0. 保持のみ」）を**起動と停止を 5 回繰り返す**（W0 の失敗は過去に 4 回中 2 回だった。直近は 0 が続いている）。毎回、腕を折りたたみ姿勢で両手で支え、10 秒保持して Ctrl-C、`[deactivate] torque OFF and Lock=1 confirmed` を確認。ログは `write_C4_W0_1.log`〜`_5.log`。
- 各回の記録: `startup_read_retries`、`[diag]` の行、`deactivated.` の行（`bus:` を含む）。
- 合格: 5 回とも起動し、腕が動かず、`startup_read_retries=0`、`bus:` の `id_mismatch=0 answered_first=0 drained_bytes=0`。`checksum mismatch` が出たら、`flushed_before_request` が 0 より大きいか（要求の前に古いバイトがあった）0 か（配線・USB の化け）を見て、**`rx=` と `flushed_before_request=` の行を全部報告**する（W0 の過去の失敗の原因の切り分け。`docs/bus_layer_fix_plan.md` の 6 章）。
- 5 回の後に、読み取り専用で状態を確認する（本文の W0 の最後と同じ。`torque_enable=0 lock=1`）。

### 6. 範囲をわずかに超えた出発点（elbow_flex +1.69009）からの計画
出発点の elbow_flex は URDF のリミット +1.69 を約 0.00009 rad 超えている。OMPL が `FixStartStateBounds` で扱えるかを、**実機で最初に**確認する（モックでは成功を確認済み。Pilz は開始状態の範囲外で拒否することを mock で確認済みなので、**Pilz は使わない**）。
- **書き込みモードで起動する**（W3.5 と同じ起動。RViz あり）。**`send_joint_delta.py` を送る前、elbow_flex が開いていない（出発点の）状態で行う。** 腕の支え方は W3 と同じ（目標が開く向きだけなので、前腕の下に触れずに構える）:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech read_only:=false use_rviz:=true \
    torque_limit:=500 2>&1 | tee -i ~/so_arm101_check/write_C4_step6.log
  ```
- 起動直後に別の端末で、実測を控える: `ros2 topic echo /joint_states --once`（elbow_flex が +1.69 付近であること）。
- RViz の MotionPlanning:
  1. Planning タブで **Planner を OMPL**（Pipeline は ompl）にする。**Pilz を選ばない**。Velocity / Acceleration Scaling は 0.1。
  2. **Goal State を `<current>` に選び直す**。Joints タブで **elbow_flex のスライダーだけ**を、**開く向きに小さく（例: −5°）**動かす（スライダーは整数の度。他の関節のスライダーは触らない。出発点付近でリミットに近い向きへは動かさない）。
  3. **Plan** を押す。Plan のアニメーションで、動かしていない関節が動いていないか、elbow_flex が開く向きだけであることを見る。
  4. 成功したら **Execute**。1 回で、動きが止まるまで待つ。**Execute の前に、Plan のアニメーションで、elbow_flex 以外の関節の変化量を確かめる。0.5° 以内の変化は許容する**（下の注意。Joints タブは整数の度なので、目標を作ると全関節が整数の度に丸められ、動かしていない関節も最大 0.5° 変わる。実機で確認済み）。**1° を超えて動く関節があれば Execute しない**（報告して止める）。0.5° 〜 1° の間は、報告してから判断する（Claude に相談）。
- **注意（`<current>` の丸めと、elbow_flex の上限）**: 出発点の elbow_flex は +1.69009 rad ＝ 96.835° で、整数の度に丸めると **97°（+1.6930 rad）**になりうる。URDF の上限 +1.69 rad（96.830°）を超えるので、**`<current>` を選んだ時点の目標（または Joints タブの elbow_flex の値）が、上限を超えた状態として表示される可能性がある**（RViz の目標の姿勢が「無効」の表示になる、スライダーが上限で止まる、など。**実際にどう見えるかは未確認。実機の画面で確認して報告する**）。扱い:
  - 目標を作る目的は elbow_flex を **−5°（約 92°、+1.606 rad。上限の内側）にすること**。スライダーを 92° に動かした時点で上限の内側になるので、無効の表示は消えるはず。**消えない（elbow_flex が 92° なのに無効の表示のまま、または他の関節が無効）場合は、Plan も Execute もせず、画面と move_group のログを報告する。**
  - 動かす前の `<current>` が無効に見えるのは、開始状態（実機の値）が範囲を 0.00009 rad 超えているためで、**この確認の対象**（`FixStartStateBounds` で開始状態が扱われるか）。Plan がこの開始状態を理由に拒否されたら、そのメッセージを報告する。
  - elbow_flex を + 側（97° 以上）にする目標は作らない。他のスライダーの変化の見積り（出発点を整数の度に丸めたとき）: pan −5.63°→−6°（−0.37°）、lift −84.97°→−85°、wrist_flex +28.84°→+29°、wrist_roll −2.77°→−3°（−0.23°）、gripper +0.87°→+1°（+0.13°）。すべて 0.5° 以内。
- Plan が失敗する・開始状態の不正で拒否される場合は、Execute せずに、ログを報告する。続けるなら（Claude と相談のうえ）`send_joint_delta.py elbow_flex -0.10 --seconds 4` で先に開いてから W3.5 へ進む。
- **実機の結果（2026-11-12）**: 実機では `<current>` の丸めは **elbow_flex 98°** になり、スライダーは端に張り付いて見えた（上の見積りの 97° ではない。elbow の出発点は +1.70697 rad と、URDF の上限より 0.017 rad 大きかった）。丸めの値は、出発点の実測によって 97°／98° と変わりうる。端に張り付いて見えても、elbow_flex を 93° にして Plan & Execute は成功した。
- 報告:
  - **Plan の成否**（RViz の表示。失敗なら、その表示の文言）。
  - **Execute の成否**（成功／中止／実行中の異常）。
  - move_group のログで、start state や bounds に関する行（`start state`・`out of bounds`・`bounds`・`FixStartStateBounds`・`INVALID_ROBOT_STATE`・`outside`・`tolerance`）:
    ```bash
    grep -a -i "start state\|bounds\|FixStartState\|INVALID_ROBOT_STATE\|outside\|tolerance" ~/so_arm101_check/write_C4_step6.log | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-300
    ```
  - **実行後の `/joint_states`**（`ros2 topic echo /joint_states --once`。出発点との差も）。
- そのまま W3.5・W4 へ続ける場合は、**開いた姿勢で Ctrl-C しない**（W4 の止め方に従う）。

### 合格の基準（C4 全体の判定。3 つに分ける）
各ステップのログの `deactivated.` の行（`retried`・`failed_cycles`・`monitors`・`rate_limited`・`port_contention`・`bus:`）と、`grep` の抜き出しで判定する。**どれか 1 つでも「不合格」に当たれば不合格。「不合格」に当たらず「要観察」に当たれば要観察。どちらにも当たらなければ合格。**

**不合格（止めて原因を調べる。次のステップへ進まない）**
- `failed_cycles>0`
- `rate_limited>0`
- `port_contention>0`
- `COMMUNICATION LOST`、または `activation failed`
- `MISMATCH`（calib の読み戻し・バックアップの比較）
- `could NOT confirm`（トルク OFF／Lock=1 を確認できなかった。**腕を支えてサーボの電源を切る**）
- （既存の `FAULT:` も同じく不合格）

**要観察（記録して続ける）**
- `retried>0` で、再試行で回復したもの（`failed_cycles=0`）
- `checksum mismatch`（**`rx=` と `flushed_before_request=` の行を全部記録する**。`flushed_before_request>0` か 0 かを見る）
- `bus:` のカウンタが 0 でないもの（`flushed_requests`・`flushed_bytes`・`drained_bytes`・`id_mismatch`・`answered_first`・`timeouts`・`bad_checksum`。**直前の WARN の `rx=` と `flushed_before_request=` の行を記録する**）
- 要観察が出たステップは、同じ条件でもう 1 回行って、再現するかを見る。再現したら Claude と相談する（順 8 の比較）。

**合格**
- 上のどちらにも当たらない（`retried=0`、`failed_cycles=0`、`rate_limited=0`、`port_contention=0`、`bus:` の全カウンタが 0、`reads` の数が従来と同じ（約 50 Hz））。

### 実機でエラーを起こす試験はしない
バスのバイトを壊す・止めるのは危険なので、実機ではしない。エラー時の挙動は偽サーボの試験で確認済み（`docs/inventory.md` の「バス層の修正」の段階 1〜4）。

---

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
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py shoulder_lift <開始時の値 - 今の値 + 0.02> --seconds 4
  ```
  **「− 0.02」は elbow_flex 専用**（戻す量が + なので、0.02 を引くと端の手前で止まる）。**shoulder_lift は + が開く向き（戻す量が −）なので「+ 0.02」**（戻す量の絶対値を 0.02 小さくする）。wrist_flex など端に近くない関節は 0.02 を足し引きしない。
  ※ 折りたたみ姿勢の shoulder_lift / elbow_flex は関節リミットの端に近い（`inventory.md` の問題2）。**リミットの外へ向かう向きは動かさない**（ドライバは校正範囲でクランプする）。
- **止め方**: 元の折りたたみ姿勢（開始時の値）に戻っている → **前腕に触れて重さを受ける** → **Ctrl-C**（脱力で腕が沈む）。
- 合格: 沈まない／指令の向きに動く／`FAULT` なし／`rate_limited` が 0。

## W3.5. W4 の前に: 出発点から elbow_flex だけを少し開く

理由: home の折りたたみ姿勢は衝突判定で Plan & Execute が中止される既知の問題がある（`inventory.md` の問題2）。RViz を使う前に、elbow_flex だけを少し開く。
**W3 に合格してから行う。** shoulder_lift は動かさない。他の関節も動かさない。

- **出発点（W3.5 と W4 の基準。トルク OFF で手でたたみ切った姿勢を、C3 の後（新しい校正の座標）に、読み取り専用の起動で測った値。ユーザー報告。丸めた値は `~/so_arm101_check/calib_C3_pose_margin.log`）**。W4 の後に戻して Ctrl-C する姿勢もこれ。（旧校正の座標＝C3 の前の値は pan -0.08362、lift -1.51134、elbow +1.66938、wrist_flex +0.50941、roll -0.07135、gripper +0.01645。`~/so_arm101_check/start_pose.log`。今は使わない。）

  | 関節 | 出発点の値 [rad] |
  |---|---|
  | shoulder_pan | -0.09820 |
  | shoulder_lift | -1.48295 |
  | elbow_flex | +1.69009 |
  | wrist_flex | +0.50327 |
  | wrist_roll | -0.04833 |
  | gripper | +0.01523 |

- **出発点の作り方（実測 2026-11-12）**: 書き込みモードの前の出発点は、**「支えて戻す」たたみ方**で作る（手で前腕を支えて、ゆっくり折りたたみの止まりまで戻す）。elbow_flex は +1.6840／+1.6855（±1 tick）に収まる。**「重力に任せて」たたむと +1.7208 になり、校正のクランプ（±1.7192）を約 1 tick 超える**（書き込みモードの起動は拒否されず、生の値を Goal にして次の周期で端へ引き込まれる。50 tick 以内は黙ってクランプ）。
- **出発点の表は毎回少しずれる**: たたみ切りの姿勢は毎回少し変わる（2026-11-12 は lift -1.49983、wrist_flex +0.41888 で、上の表とずれていた）。そこで W4 の戻し先は**「その起動のときの実測値」**とする（起動直後に `/joint_states` を控える）。**ただし elbow_flex は +1.69009 を上限にして、それより大きい値（URDF の上限を大きく超える値）には戻さない。**
- モックの MoveIt（新座標の出発点での判定。`/check_state_validity`・`/compute_fk`・`/plan_kinematic_path`、2026-11-11、mock の move_group。実機は使っていない）の判定: 出発点と、そこから elbow_flex を -0.10 / -0.15 / -0.20 / -0.30 にした姿勢は**すべて有効**（衝突なし）で、出発点からの OMPL の計画もすべて成功する。gripper 先端の高さ（base_link 基準）は、出発点（z +0.0219 m）から +0.0255 / +0.0386 / +0.0520 / +0.0794 m（旧校正の座標の出発点での判定は +0.026 / +0.039 / +0.053 / +0.081 m で、ほぼ同じ）。
- **腕の支え方**: W3 と同じ（動かすあいだは前腕の下に触れずに構える。止めるときは触れて受ける）。
- 起動（**W3.5 と W4 は同じ起動のまま続けて行う**。開いた姿勢で Ctrl-C しないため）。W4 は初めての MoveIt の実行なので `torque_limit:=500`（pan と wrist_roll に効く。lift / elbow_flex / wrist_flex は常に 1000）を付ける。RViz あり、ここでは RViz を操作しない。ログは `write_W4.log`:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false hardware:=feetech read_only:=false use_rviz:=true \
    torque_limit:=500 2>&1 | tee -i ~/so_arm101_check/write_W4.log
  ```
- 別の端末で、elbow_flex に **-0.10 を 2 回**（1 回ごとに動きが止まってから `/joint_states` で実測を確認してから次へ）:
  ```bash
  python3 ~/ros2_so-arm101/tools/send_joint_delta.py elbow_flex -0.1 --seconds 4
  ros2 topic echo /joint_states --once    # elbow_flex の実測。出発点 +1.69009 との差を計算する
  ```
- 開く向き（自重に逆らう向き）は、目標の約 0.04 rad 手前で止まる（W3・W3.5 の実測。旧校正の座標の出発点（+1.66938）での W3.5 では -0.10 を 2 回で出発点から約 -0.12、実測 +1.5620 になった。閉じる向きは約 3 tick）。実測の変化量で判断する:
  - 2 回目の後、出発点からの変化が **-0.10 〜 -0.15** → 終わり（先端は約 +0.03 〜 +0.04 m 上がる。モックで -0.10〜-0.30 は有効）。
  - -0.10 に届かない → 足りない量（**最大 -0.05**）を **1 回だけ**足す。（W3.5 では -0.05 を足しても約 -0.012 しか動かなかった。手前止まりが約 0.04 rad あるため、効く量は小さい）
  - **出発点から -0.25 を超えて開く指令は送らない**（モックで確認済みなのは -0.30 まで。新座標の出発点でも確認した。余裕を見ている）。
- 開いた姿勢で動きが止まっていること、`FAULT` が無いことを確認してから W4 へ。

## W4. MoveIt（RViz）から。速度スケーリング 0.1

- **所要時間: 10 分**（C4 では、読み取り専用の 10 分と合わせて十分とする。10 分の間に、W4 の小さな目標の Plan & Execute を数回行ってよい。10 分を超えたら、止め方に進む）。
- **腕の支え方**: W3 と同じ（前腕の下を支える）。最初の動作は**必ず W3.5 で作った少し開いた姿勢から**。
- 起動: W3.5 の起動のまま（RViz あり、`torque_limit:=500` 付き）。
- RViz の MotionPlanning で、**Velocity Scaling と Acceleration Scaling を 0.1**（既定）のまま、小さな目標（現在位置から 0.1〜0.2 rad 程度）を Plan & Execute。1 回ごとに動きが止まってから次へ。
- **最初の Execute の前（W4 の実測 2026-11-11 に基づく）**: MotionPlanning の Planning タブで **Goal State を `<current>` に選び直し**、Joints タブで**スライダーを 1 つだけ**動かす。**Plan のアニメーションで、動かしていない関節が動いていないか**を見てから Execute する。
- **注意（事実）: Joints タブのスライダーは整数の度**。目標の姿勢は**全関節が整数の度**になるため、動かしていない関節も目標が最大 約 0.5° 変わる（W4 の実測: 実行後の実機が pan +2.33°、lift −86.07°、elbow +90.29°、wrist_flex +29.54° のとき、スライダーは 2、−86、90、30）。MoveIt と arm_controller は渡された目標どおりに動いている。
- **注意: 閉じる向き（重力に沿う向き）に変わる関節は、落ちるように動くことがある。** W4 では elbow_flex が約 88.5° → 90°（約 1.8°）と閉じる向きに変わり、1 回の Execute で「カタン」と落ちる動きがあった（推測: 閉じる向きの目標でギアの摩擦から外れて自重で動いた）。Plan の前に、閉じる向きに変わる関節が無いかを Plan のアニメーションで見る。
- **目標の作り方の注意**: 出発点の elbow_flex は URDF のリミット（+1.69）を**約 0.00009 rad 超えている**（C3 の後の新座標。OUT。旧校正の座標では 0.021 rad の余裕があった）。OMPL は計画要求アダプタ `FixStartStateBounds`（`start_state_max_bounds_error: 0.1`）で扱えるので、この姿勢から Plan & Execute できる見込み（モックの move_group で計画の成功を確認。**実機は C4 で確認**）。**W4 の目標は elbow_flex を + に動かさない（現在より大きくしない）ものにする。** shoulder_lift も出発点付近でリミットに近い向き（−側）へは動かさない。
- **MotionPlanning では OMPL を使い、Pilz を選ばない**（開始状態が URDF の範囲をわずかに超えると、Pilz は計画を拒否する（`INVALID_ROBOT_STATE`）ため。出発点の elbow_flex は約 0.00009 rad 超えている）。**Pilz を使うときや、W3.5 を飛ばして起動直後に計画するときは、先に `python3 ~/ros2_so-arm101/tools/send_joint_delta.py elbow_flex -0.10 --seconds 4` で elbow_flex を開く**（開く向きは目標の約 0.04 rad 手前で止まるので、約 -0.05 動く。開いた姿勢のまま Ctrl-C しない）。
- 見ること: 計画どおりに滑らかに動く／終点で止まる／`step limit` の WARN が arm 関節で出ない／`FAULT` なし。pan の向き（+ で上から見て時計回り）が RViz の表示と実機で一致するか。
- **止め方**: **開いた姿勢のまま Ctrl-C すると腕が落ちる。** 戻す先は**その起動のときの実測値**（起動直後に控えたもの。W3.5 の表の値は目安で、毎回少しずれる）。ただし elbow_flex は +1.69009 を超える値には戻さない。戻しは RViz を使わず、`send_joint_delta.py` で 1 関節ずつ行う（理由: RViz では関節ごとの目標値を数値で指定する操作が分かりにくく、計画の衝突判定で中止される可能性もある。`send_joint_delta.py` は差分を直接指定でき、1 回ごとに実測で確認できる）。
  1. 動きが止まっていることを確認。
  2. `ros2 topic echo /joint_states --once` で各関節の実測を読む。
  3. 動かした関節ごとに、**「戻し先（起動時の実測値） − 現在の実測値」**を計算して送る（`send_joint_delta.py` は現在の実測値から目標を計算するため）。1 回の |delta| は 0.5 rad 以下。超える場合は分けて送る。順番は pan → wrist_roll → wrist_flex → shoulder_lift（動かしていれば）→ **elbow_flex を最後**。
     ```bash
     python3 ~/ros2_so-arm101/tools/send_joint_delta.py <関節名> <出発点 - 実測> --seconds 4
     ```
  4. elbow_flex は「戻し先 − 現在値」を 1 回で送る（戻し先は起動時の実測値。ただし +1.69009 を上限にする）。たとえば +1.69009 の場合（目標の +1.69009 は URDF リミット +1.69 を約 0.00009 rad 超える。`send_joint_delta.py` は URDF の範囲を見ず、ドライバは校正した範囲（±1.7192）でクランプするので、送れる。たたみ切った機械的な止まりなので、折りたたむ向きは差が約 3 tick）。
  5. 再び `/joint_states` を読み、**各関節が出発点から約 0.02 rad 以内**であることを確認する。
  6. 前腕を支える（戻せないときも手で支える）→ **launch の端末で Ctrl-C**。
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
| `deactivated. ... bus: flushed_requests=… flushed_bytes=… drained_bytes=… id_mismatch=… answered_first=… timeouts=… bad_checksum=…`（バス層の診断。読み取り専用の `deactivated (no register was written)` の行にも出る） | `drained_bytes`・`id_mismatch`・`answered_first` は、**健全なバスでは 0**（通信エラーの後に、古い応答を捨てた／取り違えた数）。`flushed_requests`・`flushed_bytes` は、要求の直前に古いバイトがあって捨てた回数と量（0 が望ましい）。`timeouts`・`bad_checksum` は通信エラーの数（`retried` と対応）。0 でなければ、その数と、直前の WARN の `[tx=… rx=… flushed_before_request=…]` を報告する。切り分け: `checksum mismatch` のとき `flushed_before_request>0` なら要求の前に古いバイトがあった、0 なら配線・USB の化け | 報告。原因の比較は `drain_quiet_ms:=0`（従来の「捨てるだけ」）で行う（C4 の順 8。起動引数の追加の承認が要る） |
| 終了時に `rviz2`（と `move_group`）が exit code -11（`process has died`）。deactivate の後に起きる（2026-11-12 の W4 のログでは、`[deactivate] torque OFF and Lock=1 confirmed` と `deactivated.` の後） | RViz 側の終了時の異常。実機のトルク OFF／Lock=1 の確認（`[deactivate]`・`deactivated.`）が済んでいれば**無視してよい**（2026-11-12 の W4 で確認） | `[deactivate]` に `could NOT confirm` が無いことだけ確認 |
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
