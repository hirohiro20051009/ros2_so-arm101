# 実機確認手順: サーボキャリブ（段階2c）— C0・C0.5

対象: `so_arm101_calib`（`so_arm101_calibration` パッケージ）。計画は `docs/stage2c_plan.md`。
**実行するのはユーザーだけ**（Claude は実機を動かさない。EEPROM の読み出しもシリアルポートを開くので、実行しない）。
この版は C0（バックアップ）と C0.5（復元経路と待ち時間の実機確認）まで。C1 以降（`measure` / `apply`）は段階2c-2 で追記する。

## 共通の約束
1. **ros2_control（launch）を止めてから使う**（シリアルポートを占有する）。実行前に確認:
   ```bash
   ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"
   ros2 node list
   ```
   どちらも空であること。`ros2 node list` だけに残り `ps` が空なら、`ros2 daemon stop; ros2 daemon start` の後にもう一度確認する。
2. 腕は出発点の姿勢で置き、トルクは OFF のまま（この段階の手順はトルクに触れない）。`so_arm101_calib` はトルクが入っていると何も書かずに止まる（終了コード 3）。
3. 起動はすべて新しい端末で、conda を activate していない状態で。ログは `tee -i ~/so_arm101_check/<名前>.log` で取る。
4. EEPROM に書くのは **Homing_Offset・Min/Max_Position_Limit の 3 項目だけ**。トルク（Torque_Enable）には書かない。書き込みの前後で Lock を 0 → 1 にする。
5. 書き込みを伴うコマンドは、`--write` と確認文字列 `WRITE-EEPROM` の手入力の両方が必要。どちらかが無ければ何も書かない。
6. 想定外の表示が出たら、そこで止めて、ログを貼って報告する。次の段階には進まない。

### 終了コード
| 値 | 意味 |
|---|---|
| 0 | 成功（dry-run を含む） |
| 1 | 失敗したが、全関節が実行前の値で Lock=1 に戻っている |
| 2 | 失敗し、一部が実行前でも目標でもない／目標のまま残った、または Lock=1 を確認できない（表に旧/新/不明が出る。`restore` のコマンド例が出る） |
| 3 | 前提の失敗（ポートを開けない、トルクが入っている、機種番号が違う、読めない）。何も書いていない |
| 4 | 入力・検査の不合格（ファイルが壊れている、確認文字列の不一致など）。何も書いていない |

## C0: バックアップを取る（読み取り専用。何も書かない）
```bash
mkdir -p ~/so_arm101_check
ros2 run so_arm101_calibration so_arm101_calib backup --port /dev/ttyACM0 \
  2>&1 | tee -i ~/so_arm101_check/calib_C0_backup.log
```
- 全関節の EEPROM（アドレス 0〜39）を 2 回読んで一致を確認し、`~/so_arm101_check/eeprom_backup_<日時>.json` に保存し、読み直して一致を確認する。
- 合格基準:
  - 終了コード 0。各関節の行に「2 回読んで一致」が出る。`保存して読み直し、一致を確認した:` が出る。
  - `Homing_Offset` と Min/Max が、現在の yaml（`so_arm101_hardware/config/follower_calibration.yaml` の `eeprom.*`）と一致する:
    pan 135 / 743 / 3452、lift 112 / 842 / 3216、elbow 122 / 862 / 3088、wrist_flex 89 / 889 / 3185、wrist_roll 1164 / 164 / 3965、gripper 107 / 1900 / 3385。
- **EEPROM の全 40 バイトを 8 バイトずつ読む**ことが実機で通るかは未確認。ERR 応答などで読めない場合は、そのメッセージを報告する（書き込みは起きていない）。
- 続けて、復元の dry-run（何も書かない）で「全部同じ」になることを確認する:
  ```bash
  ros2 run so_arm101_calibration so_arm101_calib restore --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
    --port /dev/ttyACM0 2>&1 | tee -i ~/so_arm101_check/calib_C0_dryrun.log
  ```
  `== 前提の確認（全関節）` の各行が `OK`（機種番号 777、Torque_Enable=0）で、全レジスタが「同じ（書かない）」、`dry-run: 何も書いていない`、終了コード 0。
- バックアップのファイルは `~/so_arm101_check/` に残し、**消さない**（以降の復元の元になる）。
- 報告してほしいもの: `calib_C0_backup.log`、`calib_C0_dryrun.log`、バックアップのファイル名。

## C0.5: 復元の経路と待ち時間の実機確認（最初の EEPROM 書き込み）
**ここが実機で初めて EEPROM に書く操作。実行する前に、Claude に「これから C0.5 を実行する」と伝え、承認を得てから行う。**
C0 が合格し、バックアップがあることが前提。対象は **gripper 1 関節だけ**で、書く値は**バックアップと同じ値**（値は変わらない）。

目的: (1) 書き込み → 待ち → 読み戻し → Lock=1 の経路が実機で動くことの確認、(2) `--eeprom-wait-ms` の実測。

### 手順
1. dry-run（何も書かない。`--rewrite-same` で「書き直す予定」が表示される）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib restore --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
     --joints gripper --rewrite-same --port /dev/ttyACM0 2>&1 | tee -i ~/so_arm101_check/calib_C05_dryrun.log
   ```
2. 書き込み（待ち 50 ms。確認文字列を求められたら `WRITE-EEPROM` と入力）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib restore --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
     --joints gripper --rewrite-same --write --eeprom-wait-ms 50 --port /dev/ttyACM0 \
     2>&1 | tee -i ~/so_arm101_check/calib_C05_w50.log
   ```
3. 待ち時間を変えて繰り返す: `--eeprom-wait-ms` を **0、5、10、20** にして、ログ名を `calib_C05_w0.log` などにして同じコマンドを実行する（各回、終了コードを `echo "rc=$?"` で記録する）。
4. 最後にもう一度 C0 の dry-run（`--joints` なし、`--rewrite-same` なし）で、全部同じで、`注意: ... アドレス n がバックアップと違う` が出ないこと。

### 合格基準
- 手順 2（待ち 50 ms）が終了コード 0 で、`OK: バックアップの値に戻り、Lock=1 を確認した`、`gripper Lock: 1`。
- 手順 3 の各回: 終了コード 0 なら読み戻しが一致した。**`--eeprom-wait-ms 0` などで読み戻しが一致せず失敗した場合は、失敗として扱われ（`read-back mismatch`）、Lock=1 に戻す処理が走り、終了コードは 1（または一部が残れば 2）になる。これは想定どおりで、不合格ではない**。そのときの待ち時間が「短すぎる」という測定結果として記録する。値の不一致は再試行しない（通信エラーだけ 3 回まで再試行する）。
- 失敗した回の後も、最後の C0 dry-run で全レジスタがバックアップと同じ（gripper が「実行前」＝バックアップの値のまま）であること。
- 全回を通して、`Torque_Enable` への書き込みが無い（コマンドはトルクに触れない）。腕は動かない。
- 同じ値の書き込みでは「値が変わった」ことは待ち時間では確かめられない。この段階で見るのは、終了コードと、`ERROR` が出ないこと（通信エラーは内部で 3 回まで再試行され、表示はされない）。値が変わる状況での確認は C2（gripper に新しい値を書く）で、各項目の読み戻しの一致として記録する。
- 決め方: 全回で一致した最小の待ち時間の 2〜3 倍を、`--eeprom-wait-ms` の既定（今は仮置きの 50 ms）にする（コードの定数を直す。Claude が行う）。

### 失敗したとき
- 終了コード 2 が出たら、表示された `so_arm101_calib restore --backup … --write` で戻す。`Lock=1 を確認できない` と出たら、**サーボの電源を入れ直す**（Lock は電源投入時に 1 になる。実機で確認済み）。入れ直したら、dry-run で状態を確認する。
- どの場合も、書き込みモード（`read_only:=false`）の launch は起動しない。起動しても、EEPROM と yaml が違えば起動を拒否する（段階2c-1a）が、まず復元する。

### 報告してほしいもの
- `calib_C05_dryrun.log`、`calib_C05_w50.log`、`calib_C05_w0.log`、`w5`、`w10`、`w20` の各ログと終了コード、最後の C0 dry-run のログ。
- 待ち時間を変えたとき、読み戻しが一致しなかった回があればその待ち時間。

## 注意（どの手順でも）
- `so_arm101_calib` を ros2_control と同時に動かさない。止め方: launch は Ctrl-C だけ（`kill -9` は使わない）。
- EEPROM は電源を切っても残る。書く前に必ずバックアップがあることを確認する。
