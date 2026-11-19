# 別の PC・別の腕の校正（遠隔作業者用のコマンド一覧）

読み手: 遠隔地で、そのままコピーして順に実行する作業者。端末の画面は、依頼者から見えない。
出どころ: `docs/so_arm101_calibration_check.md`（C0〜C3）、`docs/stage2c_plan.md`、`so_arm101_calibration` のソース（`src/so_arm101_calibration/src/main.cpp` ほか）。

## 前提と、この手順の約束
- **別の PC で、別の腕**を校正する。腕の EEPROM には **LeRobot の校正値**が入っている。それを、このリポジトリの校正ツール（`so_arm101_calib`）で校正し直す。
- **git 管理のファイル（`src/` の yaml を含む）は、この手順では一切変えない。** yaml は、その腕専用のコピー（`$D/follower_calibration_new_arm.yaml`）を作って、そのコピーだけを書き換える。
- **リポジトリの yaml・手順書（`docs/so_arm101_calibration_check.md` など）にある、開発用の腕の値は、合否に使わない。** 次のものが該当する: `pan 135 / 743 / 3452…` などの EEPROM の期待値、測定中の表の「あと N」（前回の端まで）、`compare` の「今の yaml との差」「zero_shift」の表。別の腕では違って当然。
- 実行するのは作業者だけ。**止め方は Ctrl-C だけ**（`kill -9`・`pkill`・`killall` は使わない）。`ros2 daemon` は触らない（stop も start もしない）。`ros2 node list` には必ず `--no-daemon` を付ける。
- 想定外の表示が出たら、そこで止めて、ログを貼って報告する。次の手順には進まない。
- コマンドの `exit=` は、パイプ（`| tee`）の直後の同じ行で読む。`$?` は使わない（`tee` の値になる）。
- 端末は**新しい端末で、conda を activate していない状態**で。新しい端末を開くたびに、0 章の「毎回の変数」を実行する（`cp` はもう実行しない）。

### 終わった後の連絡事項（依頼者へ）
次をすべて送る:
1. 各コマンドの **`exit=` の値**: backup、restore の dry-run、measure（1 回目・2 回目）、compare、apply の dry-run、apply の書き込み。
2. **ログのファイル名**（すべて `$D` = `~/so_arm101_check/` の中）: `calib_new_backup.log`、`calib_new_restore_dry.log`、`calib_new_measure_1.log`、`calib_new_measure_2.log`、`calib_new_compare.log`、`calib_new_apply_dry.log`、`calib_new_apply.log`、`calib_new_readonly.log`。
3. **`compare` の「最大の差」の行**（`calib_new_compare.log` にある）。
4. **起動の `[diag]` の行**（8 章の `grep` の出力そのまま。`servo settings vs reference` の行も含む）。
5. ファイル 4 つ: `eeprom_backup_lerobot_orig.json`、`measure_new_1.json`、`measure_new_2.json`、`follower_calibration_new_arm.yaml`。
6. 手順の途中で気づいたこと（押し切れたか、表示の見にくさ、想定外の表示）。

## 0. 変数と、この腕専用の yaml のコピー
```bash
export WS=<ワークスペースのパス 例: ~/ros2_so-arm101>
export PORT=<シリアルポート 例: /dev/ttyACM0>
export D=~/so_arm101_check
mkdir -p $D
```
- `--port` の既定は /dev/ttyACM0（`main.cpp:240`）。

**コピーが既にないかを確認する（単独で実行する）:**
```bash
ls $D/follower_calibration_new_arm.yaml
```
- `No such file or directory` と出たら（まだ無い）→ 次の `cp` へ進む。
- ファイルの名前がそのまま出たら（既にある）→ **ここで止めて報告する。上書きしない。**

**無いことを確認してから、コピーを作る:**
```bash
cp $WS/src/so_arm101_hardware/config/follower_calibration.yaml $D/follower_calibration_new_arm.yaml
export YAML=$D/follower_calibration_new_arm.yaml
ls -l $YAML
```
- 以後、**すべてのコマンドの `--calibration` と、launch の `calibration_file:=` に `$YAML`（コピー）を渡す。**
- `--calibration` を付けないと、`so_arm101_calib` は install 側の yaml（リポジトリの yaml のコピーまたはリンク。`main.cpp:148-150`）を読み、`apply` はそれを書き換える。**付け忘れない。**

**毎回の変数（新しい端末を開いたとき。`cp` は実行しない）:**
```bash
export WS=<上と同じ>
export PORT=<上と同じ>
export D=~/so_arm101_check
export YAML=$D/follower_calibration_new_arm.yaml
```

## 1. 事前確認（読むだけ）
```bash
cd $WS && source /opt/ros/humble/setup.bash && source install/setup.bash
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
ls -l /dev/ttyACM* /dev/serial/by-id/ 2>&1
id -nG | tr ' ' '\n' | grep -x dialout
echo "conda: ${CONDA_DEFAULT_ENV:-none}"
ros2 pkg executables so_arm101_calibration
```
- `ps` と `ros2 node list --no-daemon` が**どちらも空**であること。`ros2 node list` だけに残り `ps` が空なら、数秒待ってもう一度確認する。それでも残るなら報告して待つ。
- `ttyACM*` が無い、`dialout` が出ない、`conda: none` 以外、`ros2 pkg executables` が空、のどれかなら、先へ進まず報告する。
- 出どころ: `docs/so_arm101_calibration_check.md:10-17`（共通の約束 1〜3）。

## 2. backup（読み取り専用。腕はトルク OFF）
バックアップのファイルは `$D/eeprom_backup_lerobot_orig.json`（既定の保存先は `~/so_arm101_check/eeprom_backup_<日時>.json`。`main.cpp:273-277`）。
`backup` は、同じ名前のファイルがあると**確認なしで上書きする**（`eeprom_backup.cpp:185`）。次の 2 段で進める。

**(a) 単独で確認する:**
```bash
ls $D/eeprom_backup_lerobot_orig.json
```
- `No such file or directory` → (b) へ進む。
- 名前が出た（既にある）→ **止めて報告する。上書きしない。**

**(b) backup を実行する（(a) が「No such file」だったときだけ）:**
```bash
ros2 run so_arm101_calibration so_arm101_calib backup --port $PORT --calibration $YAML \
  --out $D/eeprom_backup_lerobot_orig.json 2>&1 | tee -i $D/calib_new_backup.log; echo "exit=${PIPESTATUS[0]}"
```
- 合格: `exit=0`。各関節に「2 回読んで一致」。最後に「保存して読み直し、一致を確認した」。
- **このファイルは消さない。** 9 章の `restore` の元になる。

続けて、読み戻しの確認（何も書かない）:
```bash
ros2 run so_arm101_calibration so_arm101_calib restore --backup $D/eeprom_backup_lerobot_orig.json \
  --port $PORT --calibration $YAML 2>&1 | tee -i $D/calib_new_restore_dry.log; echo "exit=${PIPESTATUS[0]}"
```
- 合格: `exit=0`、全レジスタ「同じ（書かない）」、`dry-run: 何も書いていない`（`restore.cpp:97,112`）。

## 3. measure（2 回。EEPROM には書かない）
```bash
ros2 run so_arm101_calibration so_arm101_calib measure --port $PORT --calibration $YAML \
  --backup $D/eeprom_backup_lerobot_orig.json --out $D/measure_new_1.json \
  2>&1 | tee -i $D/calib_new_measure_1.log; echo "exit=${PIPESTATUS[0]}"
```
2 回目は、`measure_new_1.json` を `measure_new_2.json` に、`calib_new_measure_1.log` を `calib_new_measure_2.log` に替えて、同じコマンドを実行する。
**端末の幅は 80 桁以上にして、端末から直接実行する**（記録中の表が `/dev/tty` に出る）。

### 腕の動かし方（表示は `measure.cpp:377-447` のとおり）
1. トルクが入っていたときだけ、最初に「腕を手で支えて Enter」と出る。**腕を支えてから** Enter を押す（Enter は合計 3 回）。トルク OFF なら出ない（Enter は 2 回）。
2. 「腕を『中央』の姿勢にして、動かさずに Enter」と出る。各関節を可動範囲のほぼ中央にして、手を離してから Enter を押す。
3. 「中央を測定中（約 0.3 秒。動かさない）」の間は動かさない。4 tick（約 0.35°）以上動くと `ERROR … 中央の測定中に動いた`（exit=4）で止まる。何も書かれていない。やり直す。
4. 「**中央の測定 OK、記録中。動かし始めてよい**」と出てから、全関節（gripper と wrist_roll を含む）を端から端まで押し切る。
   - 機械的に止まる端で止める。無理に押し込まない。
   - wrist_roll は中央から ±約 180° を超えて回さない（超えると検査 NG）。
   - 動かしていない関節は自重で垂れる。
5. 全部の端に届いたら **Enter（2 回目）**で記録を終える。Ctrl-C は何も保存せず中止。
- 記録中の表の「あと N」は、開発用の腕の範囲が基準（`main.cpp:319-322`）。**目安にしない。** 毎回、同じ強さで端まで押し切る。
- 合格: 2 回とも `exit=0`。測定結果の表の「検査」が全関節 `OK`。「測定ファイルを保存して読み直し、一致を確認した」が出る。
- 記録の行で、読み取りの失敗が 0 でない、または間隔が 20 ms から大きく外れるときは報告する。`bus:` の行は `drained_bytes=0 id_mismatch=0 answered_first=0` が健全。
- `exit=4`（検査 NG）の測定ファイルは `apply` が受け付けない（`measure.cpp:563`）。`--out` を変えてやり直す。

## 4. compare（ポートは開かない）
```bash
ros2 run so_arm101_calibration so_arm101_calib compare --calibration $YAML \
  --a $D/measure_new_1.json --b $D/measure_new_2.json 2>&1 | tee -i $D/calib_new_compare.log; echo "exit=${PIPESTATUS[0]}"
```
- 見る行は「最大の差（d_amin / d_amax / d_zero）」（`measure.cpp:618`）。
- **合否の目安: 16 tick（約 1.4°）以内。** 出どころは `docs/so_arm101_calibration_check.md:232-233` の、開発用の腕での前回のばらつき。別の腕に使える保証はないので、目安としてだけ扱う。
- gripper は `d_amin`・`d_amax`・`d_span` だけを見る。`d_zero` と `d_cent` は見ない。
- 16 tick を超えたら、`measure_new_3.json`（ログは `calib_new_measure_3.log`）で測り直し、報告する。**超えたまま `apply` に進まず、依頼者の判断を待つ。**
- 「今の yaml との差」「zero_shift」の表は無視する（比べる相手が開発用の腕の値のため）。

## 5. コピーの zero_offset_deg を 0 にする（apply の前に行う）
`apply` は、yaml の `zero_offset_deg` をそのまま持ち越す（`apply.cpp:85-93`）。先に 0 にしておけば、持ち越しの表示も WARN も出ない。**対象はコピー（`$YAML`）だけ。**
```bash
grep -n "zero_offset_deg" $YAML
sed -i 's/, zero_offset_deg: -6\.3//' $YAML
grep -n "zero_offset_deg" $YAML
```
- 1 回目の `grep` では、コメントの行（17 行目あたり）と、`elbow_flex` の行（`zero_offset_deg: -6.3`）が出るはず。
- 2 回目の `grep` では、**コメントの行だけ**が残るはず。`elbow_flex` の行そのものは消さず、`, zero_offset_deg: -6.3` の部分だけを消す。省略した関節は 0 として読まれる（`docs/vr_follow_quickstart.md:41-43`）。
- 値がこれと違っていたら、手で `zero_offset_deg: 0` にする（またはその項目を消す）。
- この変更は、元の yaml（`src/` 側）には及ばない。

## 6. apply（6 関節を 1 回で書く）
**dry-run（何も書かない）:**
```bash
ros2 run so_arm101_calibration so_arm101_calib apply --port $PORT --calibration $YAML \
  --measure $D/measure_new_1.json --backup $D/eeprom_backup_lerobot_orig.json \
  2>&1 | tee -i $D/calib_new_apply_dry.log; echo "exit=${PIPESTATUS[0]}"
```
- `--measure` に渡す測定ファイルは、`measure_new_1.json` と `measure_new_2.json` のどちらか 1 つ。
- 合格: `OK 全関節がバックアップと一致`。`書き込む値` が 6 関節ぶん（`今` は LeRobot の値）。`全関節を 1 回で書く。成功したら yaml を 1 回だけ更新する: ` の後ろのパスが **`$YAML`（コピー）**であること。`dry-run: 何も書いていない`。`exit=0`（`apply.cpp:245-299`）。
- **パスが `src/` や `install/` の yaml になっていたら、書かずに止めて報告する。**
- `--joints` は付けない（付けると yaml が更新されない）。

**書き込み（確認文字列 `WRITE-EEPROM` を手で入力して Enter）:**
```bash
ros2 run so_arm101_calibration so_arm101_calib apply --port $PORT --calibration $YAML \
  --measure $D/measure_new_1.json --backup $D/eeprom_backup_lerobot_orig.json \
  --write --eeprom-wait-ms 50 2>&1 | tee -i $D/calib_new_apply.log; echo "exit=${PIPESTATUS[0]}"
```
- 確認の行は「EEPROM に書き込む。続けるなら WRITE-EEPROM と入力して Enter:」（`main.cpp:289`）。Enter だけでは何も書かれない。
- 合格: 全関節が `[新]`。各 `Lock: 1`。`OK: 測定ファイルの値を書き、読み戻しと Lock=1 を確認した`。`yaml を更新した: …`。`exit=0`。
- exit が 0 以外なら、その場で止めて**ログ全文を報告**し、launch は起動しない:
  - 1: 全関節が元の値に戻った。
  - 2: 一部が新または不明のまま。表示された `restore --backup … --write` で戻す。`Lock=1 を確認できない` と出たら、サーボの外部電源を入れ直す（USB はつないだまま）。
  - 3: 前提の失敗。何も書いていない。
  - 4: 入力の不合格。何も書いていない。

**書いた後の確認:**
```bash
grep -n "zero_offset_deg\|kind: ros_calibrated\|applied_by" $YAML
git -C $WS status --short
```
- 1 つ目: `zero_offset_deg` の行が出ない（コメントだけ可）。`kind: ros_calibrated`。`applied_by: so_arm101_calib 0.2.0`。
- 2 つ目: **`src/so_arm101_hardware/config/follower_calibration.yaml` が出ない**こと（git 管理のファイルを変えていない確認）。出たら報告する。

## 7. colcon build は不要
**この手順では install の yaml を使わない**（すべてコピーの `$YAML` を `--calibration` と `calibration_file:=` で指定する）ので、`colcon build` は不要。
`so_arm101_calib` 自体がこの PC でビルド済みであること（1 章の `ros2 pkg executables` に出ること）だけが前提。

## 8. 読み取り専用で起動 → [diag] の確認 → 停止
**この腕で起動するときは、毎回 `calibration_file:=<コピーのパス>`（ここでは `calibration_file:=$YAML`）が要る。** 付けないと、リポジトリの yaml（開発用の腕のもの）が使われる。読み取り専用では WARN のまま起動し、書き込みモード（`read_only:=false`）では起動を拒否する。
`read_only` の既定は true（`bringup.launch.py:145`）。トルクは入らず、腕は動かない。

```bash
ros2 launch so_arm101_bringup bringup.launch.py mock:=false use_rviz:=false read_only:=true \
  serial_port:=$PORT calibration_file:=$YAML 2>&1 | tee -i $D/calib_new_readonly.log
```
別の端末で（0 章の「毎回の変数」を実行してから）:
```bash
grep -E "initialized:|\[diag\]|MISMATCH" $D/calib_new_readonly.log
```
- 合格: `[diag] EEPROM vs yaml: 6/6 joints match`（`sts3215_system.cpp:229`）。`MISMATCH` の行なし。全関節 `torque_enable=0`。
- `initialized:` の行に `ros_calibrated` と、6 章の `measure_file=` が出る。
- `servo settings vs reference` の行は、LeRobot 由来の腕でどうなるか未確認。**出た表示をそのまま報告する。**
- **停止は、launch の端末で Ctrl-C を 1 回だけ。** 終わるまで待つ。
- 停止の確認:
```bash
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
```
どちらも空であること。`ros2 node list` だけに残るなら、数秒待ってもう一度確認し、それでも残るなら報告して待つ。

### この後、この腕で他の launch を使うとき（参考）
- **`bringup.launch.py` は `calibration_file:=` を受け取れる**（`bringup.launch.py:92,107,134`）。**毎回付ける。**
- `docs/vr_follow_quickstart.md:54` の実機の起動コマンドには、`calibration_file:=` が書かれていない。この腕では、次のように付け足して実行する:
  ```bash
  ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false torque_limit:=500 calibration_file:=$YAML
  ```
  （書き込みモード。腕が動く。実行する前に依頼者に確認する。）
- `so_arm101_ik` の `vr_follow.launch.py` は、`calibration_file` の引数を持たない。ただし、キャリブレーションの yaml を読まない（`src/so_arm101_ik` に `follower_calibration`・`zero_offset` の参照がない）ので、付ける必要はない。
- `so_arm101_camera` の launch は、`calibration_file` を受け取らない。校正の yaml との照合（sha256）はメタファイル側で行う。この手順では使わない。

## 9. restore（参考。LeRobot の値に戻す）
```bash
# まず dry-run（差を見る）
ros2 run so_arm101_calibration so_arm101_calib restore --backup $D/eeprom_backup_lerobot_orig.json \
  --port $PORT --calibration $YAML 2>&1 | tee -i $D/calib_new_revert_dry.log; echo "exit=${PIPESTATUS[0]}"
# 書き込み（WRITE-EEPROM を手入力）
ros2 run so_arm101_calibration so_arm101_calib restore --backup $D/eeprom_backup_lerobot_orig.json \
  --write --eeprom-wait-ms 50 --port $PORT --calibration $YAML \
  2>&1 | tee -i $D/calib_new_revert.log; echo "exit=${PIPESTATUS[0]}"
```
- `restore` は yaml を更新しない。戻した後、コピーの yaml は新しい値のままなので、EEPROM と食い違う。**戻した後は、このコピーを使って起動しない。**
- `restore` の dry-run が「全レジスタ同じ（書かない）」になるまで、書き込みモード（`read_only:=false`）の launch は起動しない。
- 電源の入れ直しが要るときは、**サーボの外部電源だけ**を入れ直す（USB はつないだまま）。

## 終了コード（`so_arm101_calib`）
| 値 | 意味 |
|---|---|
| 0 | 成功（dry-run を含む） |
| 1 | 失敗したが、全関節が実行前の値で Lock=1 に戻っている |
| 2 | 失敗し、一部が実行前でも目標でもない／目標のまま残った、または Lock=1 を確認できない |
| 3 | 前提の失敗（ポートを開けない、トルクが入っている、機種番号が違う、読めない）。何も書いていない |
| 4 | 入力・検査の不合格（ファイルが壊れている、確認文字列の不一致など）。何も書いていない |
