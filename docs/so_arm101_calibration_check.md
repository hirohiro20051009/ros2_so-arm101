# 実機確認手順: サーボキャリブ（段階2c）— C0・C0.5

対象: `so_arm101_calib`（`so_arm101_calibration` パッケージ）。計画は `docs/stage2c_plan.md`。
**実行するのはユーザーだけ**（Claude は実機を動かさない。EEPROM の読み出しもシリアルポートを開くので、実行しない）。
C0（バックアップ）→ C0.5（復元経路と待ち時間）→ C1（測定 2 回）→ 判断 → C2（gripper に適用して戻す）→ C3（全関節に適用）。C4（書き込みモードでの W 系の再確認）は C3 合格後に、`feetech_write_check.md` を直してから行う。**前の段階が合格するまで次へ進まない。**

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
  2>&1 | tee -i ~/so_arm101_check/calib_C0_backup.log; echo "exit=${PIPESTATUS[0]}"
```
- 全関節の EEPROM（アドレス 0〜39）を 2 回読んで一致を確認し、`~/so_arm101_check/eeprom_backup_<日時>.json` に保存し、読み直して一致を確認する。
- 合格基準:
  - 終了コード 0。各関節の行に「2 回読んで一致」が出る。`保存して読み直し、一致を確認した:` が出る。
  - `Homing_Offset` と Min/Max が、現在の yaml（`so_arm101_hardware/config/follower_calibration.yaml` の `eeprom.*`）と一致する:
    pan 135 / 743 / 3452、lift 112 / 842 / 3216、elbow 122 / 862 / 3088、wrist_flex 89 / 889 / 3185、wrist_roll 1164 / 164 / 3965、gripper 107 / 1900 / 3385。
- 終了コードは、パイプの後に `; echo "exit=${PIPESTATUS[0]}"` を付けて取る（`tee` につなぐと `$?` は `tee` の値になるため）。**`PIPESTATUS` は、パイプを実行した直後の同じ行で読む**（間に別のコマンドを挟まない）。
- EEPROM の全 40 バイトを 8 バイトずつ読むことは、実機で通った（2026-11-11 の C0。合格）。
- 続けて、復元の dry-run（何も書かない）で「全部同じ」になることを確認する:
  ```bash
  ros2 run so_arm101_calibration so_arm101_calib restore --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
    --port /dev/ttyACM0 2>&1 | tee -i ~/so_arm101_check/calib_C0_dryrun.log; echo "exit=${PIPESTATUS[0]}"
  ```
  `== 前提の確認（全関節）` の各行が `OK`（機種番号 777、Torque_Enable=0）で、全レジスタが「同じ（書かない）」、`dry-run: 何も書いていない`、終了コード 0。
- バックアップのファイルは `~/so_arm101_check/` に残し、**消さない**（以降の復元の元になる）。
- 報告してほしいもの: `calib_C0_backup.log`、`calib_C0_dryrun.log`、バックアップのファイル名。

## C0.5: 復元の経路と待ち時間の実機確認（最初の EEPROM 書き込み）
**ここが実機で初めて EEPROM に書く操作。実行する前に、Claude に「これから C0.5 を実行する」と伝え、承認を得てから行う。**
C0 が合格し、バックアップがあることが前提。対象は **gripper 1 関節だけ**で、書く値は**バックアップと同じ値**（値は変わらない）。

目的: (1) 書き込み → 待ち → 読み戻し → Lock=1 の経路が実機で動くことの確認、(2) `--eeprom-wait-ms` の実測。

### 手順
`<日時>` は `20261111-163729`（バックアップのファイル名）。終了コードは、各コマンドの後ろの `; echo "exit=${PIPESTATUS[0]}"` で取る（`tee` につないでいるので `$?` は使わない）。
1. dry-run（何も書かない。`--rewrite-same` で「書き直す予定」が表示される）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib restore --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
     --joints gripper --rewrite-same --port /dev/ttyACM0 2>&1 | tee -i ~/so_arm101_check/calib_C05_dryrun.log; echo "exit=${PIPESTATUS[0]}"
   ```
2. 待ち時間を **50 → 20 → 10 → 5 → 0（ms）の順**に、1 回ずつ書き込む。各回は次の 2 つのコマンドで 1 組（`<W>` は待ち時間）:
   - (a) 書き込み（確認文字列を求められたら `WRITE-EEPROM` と入力）。ログは `calib_C05_w<W>.log`:
     ```bash
     ros2 run so_arm101_calibration so_arm101_calib restore --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
       --joints gripper --rewrite-same --write --eeprom-wait-ms <W> --port /dev/ttyACM0 \
       2>&1 | tee -i ~/so_arm101_check/calib_C05_w<W>.log; echo "exit=${PIPESTATUS[0]}"
     ```
   - (b) C0 の dry-run（`--joints` なし、`--rewrite-same` なし）。ログは `calib_C05_after_w<W>.log`:
     ```bash
     ros2 run so_arm101_calibration so_arm101_calib restore --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
       --port /dev/ttyACM0 2>&1 | tee -i ~/so_arm101_check/calib_C05_after_w<W>.log; echo "exit=${PIPESTATUS[0]}"
     ```
   - **次の待ち時間へ進んでよいのは、(a) が exit=0 で、(b) が全レジスタ「同じ（書かない）」・`注意: ... アドレス n がバックアップと違う` なし・exit=0 のとき だけ。**
3. **ある待ち時間で失敗したら（(a) が exit=0 でない、または (b) が上の条件を満たさない）、そこで止める。それより短い待ち時間は試さない。** 失敗した値は「短すぎる」という測定結果として記録する。その後は「失敗したとき」に従って状態を確認し、必要なら `--eeprom-wait-ms 50` で `restore` して戻す（これは C0.5 の追加の書き込みなので、実行前に Claude に伝える）。
4. 5 つ全部（50・20・10・5・0）が通ったら、最後に (b) をもう一度行う（ログ名 `calib_C05_final.log`）。

### 合格基準
- 待ち 50 ms の回が終了コード 0 で、`OK: バックアップの値に戻り、Lock=1 を確認した`、`gripper Lock: 1`。これが通らなければ（`restore` 自体の確認なので）以降へ進まず報告する。
- 各回の (a): 終了コード 0 なら読み戻しが一致した。**短い待ち時間で失敗した場合（`read-back mismatch` または `read-back failed`）は、その値が「短すぎる」という測定結果で、不合格ではない**。ただし、そこで止める（手順 3）。値の不一致は再試行しない（通信エラーだけ 3 回まで再試行する）。
- 失敗時の動き（コードで確認）: `restore` には「バックアップへ戻す」2 回目の書き込みは無い（`apply` にだけある）。失敗したら残りの関節へ進まず、**待ちなしで** Lock=1 を書いて読み戻し、全レジスタの状態を読んで表示する（`src/restore.cpp`、`src/eeprom_ops.cpp`）。待ち時間の指定は、各項目の書き込みと読み戻しの間にだけ効く。終了コードは、Lock=1 を確認でき、全レジスタが読めて「実行前」と同じなら 1。**Lock=1 を確認できない、またはレジスタが読めない（読み戻しが失敗するほど短いとき）と 2 になりうる**。2 のときは「失敗したとき」に従う（サーボの電源を入れ直す）。
- どの回の後も、(b) で全レジスタがバックアップと同じ（gripper が「実行前」＝バックアップの値のまま）であること。
- 全回を通して、`Torque_Enable` への書き込みが無い（コマンドはトルクに触れない）。腕は動かない。
- 同じ値の書き込みでは「値が変わった」ことは待ち時間では確かめられない。この段階で見るのは、終了コードと、`ERROR` が出ないこと（通信エラーは内部で 3 回まで再試行され、表示はされない）。値が変わる状況での確認は C2（gripper に新しい値を書く）で、各項目の読み戻しの一致として記録する。
- 決め方: **この試験では待ち時間の必要量は決められない**（同じ値の書き込みは、読み戻しが書く前から一致するため）。`--eeprom-wait-ms` の既定は 50 ms のままにする（定数は変えない）。値が変わる書き込みは C2 で 50 ms で行い、各項目の読み戻しの一致を記録する。0 ms で通ったのは「経路が実機で動き、失敗しなかった」ことだけを意味する。

### 失敗したとき
- 終了コード 2 が出たら、表示された `so_arm101_calib restore --backup … --write` で戻す。`Lock=1 を確認できない` と出たら、**サーボの電源を入れ直す**（Lock は電源投入時に 1 になる。実機で確認済み）。入れ直したら、dry-run で状態を確認する。
- どの場合も、書き込みモード（`read_only:=false`）の launch は起動しない。起動しても、EEPROM と yaml が違えば起動を拒否する（段階2c-1a）が、まず復元する。

### 報告してほしいもの
- `calib_C05_dryrun.log`、実行した回の `calib_C05_w<W>.log` と `calib_C05_after_w<W>.log`（W は 50・20・10・5・0 のうち実行したもの）、`calib_C05_final.log`（全部通ったとき）、各ログの `exit=` の値。
- 失敗して止めた場合は、その待ち時間（それより短い値は試していないこと）。

## C1: 測定を 2 回（EEPROM には書かない。Lock にも触れない）
`measure` が書くのは **`Torque_Enable=0` だけ**（トルクが入っている関節があるときだけ。腕が脱力して落ちるので、**腕を手で支えてから** Enter を押す）。EEPROM と Lock は書かない。
```bash
ros2 run so_arm101_calibration so_arm101_calib measure --port /dev/ttyACM0 \
  --backup ~/so_arm101_check/eeprom_backup_<日時>.json --out ~/so_arm101_check/measure_C1_1.json \
  2>&1 | tee -i ~/so_arm101_check/calib_C1_1.log; echo "exit=${PIPESTATUS[0]}"
```
`<日時>` は `20261111-163729`。C0.5 のあとはトルクは OFF のはずで、その場合は何も書かずに進む（`Torque_Enable=0` の書き込みは起きない）。
画面の指示に従う（**Enter はトルクが OFF なら 3 回（中央・記録開始・記録終了）、トルクが入っていれば 4 回**。余計な Enter は押さない。記録中の Enter は、どれも「記録を終える」になる）:
1. （トルクが入っているとき）腕を支えて Enter → 脱力。
2. **中央の姿勢**（各関節の可動範囲のほぼ中央。LeRobot のキャリブと同じ「真ん中」）にして、動かさずに Enter。Enter の後、約 0.2〜0.3 秒（20 ms 間隔で 11 回）読み、各関節の揺れが 4 tick（約 0.35°）以内であること（超えると `ERROR: … 中央の測定中に動いた` で止まる。何も書かれていない。やり直す）。**Enter を押す手で姿勢を崩さない**。腕を支えているときは、Enter の後に 1〜2 秒は動かさない（片手で支えたまま押すか、自重で崩れないなら手を離す）。この Enter を押した後に、次の「記録開始」の指示が出るまでは動かしてよい。
3. 「記録開始」の指示で Enter を押すと記録が始まる（`.` が出る。1 秒に約 2 個）。**全関節（gripper と wrist_roll を含む）を端から端まで**、順に動かす。wrist_roll は約 3801 tick（ほぼ 1 周）。**中央の姿勢から ±約 180° を超えて回さない**（超えると範囲が判定できず、検査で NG になる）。終わったら Enter。**途中で Enter や Ctrl-C を押すと、そこで記録が終わる／中止する**（Ctrl-C では何も保存されない）。各関節は、機械的に止まる端で止める（無理に押し込まない）。脱力しているので、動かしていない関節は自重で垂れる（動かし終わった関節も、垂れて範囲が増えることはあるが、それは実際の可動の範囲内）。
- 表（center / measured / new_h / min / max / span / samples / 検査）が出る。**検査がすべて OK**で、測定ファイルが保存される。NG の関節があるときは、保存はされるが `apply` は受け付けない（理由が出る。やり直す）。
- 同じ手順で 2 回目: `--out ~/so_arm101_check/measure_C1_2.json`、ログ `calib_C1_2.log`（コマンドの末尾の `echo "exit=…"` も同じ）。**中央の姿勢はできるだけ同じ**にする。
- 比較（ポートは開かない）:
  ```bash
  ros2 run so_arm101_calibration so_arm101_calib compare \
    --a ~/so_arm101_check/measure_C1_1.json --b ~/so_arm101_check/measure_C1_2.json \
    2>&1 | tee -i ~/so_arm101_check/calib_C1_compare.log; echo "exit=${PIPESTATUS[0]}"
  ```
  - 「再現性」: 関節ごとの 2 回の差（tick）と、最大の差。
  - 「今の yaml との差」（A と B のそれぞれ）: min / max / center の差（tick と rad）、Homing_Offset の差、**`zero_shift`（0 rad の物理位置の移動量。tick と rad）**。`zero_shift` が 0 に近いほど、今の URDF・出発点の姿勢・SRDF の数値の意味が変わらない。
- 検査に落ちる関節があると、測定ファイルは保存されるが終了コードは 4 になる。やり直すときは `--out` を変える（上書きしない）。
- 合格基準（C1 自体）: 2 回とも終了コード 0、全関節の検査が OK、トルク以外に書いていない（腕が脱力する以外に変化がない）。再現性と yaml との差には**合否の基準を置かない**（次の判断で人が見る）。
- 報告してほしいもの: `calib_C1_1.log`、`calib_C1_2.log`、`calib_C1_compare.log`、2 つの測定ファイル。

## 判断の区切り（C1 の後。C2 には自動では進まない）
`calib_C1_compare.log` を見て、ユーザーが決める:
1. **新しい値を書く**（どの測定ファイルを使うかを選ぶ。以降の C2・C3 は同じファイルを使う）
2. **今の値をそのまま書き直す**（測定ファイルは使わない。C0.5 と同じ経路の確認だけ。EEPROM は変わらない）
3. **書かない**（キャリブは今のまま。2c の残りは LeRobot 経路の archive だけ）
- 見る点: 再現性（2 回の差が大きい関節はないか）、`zero_shift`（0 rad がどれだけ動くか）、wrist_roll の範囲、検査の結果。

## C2: gripper だけに新しい値を書く（yaml は更新しない）→ バックアップへ戻す
**EEPROM に新しい値を書く最初の操作。実行する前に Claude に伝えて承認を得る。** C0.5 が合格していること。
1. dry-run（何も書かない。書く予定の値が出る）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib apply --port /dev/ttyACM0 \
     --measure ~/so_arm101_check/<選んだ測定ファイル>.json --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
     --joints gripper 2>&1 | tee -i ~/so_arm101_check/calib_C2_dryrun.log
   ```
   `今の EEPROM とバックアップの照合` が `OK 全関節がバックアップと一致`、`書き込む値` の gripper の Homing_Offset / Min / Max が測定ファイルの値であること。
2. 書き込み（`--eeprom-wait-ms` は C0.5 で決めた値）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib apply --port /dev/ttyACM0 \
     --measure ~/so_arm101_check/<選んだ測定ファイル>.json --backup ~/so_arm101_check/eeprom_backup_<日時>.json \
     --joints gripper --write --eeprom-wait-ms <C0.5 で決めた値> 2>&1 | tee -i ~/so_arm101_check/calib_C2_apply.log
   ```
   `OK: 測定ファイルの値を書き、読み戻しと Lock=1 を確認した`、`yaml は更新していない（一部の関節だけ）`、終了コード 0。
3. **サーボの電源を入れ直す**（外部電源を切って入れる）。値が残ることを確認（読み取り専用）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib backup --port /dev/ttyACM0 \
     --out ~/so_arm101_check/eeprom_after_C2.json 2>&1 | tee -i ~/so_arm101_check/calib_C2_after.log
   ```
   gripper の Homing_Offset / Min / Max が新しい値のまま、他の関節は変わっていないこと。
4. **バックアップへ戻す**:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib restore --port /dev/ttyACM0 \
     --backup ~/so_arm101_check/eeprom_backup_<日時>.json --joints gripper --write \
     --eeprom-wait-ms <C0.5 で決めた値> 2>&1 | tee -i ~/so_arm101_check/calib_C2_restore.log
   ```
   `OK: バックアップの値に戻り、Lock=1 を確認した`。続けて同じ `restore`（`--write` なし）の dry-run で、全部「同じ（書かない）」。
- 合格基準: 手順 2 と 4 が終了コード 0。手順 3 で電源を入れ直した後も値が残る。手順 4 の後、全関節がバックアップと一致。
- 失敗したとき: `apply` は書いた関節をバックアップへ自動で戻そうとする。表示される `旧 / 新 / 不明` と終了コード（上の表）を報告する。終了コード 2 は、表示された `restore --backup … --write` で戻す。**この段階で失敗したら、C3 には進まない。**
- 報告してほしいもの: `calib_C2_dryrun.log`、`calib_C2_apply.log`、`calib_C2_after.log`、`calib_C2_restore.log`。

## C3: 全関節（gripper を含む 6 関節）を 1 回で書く（yaml はそのとき 1 回だけ更新）
**実行する前に Claude に伝えて承認を得る。** C2 が合格していること。**C2 と同じ測定ファイル**を使う。
1. dry-run: C2 の手順 1 から `--joints gripper` を外す。`全関節を 1 回で書く。成功したら yaml を 1 回だけ更新する: <yaml のパス>` と出ること（パスは launch が読む `…/share/so_arm101_hardware/config/follower_calibration.yaml`。`colcon build --symlink-install` ならソースの yaml に解決される）。
2. 書き込み: C2 の手順 2 から `--joints gripper` を外し、ログ名を `calib_C3_apply.log` に。成功すると、`yaml を更新した:` が出る。終了コード 0。
3. **サーボの電源を入れ直す。** 読み取り専用で起動して診断を確認する（腕は触らない。トルク OFF のまま）:
   ```bash
   ros2 launch so_arm101_bringup bringup.launch.py mock:=false use_rviz:=false 2>&1 | tee -i ~/so_arm101_check/calib_C3_readonly.log
   ```
   `[diag] EEPROM vs yaml: 6/6 joints match`、`servo settings vs LeRobot: 6/6 joints match`、全関節 `torque_enable=0`。（EEPROM と yaml が違うと、読み取り専用では WARN のまま起動し、書き込みモードは起動を拒否する。）
4. 同じ起動のまま、腕を**出発点の姿勢**（トルク OFF で手を離しても止まる姿勢。手で作る）にして、新しい座標での値を読む:
   ```bash
   ros2 topic echo /joint_states --once 2>&1 | tee -i ~/so_arm101_check/calib_C3_start_pose.log
   ```
   停止は Ctrl-C だけ。後始末（ps と `ros2 node list`。残っていたら `ros2 daemon stop; ros2 daemon start`）。
5. **URDF の関節リミットとの余裕**（オフライン。URDF・SRDF は読むだけ）:
   ```bash
   python3 tools/pose_margin.py --pose start=<pan>,<lift>,<elbow>,<wrist_flex>,<wrist_roll>,<gripper> \
     2>&1 | tee -i ~/so_arm101_check/calib_C3_margin.log
   ```
   （値は手順 4 の `position`。順序は pan, lift, elbow, wrist_flex, wrist_roll, gripper。必要なら `--pose home=…` を足す。）校正した範囲の両端と URDF の範囲の差、姿勢ごとの URDF の上限・下限までの残りが出る。範囲の外（OUT）や余裕が小さい（LOW。仮の目安 0.05 rad）関節があれば、**C4 の前に Claude へ報告して判断を仰ぐ**（URDF・SRDF は変えない）。
- 合格基準: 手順 2 が終了コード 0、手順 3 が 6/6 MATCH（2 行とも）、手順 4・5 のログが取れている。
- 報告してほしいもの: `calib_C3_apply.log`、`calib_C3_readonly.log`、`calib_C3_start_pose.log`、`calib_C3_margin.log`。Claude が yaml の差分を確認して commit し、手順書の出発点の姿勢などを直す（C4 の前）。

## 失敗したときの共通の対処
- `apply` が終了コード 1: 全関節が実行前の値（バックアップ）に戻り、Lock=1 を確認した。原因（表示された `read-back mismatch` など）を報告する。
- 終了コード 2: 一部が新・不明のまま、または Lock=1 を確認できない。表の `旧 / 新 / 不明` を報告し、表示された `restore --backup … --write` で戻す（ros2_control は止めたまま）。`Lock=1 を確認できない` と出たら**サーボの電源を入れ直す**（Lock は電源投入時に 1 になる）。電源を入れ直したら `restore` の dry-run で状態を確認する。
- yaml が古いまま EEPROM が新しい（`yaml を更新できなかった`）: 書き込みモードの launch は起動を拒否する。退避先に出た yaml を正しい場所へ置くか、`restore` で戻す。
- どの場合も、書き込みモード（`read_only:=false`）の launch は、`restore` で戻すか C3 が合格するまで起動しない。

## 注意（どの手順でも）
- `so_arm101_calib` を ros2_control と同時に動かさない。止め方: launch は Ctrl-C だけ（`kill -9` は使わない）。
- EEPROM は電源を切っても残る。書く前に必ずバックアップがあることを確認する。
