# 実機確認手順: サーボキャリブ（段階2c）— C0〜C3

対象: `so_arm101_calib`（`so_arm101_calibration` パッケージ）。計画は `docs/stage2c_plan.md`。
**実行するのはユーザーだけ**（Claude は実機を動かさない。EEPROM の読み出しもシリアルポートを開くので、実行しない）。
C0（バックアップ）→ C0.5（復元経路と待ち時間）→ C1（測定 2 回。旧版は実施済み。改善したツールで C1c としてやり直す）→ 判断 → C2（gripper に適用して戻す）→ C3（全関節に適用）。C4（書き込みモードでの W 系の再確認）は C3 合格後に、`feetech_write_check.md` を直してから行う。**前の段階が合格するまで次へ進まない。**

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
| 4 | 入力・検査の不合格（ファイルが壊れている、**`apply` に旧形式（schema_version 1）の測定ファイルを渡した**、確認文字列の不一致など）。何も書いていない |

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

## C1c: 測定を 2 回（改善したツールでの、2 回ともやり直し。EEPROM には書かない。Lock にも触れない）
最初の C1（旧版のツール）は実施済み（2026-11-11）。結果は `docs/inventory.md`。**旧版で測った `measure_C1_1.json`・`measure_C1_2.json` は、書き込みに使わない**（`apply` が「旧形式」として拒否する。`compare` では読める）。
改善したツールでの 1 回目（`measure_C1b_1.json`、`calib_C1b_1.log`）は実施済みで、**参考として残す**（書き込みには使わない。`schema_version` 2 の正しい形式なので、`apply` は受け付けてしまう。**選ばないこと**。表示が見にくかったため、ツールの表示を直して、**2 回ともやり直す**）。ファイル名は上書きしない名前（`C1c`）にする。
改善した点: ① `compare` の再現性を生のエンコーダ値で比べる、② Enter は 2 回（中央の姿勢で Enter → そのまま記録開始 → 終わったら Enter）、③ 記録中に「前回の端まで あと N tick」をその場で表示する（**記録中は端末の表だけ。標準出力には何も出さない**。終わったら最後の表を 1 回だけ出す）。C2・C3 は、この C1c の測定ファイルを使う。
`measure` が書くのは **`Torque_Enable=0` だけ**（トルクが入っている関節があるときだけ）。EEPROM と Lock は書かない。C0.5 のあとはトルクは OFF のはずなので、何も書かずに進む。
```bash
ros2 run so_arm101_calibration so_arm101_calib measure --port /dev/ttyACM0 \
  --backup ~/so_arm101_check/eeprom_backup_20261111-163729.json --out ~/so_arm101_check/measure_C1c_1.json \
  2>&1 | tee -i ~/so_arm101_check/calib_C1c_1.log; echo "exit=${PIPESTATUS[0]}"
```
### 流れ（Enter はトルク OFF なら **2 回**、トルクが入っていれば 3 回）
0. （トルクが入っているときだけ）`腕を手で支えて Enter` → 脱力。**腕を支えてから** Enter を押す。
1. **中央の姿勢**（各関節の可動範囲のほぼ中央。LeRobot のキャリブと同じ「真ん中」）にして、**動かさずに Enter**（1 回目）。
2. `中央を測定中（約 0.3 秒。動かさない）…` と出る。**この間は動かさない**（各関節の揺れが 4 tick＝約 0.35° を超えると `ERROR: … 中央の測定中に動いた` で止まる。何も書かれていない。やり直す）。Enter を押す手で姿勢を崩さない。腕を支えているときは、Enter の後も `中央の測定 OK、記録中` が出るまで動かさない。この間に押した Enter は捨てられる（記録がすぐ終わることはない）。
3. **`中央の測定 OK、記録中。動かし始めてよい` と出てから**、全関節（gripper と wrist_roll を含む）を端から端まで、順に**押し切る**。wrist_roll は約 3801 tick（ほぼ 1 周）。**中央の姿勢から ±約 180° を超えて回さない**（超えると範囲が判定できず、検査で NG になる）。機械的に止まる端で止め、無理に押し込まない。脱力しているので、動かしていない関節は自重で垂れる。
4. 全部の端まで届いたら **Enter（2 回目）で記録を終える**。記録中に Enter を押すと、必ずそこで終わる（早すぎると幅が足りず検査で NG になる。Ctrl-C なら何も保存されずに中止）。

### 記録中の表示の見方
- **記録中は、画面に出るのは端末（`/dev/tty`）の 7 行の表だけ**（約 0.2 秒ごとに書き換わる）。標準出力（`tee` を通って画面とログに出るもの）には、記録中は何も出ない。
```
joint             今    min    max    幅  min 端まで    max 端まで
shoulder_pan   -1299  -1300   1247  2547  届いた(+43)   あと 205
```
- 表の幅は 70 桁。**端末の幅が 80 桁以上**にしておく（狭いと折り返して表示が崩れる。崩れても記録・測定の値には影響しない）。
- `今`・`min`・`max`・`幅` は、**中央の姿勢からの差（tick）**。
- `min 端まで`・`max 端まで`: 今の yaml の生の範囲（前回の端）まで、あと何 tick か。**min 側も max 側も同じ向きで読む**。`あと N` = あと N tick で前回の端。`届いた` = 前回の端と同じ位置。`届いた(+N)` = 前回の端より N tick 先まで押した。`*` は直近 1 秒に更新された（まだ押している最中）。
- 見方: **`あと N` が大きいままの端は、まだ押し切れていない**。`届いた` か `届いた(+N)` になり、`*` が消えれば、その端は押し切って止まっている。前回の端より手前（`あと N`）で止まる関節があっても、それ自体は異常ではない（前回が押しすぎだった可能性もある）。ただし、**押し切ったのに `あと N` が残る端は、機械的にそれ以上動かない可能性がある**ので、どこで止まったか（`あと` の値）を報告する。0 rad の位置（angle 関節は範囲の中点）は押し切り方で動くので、各関節を同じ強さで端まで押し切ること。
- wrist_roll の前回の範囲は 4095 をまたぐ（生の値で約 1328〜5129）が、表示は mod 4096 で折り返した差を使うので、そのまま読める。
- **Enter（2 回目）で記録を終えると、画面の表は消え、最後の時点の表が 1 回だけ出る**（同じ列。`*` は付かない。ログにも残る）。続けて `記録: 読み取り N 回（失敗 0 回）、間隔 平均 約 20 ms …、表示 K 回` の行と、測定結果の表（center / measured / new_h / min / max / span / samples / 検査）が出る。**読み取りの失敗が多い／間隔が 20 ms から大きく外れるときは報告する**。
- **`/dev/tty` が開けないとき**（端末の外から実行した場合など）: `注意: 端末（/dev/tty）を開けないので、記録中の表示は出さない` と出る。**表示なしで記録だけ続ける**（動かし終えたら Enter。最後の表と結果は出る）。表示を見ながら押し切りたいときは、端末から直接実行する。

### 結果と 2 回目
- 測定結果の表（center / measured / new_h / min / max / span / samples / 検査）が出る。**検査がすべて OK**で、測定ファイルが保存される。NG の関節があるときは、保存はされるが終了コードは 4 で、`apply` は受け付けない（理由が出る。`--out` を変えてやり直す）。
- 同じ手順で 2 回目（**2 回ともやり直す**）: `--out ~/so_arm101_check/measure_C1c_2.json`、ログ `calib_C1c_2.log`。**中央の姿勢はできるだけ同じ**にする（中央の姿勢の違いは、新しい再現性の表では結果に効かないが、同じにしておく）。
- 比較（ポートは開かない）:
  ```bash
  ros2 run so_arm101_calibration so_arm101_calib compare \
    --a ~/so_arm101_check/measure_C1c_1.json --b ~/so_arm101_check/measure_C1c_2.json \
    2>&1 | tee -i ~/so_arm101_check/calib_C1c_compare.log; echo "exit=${PIPESTATUS[0]}"
  ```
  - 「再現性」: **生のエンコーダ値（actual。Homing_Offset に依存しない）で比べる**。関節ごとに `d_amin`・`d_amax`（範囲の両端の差。tick）、`d_span`（幅の差）、`d_zero`（0 rad の物理位置＝範囲の中点の差。tick と rad。angle 関節だけ）。`d_cent` は中央の姿勢の差で、**参考であって結果には効かない**（最大の差にも入れない）。「最大の差」は `d_amin`・`d_amax`・`d_zero` から出す。**gripper** は端がそのまま 0%・100% になるので、`d_amin`・`d_amax`・`d_span` だけを見る（`d_zero` は「-」）。0 と 4095 をまたぐ関節（wrist_roll）も mod 4096 で折り返して比べる。
  - 「今の yaml との差」（A と B のそれぞれ）: min / max / center の差（tick と rad）、Homing_Offset の差、**`zero_shift`（0 rad の物理位置の移動量。tick と rad）**。`zero_shift` が 0 に近いほど、今の URDF・出発点の姿勢・SRDF の数値の意味が変わらない。
  - 前回（旧版の C1）の参考値: `zero_shift` は全関節で ±14 tick 以内（最大 0.0215 rad）。A と B の `d_zero` は pan 16、lift 2、elbow 6、wrist_flex 7、wrist_roll 0 tick。pan は 2 回目が min を 30 tick 先まで押して 0 rad が 16 tick 動いた（端の押し切り方が 0 rad に効く）。**今回は表示を見ながら、各関節を同じ強さで押し切って、この差が縮むか見る**（合否の基準は置かない）。
- 合格基準（C1c 自体）: 2 回とも終了コード 0、全関節の検査が OK、トルク以外に書いていない（腕が脱力する以外に変化がない）、測定ファイルが `schema_version` 2。再現性と yaml との差の合否は、次の「判断の区切り」の目安で見る。
- 報告してほしいもの: `calib_C1c_1.log`、`calib_C1c_2.log`、`calib_C1c_compare.log`、2 つの測定ファイル（`measure_C1c_1.json`、`measure_C1c_2.json`）。記録中の表示で気づいたこと（押し切れたか、表示の見にくさ）。

## 判断の区切り（C1c の後。C2 には自動では進まない）
`calib_C1c_compare.log` を見て、ユーザーが決める（前回の C1 では選択肢 1 を選んだ。測定ファイルは C1c のものから選ぶ。旧形式の測定ファイルは使えない）:
1. **新しい値を書く**（どの測定ファイルを使うかを選ぶ。以降の C2・C3 は同じファイルを使う）
2. **今の値をそのまま書き直す**（測定ファイルは使わない。C0.5 と同じ経路の確認だけ。EEPROM は変わらない）
3. **書かない**（キャリブは今のまま。2c の残りは LeRobot 経路の archive だけ）
- 見る点: 再現性（`d_amin`・`d_amax`・`d_zero` の 2 回の差が大きい関節はないか。`d_cent` は見なくてよい）、`zero_shift`（0 rad がどれだけ動くか）、wrist_roll の範囲、検査の結果。
- **ユーザーの判断（2026-11-11）**:
  - MIN/MAX の差は、**最後の押し込みの強さによる差**で、**1° 程度なら実運用上は問題なし**とする（1 tick = 360/4095 ≈ 0.0879°。1° ≈ 11 tick）。前の yaml の端との「あと N」（例: elbow_flex の max 側 26 tick ≈ 2.3°）も、押し込みの強さによる差として許容する。
  - **合格の目安**: C1c の 2 回で、**center（`zero_shift`、`d_zero`）とおおよその可動範囲（`d_amin`・`d_amax`）が、測るたびのばらつきと同じくらいに収まっていれば合格**。ばらつきの基準は前回の C1（最大 16 tick ≈ 1.4°）。合格なら、**どちらかの測定ファイルで C2 に進む**（以降の C2・C3 は同じファイルを使う）。
  - Claude は、結果をこの目安に照らして報告する（数値を並べる）。進むかの最終判断はユーザー。

## C2: gripper だけに新しい値を書く（yaml は更新しない）→ バックアップへ戻す
**EEPROM に新しい値を書く最初の操作（実機で初めて、値が変わる書き込み）。実行する前に Claude に「これから C2 を実行する」と伝え、承認を得る。** C0.5 と C1c が合格していること。
使うファイル: 測定ファイル `~/so_arm101_check/measure_C1c_1.json`、バックアップ `~/so_arm101_check/eeprom_backup_20261111-163729.json`。待ち時間は 50 ms（`--eeprom-wait-ms 50`）。対象は **gripper の 1 関節だけ**で、書く項目は Homing_Offset・Min・Max の 3 つだけ（トルクには書かない。腕は動かない）。
gripper に書く値（`measure_C1c_1.json`。今の EEPROM との差）:

| 項目 | 今（バックアップ） | 書く値 |
|---|---|---|
| Homing_Offset | 107 | **547** |
| Min_Position_Limit | 1900 | **1454** |
| Max_Position_Limit | 3385 | **2944** |

**C2 の間は、launch を起動しない（読み取り専用の `read_only:=true` も含む。mock も不要）。** 理由: ① `so_arm101_calib` と ros2_control は同じシリアルポートを使う（同時に動かさない）。② C2 の間、**gripper の EEPROM と yaml が食い違う**（yaml は C3 まで更新しない）。読み取り専用の launch は WARN のまま起動してしまい、gripper の位置が古い yaml で（約 440 tick ずれて）変換される。書き込みモード（`read_only:=false`）は、EEPROM と yaml が違えば起動を拒否するが、それに頼らない。launch は、C2 の手順 4 のあとの dry-run で「全部同じ」になるまで、起動しない（C3 の手順 3 まで起動しない）。
実行前の確認（共通の約束 1）:
```bash
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"
ros2 node list
```
どちらも空であること。

1. dry-run（何も書かない。書く予定の値が出る）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib apply --port /dev/ttyACM0 \
     --measure ~/so_arm101_check/measure_C1c_1.json --backup ~/so_arm101_check/eeprom_backup_20261111-163729.json \
     --joints gripper 2>&1 | tee -i ~/so_arm101_check/calib_C2_dryrun.log; echo "exit=${PIPESTATUS[0]}"
   ```
   `今の EEPROM とバックアップの照合` が `OK 全関節がバックアップと一致`、`書き込む値` の gripper が上の表の値（Homing_Offset 107 → 547、Min 1900 → 1454、Max 3385 → 2944）、`dry-run: 何も書いていない`、終了コード 0 であること。値が違う／他の関節が対象に入っているときは、書かずに報告する。
2. 書き込み（確認文字列を求められたら `WRITE-EEPROM` と入力。**Enter だけでは何も書かれない**）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib apply --port /dev/ttyACM0 \
     --measure ~/so_arm101_check/measure_C1c_1.json --backup ~/so_arm101_check/eeprom_backup_20261111-163729.json \
     --joints gripper --write --eeprom-wait-ms 50 2>&1 | tee -i ~/so_arm101_check/calib_C2_apply.log; echo "exit=${PIPESTATUS[0]}"
   ```
   `OK: 測定ファイルの値を書き、読み戻しと Lock=1 を確認した`、`gripper Lock: 1`、`yaml は更新していない（一部の関節だけ）`、終了コード 0。書く順序は Lock=0 → Homing → Min → Max → Lock=1（各項目を 50 ms 待って読み戻す。**値が変わるので、読み戻しの一致が、待ち時間が足りていることの最初の実測になる**）。
3. **サーボの電源を入れ直す**（外部電源を切って入れる。USB はつないだままでよい。C2 で確認済み。ポート名は `/dev/ttyACM0` のままだった）。値が残ることを確認（読み取り専用。launch は起動しない）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib backup --port /dev/ttyACM0 \
     --out ~/so_arm101_check/eeprom_after_C2.json 2>&1 | tee -i ~/so_arm101_check/calib_C2_after.log; echo "exit=${PIPESTATUS[0]}"
   ```
   gripper の Homing_Offset / Min / Max が **547 / 1454 / 2944** のまま、他の関節（pan 135/743/3452、lift 112/842/3216、elbow 122/862/3088、wrist_flex 89/889/3185、wrist_roll 1164/164/3965）は変わっていないこと。
4. **バックアップへ戻す**:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib restore --port /dev/ttyACM0 \
     --backup ~/so_arm101_check/eeprom_backup_20261111-163729.json --joints gripper --write \
     --eeprom-wait-ms 50 2>&1 | tee -i ~/so_arm101_check/calib_C2_restore.log; echo "exit=${PIPESTATUS[0]}"
   ```
   `OK: バックアップの値に戻り、Lock=1 を確認した`、終了コード 0。続けて、**全関節の dry-run**（何も書かない）で、全部「同じ（書かない）」であること:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib restore --port /dev/ttyACM0 \
     --backup ~/so_arm101_check/eeprom_backup_20261111-163729.json \
     2>&1 | tee -i ~/so_arm101_check/calib_C2_restore_check.log; echo "exit=${PIPESTATUS[0]}"
   ```
   18 レジスタすべて「同じ（書かない）」、`注意: ... アドレス n がバックアップと違う` なし、終了コード 0。
- 合格基準: 手順 2 と 4 が終了コード 0。手順 3 で電源を入れ直した後も値が残る（547 / 1454 / 2944）。手順 4 の後、全関節がバックアップと一致（EEPROM は C2 を行う前に戻る。yaml は C2 では更新しないので、C2 が終われば EEPROM と yaml はもとどおり一致する）。
- 失敗したとき: `apply` は書いた関節をバックアップへ自動で戻そうとする。表示される `旧 / 新 / 不明` と終了コード（上の表）を報告する。終了コード 2 は、表示された `restore --backup … --write` で戻す（`Lock=1 を確認できない` と出たら、サーボの電源を入れ直す）。**この段階で失敗したら、C3 には進まない。launch も起動しない。**
- 報告してほしいもの: `calib_C2_dryrun.log`、`calib_C2_apply.log`、`calib_C2_after.log`、`calib_C2_restore.log`、`calib_C2_restore_check.log`、各ログの `exit=` の値。

## C3: 全関節（gripper を含む 6 関節）を 1 回で書く（yaml はそのとき 1 回だけ更新）
**実行する前に Claude に「これから C3 を実行する」と伝え、承認を得る。** C2 が合格していること（合格済み。`docs/inventory.md`）。
使うファイル（**リポジトリ内の複製**。`~/so_arm101_check/` の原本と sha256 が同じ。Claude が C3 の前に置いた。コミットは C3 の合格後）:
- 測定ファイル **`calibration_records/2026-11-11/measure_C1c_1.json`**（原本 `~/so_arm101_check/measure_C1c_1.json`。C2 と同じ内容。sha256 `cfc0f592…6f47`）
- バックアップ **`calibration_records/2026-11-11/eeprom_backup_20261111-163729.json`**（原本 `~/so_arm101_check/eeprom_backup_20261111-163729.json`。sha256 `729b5015…b88f`）

待ち時間は 50 ms。**コマンドは、リポジトリの直下（`cd ~/ros2_so-arm101`）で、上の相対パスのまま実行する**（`apply` は `--measure`・`--backup` に渡した文字列を、そのまま yaml の出所に書く。絶対パスや `~` で渡すと、リポジトリの外のパスが入る）。

### yaml の扱い（C3 の前に読む）
- **更新される yaml は src 側 1 つ**: `src/so_arm101_hardware/config/follower_calibration.yaml`（git 管理）。ドライバが起動時に読む install 側（`install/so_arm101_hardware/share/so_arm101_hardware/config/follower_calibration.yaml`）は、`colcon build --symlink-install` により**そのファイルへのシンボリックリンク**で、`apply` はリンクを解決して src 側の実体を書き換える。**そろえる作業も、`colcon build` も要らない**（symlink-install でない場合は install 側がコピーなので別途必要。今の環境は symlink-install）。dry-run に出る yaml のパスは install 側の名前だが、更新されるのは src 側の実体。
- **出所の記録（`source`）**: 更新後は `kind: ros_calibrated`、`measure_file: calibration_records/2026-11-11/measure_C1c_1.json`、`backup_file: calibration_records/2026-11-11/eeprom_backup_20261111-163729.json`（**渡した相対パスのまま。リポジトリ内の複製を指す**）、`applied_at`（実行時刻）、`applied_by: so_arm101_calib 0.2.0`（版の定数 1 か所から。直した）になる。今の `source`（LeRobot の json からの変換）は消える（git の履歴に残る）。ファイル冒頭のコメントも置き換わる。`id`・`kind`・`direction`・`schema_version`・`bus`・`conversion` は引き継ぐ。
- **注意**: 測定ファイルの中の `backup_file` は、測定したときに渡した原本のパス（`/home/hirok/so_arm101_check/…`）のまま（チェックサム付きなので、複製しても書き換えない）。
- **控え**: 更新前の yaml は git の履歴にある（作業ツリーは clean。最後のコミット `1e9e968`）。念のため、リポジトリの外にも複製がある: `~/so_arm101_check/follower_calibration_before_C3.yaml`（sha256 `822dad7d…98a1`）。
- **yaml をコミットする時期**: 手順 3〜6 のログが合格してから（Claude が差分を確認してコミットする）。それまでは作業ツリーの変更のまま。**コミットには、更新した yaml と、`calibration_records/2026-11-11/` の中身（測定ファイル、バックアップ、C3 の後の `backup`（`eeprom_after_C3.json`）、`README.md`）を、同じコミットに入れる**（下の「合格後のコミット（Claude が行う）」）。
- yaml に入る値（`measure_C1c_1.json`）。Homing_Offset / min / max（`eeprom.*` も同じ）と、angle 関節の `center`:

  | 関節 | Homing_Offset | min | max | center |
  |---|---|---|---|---|
  | shoulder_pan | 144 | 724 | 3454 | 2089 |
  | shoulder_lift | 117 | 836 | 3211 | 2023.5 |
  | elbow_flex | 119 | 849 | 3090 | 1969.5 |
  | wrist_flex | 143 | 808 | 3174 | 1991 |
  | wrist_roll | 1196 | 100 | 3943 | 2021.5 |
  | gripper | 547 | 1454 | 2944 | （なし） |

### 実行前の確認
```bash
cd ~/ros2_so-arm101
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher" | grep -v -E "grep|gvfsd"
ros2 node list
git status --short src/so_arm101_hardware/config/
ls calibration_records/2026-11-11/
```
最初の 3 つは空であること（launch は止まっている。yaml に未コミットの変更がない）。`ls` は `eeprom_backup_20261111-163729.json  measure_C1c_1.json` の 2 つ。（`git status --short` 全体には `?? calibration_records/` が出る。複製は、C3 の合格後に yaml と一緒にコミットする。）

### 手順
0. **書き込みの前に、旧校正で出発点の姿勢を読む**（C3 の前後を比べる基準。何も書かない。**C3 の前後で、腕に触れずに同じ姿勢で読む**のが、正式な確認）。読み取り専用で起動し、腕を**出発点の姿勢**（トルク OFF でたたみ切って、手を離しても止まる姿勢。手で作る）にして、値を読む。**この姿勢のまま、手順 5 の読み取りまで、腕に触れない**（動かしてしまったら、手順 0 をやり直す）:
   ```bash
   ros2 launch so_arm101_bringup bringup.launch.py mock:=false use_rviz:=false 2>&1 | tee -i ~/so_arm101_check/calib_C3_before_launch.log
   ```
   別の端末で:
   ```bash
   ros2 topic echo /joint_states --once 2>&1 | tee -i ~/so_arm101_check/calib_C3_pose_before.log; echo "exit=${PIPESTATUS[0]}"
   ```
   `[diag] EEPROM vs yaml: 6/6 joints match`（旧 EEPROM と旧 yaml）を確認し、`position` の 6 つの値（順序: pan, lift, elbow, wrist_flex, wrist_roll, gripper）を控える。停止は Ctrl-C だけ（読み取り専用なので腕は動かない。トルク OFF のまま）。後始末（下の手順 5 と同じ）をしてから手順 1 へ。
1. dry-run（何も書かない）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib apply --port /dev/ttyACM0 \
     --measure calibration_records/2026-11-11/measure_C1c_1.json \
     --backup calibration_records/2026-11-11/eeprom_backup_20261111-163729.json \
     2>&1 | tee -i ~/so_arm101_check/calib_C3_dryrun.log; echo "exit=${PIPESTATUS[0]}"
   ```
   `OK 全関節がバックアップと一致`、`書き込む値` が上の表の 6 関節（`今` はバックアップの値）、`全関節を 1 回で書く。成功したら yaml を 1 回だけ更新する: …/follower_calibration.yaml`、`dry-run: 何も書いていない`、終了コード 0。違うときは書かずに報告する。
2. 書き込み（確認文字列 `WRITE-EEPROM` を入力。Enter だけでは何も書かれない）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib apply --port /dev/ttyACM0 \
     --measure calibration_records/2026-11-11/measure_C1c_1.json \
     --backup calibration_records/2026-11-11/eeprom_backup_20261111-163729.json \
     --write --eeprom-wait-ms 50 2>&1 | tee -i ~/so_arm101_check/calib_C3_apply.log; echo "exit=${PIPESTATUS[0]}"
   ```
   6 関節とも `[新]`、各 `Lock: 1`、`OK: 測定ファイルの値を書き、読み戻しと Lock=1 を確認した`、`yaml を更新した: …`、終了コード 0。続けて `git diff` で、`source` が `calibration_records/2026-11-11/…` の相対パス・`applied_by: so_arm101_calib 0.2.0` になっていること、`git diff --stat` に `follower_calibration.yaml` だけが出ること（**コミットはまだしない**）。**launch はまだ起動しない。**
3. **12 V を入れ直す**（USB はつないだまま）。まず launch なしで、EEPROM が残ったことを確認（読み取り専用）:
   ```bash
   ros2 run so_arm101_calibration so_arm101_calib backup --port /dev/ttyACM0 \
     --out ~/so_arm101_check/eeprom_after_C3.json 2>&1 | tee -i ~/so_arm101_check/calib_C3_after.log; echo "exit=${PIPESTATUS[0]}"
   ```
   6 関節とも「2 回読んで一致」で、値が上の表のとおり（Homing_Offset / Min / Max）、終了コード 0。
4. 読み取り専用で起動して診断を確認する（腕は触らない。トルク OFF のまま）:
   ```bash
   ros2 launch so_arm101_bringup bringup.launch.py mock:=false use_rviz:=false 2>&1 | tee -i ~/so_arm101_check/calib_C3_readonly.log; echo "exit=${PIPESTATUS[0]}"
   ```
   `[diag] EEPROM vs yaml: 6/6 joints match`、`servo settings vs LeRobot: 6/6 joints match`、全関節 `torque_enable=0`、`MISMATCH` の行なし。（EEPROM と yaml が違うと、読み取り専用では WARN のまま起動し、書き込みモードは起動を拒否する。）
5. **腕に触れずに**（手順 0 と同じ姿勢のまま）、新しい座標での値を読む（手順 4 の起動のまま）:
   ```bash
   ros2 topic echo /joint_states --once 2>&1 | tee -i ~/so_arm101_check/calib_C3_start_pose.log; echo "exit=${PIPESTATUS[0]}"
   ```
   **C3 の前後の差を、`zero_shift` から計算した見込みと比べる**（同じ物理姿勢なら `新 rad = 旧 rad − zero_shift × 2π/4095`。gripper は新しい範囲での %）。次を、手順 0 と手順 5 の 6 つの値に直して、リポジトリの直下で実行する:
   ```bash
   python3 - <<'EOF'
   import json, math, yaml
   BEFORE = [0, 0, 0, 0, 0, 0]   # 手順 0（C3 の前、旧校正）の position: pan, lift, elbow, wrist_flex, wrist_roll, gripper
   AFTER  = [0, 0, 0, 0, 0, 0]   # 手順 5（C3 の後、新校正）の position
   old = yaml.safe_load(open('/home/hirok/so_arm101_check/follower_calibration_before_C3.yaml'))   # C3 の前の yaml
   m = json.load(open('calibration_records/2026-11-11/measure_C1c_1.json'))                        # C3 で書いた測定ファイル
   k = 2 * math.pi / old['conversion']['angle_ticks']
   print(f"{'joint':14s}{'before':>9s}{'after':>9s}{'見込み':>9s}{'差(rad)':>9s}{'差(tick)':>9s}")
   for j, b, a in zip(m['joints'], BEFORE, AFTER):
       o = old['joints'][j['name']]
       if o['kind'] == 'angle':   # rad = direction*(raw - center)*k、raw = actual - Homing_Offset
           actual = b / k * o['direction'] + o['center'] + o['eeprom']['homing_offset']
           exp = o['direction'] * (actual - j['new_homing_offset'] - j['new_center']) * k
           tick = (a - exp) / k
       else:                      # gripper: pct = (clamp(raw,min,max) - min)/(max - min)*100、rad = radians(pct)
           actual = o['min'] + math.degrees(b) / 100 * (o['max'] - o['min']) + o['eeprom']['homing_offset']
           raw = actual - j['new_homing_offset']
           exp = math.radians((min(max(raw, j['new_min']), j['new_max']) - j['new_min']) / (j['new_max'] - j['new_min']) * 100)
           tick = float('nan')
       print(f"{j['name']:14s}{b:9.5f}{a:9.5f}{exp:9.5f}{a - exp:9.5f}{tick:9.1f}")
   EOF
   ```
   **目安: 各関節で、実測と見込みの差が 0.003 rad（約 2 tick）以内**。符号の誤り・単位の誤り・オフセットの誤りのような大きなずれを見つけられる。2026-11-11 の C3 の結果: pan 以外は 0.0001 rad 以内、pan は約 1 tick。**腕に触れてしまって同じ姿勢で読めなかったときは**、出発点の旧座標の値（pan -0.08362、lift -1.51134、elbow +1.66938、wrist_flex +0.50941、roll -0.07135、gripper +0.01645）を `BEFORE` にして比べる。姿勢を手で作り直したぶん、差は大きくなる（C3 の目安は 0.03 rad 以内）。
   停止は Ctrl-C だけ。後始末: `ps`（上の `ps aux | grep …` と同じ）と `ros2 node list` が空であること。`ros2 node list` だけに残り `ps` が空なら、`ros2 daemon stop; ros2 daemon start` の後にもう一度確認する。
6. **URDF の関節リミットとの余裕**（オフライン。URDF・SRDF は読むだけ。`source install/setup.bash` の後に実行）:
   ```bash
   python3 tools/pose_margin.py --pose start=<pan>,<lift>,<elbow>,<wrist_flex>,<wrist_roll>,<gripper> \
     2>&1 | tee -i ~/so_arm101_check/calib_C3_margin.log; echo "exit=${PIPESTATUS[0]}"
   ```
   （値は手順 5 の `position`。順序は pan, lift, elbow, wrist_flex, wrist_roll, gripper。`--calibration` は新しい yaml（既定）。）校正した範囲の両端と URDF の範囲の差、姿勢ごとの URDF の上限・下限までの残りが出る。範囲の外（OUT）や余裕が小さい（LOW。仮の目安 0.05 rad）関節があれば、**C4 の前に Claude へ報告して判断を仰ぐ**（URDF・SRDF は変えない）。**C3 の実測（2026-11-11）: 出発点（新座標）で OUT は elbow_flex だけ（+1.69009。上限 +1.69 を約 0.00009 rad 超える）。ユーザーの判断で、(a) 何も変えない（OMPL は `FixStartStateBounds`（0.1）で扱える。MotionPlanning では OMPL を使い、Pilz を選ばない。`docs/feetech_write_check.md`）。** 今後の再校正で LOW・OUT が出たら、報告して判断を仰ぐ（URDF・SRDF は変えない）。
- 合格基準: 手順 2 が終了コード 0（6 関節 `[新]`、yaml 更新）。手順 3 で 12 V を入れ直した後も値が残る。手順 4 が 6/6 MATCH（2 行とも）。手順 0 と手順 5 の前後の差が、`zero_shift` からの見込みと 0.003 rad（約 2 tick）以内。手順 6 のログが取れている（LOW・OUT は報告して判断）。
- 報告してほしいもの（**コミットはまだしない**。Claude が確認してから）: `calib_C3_before_launch.log`、`calib_C3_pose_before.log`（手順 0）、`calib_C3_dryrun.log`、`calib_C3_apply.log`、`calib_C3_after.log`、`calib_C3_readonly.log`、`calib_C3_start_pose.log`（手順 5）、`calib_C3_margin.log`、各ログの `exit=` の値、`git diff --stat` の出力、手順 5 の比較の表。Claude が yaml の差分を確認して commit し、手順書の出発点の姿勢などを直す（C4 の前）。

### 合格後のコミット（Claude が行う。ユーザーの作業は、手順 3〜6 のログの報告まで）
手順 3〜6 が合格（手順 6 の LOW・OUT は報告して判断）してから、Claude が次を行う:
1. `~/so_arm101_check/eeprom_after_C3.json` を `calibration_records/2026-11-11/eeprom_after_C3.json` に複製し、**sha256 が原本と一致すること**を確認する。
2. `calibration_records/2026-11-11/README.md` を作る（各ファイルの内容・元のパス・sha256・使い道。`measure_C1c_1.json` を C3 で使ったこと、`eeprom_backup_…` が C3 の前のバックアップで `restore` の元になること、`eeprom_after_C3.json` が C3 の後の EEPROM であること）。
3. `git diff` で yaml の差分を確認する（`source`、冒頭のコメント、各関節の `center`・`min`・`max`・`eeprom.*` だけが変わっていること）。
4. **更新した yaml と `calibration_records/2026-11-11/` を、同じコミットに入れる**（コミットメッセージに、C3 の合格と、手順 5・6 の結果の要点を書く）。
5. 手順書・`inventory.md` に C3 の結果を記録する（別のコミットでもよい）。
手順 3〜6 のどれかが不合格なら、コミットしない（yaml は `git checkout`、EEPROM は `restore` で戻す。複製は未コミットのまま残してよい）。


- 失敗したとき: 上の「失敗したときの共通の対処」。`apply` が失敗すると、yaml は更新されず、触れた全関節がバックアップへ戻る。終了コード 2 は、表示された `restore` で戻す。**どの場合も launch を起動しない**（EEPROM と yaml が食い違う）。
- **C3 の後に元に戻す手順**（C3 が合格したあとで、やはり戻すとき）: ① EEPROM: `so_arm101_calib restore --port /dev/ttyACM0 --backup ~/so_arm101_check/eeprom_backup_20261111-163729.json --write --eeprom-wait-ms 50 2>&1 | tee -i ~/so_arm101_check/calib_C3_revert.log; echo "exit=${PIPESTATUS[0]}"`（違う項目だけ書く）。② yaml: 未コミットなら `git checkout -- src/so_arm101_hardware/config/follower_calibration.yaml`、コミット済みなら `git revert <そのコミット>`（履歴は書き換えない）。③ 全関節の dry-run（`restore --backup …` に `--write` なし）が「同じ」で、読み取り専用の起動で 6/6 MATCH になるまで、書き込みモードは起動しない。`restore` は yaml を更新しない。

## 失敗したときの共通の対処
- `apply` が終了コード 1: 全関節が実行前の値（バックアップ）に戻り、Lock=1 を確認した。原因（表示された `read-back mismatch` など）を報告する。
- 終了コード 2: 一部が新・不明のまま、または Lock=1 を確認できない。表の `旧 / 新 / 不明` を報告し、表示された `restore --backup … --write` で戻す（ros2_control は止めたまま）。`Lock=1 を確認できない` と出たら**サーボの電源を入れ直す**（Lock は電源投入時に 1 になる）。電源を入れ直したら `restore` の dry-run で状態を確認する。
- yaml が古いまま EEPROM が新しい（`yaml を更新できなかった`）: 書き込みモードの launch は起動を拒否する。退避先に出た yaml を正しい場所へ置くか、`restore` で戻す。
- どの場合も、書き込みモード（`read_only:=false`）の launch は、`restore` で戻すか C3 が合格するまで起動しない。

## 注意（どの手順でも）
- `so_arm101_calib` を ros2_control と同時に動かさない。止め方: launch は Ctrl-C だけ（`kill -9` は使わない）。
- EEPROM は電源を切っても残る。書く前に必ずバックアップがあることを確認する。
