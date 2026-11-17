# 実機確認手順: オフセットキャリブ（zero-offset calibration）— 先端を机に当てた 4 姿勢を記録して fit する

**F5。実機は未実施。** 計画: `docs/zero_offset_calib_plan.md`（0.1 決定、5.2 記録の道具、5.4 当てはめの道具、4 章 合否の基準、7.5 骨組み）。道具: `tools/zero_offset_record_run.py`（記録）、`tools/zero_offset_fit_run.py`（当てはめ。表示だけ）。コード: コミット `66a0644`（F4b）までと、その前の F2〜F4。

> **この手順書のコマンドは、Claude Code が一つも実行していない（未実行）。** 実機（腕）も launch も、Claude は動かしていない。**実行するのはユーザーだけ。**
>
> **これは「オフセットキャリブ」であって、「センターキャリブ」（`so_arm101_calib measure・apply`）ではない。** 腕のトルクは**一度も入れない**（読み取り専用）。サーボに何も書かない。**この手順書は yaml を変えない**（yaml に δ を入れるのは F6 で、別の承認を得てから）。

この手順で決めること: lift・elbow・wrist_flex の `zero_offset_deg`（真の角度 = 読んだ角度 + δ）の値。elbow が約 −6.3° ずれていると見られている（`inventory.md` の C3 の調査）。**この手順の出力は、画面に表示される「貼る行」（paste）と、結果の表示だけ。** 貼るのは F6。

## 共通の約束

| 約束 | 内容 |
|---|---|
| 実行するのはユーザーだけ | 端末は A（腕の launch。読み取り専用）、D（記録・fit・確認）。A は、ログを流すだけで操作しない |
| トルク | **一度も入れない。** 読み取り専用の起動（`read_only:=true`）のまま。**出発点（折りたたみ姿勢）ではトルク OFF でも沈まない（2026-11-16 の R1 で確認）。姿勢を動かしている間、出発点へ戻す時、launch を止める前は手で支える** |
| グリッパー | **開かない。** 閉じたまま。道具は gripper の値を CSV に記録するだけで、モデルには使わない（**記録の道具の表示には gripper の値は出ない**。確認は端末 E。段 3）。トルク OFF なので動くあごが自重で開くおそれがある。**記録のたびに gripper の値を見て、段 1 で読んだ閉じた値から 0.05 rad 以上変わったら記録しない**（0.05 rad は**推測の値**）。動くあごは、必要なら指で閉じたまま押さえる（押さえる指が机や固定あごに触れないように）。あごを閉じた状態の固定あごで机に当てる |
| 止め方 | A と D は **Ctrl-C だけ**（`kill -9`・`pkill`・`killall` はしない）。A を止めてもトルクはもともと OFF（止める前は腕を支える） |
| `--no-daemon` | `ros2 topic`・`ros2 node` には付ける。`ros2 topic hz` は `--no-daemon` を受け付けないので付けない |
| ログ | `tee -i` と `; echo "exit=${PIPESTATUS[0]}"`。保存先は `~/so_arm101_check/zero_offset_<日付>/`（`<日付>` は実行する日。例 `20261118`。全部のコマンドで同じ日付に置き換える） |
| 同名のファイル | **書く前に `ls` で同名が無いことを確かめる**（`No such file or directory` が出るのが正常）。記録の CSV は追記で、同じ label は拒否されるが、`tee` は上書きする |
| コードと設定 | **この手順では何も変えない。** yaml・URDF・コードのどれも。気になることは報告だけ |
| 机に当てる力 | **押し付けない。** そっと触れる程度（たわみが誤差になる）。腕の重さは手で受ける |
| 後始末 | 停止のたびに ps と `ros2 node list --no-daemon` を確認する（段 6） |

## ユーザーがすること（段 0 の前。Claude は触らない）

cyclonedds を設定した端末（次の「端末の準備」の 4 行のあと）で、ros2 daemon を再起動する。**これはユーザーだけが行う。Claude は ros2 daemon に触らない（stop も start もしない）。**

```bash
ros2 daemon stop; ros2 daemon start; sleep 3
```

## 端末の準備（端末 A・D・E。A は launch のコマンドの前に同じ行を実行する）

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "CONDA_PREFIX=[$CONDA_PREFIX] LECONDA_SITE_PACKAGES=[$LECONDA_SITE_PACKAGES]"
```

- 期待: `CONDA_PREFIX=[] LECONDA_SITE_PACKAGES=[]`（**両方とも空**）。空でなければ、その端末は使わず、新しい端末で準備をやり直す（conda を activate していない端末）。
- `ROS_DOMAIN_ID` は設定しない（87 はモック専用）。

## 端末の割り当て

| 端末 | 用途 | 使う段 |
|---|---|---|
| A | 腕の launch（`bringup.launch.py mock:=false read_only:=true`）。ログを流す。操作しない | 1〜6 |
| D | 記録の道具（`tools/zero_offset_record_run.py`）。**Enter を受け取る端末**。記録が済んだら fit と確認のコマンド（`ls`・`ps`・`sha256sum`・`ros2 topic`）を同じ端末で | 0、2〜6 |
| E | gripper の値の確認だけ（`ros2 topic echo --no-daemon --once /joint_states`）。記録のたびに使う | 1、3 |

カメラの launch・移動の道具（`ik_move_run.py`）・RViz は**使わない**（use_rviz:=false）。

## 目標の姿勢（実物の値）と `--expected-offset`

目標は**実物（真の角度）**の傾き ε と x。ε は向き付き（90° が真下、90° 未満が前傾、90° 超が後傾）。許容は全姿勢 ε が ±5°、x が ±10 mm。

| 姿勢 | 実物の ε | 実物の x（mm） | 記録 |
|---|---|---|---|
| P1 | 90° | 100 | 2 回（P1#1、P1#2） |
| P2 | 90° | 300 | 2 回 |
| P3 | 70° | 150 | 2 回 |
| P4 | 50° | 250 | 2 回 |

- yaml に δ がまだ無い間、「読んだ角度」は実物から約 −6.3°（elbow）ずれている。そこで**初回は `--expected-offset elbow_flex=-6.3`** を渡す（実物 = 読んだ角度 + 値）。道具は、案内の表示・目標の判定・ε の下限（45°）の判定だけに、この値を使う。**記録される角度と fit には使われない**（CSV の q は読んだ値のまま）。
- 表示の ε は「実物の見込み」で、括弧の `[read …]` が読んだ値の ε。elbow の δ が −6.3° なら、読んだ値は実物より約 6.3° 大きい。
- **F6 で yaml に δ を入れた後にこの手順をもう一度行うときは、`--expected-offset` を渡さない**（読んだ角度が補正後になるため）。
- 同じ CSV に、違う値の `--expected-offset` で追記することはできない（拒否される）。

## 止め方と中止の基準（先に読む）

| 状況 | 何が起きるか | 操作 |
|---|---|---|
| 記録の道具（端末 D）の途中 | 何も送らない。読み取りだけ | `q` と Enter、または Ctrl-C（`interrupted: nothing was sent by this tool`、終了コード 130） |
| 腕の launch（端末 A） | もともとトルク OFF。**動かした姿勢では、支えていなければ落ちる** | 腕を支えてから Ctrl-C を 1 回。10 秒待って止まらなければ報告（`kill -9`・`pkill`・`killall` はしない。最後の手段は 12 V と USB） |
| 12 V | 腕が落ちる | 最後の手段。腕を支える |

**次のどれかが起きたら、そこで止めて報告する（記録を続けない）。**

1. 段 0 の確認で、期待と違う値（ps に残り、yaml の sha256 の控えと違う、`zero_offset_deg` が yaml に既にある）。
2. **机と取り付け面が同じ面でない**（段 0）。
3. 腕の launch のログに `ERROR`・`FATAL`、コントローラの失敗、`/joint_states` が来ない。
4. 表示の「model height」が、当てているときに ±60 mm を超える（`WARN`。空中か、δ が大きく外れている）。
5. 静止の判定（1 秒・0.2°）に、どう支えても通らない。腕や関節にがたつき・異音・引っかかりがある。
6. 90° の姿勢（P1・P2）で、固定あごではなく**動くあごが先に机に当たっているように見える**。
7. 先端やあごが机で滑る、傷つく、ぶつけた。
8. fit の終了コードが 0 以外で、原因が分からない（結果の読み方の表）。
9. 想定外の出力（道具のメッセージが違う、例外、終了コードが 0・2・130 以外）。
10. **グリッパーの値が閉じた値に戻らない**（段 1 で読んだ閉じた値から 0.05 rad 以上離れたまま。動くあごが開いたまま）。0.05 rad は推測の値。
11. 起動のログの `[diag]` の 2 行が 6/6 でない、停止のログの `bus:` の誤りのカウンタが 0 でない（段 1・段 6）。

## 段 0: 準備と確認（何も起動しない。端末 D）

目的: 起動前の状態を確かめ、控えを取り、保存先を作る。

端末 D（準備の行のあと）:

```bash
ls ~/so_arm101_check/zero_offset_<日付>/
mkdir ~/so_arm101_check/zero_offset_<日付>/
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher|ik_move_run|realsense2_camera_node|apriltag_node|zero_offset_" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
git -C ~/ros2_so-arm101 log --oneline -1
git -C ~/ros2_so-arm101 status --short
```

- `ls` の期待: `No such file or directory`（正常）。あれば止めて報告（違う日付にする）。
- ps の期待: **何も出ない**。`ros2 node list --no-daemon` の期待: **何も出ない**。
- `git log` の期待: `66a0644` 以降の F5 のコミット（記録の道具が F4b 入り）。`git status --short` の期待: 何も出ない。

控え（yaml・URDF・メッシュ。道具の識別に使われる）:

```bash
sha256sum ~/ros2_so-arm101/install/so_arm101_hardware/share/so_arm101_hardware/config/follower_calibration.yaml \
  ~/ros2_so-arm101/install/so_arm101_description/share/so_arm101_description/urdf/so101_new_calib.urdf \
  | tee ~/so_arm101_check/zero_offset_<日付>/S0_sha256.txt
grep -n "zero_offset_deg" ~/ros2_so-arm101/install/so_arm101_hardware/share/so_arm101_hardware/config/follower_calibration.yaml
```

- `grep` の期待: **何も出ない**（yaml に `zero_offset_deg` はまだ無い。δ_old = 0）。出たら止めて報告（その値で `--expected-offset` の扱いが変わる）。

**机と取り付け面が同じ面であることの確認**（ユーザーが定規などで）:

- 腕の取り付け面（ベースを留めている面）と、先端を当てる机の面が**同じ面**か。同じ板に留めて、その板に当てるなら同じ。段差や別の台なら違う。
- 同じ面: h の事前分布（既定 −2.4 mm、σ_h 1 mm）のまま進む。
- **違う面（段差がある）: ここで止めて報告する。** 段差を定規で測り、`--h-prior-mm` の値を一緒に決める（私が値を出す。未決定の値で進めない）。
- 机が**水平**かは、この手順では測らない（pan を変えた点が無いので、残差では見えない。未確認として残す）。机の上に物・ごみ・傷が無いことを見る。

## 段 1: 読み取り専用の起動（端末 A）

腕を折りたたんだ姿勢（出発点。トルク OFF でも沈まない）にして起動する（念のため手を添える）。

端末 A（準備の行のあと）:

```bash
ls ~/so_arm101_check/zero_offset_<日付>/S1_arm_launch.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=true use_rviz:=false 2>&1 \
  | tee -i ~/so_arm101_check/zero_offset_<日付>/S1_arm_launch.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。
- 10 秒ほど待つ。

確認（端末 D）:

```bash
ros2 topic echo --no-daemon --once /joint_states | head -30
ros2 topic hz /joint_states
```

- 期待: 6 関節（shoulder_pan・shoulder_lift・elbow_flex・wrist_flex・wrist_roll・gripper）の position が出る。`hz` が安定して出る（値を報告。Ctrl-C で止める）。
- 腕の launch のログに、**「δ ≠ 0 の関節の INFO」が出ない**こと（yaml に δ が無いので出ないのが正常）。出たら止めて報告。
- ログに `ERROR`・`FATAL` が無いこと。
- **起動のログの `[diag]` の 2 行**（端末 A のログ。`S1_arm_launch.log` でもよい）: `EEPROM vs yaml: 6/6 joints match` と `servo settings vs reference: 6/6 joints match`。**どちらかが 6/6 でなければ止めて報告**（中止の基準 11）。確認: `grep -n "\[diag\]" ~/so_arm101_check/zero_offset_<日付>/S1_arm_launch.log`。
- **gripper の閉じた値を控える**: 上の `ros2 topic echo` の `gripper` の position を `S1_gripper_closed.txt` に書き留める（段 3 でこの値と比べる。閉じたまま、手で動かしていない状態）。
- 腕を手で動かすと、position が変わること（トルク OFF の確認）。

## 段 2: 記録の道具の起動（端末 D）

端末 D:

```bash
ls ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv
python3 -u tools/zero_offset_record_run.py \
  --csv ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv \
  --expected-offset elbow_flex=-6.3 2>&1 \
  | tee -i ~/so_arm101_check/zero_offset_<日付>/S2_record.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。
- 期待する出力: 最初の行が `[zero-offset calibration] zero_offset_record_run: … NOT the center calibration …`。次に `expected offset: elbow_flex=-6.3 (a guide for the hand only; …)`。次に `Enter = record, P1..P4 = choose the target, y = confirm, q = quit`。そのあと、5 Hz で 1 行の表示が更新される（`P1#1: x … mm (…)  elevation … deg (…) [read …]  in range / OUT of range  moving … deg still / MOVING  model height … mm`）。
- 起動の検査で拒否されたとき（`refused: …`、終了コード 2）は、メッセージを報告する（識別の違い、yaml・URDF の読み込みの失敗など）。
- **Enter を受け取るのはこの端末だけ。** 他の端末で Enter を押しても記録されない。ログ（`S2_record.log`）は、`\r` で上書きされる 1 行の表示がすべて残るので長くなる（1 分で約 300 行分）。

## 段 3: 記録（P1〜P4、各 2 回。計 8 回）

目標の順: P1#1、P1#2、P2#1、P2#2、P3#1、P3#2、P4#1、P4#2（道具が順に進める。`P3` と打って Enter で、その姿勢の目標に切り替えられる）。

### 腕の支え方（トルク OFF で、静止の判定 1 秒・0.2° に通る支え方）

静止の判定: 直近約 1 秒（20 サンプル）のどの関節も、最大と最小の差が **0.2° 以下**（サーボの 1 tick は約 0.088°）。表示が `still` のときだけ記録できる。`MOVING` の間は記録されない。

1. **腕の重さは手で受ける。** 先端は机に**そっと触れさせる**だけで、押し付けない。机に体重をかけると、たわみ・歯車の遊びが誤差になる。
2. **手首の近くと前腕を、片手の指で挟むように支える**（腕を握りしめない。握力で関節が動く）。支える手は、机か腕の台に前腕を置いて**固定**し、手が震えない姿勢にする。
3. **目で見て先端が机に当たっている**ことを確かめる（表示の model height が 0 に近いのが目安。±60 mm 超は警告）。
4. 動かしたあとは、**1〜2 秒待つ**（腕が揺れている間は `MOVING`）。表示が `still` になってから、もう一方の手（または別の人）で Enter。
5. P2（90°、x 300）は腕が遠くまで伸び、てこで手首に大きな力がかかる。**前腕を支える位置を先端に近づけ、無理なら別の人に腕を支えてもらい、Enter だけ押す。** 無理をしない。
6. 2 回目は、**一度先端を机から離して**、もう一度当てる（同じ位置に置き直さない。当て方のばらつきを測るため）。
7. Enter を押す瞬間に腕を動かさない（Enter の指の動きが腕に伝わらない姿勢で）。
8. どうしても `still` にならない（5 の項目）なら、そこで止めて報告する。

### 各姿勢の手順

1. 表示の目標（`P1#1` など）を見て、先端を机の目標の x あたりに置く。
2. 表示の `(…)` の差（Δx は mm、Δε は度）が範囲（x ±10 mm、ε ±5°）に入り、`in range` になるまで、手で姿勢を変える。
3. 先端を机に当て、`still` を待つ。**Enter の前に、端末 E で gripper の値を見る**: `ros2 topic echo --no-daemon --once /joint_states | grep -A8 "^position"`（`gripper` は 6 番目の値）。段 1 の閉じた値（`S1_gripper_closed.txt`）から **0.05 rad 以上**（推測の値）変わっていたら、記録しない。動くあごを閉じた位置に戻し（必要なら指で押さえる。指が机や固定あごに触れないように）、`still` を待ってからやり直す。戻らなければ中止の基準 10。問題なければ Enter。
4. **90°（P1・P2）は、固定あごの先が机に当たっていること（動くあごが先に当たっていないこと）を目で確認する。** あごは閉じたまま。動くあごが先に当たっているように見えたら、記録せず止めて報告（中止の基準 6）。
5. 記録されると `recorded P1#1 (row 1): x … mm, elevation … deg (read …), model height … mm` が出る。
6. 範囲外で Enter を押すと `outside the target (…)` と出て記録されない。**範囲に戻す**のが普通。意図して範囲外で記録するときだけ `y` と Enter（fit の姿勢の検査が評価する）。
7. `not recorded: …` が出たら理由を読む（`the arm is not still`＝静止しなかった、`the elevation … is below 45`＝傾きが浅すぎる、`no fresh /joint_states`＝購読が来ていない）。
8. 6 回目の記録から、記録のたびに `provisional (display only, not a result): …` が出る（暫定。止め時の目安。合否は fit で）。

全 8 回の記録のあと、`all poses recorded` と表示される。`q` と Enter で終了（終了コード 0）。

## 段 4: 記録の確認（端末 D）

```bash
ls -l ~/so_arm101_check/zero_offset_<日付>/
head -3 ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv | cut -c1-200
wc -l ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv
cat ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv.meta.yaml
sha256sum ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv \
  ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv.meta.yaml \
  | tee ~/so_arm101_check/zero_offset_<日付>/S4_sha256.txt
```

- CSV の 1 行目が `# zero-offset calibration (floor touch). Not the center calibration.`、2 行目がヘッダー。
- `wc -l` の期待: **10**（コメント 1 + ヘッダー 1 + 記録 8）。
- meta の期待: `calibration_kind: zero_offset`、`identity.zero_offset_deg` が全関節 0.0、`settings.expected_offset_deg.elbow_flex: -6.3`、`settings.sigma_90_extra_mm: 0.0`、`h_prior_mm: -2.4`。
- 記録した行の `eps_deg`（読んだ値）と `eps_expected_deg`（実物の見込み）が、約 6.3° 違うこと（elbow の見込み）。

## 段 5: fit の実行（端末 D。表示だけ。yaml も CSV も書かない）

```bash
ls ~/so_arm101_check/zero_offset_<日付>/S5_fit.log
python3 -u tools/zero_offset_fit_run.py ~/so_arm101_check/zero_offset_<日付>/zero_offset_<日付>.csv 2>&1 \
  | tee -i ~/so_arm101_check/zero_offset_<日付>/S5_fit.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。
- 外れ値の疑いがあるときだけ、`--exclude P3#2` を付けて**もう一度**（別のログ名で）実行できる。**外した結果をそのまま採用しない。** 外した理由を報告する。
- **fit の道具は yaml を書かない。** 表示された「貼る行」を yaml に入れるのは F6（別の承認）。

### 結果の読み方

| 終了コード | 意味 | 次の操作 |
|---|---|---|
| 0 | 合格。`result: accepted`（警告つきは `accepted (with N warning)`）。`paste:` の行が出る | 結果を報告する。貼るのは F6 |
| 2 | 不合格または拒否。`result: rejected (nothing to paste)`。貼る行は 1 行も出ない | 全項目（`[FAIL]` の行）を報告する。原因を見る（下） |
| 1 | 引数・読み込みの誤り（ファイルが無い、`--exclude` の label が無い） | メッセージを報告する |

表示の順: 姿勢の一覧、モデル（`free`＝wrist も当てはめた値、`fixed`＝wrist を 0 に固定）、各記録の残差、h と RMS、関節ごとの δ と sd と今の yaml の値、相関、検査の一覧（`[ok]`・`[WARN]`・`[FAIL]`）、`paste:` の行。

主な検査（基準は計画の 4 章。推測の値）:

| 検査 | 基準 | 不合格のときに疑うこと |
|---|---|---|
| poses・records | 姿勢 4 以上、記録 8 以上 | 記録が足りない |
| tilt_groups | ε が 10° 以上離れた 3 群 | 傾きの種類が足りない（fit の ε は読んだ値で見る） |
| x_spread | x の広がり 150 mm 以上 | x の位置が近すぎる |
| eps_min | 読んだ値の ε が 45° 以上 | 浅すぎる（fit は読んだ値で見るので、記録のときより甘い） |
| condition | 条件数 1e3 以下 | 姿勢の組が悪い（決まらない） |
| residual_rms / residual_max | 1.5 mm 以下 / 3 mm 以下 | 当て方のばらつき、メッシュと実物の差、机の傾き、h の事前分布が違う |
| sd_… | lift 0.5°、elbow 1.0°、wrist 1.0° 以下 | 当て方のばらつきが大きい |
| leave_one_out_… | 1 回外した変化が各 0.5°・1.0°・1.0° 以下 | 外れ値の疑い（その記録の当て方を見直す） |
| delta_abs_… | 10° 以下 | 実物が大きく外れている |
| h_estimate | −2.4 mm から 3 mm 以内（5 mm 超は不合格） | 机と取り付け面が違う、机が傾いている |

- **wrist の貼る行は、δ_wrist の絶対値が 1.5° 以上で sd が 1.0° 以下のときだけ出る。** それ以外は「wrist は 0 のまま」と表示される（不合格ではない）。このとき lift と elbow の貼る行は、wrist = 0 に固定して当てはめ直した値。
- 期待の目安（推測。実機で見直す）: elbow が約 −6.3°（カメラの値 −6.31 ± 0.38、物差しの −5.77 と比べる）、lift と wrist は 0 に近い。**合わなくても結果を直さず、そのまま報告する**（メッシュと実物の差、90° の当たり方、机の傾きを疑う）。
- 表示の先頭近くに `expected offset of the record display: elbow_flex=-6.3 (… the fit does not use it …)` の 1 行が出る。fit の計算には使われていない。
- 表示の ε は、読んだ値の ε（実物より約 6.3° 大きい）。

## 段 6: 後始末

腕を**支えてから**、端末 A で Ctrl-C を 1 回。端末 D の記録の道具が動いていたら先に `q` と Enter。

```bash
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|static_transform_publisher|ik_move_run|realsense2_camera_node|apriltag_node|zero_offset_" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
```

- 期待: **どちらも何も出ない。** `--no-daemon` だけに残って ps が空なら、数秒待ってもう一度確認する。それでも残るなら報告して指示を待つ（`ros2 daemon` には触らない。stop も start もしない）。
- ps に残るときは、報告する（`pkill`・`killall` はしない）。

停止のログの確認（端末 D。停止の後）:

```bash
grep -n "deactivated (no register was written)" ~/so_arm101_check/zero_offset_<日付>/S1_arm_launch.log
```

- 期待: `deactivated (no register was written). reads=… retried=… failed_cycles=… bus: flushed_requests=… flushed_bytes=… drained_bytes=… id_mismatch=… answered_first=… timeouts=… bad_checksum=…` の行が **1 行**ある（`no register was written` ＝ サーボに何も書いていない）。
- **`retried=0`、`failed_cycles=0`、`id_mismatch=0`、`answered_first=0`、`timeouts=0`、`bad_checksum=0`**（`bus:` の誤りのカウンタ。`flushed_requests`・`flushed_bytes`・`drained_bytes` は値を報告する。0 でなくてもよいかは未確認）。
- **この行が無い、または 0 でないカウンタがあれば、止めて報告する**（中止の基準 11）。

## 記録（報告すること）

1. 保存先 `~/so_arm101_check/zero_offset_<日付>/` の一覧と、`S0_sha256.txt`・`S4_sha256.txt`。
2. 段 1 の確認（6 関節、hz、INFO が出ないこと、`[diag]` の 2 行、gripper の閉じた値）と、段 6 の停止のログの行（`deactivated …` と `bus:` のカウンタ）。
3. 段 3 の各記録の表示（`recorded …` の行）と、各記録のときの gripper の値と、`not recorded` が出た回数と理由、90° の当たり方の目視の結果（固定あごが先か）。
4. 段 5 の fit の全出力（`S5_fit.log`）と終了コード。
5. 段 6 の ps と `ros2 node list --no-daemon` の結果。
6. 気になったこと（がたつき、異音、静止の取りにくさ、支え方の工夫、机の様子）。

材料は `inventory.md` に sha256 とともに控える（Claude が行う）。**yaml は変えない。** 貼る行を yaml に入れるのは F6 で、オフセットキャリブの結果をあなたと確認し、承認を得てから行う。
