# 実機確認手順: カメラの段 C2（腕を動かして 13 点を記録する）

2026-11-16 に実施し、すべて合格（記録は `docs/inventory.md` の「C2 の実機の確認（2026-11-16）」）。

計画: `docs/camera_stage_plan.md`（7 章 点の選び方、8 章 記録と道具、10 章 C2 の骨組み）。点の表と根拠: `docs/camera_pose_study.md`（1.1）。記録の道具: `tools/camera_record_run.py`（コミット `832b882`、`7ae0fbc`、記録は `docs/inventory.md` の「C2-a の記録の道具」）。測定値の書き込み: コミット `f566392`（`tag_mount.yaml`）。

> **この手順書のコマンドは、Claude Code が一つも実行していない（未実行）。** 実機（腕）もカメラも launch も、Claude は動かしていない。**W0 以降（トルク ON と移動）はユーザーだけが実行する。** 書いた内容は、リポジトリのファイルを読んで確かめた（確かめた場所は `ファイル:行` で書く）。読んで確かめられなかったことは「見込み」「推測」と書く。
>
> 段の名前の W は、**書き込み（トルク ON・移動）の段**。W の付かない段（0、1、2）は、腕のトルクを入れない。

この段で確かめること: (1) タグの取り付けの値が表示で合うこと、(2) 検出（`/detections`）とタグの TF（`/tf`）の stamp が実機で一致すること、(3) 13 点（P01〜P13）で、移動・整定・記録ができること。**Kabsch は C3。この段では計算しない。**

## 共通の約束

| 約束 | 内容 |
|---|---|
| 実行するのはユーザーだけ | 端末は A（腕の launch）、B（カメラの launch）、C（移動）、D（記録と確認）。A と B は、ログを流すだけで操作しない。止めるのは Ctrl-C だけ（`kill -9`・`pkill`・`killall` はしない） |
| 新しい端末 | **conda を activate していない**端末。端末 C と D は、それぞれ `cd ~/ros2_so-arm101` と準備の 3 行を実行する（下の「端末の準備」）。**`ROS_DOMAIN_ID` は設定しない（87 はモック専用）** |
| 腕を支える | 書き込みモードの起動の前に、腕を折りたたみ姿勢にして手で支える。止める前にも支える（Ctrl-C でトルクが OFF になり、腕は落ちる）。**この段は pitch −30° で、腕が x 0.315 m まで伸びる。M2・M3' より遠くへ出る** |
| 床 | 先端 z が 0.045 m 以上。腕の周り（x 0.45 m、y ±0.25 m の範囲）に物を置かない |
| 非常停止 | 12 V を切る。**常に片手を電源の近くに置く** |
| `--no-daemon` | `ros2 topic`・`ros2 node`・`ros2 param` には付ける。`ros2 control list_controllers` と `ros2 topic hz` は `--no-daemon` を受け付けないので付けない |
| ログ | `tee -i` と `; echo "exit=${PIPESTATUS[0]}"`。保存先は `~/so_arm101_check/camera_<日付>_C2/`（`<日付>` は実行する日。例 `20261117`。全部のコマンドで同じ日付に置き換える） |
| 同名のファイル | **書く前に `ls` で同名が無いことを確かめる**（`No such file or directory` が出るのが正常）。ik_move_run の `--csv` と `tee` は上書きする。camera の CSV は追記で、同じ label は拒否される |
| コードと設定 | **この段では何も変えない。** 気になることは報告だけ |
| 後始末 | 停止のたびに ps と `ros2 node list --no-daemon` を確認する（W5） |

**install 側の tag_mount.yaml は symlink ではなくコピー**（2026-11-16 に `ls -l` で確認。通常のファイル）。**yaml を直したら、`colcon build --packages-select so_arm101_camera` が要る。** この段では直さない。

## 端末の準備（端末 C・D。A・B は launch のコマンドの前に同じ 3 行を実行する）

```bash
cd ~/ros2_so-arm101
source /opt/ros/humble/setup.bash
source ~/ros2_so-arm101/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

（`ROS_DOMAIN_ID` は設定しない。）

## 端末の割り当て

| 端末 | 用途 | 使う段 |
|---|---|---|
| A | 腕の launch（`bringup.launch.py`）。ログを流す。操作しない | 2（読み取り専用）、W0〜W5（書き込みモード） |
| B | `camera.launch.py use_apriltag:=true`。ログを流す。操作しない | 2〜W5（途中で止めない。Ctrl-C は W5） |
| C | 移動（`tools/ik_move_run.py`。`--dry-run` も） | W1〜W3 |
| D | 記録（`tools/camera_record_run.py`）と確認のコマンド（`ls`・`ps`・`ros2 topic echo`・`sha256sum`） | 0〜W5 |

C と D を分ける理由: 両方が確認の Enter を標準入力で受け取る。

## 止め方（先に読む）

| 状況 | 何が止まるか | 操作 |
|---|---|---|
| ik_move_run の**確認の画面**（`Start from this state?`、`Send this trajectory?`） | **何も送らずに終わる**（腕は動いていない） | `n` と Enter、または Ctrl-C（終了コード 2 または 130）。30 秒の無入力でも `NOT_CONFIRMED` |
| 計画の最中 | 何も動いていない | Ctrl-C（130） |
| **送った後**（`ExecuteTrajectory` の実行中） | **道具の Ctrl-C は腕を止めない。** 進行中のゴールは軌道の最後まで進む | 道具の Ctrl-C は、何も取り消さない（終了コード 130） |
| **腕を止めたいとき** | S5: 端末 A で Ctrl-C を 1 回（**トルク OFF で腕が落ちる。腕を支える**）。S6: 12 V を切る（最後の手段） | これだけ（Servo は削除済みで、S1〜S4 は無い） |
| camera_record_run | 何も送らない。Ctrl-C は `interrupted: nothing was sent by this tool`、終了コード 130 | Ctrl-C |
| 端末 B（カメラの launch） | 腕は落ちない | Ctrl-C を 1 回。10 秒待って止まらなければ報告（`kill -9`・`pkill`・`killall` はしない。最後の手段は USB ケーブル） |

**動いている間は、片手を 12 V の電源の近くに置き、もう片手で腕の近くを支える**（落ちるときに受ける）。

**RViz と道具の排他**（RViz は端末 A の launch で開く）:

- W1 の T への移動と W4 の出発点への戻しだけ、RViz を使う。
- **W1 の終わりから W3 の終わりまで、RViz の Plan・Execute・マーカー操作に触れない**（RViz のウィンドウは開いたままでよい）。
- W1・W4 の RViz の操作中は、ik_move_run を起動しない。
- W4 の前に、RViz の Planning タブで Start State と Goal State を `<current>` に選び直す。

## 終了コード

**ik_move_run**（`tools/ik_move_run.py:23`、`large_move_real_check.md` の表と同じ）

| コード | 意味 | 扱い |
|---|---|---|
| 0 | `REACHED`（先端の誤差 10 mm 以内）、または `DRY_RUN` | 続ける |
| 2 | 停止（確認の拒否、`PLAN_REFUSED`、`ARM_MOVED`、`OTHER_GOAL`、`ALREADY_AT_GOAL`、引数の拒否） | **再試行しない。理由を報告**（`NOT_CONFIRMED` は何も送られていない。`ALREADY_AT_GOAL` は動いていない） |
| 3 | **開始の拒否（`START_REFUSED`）。確認の前なので、何も動かず何も失わない** | 中止して報告 |
| 4 | `SHORT`（先端の誤差が 10 mm 超） | **失敗ではない。止めない。** 値を記録して続ける。止める条件は「中止・相談の基準」（`stuck` が 0.05 rad 超、先端の z が 0.045 m 未満） |
| 130 | Ctrl-C | 進行中のゴールは止まっていない |

**camera_record_run**（`record.py:56-60`、`tools/camera_record_run.py:13`、`record.py:519-566`）

| コード | 意味 | 扱い |
|---|---|---|
| 0 | `--investigate` では「タグが見えた」（何も書かない）。記録では「行が書かれ、tag_seen が 1」 | 続ける |
| 2 | 拒否（tag_mount.yaml が null か不正、label の重複、meta の食い違い）、または確認が yes でない（Enter 以外の答え、30 秒の無入力）。**何も書かれていない** | 理由を読む。label の重複は別の label にしない（報告） |
| 3 | **タグが見えない。次の 2 つがある（`recorded … as row N` の行で見分ける）** | 下の 2 つ |
| 5 | 新しい `/joint_states` が無い。または、新しい CSV で camera_info と `camera_link` から光学フレームの TF が 5 秒以内に来ない（`tools/camera_record_run.py:210-217`）。何も書かれていない | 腕の launch とカメラの launch を確認して報告 |
| 130 | Ctrl-C。何も送っていない | |

camera の終了コード 3 の 2 つ:

1. **調査で見えない（何も書かれない）。** 最初の測定で一致したフレームが 8（`MATCH_MIN`、`record.py:49`）未満。確認の前に終わる（`record.py:539-541`）。`the tag is not seen: nothing was recorded. Stop and ask.` と出る。**`recorded` の行は無い。**
2. **再測定で見えない（tag_seen が 0 の行が書かれる）。** 調査では見えて Enter を押したあと、2 回目の測定で一致が 8 未満（`record.py:547-566`。まれなはず［推測］）。**`recorded P## as row N of …` の行が出る。** tag_seen が 0 の行が CSV に残る。

**「見えた」は一致したフレームの数だけで決まる**（`record.py:129-131`）。タグと視線の角度は使わない（表示だけ。`record.py:219`）。`cam_n`（外れ値を除いたあとの数）が 8 未満でも tag_seen は 1 のまま（`record.py:238`、`:251`、`:254`）。**同じ 3 でも、ik の 3 は「開始の拒否」、camera の 3 は「タグが見えない」。意味が違う。**

## 中止・相談の基準（どの段でも）

次のときは止めて、報告する（再試行しない）:

| 状況 | 扱い |
|---|---|
| **P01 が見えない**（camera 3 の 1 つ目） | 止める。D2（pitch）に戻る相談。−15° への切り替えは別の相談（12 点の表の作り直しが要る） |
| **P02 以降が見えない**（camera 3） | `--investigate` を **1 回だけ**やり直す。それでも 3 なら、ログに残して**次の点へ進む**（その点の行は無い。次の点の dry-run は着いた位置から検査される）。**見えない点が合計 3 つになったら止めて相談する** |
| **unmatched が多い**（stamp の不一致の疑い） | 調査の表示で、`detections` が 8 以上あるのに `matched` が 8 未満なら止める。`unmatched detections` が `detections` の半分を超えるときは注意して報告（半分は推測の値） |
| **still が 0**（`arm stopped: NO`） | 確認で `n` と答える（何も書かれない）。5 秒待って、同じ label でやり直すのは **1 回だけ**。それでも NO なら止めて報告 |
| ik の `SHORT`（終了コード 4） | 止めない（記録して続ける。`fk_` は止まってから測った `q_meas` から計算するので、データに影響しない）。**止めるのは、`stuck`（`desired − q_meas`）が 0.05 rad 超（推測の値）のとき、または先端の z が 0.045 m 未満のとき** |
| ik の `START_REFUSED`・`PLAN_REFUSED`・`ARM_MOVED`・`OTHER_GOAL`・`DESIRED_CHANGED`・`NOT_SETTLED` | 再試行しない |
| 計画の長さが 20 秒を超える（`DURATION`） | 再試行しない |
| ドライバの WARN、異音、`FAULT`・`COMMUNICATION LOST`・`read failed` | 止めて、支えながら Ctrl-C。原因を報告してから再開する |
| 停止後の `[deactivate]` の 2 行が無い | 腕を支えて 12 V を切る |
| ps が `static_transform_publisher` の数で合わない（下の「static_transform_publisher の確認」） | 止めて報告 |

**T でタグが見えなくても中止しない**（W1）。

## 0. 準備（ユーザー）

端末 D で（`cd` と準備の 3 行は上の「端末の準備」）:

1. 準備の 3 行を実行する（conda なし）。
2. **ros2 daemon を起動し直す**（cyclonedds を設定したこの端末で。ユーザーが行う。Claude は触らない）:

   ```bash
   ros2 daemon stop; ros2 daemon start; sleep 3
   ros2 daemon status
   ```
3. 前回の起動の残りが無いこと:

   ```bash
   ps aux | grep -E "realsense2_camera_node|apriltag_node|static_transform_publisher|ros2_control_node|move_group|robot_state_pub|rviz2|spawner|ik_move_run|camera_record_run" | grep -v -E "grep|gvfsd"
   ros2 node list --no-daemon
   ```

   **どちらも空であること**（`static_transform_publisher` は 0）。残っていたら止めて報告。
4. 保存先を作る。カメラが見える:

   ```bash
   mkdir -p ~/so_arm101_check/camera_<日付>_C2
   ls /dev/video* | head -3
   ```
5. リポジトリの状態（コードと設定が、記録のコミットのままであること）:

   ```bash
   git log -1 --format='%h %s'
   git status --short
   ```

   見込み: `git status --short` は空。`git log -1` は、この手順書のコミットか、その後のコミット。
6. install 側の tag_mount.yaml が、リポジトリのものと同じこと:

   ```bash
   sha256sum ~/ros2_so-arm101/src/so_arm101_camera/config/tag_mount.yaml ~/ros2_so-arm101/install/so_arm101_camera/share/so_arm101_camera/config/tag_mount.yaml
   ```

   **合格: 2 つの sha256 が同じ**（2026-11-16 の値は `51fc015e476340de33a3a3c1577059aaed0145959c9abaa54273bb296c1044fa`）。違えば止めて報告（ビルドが要る）。

カメラを USB 3 のポートに挿す。**カメラが挿さっていないと、ドライバは起きるまで待ち続ける**（C1 と同じ。Ctrl-C で止めて報告）。

## 1. tag_mount.yaml の測定値と T の表示（腕・カメラ・launch なし）

**測定値の書き込みは済んでいる**（コミット `f566392`）。`config/tag_mount.yaml` の値:

- `reference_point_gripper_frame_m: [0.0, 0.0, 0.0063]`（固定あごの先端。URDF と mesh から計算）
- `ref_to_tag_gripper_frame_m: [-0.038, 0.0, -0.062]`（ノギスの測定。前 38 mm は台紙の厚みを含めた印刷面まで、上は先端の端から 62 mm、左右 0）
- 足した値（FK が使うオフセット）: (−0.038, 0.0, −0.0557) m

何も起動していない端末 D で、表示を確かめる（ノードは作るが、何も出版せず、カメラもシリアルも開かない）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/S1_display.log
timeout 60 python3 -u tools/camera_record_run.py --investigate 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/S1_display.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。
- **合格:** 次の 2 行のあと、`no fresh joint angles: nothing was recorded`、`exit=5`（約 20 秒。`tag_mount.py:183-192`）。
  - `tag centre seen from the origin of gripper_frame_link (tip pointing down): front 38.0 mm, up 55.7 mm, left 0.0 mm`
  - `  of which reference point to tag centre (measured): front 38.0 mm, up 62.0 mm, left 0.0 mm`
- 数字が違う、または `refused:` が出る（`exit=2`）なら、止めて報告。

## 2. 軸の対応表と、見え方の確認（読み取り専用。トルク OFF）

ここでは**腕のトルクを入れない**。次の 2 つを、動かす前に確かめる:

1. gripper_frame_link の軸の対応表を、実物で確かめる。
2. `/detections` と `/tf` のタグの stamp が、実機で一致するか。

**軸の対応表**（T の姿勢、先端が真下のとき。`tag_mount.py:49-53`）:

| gripper_frame_link の軸 | T の姿勢での向き | 表示の語 |
|---|---|---|
| +x | 後ろ（ロボットの根元の方） | 前は −x |
| +y | 左 | 左は +y |
| +z | 下 | 上は −z |

**この表は T の姿勢だけで使える。pitch −30° の 12 点では、前と上が混ざるので使わない。**

起動（腕は折りたたみ姿勢のまま。読み取り専用）。

端末 A（準備の 3 行のあと）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/S2_arm_launch.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=true use_rviz:=false 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/S2_arm_launch.log; echo "exit=${PIPESTATUS[0]}"
```

端末 B（準備の 3 行のあと）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/camera_launch.log
ros2 launch so_arm101_camera camera.launch.py use_apriltag:=true 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/camera_launch.log; echo "exit=${PIPESTATUS[0]}"
```

- 各 `ls` の期待: `No such file or directory`（正常）。
- **端末 B は、W5 まで止めない**（カメラの launch は 1 回だけ。ログは 1 本）。
- 10 秒ほど待つ。

### static_transform_publisher の確認（順に）

端末 D で、**3 回**行い、数を記録する（`S2_stp.txt` に追記。書く前に `ls`）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/S2_stp.txt
ps aux | grep -E "static_transform_publisher" | grep -v -E "grep|gvfsd" | wc -l
```

| 時点 | 期待 | 理由 |
|---|---|---|
| 腕の launch の前（段 0 の ps） | **0** | 前回の起動の残りが無い |
| 腕の launch の後（端末 A を起動して 10 秒後。カメラの launch の前でも後でもよい） | **1** | moveit の仮想関節（`world` から `base_link`） |
| カメラの launch の後 | **1 のまま（増えない）** | `camera.launch.py` は extrinsics が無いと TF を出さない。camera_link の static_transform_publisher は出ない（C1-1d の合格） |

さらに、`/tf_static` に `child_frame_id: camera_link` が無いこと（C1-1d と同じ）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/S2_tf_static_raw.txt
timeout 10 ros2 topic echo --no-daemon --once --qos-durability transient_local --qos-reliability reliable /tf_static \
  > ~/so_arm101_check/camera_<日付>_C2/S2_tf_static_raw.txt 2>&1
grep -c "child_frame_id: camera_link" ~/so_arm101_check/camera_<日付>_C2/S2_tf_static_raw.txt
```

- **合格: 0**。

**数が 1 でない（0 や 2）なら止めて報告**（TF の二重の発行、または仮想関節が無い。bringup は変えない）。

### 調査（読み取り専用。腕は手で置く）

**まず、腕をそのまま（折りたたみ姿勢で）**、端末 D で:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/S2_inv_rest.log
python3 -u tools/camera_record_run.py --investigate 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/S2_inv_rest.log; echo "exit=${PIPESTATUS[0]}"
```

- 見える見込みは低い（折りたたみ姿勢は床の下で、カメラから隠れるかもしれない［推測］）。**見えなくても（終了コード 3）、中止しない。** `frames:` の行（detections、tag poses、matched、unmatched）を控える。
- **次に、腕を手で T の付近に置く**（先端が真下、ほぼ T。トルクが OFF なので、手で支える。手でタグを隠さない）。支えたまま、同じコマンドを `S2_inv_T.log` に:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/S2_inv_T.log
python3 -u tools/camera_record_run.py --investigate 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/S2_inv_T.log; echo "exit=${PIPESTATUS[0]}"
```

- **腕を手で支えているので、`arm stopped: NO` は出てよい**（この段では still を合否にしない）。
- **見るもの:** `frames: detections N, tag poses N, matched N, unmatched detections N, unmatched poses N`。
  - **合格（stamp）: `detections` と `tag poses` が 0 でなく、`matched` が `detections` の半分以上。**
  - **止める: `detections` が 8 以上あるのに `matched` が 8 未満**（stamp の不一致）。ログを保存して報告（腕は動かさない）。
  - `detections` が 0 なら、見えていない（タグが画像の外、遠い、斜めすぎるなど）。置き方を変えて 1 回だけやり直す。それでも 0 なら報告。
- **軸の対応表を実物で確かめる:** T の付近で、**タグの中心が、先端（固定あごの先端）より前（ロボットの外向き）で、上にある**こと。表示の `front 38.0 mm, up 55.7 mm`（上の段 1）と、見た目が合うこと。左右は 0 で、y の符号は実物では確かめられない（結果にも影響しない）。合わなければ止めて報告。

**停止:** 端末 A（腕の launch）を、腕を支えながら Ctrl-C で 1 回（読み取り専用なので、トルクは入っていない）。`deactivated (no register was written)` の行と `exit=0`。**端末 B（カメラ）は止めない。**

## W0. 書き込みモードで起動（トルク ON）

**ここからトルク ON。** 腕を折りたたみ姿勢にして、前腕の下を支える。

端末 A:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/W0_arm_launch.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false torque_limit:=500 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/W0_arm_launch.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。bag は取らない。

端末 D で（10 秒ほど待ってから）:

```bash
ros2 control list_controllers
ls ~/so_arm101_check/camera_<日付>_C2/C2_start_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C2/C2_start_pose.txt
ps aux | grep -E "static_transform_publisher" | grep -v -E "grep|gvfsd" | wc -l
```

- `ros2 control list_controllers`（`--no-daemon` は付けない）で `arm_controller` が active。
- コマンド A（`servo_real_check.md:47`。`large_move_real_check.md` の M1 と同じ）で `STOPPED`。ドライバの WARN なし。
- **`C2_start_pose.txt` が、出発点の実測。W4 で戻す目標の元になる。**
- `ls` の期待: `No such file or directory`（正常）。
- `wc -l` は **1**（moveit の仮想関節。カメラの launch で増えていない）。

## W1. RViz で T へ。T で調査

RViz（端末 A の launch で開く）の MotionPlanning で、**Planner は OMPL**、**Velocity Scaling と Acceleration Scaling は 0.1 のまま**。Planning タブで **Start State と Goal State を `<current>` に選び直してから**、Joints タブで整数の度に設定する: **shoulder_pan 0、shoulder_lift −22、elbow_flex +30、wrist_flex +83、wrist_roll 0**（gripper は触らない）。Plan してアニメーションを見てから（先端が下がらない・動かさない関節が動かない）、Execute。**動いている間は、片手を 12 V の近くに置き、もう片手で腕を支える。**

止まったら、端末 D で実測を控える:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/C2_T_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C2/C2_T_pose.txt
```

次に、T で調査する（何も送らない）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/W1_inv_T.log
python3 -u tools/camera_record_run.py --investigate 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/W1_inv_T.log; echo "exit=${PIPESTATUS[0]}"
```

- **T ではタグが見えないことがありうる**［推測］。道具は、タグと視線の角度では「見えた」を決めない（一致したフレームが 8 未満なら終了コード 3。`record.py:129-131`）。60° は研究の仮の値で（`camera_pose_study.md:50`）、C1 で検出を確かめたのは約 38° まで。T でのタグは、研究の見積もりではカメラ A から 63°（`camera_pose_study.md:106`）。
- **終了コード 3 でも中止しない。** `frames:` の行（detections、tag poses、matched）を報告する。
- **止める: `detections` が 8 以上あるのに `matched` が 8 未満**（stamp の不一致）。

動かさない確認として、P01 の dry-run を T から実行してよい（W2 の P01 (a) と同じ。ここで通れば、W2 の P01 の dry-run は繰り返す）。

## W2. 13 点

### 点の表（`camera_pose_study.md` の 1.1、21〜32 行の表から写した。`--goal` は m、全点に `--pitch-deg -30.0 --roll-deg 0`）

| 点 | goal x y z | 元の行 | 前の点からの先端の移動（概算） |
|---|---|---|---|
| P01 | 0.245 -0.020 0.140 | 21 | T から約 108 mm |
| P02 | 0.315 -0.020 0.140 | 22 | 70 mm |
| P03 | 0.315 0.040 0.140 | 23 | 60 mm |
| P04 | 0.245 0.040 0.140 | 24 | 70 mm |
| P05 | 0.245 0.100 0.140 | 25 | 60 mm |
| P06 | 0.315 0.100 0.140 | 26 | 70 mm |
| P07 | 0.315 0.100 0.060 | 27 | 80 mm |
| P08 | 0.245 0.100 0.060 | 28 | 70 mm |
| P09 | 0.245 0.040 0.060 | 29 | 60 mm |
| P10 | 0.315 0.040 0.060 | 30 | 70 mm |
| P11 | 0.315 -0.020 0.060 | 31 | 60 mm |
| P12 | 0.245 -0.020 0.060 | 32 | 70 mm |
| P13 | 0.245 -0.020 0.140 | 21（P01 の再訪） | 80 mm |

全区間が、先端の移動量の上限 0.15 m（`ik_move.py:57`）より小さい。**P12 から P13 と、P13 から T は、研究の検査の連鎖に入っていない**（研究は P12 まで）。dry-run で確かめる。P12 から T は 1 回では通らない（wrist_flex が 1.0 rad を超える）ので、P13（P01 の再訪）を経由する。

### 1 点分の手順

点ごとに、次の 4 つを順に行う。**(b) の移動だけが、腕を動かす。** (a) の dry-run と (c)(d) は、何も送らない。

**P01 の完全なコマンド**（P02 以降は、点の名前と `--goal` の 3 つの数と、ファイル名の `P01` を表の値に置き換える）:

**(a) dry-run**（端末 C。何も送らず、計画もせず、CSV も作らない。今の測定の位置から検査する）:

```bash
cd ~/ros2_so-arm101
ls ~/so_arm101_check/camera_<日付>_C2/P01_dry.log
python3 -u tools/ik_move_run.py --dry-run --goal 0.245 -0.020 0.140 --pitch-deg -30.0 --roll-deg 0 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/P01_dry.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。
- **合格: `DRY-RUN: the goal passes every check`、`exit=0`。** 通らなければ（`exit=2` や `3`）止めて報告（その点は飛ばさない）。

**(b) 移動**（端末 C。**トルク ON のまま動く。** 確認は 2 回、各 30 秒）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/P01_move.csv ~/so_arm101_check/camera_<日付>_C2/P01_move.log
python3 -u tools/ik_move_run.py --goal 0.245 -0.020 0.140 --pitch-deg -30.0 --roll-deg 0 \
  --csv ~/so_arm101_check/camera_<日付>_C2/P01_move.csv 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/P01_move.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: 2 つとも `No such file or directory`（正常）。
- 手順: ① 開始の確認（`GOAL` の表）を読んで Enter。② 計画の行（`PLAN OK`、長さ、`lowest z`、`last vs goal`）を読む。`PLAN REFUSED` なら何も動かない（報告）。③ **腕を支え、12 V の近くに手を置いて**、送信の確認で Enter。④ 止まるのを待つ（整定 1.5〜8 秒）。⑤ `ARRIVAL` の表を読む。
- 動く時間の見込み（推測）: 60〜80 mm の移動で約 3.5〜5 秒（M2 の 32 mm は約 3 秒、M3' の 86 mm は約 5 秒）。**T から P01 と、P13 から T は 6〜8 秒**。計画の行で長さを読み、10 秒を超えるときは報告（20 秒を超えたら `DURATION` で拒否される）。
- **見るもの:** `ARRIVAL` の先端の誤差 mm、`desired − q_meas`（`stuck`）、`lowest z`（0.045 以上）。
- **合格:** `REACHED`（終了コード 0）。`SHORT`（終了コード 4）も、次の条件で続ける: `stuck` が 0.05 rad 以下で、先端の z が 0.045 m 以上。値を報告用に控える。

**(c) 調査**（端末 D。**P01 だけ別に実行**。何も書かない。P02 以降は (d) の最初の表示が調査になるので、別には行わない）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/P01_inv.log
python3 -u tools/camera_record_run.py --investigate 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/P01_inv.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。
- **見るもの:** `frames:` の行（`matched` が 8 以上）、`arm stopped: yes`、`detection rate`、`incidence`（表示だけ）、`tag seen: yes`。
- **P01 が `exit=3`（見えない）なら、止めて報告。** P01 の調査は、最初の stamp の確認でもある（stamp の不一致は「中止・相談の基準」）。

**(d) 記録**（端末 D。調査 → Enter → 記録）:

P01（新しい CSV）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/c2_points.csv ~/so_arm101_check/camera_<日付>_C2/c2_points.csv.meta.yaml ~/so_arm101_check/camera_<日付>_C2/P01_rec.log
python3 -u tools/camera_record_run.py --csv ~/so_arm101_check/camera_<日付>_C2/c2_points.csv --label P01 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/P01_rec.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待（P01）: **3 つとも `No such file or directory`（正常）**。CSV か meta が**ある**なら、前回の残り。**実行せず、止めて報告**（`refused:` になる）。

P02〜P13（同じ CSV に追記。例は P02）:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/c2_points.csv ~/so_arm101_check/camera_<日付>_C2/c2_points.csv.meta.yaml
ls ~/so_arm101_check/camera_<日付>_C2/P02_rec.log
python3 -u tools/camera_record_run.py --csv ~/so_arm101_check/camera_<日付>_C2/c2_points.csv --label P02 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/P02_rec.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待（P02 以降）: **1 つ目の `ls` は 2 つとも見つかる（正常。`No such file` が出たら、CSV の名前か場所が違う。実行せず報告）。2 つ目の `ls`（`P02_rec.log`）は `No such file or directory`（正常）。**
- 流れ: 調査の表示（`frames:` など）→ `Record this point as P02? (Enter = yes, anything else = no)` → **Enter**（30 秒以内）→ 再測定 → `recorded P02 as row N of …`。
- **Enter の前に、調査の表示を読む:** `tag seen: yes`、`arm stopped: yes`、`matched` が 8 以上。`arm stopped: NO` なら `n` と答える（何も書かれない。5 秒待って 1 回だけやり直す）。
- **合格:** `recorded … as row N`、`exit=0`。

見えないとき（`exit=3`）の扱い:

- **P01:** 止めて報告。
- **P02 以降:** 調査で見えない（`recorded` の行が無い）なら、`--investigate` を 1 回やり直す（ログ `P02_inv.log`。書く前に `ls`）。それでも `exit=3` なら、**ログに残して次の点へ進む**（その点の行は無い）。**見えない点が合計 3 つになったら、止めて相談する。**
- 再測定で見えない（`recorded P02 as row N` の行がある。tag_seen が 0 の行が書かれた）: 同じ label では書き直せない。そのまま次の点へ進み、見えない点として数える。

### 順路

1. **T → P01 → P02 → … → P12 → P13（P01 の再訪）。** 各点で (a)〜(d) を行う（P01 以外の (c) は省く）。
2. P13 は、P01 と同じ `--goal 0.245 -0.020 0.140` を、label `P13` で。P12 の下の層から上の層へ上がるので、P01 と P13 の差が、行きと帰りの差（戻りの再現性）の目安になる。
3. 1 点で 2 分前後、13 点で約 30 分（推測）。

## W3. 点 1 から T へ

P13 の記録のあと、端末 C で、まず dry-run:

```bash
cd ~/ros2_so-arm101
ls ~/so_arm101_check/camera_<日付>_C2/W3_T_dry.log
python3 -u tools/ik_move_run.py --dry-run --joints-deg 0 -22 30 83 0 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/W3_T_dry.log; echo "exit=${PIPESTATUS[0]}"
```

通ったら（`exit=0`）、移動:

```bash
ls ~/so_arm101_check/camera_<日付>_C2/W3_T.csv ~/so_arm101_check/camera_<日付>_C2/W3_T.log
python3 -u tools/ik_move_run.py --joints-deg 0 -22 30 83 0 \
  --csv ~/so_arm101_check/camera_<日付>_C2/W3_T.csv 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C2/W3_T.log; echo "exit=${PIPESTATUS[0]}"
```

- `ls` の期待: `No such file or directory`（正常）。
- 6〜8 秒動く（推測）。**手を電源の近くに。**
- 止まったら、コマンド A で `STOPPED`。`C2_T_pose2.txt` に実測を控える（`ros2 topic echo --once --no-daemon --field position /joint_states`。書く前に `ls`）。
- `ALREADY_AT_GOAL`（終了コード 2）は、すでに T にいて動いていないという意味（報告だけ）。

## W4. RViz で出発点へ戻す

**出発点は床の下（先端 z ≈ 0.033 m）なので、ik_move_run では戻せない**（`GOAL_FLOOR` で拒否される）。RViz で戻す。

1. コマンド A で `STOPPED`。
2. RViz の Planning タブで、**Start State と Goal State を `<current>` に選び直す**（ik_move_run で動かしたので、RViz の表示の状態は古い）。
3. Joints タブで、**`C2_start_pose.txt` の実測に近い整数の度**に設定する（rad から度に直して四捨五入）。2026-11-15 の V8 の目標（pan −4、lift −85、elbow +96、wrist_flex +25、roll −3 度）は補正前の座標の参考で、**F6 の補正の後は使わない**（elbow +96° は補正後の真の上限 +92.2° を超える）。今日の起動直後の実測を使う。**elbow_flex は +91°（+1.588 rad）を超える値にしない**（補正後の真の上限は +92.2° = +1.6093 rad。+1.69009 は補正前の座標の値）。
4. Plan → アニメーションを見る（先端が床に近づきすぎない）→ Execute。実行の開始位置と実測の差が 0.01 rad 以内であること（超えると実行が拒否される。拒否されたら、実測を読み直して計画し直す）。

## W5. 停止と後始末

1. **腕を折りたたみ姿勢で支える。** 端末 A で Ctrl-C を 1 回（S5）。
2. 端末 A のログに、次の 2 行があること:
   - `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints`
   - `deactivated. reads=…`

   無ければ、腕を支えて 12 V を切る（S6）。`move_group`・`rviz2` の exit code −11 は、`deactivated.` の後なら無視してよい。
3. 端末 B（カメラ）で Ctrl-C を 1 回（10 秒待つ）。
4. **数秒待ってから**、端末 D で:

   ```bash
   ps aux | grep -E "realsense2_camera_node|apriltag_node|static_transform_publisher|ros2_control_node|move_group|robot_state_pub|rviz2|spawner|ik_move_run|camera_record_run" | grep -v -E "grep|gvfsd"
   ros2 node list --no-daemon
   ls /dev/video* | head -3
   ```

   **合格: ps が空、`ros2 node list --no-daemon` が空、カメラが見える。**
5. `ros2 node list --no-daemon` にだけ残り、ps が空なら、数秒待ってもう一度確認する。それでも残るなら、報告して指示を待つ。**ros2 daemon には触らない**（`stop`・`start` をしない）。ps にプロセスが残るときは、`kill -9`・`pkill`・`killall` をしない（報告して指示を待つ）。

## 合格の基準

| 項目 | 基準 |
|---|---|
| 段 0・1 | ps と node list が空。tag_mount.yaml の sha256 が src と install で一致。T の表示が `front 38.0, up 55.7, left 0.0` と `front 38.0, up 62.0, left 0.0`、`exit=5` |
| 段 2 | `static_transform_publisher` の数が 0、1、1。`child_frame_id: camera_link` が 0 件。stamp が一致（`matched` が `detections` の半分以上）。軸の対応表が実物と合う |
| 移動（P01〜P13、T への戻し） | 各移動が `REACHED`、または `SHORT` で `stuck` が 0.05 rad 以下かつ先端の z が 0.045 m 以上。`FAULT`・WARN・異音なし |
| 記録 | **P01 が見える。12 点（P01〜P12）のうち見える点が 10 以上**（見えない点が 2 つ以内）。記録した行は still が 1。**tag_seen が 1 の行の `cam_n` が 8 以上** |
| 停止 | `[deactivate]` の 2 行。ps と node list が空 |

（`cam_n` の 8 は、`MATCH_MIN` に合わせた推測の値。`cam_n` が 8 未満でも tag_seen は 1 のままなので、行を数えるときに別に見る。）

**C2 は、ここで Kabsch を計算しない。** 合否の最終判断は、C3 の入力として使えるかで決める。

## 記録に残すもの（保存先 `~/so_arm101_check/camera_<日付>_C2/`）

- 段ごとのログ: `S1_display.log`、`S2_*.log`、`S2_stp.txt`、`S2_tf_static_raw.txt`、`W0_arm_launch.log`、`camera_launch.log`、`W1_inv_T.log`。
- 点ごとのログと CSV: `P*_dry.log`、`P*_move.csv`、`P*_move.log`、`P01_inv.log`、`P*_rec.log`。見えなかった点の再調査のログ（`P*_inv.log`）。
- カメラの記録: `c2_points.csv` と `c2_points.csv.meta.yaml`。
- 腕の実測: `C2_start_pose.txt`、`C2_T_pose.txt`、`C2_T_pose2.txt`、`W3_T_dry.log`、`W3_T.csv`、`W3_T.log`。
- **すべてのファイルの sha256**（書く前に `ls`）:

  ```bash
  ls ~/so_arm101_check/camera_<日付>_C2/SHA256SUMS.txt
  cd ~/so_arm101_check/camera_<日付>_C2 && sha256sum * 2>/dev/null | tee SHA256SUMS.txt
  cd ~/ros2_so-arm101
  ls -l ~/so_arm101_check/camera_<日付>_C2/
  git log -1 --format='%H %s'
  ```
- tag_mount.yaml の sha256（meta の `identity` にも入る）。使ったコミットの hash（`git log -1`）。
- 気づいたこと（異音、見え方、タグの位置、P01 と P13 の差、still が 0 だった点）。

結果の記録（`docs/inventory.md`）は、あとで Claude が docs だけのコミットで行う。**CSV の内容は、Claude が読み取りだけで確かめる。**

## 報告してほしいもの

- 段 1（`S1_display.log`）の 2 行と `exit=`。
- 段 2: `static_transform_publisher` の数 3 つ、`camera_link` が子の行の数、調査の `frames:` の行（折りたたみ姿勢と T の付近）、軸の対応表の見た目の結果。
- W0〜W1: `C2_start_pose.txt`、`C2_T_pose.txt`、T の調査の `frames:` の行と `exit=`。
- 13 点の表: 点ごとの `exit=`（dry-run、移動、記録）、移動の `REACHED` か `SHORT`、見えない点があれば、それが「調査で見えない」か「再測定で見えない」か。
- 停止後の 2 行と、ps・node list の結果。
- 保存したファイルの置き場所。
