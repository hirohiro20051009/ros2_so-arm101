# 実機確認手順: F6 の補正後の確認（elbow_flex の zero_offset_deg = −6.3 を入れたあと）

**F6b。** 計画: `docs/f6_plan.md`（4 章が本書の元。項目 6 の決定を含む）。コード・yaml: コミット `76d5654`（F6a）。元の計画: `docs/joint_offset_plan.md`（7.4 の R0〜R3 は本書の a〜f に置き換わる）、`docs/zero_offset_calib_plan.md`（6 章）。

> **この手順書のコマンドは、Claude Code が一つも実行していない（未実行）。** 実機（腕）も launch も、Claude は動かしていない。**実行するのはユーザーだけ。**
>
> **補正が効く時点（F6a のコミット `76d5654` 以降）**: `install/so_arm101_hardware/share/so_arm101_hardware/config/follower_calibration.yaml` は src の yaml への symlink（`ls -l`: `follower_calibration.yaml -> /home/hirok/ros2_so-arm101/src/so_arm101_hardware/config/follower_calibration.yaml`）。**F6a のコミットの後は、再ビルドなしで、次の実機の launch から elbow の補正（−6.3°）が効く**。ドライバは起動のとき（on_init）に 1 回だけ読む。走っている launch には効かない。**この手順書の a 段が、補正つきの最初の実機の launch になる。**
>
> **これは補正の確認であって、センターキャリブでもオフセットキャリブでもない。** yaml・URDF・コードは変えない。気になることは報告だけ。
>
> **−6.3° は暫定。** 根拠（カメラ C2 の −6.31 ± 0.38、R1 の物差し）は、どちらも wrist_roll のねじが緩んだ状態（修理前）のデータ。床当て（2026-11-17）は使わない。この手順の b・c の物差しは、修理後の最初の独立の確認になる。決着は C4（机の ID 1）。

この手順で確かめること: (1) 新しいバイナリが yaml の補正を読んだこと（起動の INFO）、(2) 補正後の読みが予測の範囲にあること、(3) 修理後の wrist_roll の出発点の読みが修理前と同じ（ホーンの歯 1 つ分のずれが無い）こと、(4) 物差しの予測との一致（出発点 31.3 mm、T 53.3 mm）、(5) 補正後に有効化・T への移動・大きな移動・DiffMode の小さな 1 歩・出発点への戻しが問題なく動くこと。

## 共通の約束

| 約束 | 内容 |
|---|---|
| 実行するのはユーザーだけ | 端末は A（腕の launch）、C（移動の道具）、D（確認のコマンド）。A はログを流すだけで操作しない。端末の準備・割り当て・`ROS_DOMAIN_ID` を設定しないこと・`--no-daemon` の扱いは `camera_c2_real_check.md` の「共通の約束」「端末の準備」と同じ（カメラの端末 B は使わない） |
| 腕を支える | トルク ON の起動の前に、腕を折りたたみ姿勢にして手で支える。止める前にも支える（Ctrl-C でトルクが OFF になり、腕は落ちる）。**動いている間は、片手を 12 V の電源の近くに置く** |
| 床 | 先端 z が 0.045 m 以上。腕の周り（x 0.45 m、y ±0.25 m）に物を置かない |
| ログ | `tee -i` と `; echo "exit=${PIPESTATUS[0]}"`。保存先は `~/so_arm101_check/zero_offset_apply_<日付>/`（`<日付>` は実行する日。例 `20261118`）。先に `mkdir -p` |
| 同名のファイル | **書く前に `ls` で同名が無いことを確かめる**（`No such file or directory` が出るのが正常） |
| コードと設定 | **この手順では何も変えない。** yaml・URDF・コードのどれも |
| 後始末 | 停止のたびに ps と `ros2 node list --no-daemon` を確認する（f 段）。ros2 daemon には触らない |
| 非常停止 | 12 V を切る（最後の手段） |

### 共通の止める条件（どの段でも。出たら、その場で止めて、腕を支えて Ctrl-C。原因を報告してから再開する）

- A のログに `FAULT:`・`COMMUNICATION LOST`・`read failed`・`step limit`・**`clamp: '`**（J2 の WARN。文面は `clamp: 'elbow_flex' command maps to raw …`）が出る。
- 異音・腕の予期しない動き。
- **RViz の Joints タブで elbow_flex を +91° より上にしない**（段 c の T、段 e の戻し先を含む。補正後の真の上限は +92.20° = +1.6093 rad。+91° = +1.588 rad を超える Goal は Plan しない）。

## 保存先を作る

```bash
mkdir -p ~/so_arm101_check/zero_offset_apply_<日付>
```

以下、ファイル名の前の `~/so_arm101_check/zero_offset_apply_<日付>/` は `$D` と略す（実際には全部書く）。

## 段 a. 読み取り専用で起動（腕は動かさない。トルク OFF。出発点の折りたたみ姿勢）

**補正つきの最初の起動。** 腕を、前回までと同じ「支えて戻す」たたみ方で、折りたたみ姿勢にする（手で支える。トルク OFF なので自重で動く）。

端末 A:

```bash
ls ~/so_arm101_check/zero_offset_apply_<日付>/a_arm_launch.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=true 2>&1 \
  | tee -i ~/so_arm101_check/zero_offset_apply_<日付>/a_arm_launch.log; echo "exit=${PIPESTATUS[0]}"
```

端末 D（10 秒ほど待ってから）:

```bash
grep -n "zero_offset_deg\|\[diag\]" ~/so_arm101_check/zero_offset_apply_<日付>/a_arm_launch.log
ls ~/so_arm101_check/zero_offset_apply_<日付>/a_start_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/zero_offset_apply_<日付>/a_start_pose.txt
```

`a_start_pose.txt` の 6 つの値は、pan、lift、elbow、wrist_flex、wrist_roll、gripper の順（rad）。

### a-1. 起動の INFO と補正後の elbow の読み

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| INFO 行（A のログ。`zero_offset_deg:` を含む行の数） | **ちょうど 1 行**（elbow_flex だけ）: `zero_offset_deg: elbow_flex -6.30 deg (reported position = measured position + this); clamp tolerance low=50 high=122 ticks (default 50)` | 行が無い（古いバイナリか、yaml が読まれていない）／2 行以上／関節名が elbow_flex でない／値が `-6.30` でない／**`low=50` でない／`high=122` でない** |
| high の根拠（コード。F6a の計画 D） | `calibration.cpp:183` の s = −direction × zero_offset_deg × 4095 / 360 = +71.66（direction +1、δ −6.3）。`:184-185` で high = 50 + ceil(s) = **122**。`:181` の初期値から low = 50 | 上に同じ |
| `[diag]` | 2 行とも `6/6 joints match`（EEPROM は書いていない） | 6/6 でない行がある |
| elbow_flex の読み（3 番目の値） | **約 +89.7°（+1.565 rad）**。根拠: R1 の出発点 +1.67475 rad（95.95°）− 6.3° = 89.65°。補正前の出発点は毎回 1〜2° ばらつく（過去 1.6747、1.6824、1.7054 rad → 補正後 89.65°、90.09°、91.42°）。**許容: +88.0° 以上 +92.20° 以下（+1.6093 rad）** | 88.0° 未満／**92.20° 超**（下の a-3） |
| gripper の読み（6 番目） | 約 0.028（閉じた値 0.02811。変動 ±0.0012） | 0.02811 から 0.005 rad 超ずれる |
| 停止後 | f 段と同じ | f 段 |

### a-2. 修理前の出発点との比較（wrist_roll のねじの修理。ホーンは外して付け直した。印を付けて戻したので、ずれはほぼ無い見込み）

目的: ホーンのスプラインの歯 1 つ分のずれ（約 14°。歯数 25 は推測。歯数は数えない）が無いことを確かめる。修理前の出発点の読み（センターキャリブ C3 の後の座標。補正なし。rad。`~/so_arm101_check/` の下の材料。読むだけ）:

| 材料 | pan | lift | elbow（補正前） | wrist_flex | wrist_roll | gripper |
|---|---|---|---|---|---|---|
| `camera_20261116_C2/C2_start_pose.txt`（2026-11-16 17:37） | −0.06598 | −1.50597 | +1.70544 | +0.66284 | −0.04373 | 0.02811 |
| `camera_20261116_C2/C2_end_pose.txt`（18:07） | −0.06598 | −1.49830 | +1.67475 | +0.66898 | −0.04680 | 0.02811 |
| `camera_20261116_C3/R1_js.txt`（19:49） | −0.06905 | −1.49830 | +1.67475 | +0.66898 | −0.04833 | 0.02811 |
| `zero_offset_20261117/S1_gripper_closed.txt`（2026-11-17 15:50） | −0.11047 | −1.49369 | +1.68242 | +0.85003 | −0.05294 | 0.02811 |
| 範囲 | −0.1105〜−0.0660 | −1.5060〜−1.4937 | 補正後 +1.5648〜+1.5955 | +0.6628〜+0.8500 | **−0.0529〜−0.0437** | 0.02811 |

修理前の wrist_roll の 4 回の幅は 0.0092 rad = **0.53°**（−2.51°〜−3.03°）。

今日の `a_start_pose.txt` を上の範囲と比べる。**wrist_roll は、範囲の外へ出た量 |Δ|（範囲の内なら 0）で判定する。**

| 関節 | 予測 | 判定 |
|---|---|---|
| wrist_roll | 範囲 −0.0529〜−0.0437 の内、または外へ 0.02 rad（1.1°）以内 | **0.02 rad 以内: 合格**。**0.02〜0.052 rad（1.1〜3.0°）: 注意域。続行してよいが、値を記録に残し、報告に書く**。**0.052 rad（3.0°）超: 止める**（書き込みへ進まない。下の読み方で報告） |
| pan | 範囲の外へ 0.03 rad（1.7°）以内 | 0.03 rad 超なら止める |
| lift | 範囲の外へ 0.03 rad 以内 | 0.03 rad 超なら止める |
| wrist_flex | 範囲 +0.6628〜+0.8500 の外へ 0.10 rad（5.7°）以内 | 0.10 rad 超なら止める |

- ばらつきの幅 0.53° の約 6 倍が止める値 3.0°（偶然のばらつきではまず超えない）、歯 1 つ分 14.4° の約 1/5（歯 1 つ分のずれなら確実に超える）。注意域の 1.1° は、ばらつきの幅の約 2 倍（印を付けて戻した小さなずれの範囲）。
- **止めたとき（wrist_roll が 0.052 rad 超）の読み方**（|Δ| で）:
  - **0.052〜0.15 rad（3.0〜8.6°）**: 歯 1 つ分ではない（組み付けのわずかなずれ、または出発点のたたみ方）。**止めて報告**。
  - **0.15〜0.33 rad（8.6〜19°。歯 1 つ分 0.251 rad の前後）**: **ホーンの歯のずれを疑う**。止めて報告し、センターキャリブ（wrist_roll の center）の確認を先に行う（`so_arm101_calibration_check.md`。ユーザーが実施）。
  - **0.33 rad 超**: ほかの原因（ホーンが別の向き、中心が大きく動いた）。止めて報告。
- **0.02 rad を超えて 0.25 rad 以内に外れたときの扱い**: 止めて報告する（0.02〜0.052 rad の注意域は続行可）。ただし、**wrist_roll を手で真っ直ぐに戻して（トルク OFF なので手で動かせる。固定あごが下の向きに揃える。無理に押し込まない）、1 回だけ読み直してよい**。読み直した値で上の表を当てはめ直す。読み直しても外れていれば、そのまま止めて報告（再試行はしない）。
- 目視の補助（任意）: 付けた印が合っていること、固定あごが下・動くあごが上の向きが修理前と同じに見えること。
- 前提: たたんだ出発点の wrist_roll の読みが毎回ほぼ同じこと。修理前の 4 回（2 日間）で 0.53° に収まっていた。なぜ収まるかは未確認。
- この比較は、elbow の補正の確認とは別の確認。合否は elbow の δ の判定に使わない。

### a-3. elbow の読みが +92.20° を超えたとき

- 読みは切られない（`sts3215_system.cpp:526`、`calibration.cpp` の `raw_to_rad`）。92.20° 超は raw が raw_max（3090）を超えた読みで、そのまま `/joint_states` に出る。MoveIt の位置の範囲は URDF の上限 +1.69 rad（96.83°）なので、92.20〜96.83° は start state out of bounds にならない。
- 書き込みへ進むと、有効化で `hw_commands_ = hw_positions_`（`:854`）。最初の write で want が raw_max を n tick 超える。n ≤ 122 なら異常にならず（`:991`）、`clamp:` の WARN が 1 回出て（`:994`）、Goal は raw_max に切られる（`:1005`）。**腕は有効化の直後に n tick（n × 0.088°）内側へ動く。** 「有効化で動かない」の確認（c 段）が成り立たない。そこで、**超えたら書き込みへ進まない**。
- 超えたとき: 読み取り専用の launch を止め（f 段）、腕を手で少し開いて（elbow を緩めて）たたみ直し、a 段をやり直す。**2 回やり直しても超えるなら、報告して指示を待つ**（δ・center の見直しの材料。その場で値を変えない）。

## 段 b. 出発点で物差しを当てる（独立の確認。腕は動かさない。トルク OFF のまま）

固定あごの一番低い点の、机（腕の取り付け面）からの高さを、前回（`inventory.md` の C3 の節の 3）と同じ測り方で、**2 回**測る。2 回の差が 3 mm を超えたら測り直す。

| 見るもの | 予測 | 判定 |
|---|---|---|
| 一番低い点の高さ（R1 の姿勢） | **31.3 mm**（F6a の計算 `lowest_point_f6.py`: 補正後の R1 で 31.28 mm。補正前は 4.77 mm。1° あたり約 4.3 mm）。**28.3〜34.3 mm なら確定**（予測 ± 3 mm = ±0.7°。決定済み） | 外れたら止めて報告（δ の見直しの材料。その場で値を変えない） |
| 予測が使える条件 | 今日の a 段の読みが、R1 の補正後の値（pan −0.0690、lift −1.4983、elbow **+1.5648**、wrist_flex +0.6690 rad）に、**4 関節とも 0.01 rad 以内**（elbow の 0.01 rad は先端で約 2.5 mm） | 離れていたら 31.3 は使えない。**今日の読みから `lowest_point.py` を計算し直す**（Python。Claude に依頼して承認を得る）。その値 ± 3 mm で判定 |

- 注意: 物差しは 2026-11-16（修理前）の測定 29 ± 3 mm の測り直しで、δ の根拠の一部（物差し −5.77）の再測定。カメラ（−6.31 ± 0.38）とは独立。**修理後（ねじ・ホーン）の最初の測定になる。**

b 段が終わったら、A を止める（f 段の手順。ps と `ros2 node list --no-daemon` が空になるまで）。

## 段 c. 書き込みモードで起動し、RViz から T へ（トルク ON）

**ここからトルク ON。** 腕を折りたたみ姿勢にして、前腕の下を支える。

端末 A:

```bash
ls ~/so_arm101_check/zero_offset_apply_<日付>/c_arm_launch.log
ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=false torque_limit:=500 2>&1 \
  | tee -i ~/so_arm101_check/zero_offset_apply_<日付>/c_arm_launch.log; echo "exit=${PIPESTATUS[0]}"
```

端末 D（10 秒ほど待ってから）:

```bash
ros2 control list_controllers
ls ~/so_arm101_check/zero_offset_apply_<日付>/c_start_pose.txt
ros2 topic echo --once --no-daemon --field position /joint_states 2>&1 \
  | tee ~/so_arm101_check/zero_offset_apply_<日付>/c_start_pose.txt
```

コマンド A（`servo_real_check.md:47`）で `STOPPED`。

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| 有効化の瞬間 | **動かない**（`rad_to_raw(raw_to_rad(raw)) == raw`。J1 の試験）。目で見て動かない。`c_start_pose.txt` を **10 秒あけてもう 1 回**（`c_start_pose2.txt`。書く前に `ls`）読んだ差は 0〜1 tick（0.0015 rad）。**a 段とは、b 段のあとでたたみ直しているので比べない**（c の読みが a-1 の範囲、elbow +88.0〜+92.20° に入ることだけ確かめる） | **どの関節でも 2 回の差が 0.005 rad（約 3 tick）超**／目で見て動く／elbow が +92.20° 超 |
| INFO 行 | a 段と同じ 1 行 | a 段の条件 |
| `arm_controller`・コマンド A | active／`STOPPED`。WARN なし | 違う |

### T へ（RViz。`camera_c2_real_check.md` の W1 と同じ）

MotionPlanning で **Planner は OMPL**、**Velocity Scaling と Acceleration Scaling は 0.1 のまま**。Planning タブで **Start State と Goal State を `<current>` に選び直してから**、Joints タブで整数の度に設定する: **shoulder_pan 0、shoulder_lift −22、elbow_flex +30、wrist_flex +83、wrist_roll 0**（**gripper は触らない**。elbow は +91° 以下）。Plan してアニメーションを見てから、Execute。**動いている間は、片手を 12 V の近くに置き、もう片手で腕を支える。** 止まったら `/joint_states` を `c_T_pose.txt` に控える。

**T の予測（F6a の計算。`t_fk_f6.py`、`lowest_point_f6.py`。補正後は物理的な姿勢が FK と一致する）**:

| 項目 | 予測 |
|---|---|
| T の先端（gripper_frame_link）の FK | (x 176.66、y −0.01、z 58.77) mm |
| 補正前の物理姿勢（真の elbow 23.7°）との差 | **dx −18.20、dz −14.61 mm（大きさ 23.3 mm）。補正後は約 18.2 mm 手前、約 14.6 mm 低い** |
| T の一番低い点の高さ（補正後） | **53.3 mm**（動くあごの頂点。固定あごの先端の点は 54.9 mm）。補正前は 69.2 mm |
| T の感度（elbow 1° あたり） | 一番低い点 −2.49 mm、先端 dx −3.01・dz −2.16 mm |

- Execute の前（アニメーション）: 先端が出発点より下がらない（経路の先端 z の最小は出発点）。**先端が机に近づく動きに見えたら Execute しない**。
- **gripper の値（確認する）**: **53.3 mm の予測は gripper = 0.02811 で計算した**（動くあごが一番低い点なので、gripper の値で高さが変わる）。T へは arm グループ（pan〜wrist_roll の 5 関節。SRDF の group arm）だけを送る。gripper は group hand（別）で、Execute では動かない。有効化で指令は読んだ値（保持）。したがって **T での gripper の読みは、a・c の起動直後の読み（約 0.02811）と同じはず**。`c_T_pose.txt` の 6 番目の値が **0.02811 ± 0.005** に入っていること。**違えば物差しを当てずに報告**（53.3 mm の予測が使えない。感度は計算していない（Python 不実行）。推測: 動くあごの頂点が回転軸から約 0.1 m 以内なら 0.005 rad で 0.5 mm 以下）。

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| 着いた後の関節の読み（`c_T_pose.txt`）と目標の差 | 5 関節とも **0.0082 rad 以内**（V8 の実績の最大。MoveIt の終点のずれ）。elbow は +30.0° ± 0.6° | **0.02 rad 超** |
| ログ | `clamp: '` なし、WARN なし | 共通の止める条件 |

### T の物差し（項目 6。決定）

止まって 10 秒後に、T の一番低い点（動くあごの下端が最低の見込み）の机からの高さを、2 回測る（差が 3 mm 超なら測り直す）。

| 予測との差（測った値 − 53.3 mm） | 判定 |
|---|---|
| **4 mm 以内**（49.3〜57.3 mm） | 合格 |
| **4 mm 超 6 mm 以内** | **記録して報告する。elbow の不合格とは決めない**（lift の −2° 前後（未決着）の可能性があるため）。値を `c_T_ruler.txt` に書く（測った値、差、gripper の値、a 段の読み）。**d 段へ進むかは、報告を見てから**（判断を仰ぐ） |
| **6 mm 超**、または**測った高さが 45 mm 未満** | **止める。** 腕を支えて、動かさずに報告（補正が効いていなければ 69.2 mm、補正が二重にかかれば約 37.6 mm で、どちらもこの条件に当たる） |

- 許容 4 mm の根拠: 物差し ±3 mm と、δ の ±0.7° × 2.49 mm/° = 1.7 mm の二乗和 3.47 mm を切り上げ。
- 参考: T の先端の FK の z は 58.77 mm。DiffMode の床の規則（`floor_z` 0.048 m、測った 0.045 m）との余裕は 10.8 mm、13.8 mm。**読みが同じなら FK も同じ**なので、道具の見る値は補正の前後で変わらない（変わるのは物理的な姿勢）。

## 段 d. 大きな移動と DiffMode の小さな 1 歩（T から。`joint_offset_plan.md` 9.1 の項目 4）

端末 C で、`large_move_real_check.md` の M1〜M3' と同じコマンドを使う（ファイル名は `d_…`）。**各実行の前に `--dry-run` を見る。**

```bash
ls ~/so_arm101_check/zero_offset_apply_<日付>/d_M2dry.log
python3 -u tools/ik_move_run.py --dry-run --goal 0.200 0.000 0.060 --pitch-deg -91.0 --roll-deg 0 2>&1 \
  | tee -i ~/so_arm101_check/zero_offset_apply_<日付>/d_M2dry.log; echo "exit=${PIPESTATUS[0]}"
ls ~/so_arm101_check/zero_offset_apply_<日付>/d_M3pdry.log
python3 -u tools/ik_move_run.py --dry-run --goal 0.230 0.060 0.060 --pitch-deg -91.0 --roll-deg 0 2>&1 \
  | tee -i ~/so_arm101_check/zero_offset_apply_<日付>/d_M3pdry.log; echo "exit=${PIPESTATUS[0]}"
```

dry-run の見込み（`large_move_real_check.md` M1 のモックの結果。IK は yaml・補正を使わないので補正前後で同じ。開始点が違えば少し変わる）: 2 つとも `DRY-RUN: the goal passes every check`、`exit=0`。`q_goal [deg]`: M2 が −0.003、−10.457、**18.326**、83.132、0。M3' が −17.428、9.235、**−5.265**、87.030、0（elbow は太字）。

M2 → M2 の戻し（T へ）→ M3' → M3' の戻しの順（`large_move_real_check.md` の M2・M3' の手順。**腕を支え、12 V の近くに手を置いて**、送信の確認で Enter）。コマンドは同じで、CSV・ログの名前だけ `d_M2.csv`・`d_M2.log`・`d_M2_return.csv`・`d_M2_return.log`・`d_M3p.csv`・`d_M3p.log`・`d_M3p_return.csv`・`d_M3p_return.log` に変える（書く前に `ls`）。

続けて DiffMode の 1 歩（T から。`diff_mode_real_check.md` の V3 と同じ。**M3' の戻しで T に着いたあと**）:

```bash
ls ~/so_arm101_check/zero_offset_apply_<日付>/d_V3_px.csv
python3 tools/diff_mode_run.py --route direct --max-steps 1 --offset 0.005 0 0 \
  --csv ~/so_arm101_check/zero_offset_apply_<日付>/d_V3_px.csv
```

（① の開始の確認は 30 秒で `NOT_CONFIRMED`。タイムアウトしたら別名の CSV で再実行。）

| 見るもの | 予測（補正前の実績。`inventory.md` の M2・M3' の節: 誤差 6.38・4.00・7.05・3.57 mm） | 止める条件 |
|---|---|---|
| dry-run の `q_goal` の elbow_flex | 18.3° と −5.3°（**+90.5° = 1.58 rad 以下**。補正後の真の上限 1.6093 rad から 0.03 rad の余裕） | 1.58 rad 超 → 送らない（`n`）。報告 |
| M2・M3'・それぞれの戻しの `Outcome` | `REACHED`（先端の誤差 10 mm 以内、終了コード 0）。誤差は補正前の実績（3.6〜7.1 mm）と同程度、悪化しない見込み | `SHORT`（終了コード 4）・`NOT_SETTLED`・`PLAN_REFUSED`・タイムアウト・`FIRST_DESIRED`・`GOAL_FLOOR`。`SHORT` は失敗ではない（記録して報告）が、次の段へは進まない |
| 先端の高さ（道具の表示。着いた点の z − 0.045） | 補正前の実績 +10.2〜+11.2 mm。**z ≥ 0.045 m** | z < 0.045 m |
| 手前止まり（`desired − q_meas` の最大） | 補正前の実績 0.0368 rad 以内 | **0.05 rad 超**（報告して指示を待つ。M2 と同じ基準） |
| V3 の `Outcome` | `LIMITED`（`MAX_STEPS`）または `CONVERGED`、終了コード 0。1 歩目の δ は約 2.3 mm（前回の実績: lift +0.0199、elbow −0.0199 rad） | `REFUSED`・`NOT_SETTLED`・`DESIRED_MISMATCH`・`SPIN`・タイムアウト |
| V3 の `windup_after` | 0.06 rad 以下（記録だけ。`tip_move_over_delta` も記録だけで合否に使わない） | 0.06 rad 超 |
| ドライバのログ | WARN なし | 共通の止める条件 |

## 段 e. 出発点へ戻す（RViz。`camera_c2_real_check.md` の W4 と同じ）

- 戻し先: 段 a の `a_start_pose.txt` の 5 関節を、度に直して四捨五入した整数。**elbow_flex は約 +90°（予測。a 段の読み 89.7° の四捨五入）。ただし +91° を超えない。** 理由（計算）: 補正後の真の上限は +92.20°（1.6093 rad）。+91° = 1.5882 rad で、上限まで 1.2°（約 14 tick）。**+96° は使えない**（上限を 3.8°（約 43 tick）超える。許容 122 tick の内側なので黙ってクランプではなく WARN つきで、腕は約 +92.2° で止まり、戻し先より 2.5° 先に曲がる）。
- pan・lift・wrist_flex・wrist_roll: 今日の a 段の読み（R1 の補正後の値は約 pan −3.96°、lift −85.8°、wrist_flex +38.3°、roll −2.77°。たたみ方でばらつく）。2026-11-15 の V8 の目標（pan −4、lift −85、elbow +96、wrist_flex +25、roll −3）は補正前の座標の実績で、使わない。

手順: 端末 A のコマンド A で `STOPPED` → RViz で Start State と Goal State を `<current>` に選び直す → Joints タブで上の整数の度に設定（**gripper は触らない**）→ Plan → アニメーションを見る（先端が床に近づきすぎない）→ Execute。

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| Goal の elbow_flex（Joints タブの値） | **+90°** | **+91° 超** → Plan しない |
| 実行の開始位置と実測の差 | 5 関節とも 0.01 rad 以内（`allowed_start_tolerance`） | 拒否された → 実測を読み直して計画し直す（再試行は 1 回まで。2 回目も拒否なら報告） |
| 着いた後の読み（`e_end_pose.txt`）と、**送った戻し先の目標値**（RViz の Joints タブに入れた整数の度を rad に直した値）の差 | 5 関節とも **0.0082 rad 以内**（V8 の実績の最大。MoveIt の終点のずれ。段 c の T と同じ扱い）。a 段の読みとは比べない（a の elbow の許容幅 88.0〜92.20° のせいで、正常でも差が大きく出うる） | **0.02 rad 超** |
| A のログ | `clamp: '` の WARN が **0 行** | 1 行でもあれば報告（目標が上限を超えた印） |

## 段 f. 後始末

1. **腕を折りたたみ姿勢で支える。** 端末 A で **Ctrl-C を 1 回**押す。
2. A に出る最後の行 `exit=…` を貼る（`exit=0` が期待）。**貼ってから、ps を実行する**（順序は崩さない）。
3. A のログの確認: `[deactivate] torque OFF and Lock=1 confirmed (read back) on all joints` と `deactivated. reads=…` の 2 行。bus のカウンタ（`retried`・`failed_cycles` など）がすべて 0。`grep -c "clamp: '" ~/so_arm101_check/zero_offset_apply_<日付>/<そのときの A のログ>.log` が **0**。
4. 数秒待ってから、端末 D で（`--no-daemon` を付ける）:

```bash
ps aux | grep -E "ros2_control_node|move_group|robot_state_pub|rviz2|spawner|ik_move_run|diff_mode_run" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
```

| 見るもの | 予測 | 止める条件 |
|---|---|---|
| `exit=` | 0 | 0 以外（`move_group`・`rviz2` の −11 は `deactivated.` の後なら無視） |
| 2 行（`torque OFF and Lock=1 confirmed`・`deactivated`） | 両方ある | 無い → 腕を支えて 12 V を切る。報告 |
| bus のカウンタ | すべて 0 | 0 でない |
| ps | 空 | プロセスが残る → `kill -9`・`pkill`・`killall` はしない。報告して指示を待つ |
| `ros2 node list --no-daemon` | 空 | `--no-daemon` にだけ残り ps が空なら、数秒待ってもう一度。それでも残るなら報告して指示を待つ。ros2 daemon には触らない（stop・start をしない） |

（a 段・b 段の後の停止でも、同じ確認をする。トルク OFF なので、止める前に腕を支える。）

## 合格の基準（まとめ）

- a: INFO が 1 行（`-6.30`、`low=50 high=122`）、`[diag]` 6/6、elbow の読みが +88.0〜+92.20°、wrist_roll が修理前の範囲の外へ 3.0° 以内、gripper が 0.02811 ± 0.005。
- b: 一番低い点が 28.3〜34.3 mm。
- c: 有効化で動かない（10 秒あけた 2 回の読みの差 0.005 rad 以内）、T の gripper が 0.02811 ± 0.005、T の一番低い点が 53.3 ± 4 mm（4〜6 mm は記録して報告）。
- d: M2・M3'・戻しが `REACHED`、z ≥ 0.045 m、V3 が `LIMITED` か `CONVERGED`。
- e: 戻し先の elbow が +91° 以下、`clamp: '` が 0 行。
- f: `exit=0`、2 行、bus カウンタ 0、ps と `ros2 node list --no-daemon` が空。
- どの段でも: `FAULT`・WARN・異音なし。

## 記録に残すもの（保存先 `~/so_arm101_check/zero_offset_apply_<日付>/`）

- A のログ（`a_arm_launch.log`・`c_arm_launch.log` ほか）、`a_start_pose.txt`・`c_start_pose.txt`・`c_T_pose.txt`・`e_end_pose.txt`、`c_T_ruler.txt`（4〜6 mm のとき必須）、`d_*.csv`・`d_*.log`。
- 物差しの測定値（b の 2 回、c の 2 回）と、測った日時・測り方。
- 材料の sha256（`sha256sum`）を `inventory.md` の F6 の節に。

## 報告してほしいもの

- a: INFO 行の全文、`[diag]` の行、`a_start_pose.txt` の 6 値、修理前との比較の表（wrist_roll の外れた量と判定）。
- b: 物差しの 2 回の値と判定。
- c: 有効化の前後の差、`c_T_pose.txt` の 6 値（gripper を含む）、T の物差しの 2 回の値と判定。
- d: 各実行の `Outcome`・終了コード・先端の誤差・z・手前止まり。
- e・f: 戻し先、`exit=`、2 行、bus のカウンタ、ps と `ros2 node list --no-daemon`。
- 止めた場合: どの段のどの値がどの閾値を超えたか。
