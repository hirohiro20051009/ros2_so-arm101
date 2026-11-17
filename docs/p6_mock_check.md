# P6 b3: モックでの VR 追従の確認（2026-11-17）

偽入力（`examples/vr_fake_input.py`）で、VR 追従ノード（`vr_follow`）をモックの腕に対して確かめた。実機は使っていない。

## 1. 条件

- モック: ROS_DOMAIN_ID=87、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp。起動の順は、`bringup.launch.py mock:=true use_rviz:=false`（前景。PID を控え、SigIgn に SIGINT（0x2）が立っていないことを確認）→ `vr_follow.launch.py`（既定の値。scale 1.0、rate_hz 10、point_time_s 0.08、C1〜C4 は既定）→ 偽入力。
- コミット: C1 のメッセージごとの判定まで（9027f53）。b3 のスクリプトは、この確認の後にコミットする。
- 準備: モックの初期姿勢（折りたたみ。wrist_flex 0.8。先端の z が約 0.03 m で、クラッチの前の検査 0.045 m に当たる）のまま始められないので、スクリプトが最初に作業姿勢 [0, 0, 0.8, -0.6, 0]（先端の z 約 0.10 m）へ、arm_controller に軌道を送って動かす（追従ノードを通さない。モックのみ）。
- 偽入力: Joy 50 Hz、PoseStamped 90 Hz（base_link）。手の基準は (0.30, 0, 0.30) で固定（相対動作なので、動きだけが意味を持つ）。ボタンは clutch 0、open 1、close 2。
- 監視: `/vr_follow/status`、`/arm_controller/joint_trajectory`（点）、`/vr_follow/gripper`、`/gripper_controller/gripper_cmd/_action/status`、`/joint_states`。「状態が TRACKING でない間に点が来ない」は、状態の変化の前後 0.08 s を除いて判定する。

## 2. 結果（7 シナリオ、すべて合格）

1 回目（`b3_run1.txt`）は rotation の 1 項目だけが不合格で、その項目の期待値を直して rotation だけ再実行し（`b3_run2.txt`）、合格した（4 章）。

| シナリオ | 状態の遷移（時刻 s） | 確かめた内容と結果 |
|---|---|---|
| line | IDLE → TRACKING（1.0）→ IDLE RELEASED（7.0） | 点 59 個。TRACKING 以外で点なし。手を +40 mm で先端が追従（6 mm 以内）、戻りも 6 mm 以内 |
| jump | TRACKING（1.0）→ HOLD_JUMP TARGET_JUMP（3.0。80 mm の階段）→ TRACKING（4.5。手が止まった目標の 20 mm 以内に戻る）→ HOLD_JUMP（6.0。1 メッセージだけのスパイク）→ TRACKING（6.1）→ IDLE（8.0） | 点 53 個。HOLD 中に点なし。先端は 80 mm の飛びを追わない（+30 mm 未満） |
| stale | TRACKING → STALE POSE（3.5。pose を 1 s 止めた）→ TRACKING（4.0。同じ位置に戻る）→ STALE POSE（5.5）→ STALE WAIT_RETURN（6.2。遠い位置で再開）→ IDLE RELEASED（7.5）→ TRACKING（8.5）→ IDLE JOY_TIMEOUT（11.0。Joy を 1.5 s 止めた） | 点 63 個。STALE 中に点なし。Joy が戻っても、クラッチを押したままでは再開しない |
| range | TRACKING → HOLD_RANGE FLOOR（2.9）→ TRACKING（5.9）→ HOLD_RANGE ELBOW_LIMIT（10.0）→ HOLD_RANGE JOINT_RANGE（10.2）→ IDLE（14.1） | 点 58 個。先端の z の最小が 0.043 m より上（床の判定 0.045 m。停止時の z 0.0427 m の目標は送らず、測定の先端は 0.043 m 以上）。elbow は +91°（1.588 rad）未満。HOLD_RANGE の間、点なし。理由が ELBOW_LIMIT から JOINT_RANGE に変わったのは、手が進み続けたため（状態は HOLD_RANGE のまま） |
| rotation | IDLE → TRACKING（1.0）→ IDLE（9.0） | 点 79 個。手を 20° 回して wrist_roll が 0.2 rad 以上動く。投影で捨てた角度 `proj_deg` は、roll のとき約 4°、yaw 15° のとき約 14.6°（腕が作れない yaw を報告） |
| gripper | TRACKING（5.6）→ HOLD_JUMP（7.1）→ TRACKING（7.2）→ STALE（9.1）→ TRACKING（9.6）→ IDLE（11.1） | `/vr_follow/gripper` は 1.0（CLOSE）と 0.0（OPEN）の 2 回だけ。action の goal は 2 個（OPEN と CLOSE の同時押しでは 0、腕の HOLD・STALE でも 0）。モックの gripper は、OPEN で 1.0 rad に開き（> 0.8）、CLOSE の目標 0.05 rad（開始位置が 0 のため、変化は小さい。< 0.3）。停止の間（4.6〜12 s）、gripper の位置の変化は 0.02 rad 未満 |
| busy | TRACKING（1.0）→ BUSY（3.1。追従中に `FollowJointTrajectory` の goal を 1 つ送った）→ IDLE RELEASED（9.1）→ TRACKING（10.1。新しいクラッチ）→ IDLE（12.1） | 点 38 個。BUSY の理由は `…/follow_joint_trajectory/_action/status: 1 active goal(s)`。goal が終わっても BUSY のままで、新しいクラッチまで点なし |

## 3. その他の確認

- **C3（BLOCKED）はモックでは出なかった**（`BLOCKED (C3) seen: False`）。モックは指令どおりに動くため、出ない見込みどおり。実機の c0（read_only:=true。測定が動かない）で、TRACKING → 指令が 0.2 rad 進んだところで BLOCKED になることを確かめる（P6c）。
- ノードのログの ERROR は 4 件で、すべて期待どおり（HOLD_RANGE の FLOOR・ELBOW_LIMIT・JOINT_RANGE、BUSY）。ログにエラーの詳細（`message`）が付く。
- gripper_controller はモックで動いている（OPEN の goal で位置が 1.0 rad に変わった）。
- ノードの落ち・例外・警告なし（ログの WARNING は、停止のときの launch の SIGINT の 1 件だけ）。

## 4. 1 回目の不合格と直し

rotation の「roll だけ回しているとき `proj_deg` が 3° 未満」が、実測 4.0° で不合格だった。原因は期待値の誤り: tool 軸が base_link の x 軸から約 11°（pitch −0.2 rad）ずれているため、x 軸まわりの手の回転は純粋な roll にならず、約 4° が pan 平面の外へ出て報告される。ノードの動きは正しい。期待値を 6° 未満に直して再実行し、合格した（yaw 15° のときは約 14.6° が報告される）。

## 5. 停止の確認

vr_follow（PID を控えた launch）を kill -INT で止め（clean に終了）、続いて bringup を kill -INT で止めた。ps に関係するプロセスなし、`ros2 node list --no-daemon` は 0 件。bringup のログの終わりに move_group の exit code -11（既知の終了時の挙動）。

## 6. 材料と sha256（`~/so_arm101_check/p6/`、git 管理外）

`b3_run1.txt` `c322de1628ed2bebb11711b00462b07ba83567374222cfdb3282e92ae4688f01`、`b3_run2.txt` `9b0ec613fd2df5fd64ec1051f186c7682aceda91d491ce748baf100de4ebdeaf`、`b3_node.log` `7e1d7f6b250d0f142bf9be59b8e9858fef75c752df59b0e371b09187e5a20152`、`b3_bringup.log` `5aa2bbb46a9f24b41b4693952838269cf76bd3e26461ab7f893ac7b8c0ccd46c`、`b3_run1/busy.csv` `297d7c66271733a6a99ed99740b867c194f5dfa86d8513ba5ade97442cf094cd`、`b3_run1/gripper.csv` `9885f679130d9cdf85b765a9b13ce40f37071735001b0a16b136496acbc2ea4d`、`b3_run1/jump.csv` `d69709214fb3479599275a82bbb79666096f51c14216921d4b7fc31c799b9873`、`b3_run1/line.csv` `3da054b9a35d0017bb44c11ed8aeffc191518bb96b19d989d9513bf16de9b644`、`b3_run1/range.csv` `aaef49da0a96c5cdef216d2a6a42b21a9c4d650270d2acf5038f323a03be5bae`、`b3_run1/rotation.csv` `56a7101f85b88eabe69fd1021c3fe2140392e54eb0b570c13f7c32cb29b97dbe`、`b3_run1/stale.csv` `08367f287ae58e33627a24e46537f1859945c55a8d31aa81f644571b242f7542`、`b3_run2/rotation.csv` `1cd7c78c4c9a1ba940f7d737015bafc57fa1997cab4e091aa7bf022acc22f11b`。

## 7. モックでは確かめていないこと（P6c で確かめる）

- 実機の遅れ（JTC の遅れ 0.06 rad とサーボの遅れ）と、C3（0.2 rad）の余裕。C2（0.15 m）の余裕。
- 実機のグリッパー（open_rad・closed_rad の値、物をつかんだときの goal の扱い）。
- scale を実行中に変えたときの動き（単体試験では確認済み。モックのシナリオには入れていない）。
- 実際の VR の座標の合わせ込み（次の担当者）。
