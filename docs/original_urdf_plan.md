# オリジナルの URDF への置き換えの調査（2026-11-12。調査のみ。コード・設定・実機は変えていない）

対象: 今の xacro / URDF を、SO-ARM101 のオリジナル URDF（TheRobotStudio/SO-ARM100）から「変更せずに置き、薄い xacro で include する」形にできるか。
`docs/lerobot_removal_plan.md` 10 章の調査項目に対応する。

表記: **[事実]** = diff・ファイル・コマンド・ログで確認したこと。**[推測]** = 確認していないこと。**[未確認]** = 今回は確認していない・できなかったこと。

---

## 0. 結論（先に）

1. **[事実]** 今の `so101_new_calib.urdf` は、オリジナル（SO-ARM100 の 385e8d7、2025-07-02）と**リンク 8・関節 7 の全要素（origin・axis・limit・inertia・visual・collision）が一致**する。13 個のメッシュ STL も sha256 が全て一致する。差は 3 つだけ: ①メッシュのパスの書き方、②`<transmission>` ×6 が無い、③`world` のコメントブロックが有る。**0 の位置・向き・範囲・リンク名・関節名は、置き換えても変わらない。**
2. **[事実]** 今の側にだけあるリンク・フレームは**無い**。`gripper_frame_link` もオリジナルに有る。
3. **[事実]** オリジナルは **そのままでは使えない**。メッシュが `assets/xxx.stl`（相対パス、`package://` なし）で、ROS の `robot_description`（文字列）としては解決できない（**[推測]**: RViz・MoveIt は resource_retriever で解決するため。置き換え後に確認する）。したがって「ファイルの内容を 1 バイトも変えずに include」は**できない**。できるのは「**オリジナルを無変更で置き、ビルド時に `assets/` → `package://so_arm101_description/meshes/` の置換だけを機械的に行う**」形（9 章の案 A）。
4. 置き換えで増えるのは `<transmission>` ×6（ROS 1 流の `hardware_interface/PositionJointInterface`）。**[推測]** ros2_control（Humble 2.54.3）は `<ros2_control>` の外の `<transmission>` を読まない。ただし**未確認**（モックで確認する）。
5. **[事実]** 実際に動いている経路は、`so_arm101_moveit_config/config/so101_new_calib.urdf.xacro` が `so_arm101_description/urdf/so101_new_calib.urdf` を include する形。description 側の `*.urdf.xacro`・`*.ros2_control.xacro`・`*_with_transmission.xacro`・`config/*`・`usd/`・`joints_properties.xml` は**どこからも使われていない**（grep で確認）。
6. 由来: **[事実]** 今の URDF・メッシュは、nimiCurtis/so101_ros2 の `so101_description` と（パッケージ名以外）同一で、nimiCurtis の URDF は TheRobotStudio のもの（385e8d7 と一致）。オリジナルのライセンスは **Apache-2.0**（SO-ARM100 の LICENSE。NOTICE ファイルなし）。

---

## 1. オリジナルの取得元（`/tmp` に取得。リポジトリには入れていない）

| 項目 | 内容 |
|---|---|
| 取得元 | https://github.com/TheRobotStudio/SO-ARM100 （`git clone`、全履歴） |
| 取得したコミット（HEAD） | `a758567c3978dfeefe282ede0500085a48fe8f78`（2026-10-06 `docs: list Seeed Studio and WowRobo first in kits section (#188)`）。タグ `v0.1.1` あり |
| 場所 | `Simulation/SO101/so101_new_calib.urdf`、メッシュは `Simulation/SO101/assets/*.stl` |
| そのファイルの最終変更 | `385e8d7`（2025-07-02 `fix(urdf): fixing multiple issues related to the last URDF update (#117)`、Caroline Pascal）。HEAD まで変更なし |
| ファイルの sha256 | `so101_new_calib.urdf` = `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c` |
| 履歴 | `4026fe1`（2025-05-19、追加）→ `bbb9734`（05-21）→ `0c8e289`（06-25。「fixed direction and frame」）→ `385e8d7`（07-02。`_link` 接尾辞、`baseframe` 削除、`gripper_frame_link` へ改名） |
| 同じ場所の他のファイル | `so101_leader_new_calib.urdf`（リーダー用）、`so101_new_calib_camera.urdf`、`so101_old_calib.urdf`、`*.xml`（MuJoCo）、`joints_properties.xml`、`README.md` |
| 生成元（URDF のコメント） | onshape-to-robot で Onshape から生成。Onshape の URL が URDF の 2〜3 行目にある（今の URDF にも同じコメントが残っている） |
| オリジナルの README の説明 | 「相対パスのメッシュに直した（`package://` ではない）」「base の collision メッシュは外した」「新校正は各関節の 0 を範囲の中央にする」 |

取得先のローカルの場所（`/tmp` なので消える）: `/tmp/orig_so100/repo`。比較用の生成物: `/tmp/orig_so100/gen/{current,thin,orig_patched}.*`。

---

## 2. 差の表（今の xacro から生成した URDF と、オリジナル）

比較の方法 **[事実]**:
- 今の生成物: `xacro so_arm101_moveit_config/config/so101_new_calib.urdf.xacro initial_positions_file:=… use_fake_hardware:=true`（実際に動く経路。rc=0、`check_urdf` 成功、`base_link` が根）。
- オリジナル: 385e8d7 の URDF。
- リンクと関節を要素ごとに XML で比較（メッシュのパスの書き方だけ正規化）。結果は「**15 要素（リンク 8 + 関節 7）すべて一致**」。さらにテキストの `diff` を全行読んだ。

| 項目 | 今の側 | オリジナル | 一致 | 備考 |
|---|---|---|---|---|
| リンク名 | `base_link`, `shoulder_link`, `upper_arm_link`, `lower_arm_link`, `wrist_link`, `gripper_link`, `gripper_frame_link`, `moving_jaw_so101_v1_link` | 同じ | ✔ | |
| 関節名 | `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`, `gripper_frame_joint`（固定） | 同じ | ✔ | |
| 関節の origin / axis | 全関節 axis `0 0 1`（子フレーム） | 同じ | ✔ | origin は全て同一（rpy の桁も同じ） |
| 関節の limit（lower, upper） | pan ±1.91986、lift ±1.74533、**elbow ±1.69**、wrist_flex ±1.65806、wrist_roll −2.74385〜+2.84121、gripper −0.174533〜+1.74533。effort 10、velocity 10 | 同じ | ✔ | elbow の `+1.69` は元から（オリジナルの URDF に「5 度の校正オフセットをリミットに適用」というコメントがある） |
| inertia / mass / inertial origin | 全リンク | 同じ | ✔ | |
| collision の形状 | visual と同じ STL メッシュ（base 以外も全て）。box・cylinder は無い | 同じ | ✔ | オリジナルの README は「base の collision を外した」と書くが、385e8d7 の URDF には base_link の collision が**ある**（上の URDF 参照）。README と中身が合っていない **[事実]** |
| visual / material | `3d_printed`（黄）、`sts3215`（黒） | 同じ | ✔ | |
| `gripper_frame_link`（TCP のダミー） | 有（固定関節 `gripper_frame_joint`、origin `-0.0079 -0.000218121 -0.0981274` rpy `0 3.14159 0`） | 同じ | ✔ | **今の側だけではない**。SRDF の `arm` グループの tip・servo.yaml の `ee_frame_name` が使う |
| `<link name="gripper_frame_link">` の中の `<origin>` | 有 | 有 | ✔ | URDF としては不正な要素だが、urdfdom は読む（`check_urdf` 成功）。そのまま |
| メッシュのパス | `package://so_arm101_description/meshes/xxx.stl`（全 34 か所（visual と collision。2026-11-12 の再確認で訂正。初版は 26 と誤記）） | `assets/xxx.stl`（相対） | **✘** | 差 ① |
| メッシュのファイル | `src/so_arm101_description/meshes/*.stl` 13 個 | `Simulation/SO101/assets/*.stl`（18 個。残り 5 つは leader・カメラ用） | ✔ | 13 個とも sha256 一致（下） |
| `<transmission>` | **無い**（URDF から除去され、`so101_new_calib_with_transmission.xacro` に分離。この xacro は動く経路で include されない） | 有（6 個。`transmission_interface/SimpleTransmission`、`hardware_interface/PositionJointInterface`、motor1〜6） | **✘** | 差 ② |
| `world` リンクと `world_to_base` 固定関節 | **コメントアウト**（生成物にコメントとして残る） | 無い | **✘（コメントだけ）** | 差 ③。URDF の意味は同じ（根は `base_link`） |
| ファイル先頭のコメント | 同じ | 同じ | ✔ | 末尾の改行の有無だけ違う |
| `<robot name>` | `so101_new_calib` | `so101_new_calib` | ✔ | |
| ros2_control の `<ros2_control>` ブロック | xacro が追加（`so101_new_calib.ros2_control.xacro`） | 無い | — | 包む側の仕事 |

### 2.1 メッシュの sha256（13 個、今の `meshes/*.stl` とオリジナルの `assets/*.stl`）

**[事実]** 全て `SAME`（`sha256sum` を比べた）: base_motor_holder_so101_v1 / base_so101_v2 / motor_holder_so101_base_v1 / motor_holder_so101_wrist_v1 / moving_jaw_so101_v1 / rotation_pitch_so101_v1 / sts3215_03a_no_horn_v1 / sts3215_03a_v1 / under_arm_so101_v1 / upper_arm_so101_v1 / waveshare_mounting_plate_so101_v2 / wrist_roll_follower_so101_v1 / wrist_roll_pitch_so101_v2。
例: `base_so101_v2.stl` = `bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d`（両方）。
オリジナルだけにある STL（URDF が使わない）: `Handle_SO101`、`Trigger_SO101`、`Wrist_Roll_SO101`（leader 用）、`wrist_camera_mount_so101_v1`、`wrist_camera_so101_v1`（カメラ用）。

### 2.2 「今の xacro の生成物」と「オリジナルを include した薄い xacro の生成物」の比較（実験）

**[事実]**（`/tmp` で実験。リポジトリは変えていない）オリジナルのメッシュのパスだけを `sed` で置換したものを、今の moveit_config の xacro と同じ形で include して生成し、今の生成物と `diff`:
- 差は **(a) `world` のコメントブロック（今の側にだけ有る）、(b) `<transmission>` ×6（オリジナル側にだけ有る）** の 2 種類だけ。リンク・関節・ros2_control ブロックは**同一**。
- `check_urdf` はどちらも成功。

---

## 3. 差の分類

| # | 差 | 包む側の xacro で吸収できる | オリジナルを変えないと無理 | 実機の動きが変わる |
|---|---|---|---|---|
| ① | メッシュのパス（`assets/` → `package://…/meshes/`） | **✘** xacro は include したファイルの属性を書き換えられない **[推測]**（今回は xacro の機能を調べただけで、書き換えの方法は見つからなかった） | 変えたくないなら、**ビルド時の機械的な置換**で生成する（9 章の案 A）。置換の内容は 1 種類だけで、diff が空になることを検査できる | 変わらない（見た目・衝突のメッシュは同じファイルを指す） |
| ② | `<transmission>` ×6 が増える | ✔ 吸収不要か、案 A の置換で除くこともできる | 不要 | 変わらない **[推測]**（ros2_control は `<ros2_control>` の外の transmission を使わない。**未確認**。ros2_control のバージョンは Humble 2.54.3） |
| ③ | `world` のコメントが消える | ✔（コメントなので意味なし） | 不要 | 変わらない |
| ④ | ros2_control ブロック | ✔ 今のとおり別 xacro のマクロ | 不要 | 変わらない |
| ⑤ | 追加のフレーム | ✔ ただし**今は追加するものが無い**（`gripper_frame_link` はオリジナルに有る。`world` は SRDF の `virtual_joint` が TF だけで出す） | 不要 | 変わらない |
| ⑥ | 0 の位置・向き・範囲・リンク名・関節名 | 差が無い | 差が無い | **変わらない** |

### 3.1 実機で確かめた事実との整合（URDF の数値計算で照合した。実機・ポートは使っていない）

**[事実]**（numpy で URDF の FK を計算。`pan=0` で `gripper_frame_link` は base_link で (0.3914, 0, 0.2265)）:

| 実機の事実（`docs/feetech_write_check.md`・`docs/inventory.md` の記録） | URDF の FK（今の URDF = オリジナル） | 一致 |
|---|---|---|
| pan の + は上から見て時計回り | pan +0.3 で先端は y −0.1042（+x から −y へ。上から見て時計回り） | ✔ |
| lift の + は先端が下がる | lift +0.1 で先端 z −0.0327 | ✔ |
| 開く向きは lift +、elbow −、wrist_flex − | elbow −0.1 で先端 z +0.0294、wrist_flex −0.1 で z +0.0159（先端が上がる＝開く） | ✔ |
| 校正の 0 rad は範囲の中央（`center = (min+max)/2`） | オリジナルの README「新校正は各関節の 0 を範囲の中央にする」と同じ考え方。URDF のリミットは wrist_roll と gripper 以外が 0 を中心に対称 | ✔（考え方が一致） |

校正した物理の範囲（`follower_calibration.yaml` の min/max/center から計算）と URDF のリミット **[事実]**:

| 関節 | 校正の範囲（rad） | URDF の範囲（rad） | URDF の余裕（下側 / 上側。負 = 校正の範囲のほうが広い） |
|---|---|---|---|
| shoulder_pan | ±2.0944 | ±1.91986 | −0.1745 / −0.1745 |
| shoulder_lift | ±1.8220 | ±1.74533 | −0.0767 / −0.0767 |
| elbow_flex | ±1.7192 | ±1.69 | **−0.0292 / −0.0292** |
| wrist_flex | ±1.8151 | ±1.65806 | −0.1571 / −0.1571 |
| wrist_roll | ±2.9483 | −2.74385〜+2.84121 | −0.2044 / −0.1071 |
| gripper | 0〜1.7453（%→rad） | −0.174533〜+1.74533 | 下側は URDF のほうが広い。上側は同じ |

- これは**今の状態と同じ**（URDF が同一のため）。置き換えで改善も悪化もしない。
- elbow の上限 +1.69 に対し、出発点（たたみ切り）の実測が +1.69009 で 0.00009 rad 超える件（`docs/feetech_write_check.md` 6 節、対応は OMPL を使う）は、**置き換えても同じ**。オリジナルを書き換えて elbow の上限を広げることは、**この案では不可**（オリジナルを変えない）。広げるなら、包む側で `<joint>` を上書きする別の手段（MoveIt 本体の設定の変更。承認制）が要る。

---

## 4. 名前・範囲に依存する設定への影響

置き換えでリンク名・関節名・origin・axis・limit が**変わらない**ので、下の設定は**変更不要**（**[事実]**: 名前を grep で照合した）。

| ファイル | 依存するもの | 影響 |
|---|---|---|
| `so_arm101_moveit_config/config/so101_new_calib.srdf` | リンク名・関節名 | 変更不要（下の 4.1 の 1 件を除く） |
| `so_arm101_moveit_config/config/moveit_controllers.yaml` | 関節名（arm 5 関節、gripper） | 変更不要 |
| `so_arm101_controller/config/controllers.yaml`（動く経路）、`so_arm101_moveit_config/config/ros2_controllers.yaml`（未使用） | 関節名、`gripper_controller` の `joint: gripper` | 変更不要 |
| `so_arm101_moveit_config/config/joint_limits.yaml` | 関節名。**速度・加速度だけ**（位置は URDF から） | 変更不要。URDF の limit が同じなので位置の範囲も同じ |
| `so_arm101_moveit_config/config/kinematics.yaml` | グループ名 `arm`（KDL、解像度 0.005、timeout 0.005） | 変更不要。チェーンは `base_link`→`gripper_frame_link` で同一 |
| `so_arm101_moveit_config/config/servo.yaml` | `planning_frame: base_link`、`ee_frame_name: gripper_frame_link`、`robot_link_command_frame: base_link` | 変更不要 |
| `so_arm101_moveit_config/config/initial_positions.yaml` | 関節名（mock の初期値。pan 0、lift −1.57、elbow 1.57、wrist_flex 0.8 …） | 変更不要。elbow 1.57 は上限内 |
| `so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro` | 関節名 6 つ、`initial_positions`、`mock_components` / `Sts3215System` | 変更不要 |
| `so_arm101_hardware/config/follower_calibration.yaml` | 関節名、`direction`、`center`、min/max（tick）。**URDF の値は持たない** | 変更不要（0 の位置が同じ） |
| `tools/pose_margin.py` | 既定の `--xacro` が moveit_config の xacro | 変更不要（出力の URDF は同一の内容） |
| `so_arm101_description/config/*`、`urdf/*.xacro`、`usd/`、`joints_properties.xml` | — | **未使用**（grep で参照なし）。**[事実]** `joints_properties.xml` はオリジナルの `Simulation/SO101/joints_properties.xml` と末尾の改行以外同一。`description/config/initial_positions.yaml` は moveit_config のものと値が違う（lift −1.75 等）が、使われていない |

### 4.1 SRDF の全行の照合（実在リンク名・URDF の親子と比べた）

**[事実]** URDF のリンク（8 つ）: `base_link`, `shoulder_link`, `upper_arm_link`, `lower_arm_link`, `wrist_link`, `gripper_link`, `gripper_frame_link`, `moving_jaw_so101_v1_link`。

| 行 | 内容 | 結果 |
|---|---|---|
| group `arm` | chain `base_link` → `gripper_frame_link` | ✔ 実在。URDF の親子の連鎖 |
| group `hand` | joint `gripper` | ✔ |
| group_state `center` / `home` / `initial` / `open` / `close` | 関節名 6 つ（値は 0、home は elbow 1.57・lift −1.57・wrist_flex 0.8、initial は wrist_flex 1.57、open は gripper 1） | ✔ 関節名は実在。値は全て URDF の範囲内 |
| end_effector `ee` | parent_link `gripper_link`、group `hand`、parent_group `arm` | ✔ |
| virtual_joint | `type="fixed" parent_frame="world" child_link="base_link"` | ✔ 下の 4.2 |
| `disable_collisions` ×16 | 下の表 | **1 行が誤り（:45）** |

`disable_collisions` 16 行（`so101_new_calib.srdf`）:

| 行 | link1 × link2 | reason | リンク名 | URDF の親子 | 判定 |
|---|---|---|---|---|---|
| 38 | base_link × shoulder_link | Adjacent | ✔ | 親子 ✔ | OK |
| 39 | base_link × upper_arm_link | Never | ✔ | — | OK |
| 40 | gripper_link × lower_arm_link | Never | ✔ | — | OK |
| 41 | gripper_link × moving_jaw_so101_v1_link | Adjacent | ✔ | 親子 ✔ | OK |
| 42 | gripper_link × wrist_link | Adjacent | ✔ | 親子 ✔ | OK |
| 43 | gripper_link × shoulder_link | Never | ✔ | — | OK |
| 44 | gripper_link × upper_arm_link | Never | ✔ | — | OK |
| **45** | **gripper_link × `elbow_flex`** | Never | **✘ `elbow_flex` は関節名（リンクではない）** | — | **誤り**（既知。:45）。起動のたびに `move_group` が警告 |
| 46 | lower_arm_link × moving_jaw_so101_v1_link | Never | ✔ | — | OK |
| 47 | lower_arm_link × upper_arm_link | Adjacent | ✔ | 親子 ✔ | OK |
| 48 | lower_arm_link × wrist_link | Adjacent | ✔ | 親子 ✔ | OK |
| 49 | moving_jaw_so101_v1_link × wrist_link | Never | ✔ | — | OK |
| 50 | shoulder_link × upper_arm_link | Adjacent | ✔ | 親子 ✔ | OK |
| 51 | shoulder_link × lower_arm_link | Never | ✔ | — | OK |
| 52 | shoulder_link × wrist_link | Never | ✔ | — | OK |
| 53 | shoulder_link × gripper_frame_link | Never | ✔（ただし gripper_frame_link は collision を持たない） | — | 意味なし（害はない） |

（行番号は `grep -n disable_collisions` で確認した実際の行。`:45` が `elbow_flex` の行。）

- **[事実]** `Adjacent` と書かれた 6 行（base-shoulder、gripper-jaw、gripper-wrist、lower-wrist、lower-upper、shoulder-upper）は、URDF の親子と**全て一致**する。URDF の親子 6 組（base-shoulder、shoulder-upper、upper-lower、lower-wrist、wrist-gripper、gripper-jaw）の全てが SRDF にある（`gripper_frame_link` の固定関節だけは対象外）。
- **[事実]** 同じ組（順序を無視）の重複は見つからなかった（`lerobot_removal_plan.md` 10.1 に「`gripper_link`×`lower_arm_link` などはすでにある」とあるが、今回の照合では重複行は検出されなかった。10.1 の書き方は、`gripper_link` を含む行が多いことを指していたのかもしれない）。
- 置き換えでリンク名が同じなので、**SRDF の修正が必要になる理由は、置き換えにはない**。`elbow_flex` の誤りは独立した件（置き換えと別に直せる。SRDF の変更は承認制）。

### 4.2 virtual_joint の `world`

- **[事実]** SRDF: `virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"`。
- **[事実]** URDF（今もオリジナルも）に `world` リンクは**無い**（今の側はコメントアウト）。根のリンクは `base_link`。
- **[事実]** `so_arm101_bringup/launch/bringup.launch.py:153` が `static_virtual_joint_tfs.launch.py` を起動し、`world → base_link` を静的 TF で出す。`planning_frame` は servo.yaml で `base_link`（MoveIt 本体の `planning_frame` は SRDF・URDF の根から決まる **[推測]**。今回は move_group の起動はしていないので**未確認**）。
- 置き換えの影響: **無い**（オリジナルにも `world` は無く、状況が同じ）。

---

## 5. 今の xacro・URDF がオリジナルから作られた経緯（git log・ファイル比較）

### 5.1 このリポジトリ（ros2_so-arm101）の履歴 **[事実]**

```
29ff6bf 2026-11-05 Stage 1: import kept packages from legacy workspace      ← description を旧ワークスペースから取り込み
19273db 2026-11-05 Stage 2a-1: rename packages to so_arm101_*                ← package 名の変更のみ
3bc04f8 2026-11-05 Stage 2a-2: unified bringup launcher …
7b5360b / b58df44 / 759893f (2026-11-12)  L1・L3・不要ファイルの削除（URDF・メッシュには触れていない）
```

- 今の `so101_new_calib.urdf` を、旧ワークスペースの `so101_ros2/so101_description/urdf/so101_new_calib.urdf` と比べると、**`so101_description` → `so_arm101_description`（メッシュのパス）の置換以外は同一**（`diff` が空）。つまり**このリポジトリでは URDF の内容を一切変えていない**。
- 旧ワークスペース（`~/ros2_so101`）の git は「Track so101_ros2 source in parent repository」（2026-10-27、1 コミット）で取り込んでいて、上流の履歴は持たない。
- `src/so_arm101_description/meshes/` の 13 個の STL は、旧ワークスペースの `meshes/` と同一（`diff -rq` で差なし）。`assets/step`・`assets/stl` は 759893f で削除済み。

### 5.2 nimiCurtis/so101_ros2 の履歴 **[事実]**（`git clone` して確認。HEAD `b50872d`、2025-12-13）

- `so101_description/urdf/so101_new_calib.urdf` の履歴: 2025-06-13（`845272a`）に作成 → 2025-07-08（`efe59a5`）に `base_link` など `_link` の名前へ更新 → 2025-10-12（`026fb42`）に `world` のコメントブロックが最初に現れる。
- 最初の版（06-13）のリンク名は `base`・`shoulder`…（TheRobotStudio の `0c8e289` 以前の形）、`<transmission>` は URDF の中にあった。
- nimiCurtis の **HEAD の URDF は、TheRobotStudio の 385e8d7 と 15 要素すべて一致**（メッシュのパスを正規化して比較）。メッシュ（例 `base_so101_v2.stl`）も sha256 が TheRobotStudio と一致。
- `<transmission>` の `with_transmission.xacro` への分離と `world` のコメントは、**nimiCurtis 側の変更**（今の URDF の差 ②③ の由来）。**[推測]**: 2025-10-12 前後の変更。

### 5.3 まとめ（由来）

- **[事実]** URDF の本体（リンク・関節・inertia・メッシュ）= TheRobotStudio/SO-ARM100 の 385e8d7（2025-07-02）と同じ内容。
- **[事実]** 差 ①（`package://` の書き方）・②（transmission の分離）・③（world のコメント）= nimiCurtis/so101_ros2 の変更。このリポジトリで加えた変更は、package 名の置換だけ。
- **[事実]** description の meshes は、TheRobotStudio のファイルそのもの（sha256 一致）。nimiCurtis も同じファイルを使っている。
- **[推測]** 旧ワークスペースの `so101_description` は、nimiCurtis の 2025-07-08 以降のどこかの版のコピー（`_link` 名で、transmission 分離前後は未確認）。取り込み時点の正確な版は**未確認**。
- **[事実]** `so_arm101_controller`（`controllers.yaml`）・`so_arm101_bringup`・`so_arm101_description` の `package.xml` に `<author>Nimrod (nimicu21@gmail.com)</author>` や maintainer の記載が残る。`controllers.yaml` の中身は nimiCurtis の `so101_controllers.yaml` の値を引き継いだもの（`controllers.yaml` 冒頭のコメント）。**controller のうち、TheRobotStudio 由来のものは無い**（TheRobotStudio のリポジトリに ROS 2 の controller は無い **[事実]**: SO-ARM100 の `Simulation/SO101` に無い）。
- **[事実]** description の `usd/`（`so101_new_calib.usd` 等）は、TheRobotStudio の `Simulation/SO101` に無い。nimiCurtis 由来と**[推測]**（nimiCurtis の `so101_description/usd` が元。取り込み元の確認は**未確認**）。
- `LICENSE.so101_ros2` を残すかの判断材料（7 章に統合）: URDF・メッシュは TheRobotStudio（Apache-2.0）、controller・launch・xacro（`mode` 切替など）・package.xml の author は nimiCurtis（MIT）。URDF とメッシュだけを TheRobotStudio から取り直しても、**`controllers.yaml`・`package.xml` の author・description のその他のファイル（xacro・usd・rviz・config）が nimiCurtis 由来のままなら、`LICENSE.so101_ros2` は残す**必要がある。

---

## 6. 置き換えた後に必要な確認

### 6.1 Claude ができる確認（オフライン。実機・ポートは使わない）

1. **生成結果の diff**: 置き換え前後で `xacro so101_new_calib.urdf.xacro …`（`use_fake_hardware:=true/false`、`hardware:=feetech`、`read_only`、`calibration_file` の組み合わせ）の出力を比べる。許容する差は「world のコメント」と「`<transmission>` ×6」だけ。リンク・関節 15 要素は XML で一致すること（今回の `cmp.py` と同じ比較）。
2. **オリジナルとの一致の検査**: 取り込んだオリジナルの sha256 が `a758567`（または 385e8d7）の値と一致すること。ビルド時の置換の結果が、「`assets/` → `package://…/meshes/` の置換だけ」で再現できること（置換後と元の `diff` で、メッシュの行以外が空）。
3. **メッシュのパス**: 生成 URDF の全 `filename` が `package://so_arm101_description/meshes/` で、参照先のファイルが全て存在すること（インストール先 `install/` で確認）。
4. `check_urdf`、`colcon build`（description・moveit_config・bringup）、`colcon test`（関係するパッケージ）。
5. **モック**（`mock:=true`）: `ros2_control_node` が起動し、`/joint_states` が出る。`<transmission>` ×6 があっても `controller_manager` が警告・エラーを出さないこと（4 章の「[推測]」の確認）。`move_group` の警告が `elbow_flex` の 1 件だけ（増えていない）こと。MoveIt の RViz でロボットの見た目（メッシュ）が表示されること、衝突のメッシュが読めること（`move_group` のログに mesh のエラーが無い）。`ros2 node list` と `ps` で残りが無いこと（CLAUDE.md の規則）。
6. `tools/pose_margin.py` の出力が置き換え前と同じ（URDF のリミット）であること。

### 6.2 ユーザーが行う実機確認（Claude は実機を動かさない）

URDF の内容は同一なので、**動作の確認は軽くてよい**が、メッシュ・transmission の経路が変わるので次を提案する（手順書: `docs/feetech_write_check.md`）:
- 読み取り専用の起動（`read_only:=true`）で、`/joint_states` と RViz の表示が置き換え前と同じ姿勢を示す（同じ姿勢で、RViz のモデルが実機と同じ形）。
- 書き込みモードの W4（MoveIt の Plan & Execute、pan の向き・elbow の開く向き）は、**URDF が同一なので省略可能**（ユーザーの判断。省略しない場合は、pan の +1° と、elbow を開く向きに小さく、の 2 項目だけでよい）。
- `elbow_flex` の出発点が URDF の上限を 0.00009 rad 超える件は、置き換え前と同じ（OMPL で計画する）。

---

## 7. オリジナルのライセンスと帰属表示

**[事実]**:
- TheRobotStudio/SO-ARM100 の `LICENSE` は **Apache License 2.0**（全文。末尾の "Copyright [yyyy] [name of copyright owner]" はテンプレートのまま。特定の著作権者名は記載されていない）。`NOTICE` ファイルは**無い**。`README.md`・`Simulation/SO101/README.md` にライセンスの追加の記述（ハードウェアのライセンスなど）は見つからなかった（grep で `licen|apache|copyright` を検索）。
- URDF は onshape-to-robot（Rhoban。ライセンスは**未確認**）で生成。モータの物性（`joints_properties.xml`）は Open Duck Mini（apirrone。ライセンスは**未確認**）から引用と README に記載。
- nimiCurtis/so101_ros2 の LICENSE は **MIT**（Copyright (c) 2025 nimiCurtis）。

**Apache-2.0 の義務（一般論。法的助言ではない）**: 再配布するときは (a) LICENSE のコピーを添える、(b) 変更したファイルに変更した旨を明記する、(c) 元の著作権・帰属の表示を残す、(d) NOTICE があればそれも。**[推測]** 今回は NOTICE が無いので (d) は不要。

**案**（ユーザーの判断。今は何もしない）:
1. オリジナルの URDF とメッシュを、**配置するディレクトリごとに `LICENSE.SO-ARM100`（Apache-2.0 のコピー）と、出典の URL・コミット・ファイルの sha256 を書いた `README` を置く**（`docs/third_party.md` にも記録）。
2. ビルド時の置換（`assets/` → `package://…`）は「変更」にあたる可能性があるので、**生成物に変更の旨を 1 行のコメントとして入れる**（生成 URDF の先頭に XML コメント。ファイルの意味は変わらない）。
3. `LICENSE.so101_ros2`（MIT）は、nimiCurtis 由来のファイル（`controllers.yaml`、package.xml の author、description のその他のファイル）が**残る間は残す**。URDF・メッシュだけを TheRobotStudio へ付け替えても、それだけで削除はできない。
4. `package.xml` の `<license>BSD</license>`（description・moveit_config）は、実際のライセンス（URDF・メッシュは Apache-2.0）と合っていない **[事実]**。置き換えの段階で `Apache-2.0` と、その他の由来を整理することを提案する。

---

## 8. 確認できた範囲・できなかった範囲

**確認できた（事実）**: オリジナルの場所・コミット・履歴、URDF のリンク・関節の全要素の一致、メッシュ 13 個の sha256、今の生成物とオリジナルを include した生成物の diff、SRDF の全行とリンク名・親子の照合、設定ファイルの名前の依存、今の URDF の FK と実機の向きの一致、校正範囲と URDF リミットの差、旧ワークスペース・nimiCurtis の履歴と LICENSE。

**確認できなかった・していない（未確認）**:
- ros2_control（Humble 2.54.3）が、`<ros2_control>` の外の `<transmission>` を無視するか（モックで確認する）。
- `assets/` の相対パスが、RViz・MoveIt（robot_state_publisher 経由の `robot_description`）で本当に解決できないか（今回は RViz・`move_group` を起動していない）。
- `move_group` の起動（`planning_frame`、SRDF の警告）。今回は何も起動していない（プロセスは起動していない。`ps` の確認は不要）。
- onshape-to-robot・Open Duck Mini のライセンス。
- 旧ワークスペースの `so101_description` を取り込んだ、nimiCurtis の正確な版。
- `usd/` の由来の確認。
- 実機の動作（実機・ポートは触っていない）。

---

## 9. 置き換えの案（比較）と、判断が要る点

| 案 | 内容 | 長所 | 短所 |
|---|---|---|---|
| **A（推奨の候補）** | オリジナル `so101_new_calib.urdf` と `assets/*.stl`（使う 13 個）を、無変更で `third_party/` 等に置く（LICENSE・出典・sha256 付き）。`so_arm101_description` の `CMakeLists.txt` が、ビルド時に `assets/` → `package://so_arm101_description/meshes/` を置換した URDF を `share/…/urdf/` に生成・インストールし、薄い xacro（今の moveit_config の xacro と同じ）がそれを include | オリジナルをそのまま追従・検証できる（sha256、diff）。URDF の内容は同一 | CMake に置換の処理が要る。`<transmission>` ×6 が増える（[推測] 無害） |
| B | 今のまま（nimiCurtis の URDF）を残し、出典と LICENSE の記録だけ整理（`third_party.md`、`package.xml`） | 変更が最小。リスクがゼロ | 「オリジナルを引用する形」にならない |
| C | オリジナルのメッシュのパスだけを書き換えたコピーを置く（置換済みの 1 ファイルをコミット） | 簡単 | 手で書き換えたコピーで、オリジナルとの一致を毎回検査する必要がある（今とほぼ同じ） |

**ユーザーに判断してほしい点**:
1. 案 A・B・C のどれにするか。**今の URDF は既にオリジナル（385e8d7）と内容が同じなので、置き換えの利点は「出典・追従・検証の明確さ」が中心で、動作は変わらない。** 費用対効果で B も妥当。
2. 案 A なら、オリジナルの置き場所（例 `src/so_arm101_description/third_party/SO-ARM100/`）と、使うオリジナルのコミット（`a758567` の HEAD か、URDF が最後に変わった `385e8d7` か）。メッシュの `assets/` の扱い（13 個だけコピーするか、全部か）。
3. `<transmission>` ×6 を、置換のときに除くか、残すか（残す案を推奨。モックで確認してから）。
4. SRDF の `elbow_flex` の誤り（:45）を、置き換えと別に直すか（承認制）。
5. `package.xml` の `<license>` の整理と、`LICENSE.so101_ros2` の扱い（7 章の案）。
6. description の未使用ファイル（`urdf/*.xacro`、`config/*`、`usd/`、`joints_properties.xml`）を、置き換えと一緒に削除候補にするか（`git rm`、承認制。`docs/inventory.md` に戻し方を記録する）。**今回は何も削除していない。**

### 9.1 実施の段階（案。承認後）

1. （計画の承認）→ 2. オリジナルの取り込みと LICENSE・出典の記録（`docs/third_party.md`）→ 3. CMake の置換と薄い xacro → 4. 6.1 の確認（xacro の diff、`colcon build`・`colcon test`、モック、`ps` と `ros2 node list`）→ 5. コミット（1 段階ごと）→ 6. 6.2 のユーザーの実機確認。

---
---

# 追記（2026-11-12、第 2 回。調査の補足と、A 案の実施計画。コード・xacro・URDF・SRDF・CMake・package.xml は変更していない）

## 10. 補足の確認（ユーザーの 1〜4 への回答）

### 10.1 `ps` で見えた ros 関連のプロセス 1 件（触っていない）

**[事実]** 該当行（`ps -eo pid,ppid,lstart,cmd`）:

```
 373597   11609 Thu Nov 12 15:07:11 2026 /usr/bin/python3 -c from ros2cli.daemon.daemonize import main; main() --name ros2-daemon --ros-domain-id 0 --rmw-implementation rmw_fastrtps_cpp
```

- 起動と終了の操作はしていない（止めていない・触っていない）。これは **ros2 の daemon**（`ros2 node list` などが自動で起動する常駐プロセス）で、launch したノードではない。**[推測]** 起動した時刻（15:07）は、私の調査の時間帯ではなく、ユーザーの端末の `ros2` コマンド（`ros2 node list` など）が起動したものと思われる。私は `ros2` コマンドを一度も実行していない（`xacro`・`check_urdf`・`git`・`python3`・`sha256sum` だけ）。**確定はできない**（ppid 11609 の中身は確認していない）。

### 10.2 「description の xacro・config は未使用」と、L1 の修正・7 通りの比較との関係

**食い違いではない。「未使用」は「どの launch・試験・tools・CMake も読まない」という意味**。L1 で description の xacro を直したのは「読まれてはいないが、`So101HardwareBridge` の参照を残さない」ための安全側の修正で、7 通りの比較は **launch を通さず `xacro` コマンドを手で実行**して行ったもの（`inventory.md` 1698 行・1801 行）。L1 の記録にも「この xacro を実行時に読むものは無いことを grep で確認した」とある。**今回の grep（`xacro|urdf|robot_description|MoveItConfigsBuilder|so_arm101_description|get_package_share` を `src`・`tools`・`setup_so101.sh` に対して）でも同じ結果**。

launch ファイルごとの、読む xacro・URDF（**[事実]** 起動はせず、grep とファイル読みだけ）:

| launch ファイル | robot_description の作り方 | 読む xacro | その先で読む URDF |
|---|---|---|---|
| `so_arm101_bringup/launch/bringup.launch.py` | `MoveItConfigsBuilder("so101_new_calib", "so_arm101_moveit_config").robot_description(mappings={use_fake_hardware, hardware, serial_port, calibration_file, read_only, acceleration, torque_limit, apply_servo_config, probe_lock_sram})`。rsp と `ros2_control_node` に同じ `moveit_config.robot_description` を渡す | **moveit_config** の `config/so101_new_calib.urdf.xacro`（`.setup_assistant` に `modified_urdf.xacros` があり、`MoveItConfigsBuilder` が `config/<robot_name>.urdf.xacro` を選ぶ。moveit_configs_builder.py:170） | **description の `urdf/so101_new_calib.urdf`**（`$(find so_arm101_description)/urdf/so101_new_calib.urdf` を `xacro:include`） |
| 同 `include()` で起動する `static_virtual_joint_tfs.launch.py`、`spawn_controllers.launch.py`、`move_group.launch.py`、`moveit_rviz.launch.py`、`warehouse_db.launch.py`（moveit_config の launch） | `MoveItConfigsBuilder(...).to_moveit_configs()`（`.robot_description(mappings=…)` を**呼ばない**） | 同じ moveit_config の xacro（引数は xacro の既定値: `use_fake_hardware=true` など。`move_group` などは ros2_control のブロックを使わないので影響なし **[推測]**） | 同じ |
| `so_arm101_moveit_config/launch/rsp.launch.py` | `.robot_description(mappings={"use_fake_hardware": …})` | 同じ moveit_config の xacro | 同じ |
| `servo.launch.py`、`setup_assistant.launch.py` | `.to_moveit_configs()` | 同じ moveit_config の xacro | 同じ |
| `so_arm101_description/launch/`、`so_arm101_controller/launch/` | （ディレクトリが空。git 管理にも無い） | — | — |
| `tools/pose_margin.py`（launch ではない） | `xacro <xacro> use_fake_hardware:=true` を実行 | moveit_config の xacro（既定値） | 同じ |

- **description の `urdf/*.xacro` 3 つ、`config/*`、`usd/`、`joints_properties.xml` を読む launch・試験・tools・CMake は無い**。description の CMake は `urdf/`・`meshes`・`config`・`launch`・`rviz` を**インストールするだけ**（読む側ではない）。
- 使われているのは description の **`urdf/so101_new_calib.urdf`（plain）と `meshes/*.stl`（13 個）だけ**。
- `package.xml` の `exec_depend so_arm101_description`（bringup・controller・moveit_config）は、この URDF とメッシュのため。
- `.setup_assistant` は `urdf: package: so_arm101_description, relative_path: urdf/so101_new_calib.urdf` を持つ（Setup Assistant 用のメタデータ。実行時の経路ではない）。**URDF のパス・名前を変えると、この記述も合わなくなる**ので、A 案では**生成した URDF を同じパス・同じ名前でインストールする**（11.2）。

### 10.3 「全要素が一致」の比べ方（初回の方法の弱点と、やり直した結果）

**初回（`/tmp/orig_so100/cmp.py`）の方法 [事実]**: `xml.etree.ElementTree` で読み、`<link>`・`<joint>` ごとに `ET.tostring` した文字列から**空白を全て削除**し、`package://so_arm101_description/meshes/` を `assets/` に置換してから**文字列として完全一致**を見た。数値の許容差は**無し**（文字列一致）。コメント・`<material>`・`<transmission>`・`<ros2_control>` は対象外。
**弱点 [事実]**: 空白を全部消すため、`xyz="1 2"` と `xyz="12"` のような差を見逃す可能性があった（実際に差があるわけではない）。属性の順序にも依存した。

**やり直し（`/tmp/orig_so100/cmp2.py`。リポジトリには入れていない）**:
- 方法: 要素ごとに、タグ・属性（値は空白で分割し、数として読めるものは `float`）・テキスト（前後の空白のみ除去）・子要素を**再帰的に**比べる（子の順序も一致を要求）。名前を持つ最上位の要素（`material`・`link`・`joint`・`transmission`・`ros2_control`）を**名前で対応づけ**、片方にしか無いものも報告する。`filename` の `package://so_arm101_description/meshes/` だけ `assets/` に正規化。コメントは XML パーサが捨てる（比較外）。
- 許容差: **0（完全一致）、1e-9、1e-6 の 3 通りで実行**。
- 感度の確認: 生成物の elbow の上限を `1.69` → `1.6901` に書き換えると、許容差 1e-6 では**不一致として検出**され、許容差 1e-3 では検出されない（意図どおり）。
- 結果 **[事実]**（385e8d7 のオリジナル vs 今の moveit_config の xacro の生成物）: 3 通りとも**同じ**。`material` 2・`link` 8・`joint` 7 は**全て完全一致**。不一致は `transmission` ×6（オリジナルにだけ有る）と `ros2_control`（今の側にだけ有る。xacro が追加）の 7 件のみ。

**`so101_old_calib.urdf` との違い [事実]**（同じ `cmp2.py` で比較）:
- 今の URDF と old_calib は一致しない（不一致 29 件）。old_calib は**リンク名が `base`・`shoulder`・`upper_arm`・`lower_arm`・`wrist`・`gripper`・`jaw`**（`_link` 無し）で、`gripper_frame_joint` が無い。
- 関節の範囲も違う（old: elbow −0.1745〜+3.1416、shoulder_lift −3.3161〜+0.1745、wrist_roll ±2.79253、gripper −0.174533〜1.74533、pan ±1.91986、wrist_flex ±1.65806。今: elbow ±1.69、lift ±1.74533、wrist_roll −2.74385〜+2.84121 …）。関節の origin の rpy も違う（0 の位置が違う。new は「範囲の中央が 0」、old は「水平に伸ばした姿勢が 0」）。
- したがって、**今の URDF は `so101_new_calib.urdf` に一致し、`so101_old_calib.urdf` とは別物**。校正の方式（0 rad は範囲の中央）とも合うのは new のほう。

### 10.4 `inventory.md` への記録

**[事実]** 初回の調査の後、`docs/inventory.md` には**記録を入れていなかった**（`grep original_urdf` で 0 件）。今回、最終の節に追記する（docs のみ）。

### 10.5 初版の訂正

- メッシュの参照は**全 34 か所**（初版の「26」は誤り。`grep -c 'filename="assets/'` で 34。オリジナルも 34）。3 章の分類・結論には影響しない。
- ユーザーの「差は mesh のパスと transmission だけになるはず」について: **変更の前後（今の moveit_config の xacro の生成物と、置き換え後の生成物）**では、メッシュのパスは**同じ**になる（生成物が今と同じ `package://so_arm101_description/meshes/…` を出すため）。差になるのは (a) `<transmission>` ×6、(b) `world` のコメント 2 ブロックの消失、(c) 生成物に入れる出所のコメント 1 行 だけ。「オリジナルとの差」としては、メッシュのパスと transmission、world のコメント。

---

## 11. A 案の実施計画（承認前。実施は承認後。1 段階ごとに「承認 → `colcon build` → 関係パッケージの `colcon test` → 試験の件数の前後比較 → コミット」）

### 11.1 オリジナルを無変更で置く場所と、記録の方法

```
src/so_arm101_description/
  third_party/SO-ARM100/
    so101_new_calib.urdf   ← オリジナル（385e8d7）を 1 バイトも変えずに置く
    LICENSE                ← SO-ARM100 の LICENSE（Apache-2.0）のコピー
    SOURCE.md              ← 取得元・コミット・日付・ファイルの対応・「無変更」の宣言
    SHA256SUMS             ← sha256sum 形式。URDF 1 つと、メッシュ 13 個
  meshes/                  ← 今のまま。オリジナルの Simulation/SO101/assets の 13 個と同一（sha256 一致を確認済み）。移動しない
```

- **メッシュは動かさない**: 今の `meshes/*.stl` が、385e8d7 の時点の `assets/*.stl` と**バイト一致**であることを確認した **[事実]**（13 個とも `git show 385e8d7:… | sha256sum` と一致）。コピーを増やさず（16 MB）、`meshes/` を「SO-ARM100 の assets（13/18）の無変更のコピー」と `SOURCE.md` に明記し、`SHA256SUMS` に載せる。
- **記録する値 [事実]**:
  - 取得元: https://github.com/TheRobotStudio/SO-ARM100 、パス `Simulation/SO101/so101_new_calib.urdf`、固定リンク `…/blob/385e8d7c68e24945df6c60d9bd68837a4b7411ae/Simulation/SO101/so101_new_calib.urdf`。
  - コミット: ファイルの最終変更 `385e8d7c68e24945df6c60d9bd68837a4b7411ae`（2025-07-02）。取得時の HEAD `a758567c3978dfeefe282ede0500085a48fe8f78`（2026-10-06）。HEAD まで同じ内容。
  - URDF: git blob `9552a231d8b23bed68ec15779eba620c5d875ec4`、sha256 `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`、34 か所のメッシュ参照（すべて `filename="assets/`）。
  - LICENSE の sha256: `c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4`（385e8d7 と HEAD で同一）。
  - メッシュ 13 個の sha256: 2.1 節の一覧と、実施時に `sha256sum` で `SHA256SUMS` に書く。
- **記録の方法**: 取得は `git show 385e8d7:<path> > <dest>` のように**コミットを固定して**行う（作業ツリーの HEAD ではなく）。`SOURCE.md` に取得の手順と日付、`docs/third_party.md` に出典の表（ライセンス・場所）、`docs/inventory.md` に実施の記録（戻すためのコミット）を書く。
- **置き換えで消えるファイル**: 今の `urdf/so101_new_calib.urdf`（nimiCurtis 版）は `git rm`（承認制）。**戻し方は `docs/inventory.md` に「削除の直前のコミット `<hash>` から取り出せる」と書く**（CLAUDE.md の規則）。git 管理外のものは対象に無い。

### 11.2 パスの置換は「ビルド時に生成する別ファイル」にだけ行う

- `third_party/SO-ARM100/so101_new_calib.urdf` は**書き換えない**。置換は `so_arm101_description/CMakeLists.txt` が行い、**ビルドツリー（`build/`）に別ファイルを生成**して `share/so_arm101_description/urdf/so101_new_calib.urdf` にインストールする。
- **インストール先のパスと名前は今と同じ**にする（`$(find so_arm101_description)/urdf/so101_new_calib.urdf`）。これにより **moveit_config の xacro（= 薄い xacro）・`.setup_assistant`・launch・`tools/pose_margin.py` は変更不要**（A 案でも moveit_config の xacro は今のまま。「薄い xacro」はすでにその形）。
- 置換の仕様（`cmake -P` で実行する小さなスクリプト `cmake/gen_urdf.cmake`。`file(READ)`・`string(REPLACE)`・`file(WRITE)` だけで、Python 等に依存しない）:
  1. `filename="assets/` → `filename="package://so_arm101_description/meshes/` の**この 1 種類だけ**。
  2. 置換前に `filename="assets/` が**ちょうど 34 か所**、`assets` を含む行も 34 か所であることを検査し、違えば**ビルドを失敗**させる（オリジナルが変わったときに気づくため）。
  3. 生成物の 2 行目（`<?xml …?>` の次）に出所のコメントを 1 行入れる（例: `<!-- GENERATED at build time from third_party/SO-ARM100/so101_new_calib.urdf (sha256 …, SO-ARM100 385e8d7, Apache-2.0). Only change: mesh filename prefix assets/ -> package://so_arm101_description/meshes/ -->`）。これは Apache-2.0 の「変更の明記」を兼ねる（7 章）。
  4. 入力の sha256 が `SHA256SUMS` と違えば、ビルドを失敗させる（無変更の保証）。
- CMake の骨子（案。実施時に確定）: `add_custom_command(OUTPUT …/urdf/so101_new_calib.urdf COMMAND ${CMAKE_COMMAND} -DIN=… -DOUT=… -P cmake/gen_urdf.cmake DEPENDS …)`、`add_custom_target(gen_urdf ALL DEPENDS …)`、`install(FILES …/urdf/so101_new_calib.urdf DESTINATION share/${PROJECT_NAME}/urdf)`。`install(DIRECTORY urdf/ …)` は、`urdf/` に何も残らなければ削除（12 章の候補を承認した場合）。`--symlink-install` でも、生成ファイルは `build/` から `install/` に**コピー**されるので、挙動は同じ **[推測]**（実施時に確認）。

### 11.3 置いたファイルの sha256 を見る試験

- `so_arm101_description` の `BUILD_TESTING` に、`add_test()` で **CMake スクリプトの試験**を 2 つ追加（`ament_cmake` のみ。Python 不要）:
  1. **`urdf_original_checksums`**: `third_party/SO-ARM100/SHA256SUMS` の全行（URDF 1 つ、メッシュ 13 個、LICENSE）について、`file(SHA256 …)` の結果が記録値と一致すること。ファイルの欠け・余りも検出（`meshes/` の STL が 13 個ちょうどで、`SHA256SUMS` と同じ集合であること）。
  2. **`urdf_generated_matches_original`**: 生成された URDF の 2 行目（出所のコメント）を除き、`package://so_arm101_description/meshes/` → `assets/` と**逆置換**したものが、オリジナルと**バイト一致**すること。生成物のメッシュ参照が 34 か所で、参照先のファイルが全て `meshes/` に存在すること。
- 現在の `so_arm101_description` の試験は lint のみ（`ament_lint_auto`）。**件数の前後を記録**する（前: 実施前に `colcon test` で数える。後: 追加の 2 件＋ lint）。
- 手動の確認用に、`tools/urdf_compare.py`（10.3 の `cmp2.py` を整えたもの。新規ファイル。`xacro` の出力を入力にでき、許容差と無視する要素を引数で指定）を提案する。試験（colcon test）には入れない。

### 11.4 `<transmission>` ×6 は無変更を優先して残し、ros2_control が無視することを確かめる手順

方針: **オリジナルの `<transmission>` を除かずに残す**。もし問題が出たら、そのときに初めて、生成の置換に「`<transmission>` を除く」を**追加**する（その場合は 11.3 の逆置換の試験も、その差を許す形に直す。**無変更を優先する方針は、問題が無い限り変えない**）。

確認の手順（**モックを使う手順は、ユーザーが「実機を使っていない」と返答するまで実施しない**）:

1. **オフライン（今すぐ可。起動なし）**: ros2_control（Humble 2.54.3）の URDF の読み込みのコードの範囲を確認する。**[事実]** `libhardware_interface.so` に "Missing <plugin> tag of <transmission> element in your URDF." という文字列がある。これは `<ros2_control>` の中の `<transmission>` の検査の文言と **[推測]**。`<ros2_control>` の外の `<transmission>` を読むかは、**ソースで確認できていない**（`apt` のソースは無い。GitHub の ros2_control の humble ブランチの `component_parser.cpp` を読む予定）。
2. **モック（ユーザーの返答の後）**: 置き換えの**前**に、基準の起動を 1 回行う（`ros2 launch so_arm101_bringup bringup.launch.py mock:=true use_rviz:=false`。自分で起動して **PID を控え**、`kill -INT <PID>` だけで止める。`pkill` 等は使わない。止めた後は `ps` と `ros2 node list` の両方で残りが無いこと、`ros2 node list` だけ残るなら `ros2 daemon stop; ros2 daemon start` の後にもう一度）。ログを `~/so_arm101_check/urdf_before_mock.log` に保存。
3. 置き換えの**後**に、同じ起動を行い `~/so_arm101_check/urdf_after_mock.log` に保存して、次を確かめる:
   - `ros2_control_node` のログに、`transmission` を含む警告・エラーが**無い**（`grep -i transmission`）。
   - `ros2 control list_hardware_components` が、前と同じ（`SO101System`、mock、active）。`ros2 control list_controllers` が前と同じ（`joint_state_broadcaster`・`arm_controller`・`gripper_controller` が active）。
   - `ros2 topic echo /joint_states --once` が前と同じ関節名・初期値。
   - `ros2 param get /robot_state_publisher robot_description` に `<transmission>` が 6 つ、メッシュが `package://…` であること。
   - `move_group` のログの警告が、前と**同じ**（`Link 'elbow_flex' …` の 1 件のみ。増えていない）。
   - メッシュを読めない旨のエラーが無い（`move_group`・RViz のログ）。
4. 上の 3 つのいずれかが失敗したら、変更を戻して（`git revert`。または未コミットのまま破棄）報告する。

### 11.5 変更の前後で robot_description を比べる方法と、実機での確認の範囲の案

**robot_description の前後の比較（オフライン。launch は使わない）**:
- **前（基準）**を、置き換えの**前に**生成して `~/so_arm101_check/urdf_before/` に保存し、sha256 を `docs/inventory.md` に記録する。組み合わせ（launch と同じ引数）: ①`use_fake_hardware:=true`（mock）、②mock で `hardware` 未指定、③`use_fake_hardware:=false hardware:=feetech read_only:=true calibration_file:=<yaml>`、④同 `read_only:=false`（`acceleration`・`torque_limit`・`apply_servo_config`・`probe_lock_sram` の既定のまま）。さらに `apply_servo_config:=true`、`probe_lock_sram:=true` を各 1 通り。計 6 通り（以前の 7 通りのうち description 側の 3 通り（`mode:=real|isaac|gazebo`）は、12 章の削除を承認する場合は対象外。承認しない場合は、同じ 3 通りも前後で比べる）。
- **後**を同じ引数で生成し、①`diff`（テキスト）、②`tools/urdf_compare.py`（構造比較。許容差 0）で比べる。
- **期待する差（これ以外が出たら中止して報告）**: (a) `<transmission>` ×6 が増える、(b) `world` のコメントの 2 ブロックが消える、(c) 出所のコメント 1 行が増える、(d) xacro が付ける先頭の生成コメント（xacro のパスの行）。**メッシュのパス・リンク・関節・`<ros2_control>` は同一**。今回の試験（`/tmp` で、`world` のコメントと transmission 以外は同一）で、この形になることを**事前に確認済み [事実]**（2.2 節）。
- あわせて、`xacro` の出力の `check_urdf` が成功すること、`tools/pose_margin.py` の出力（URDF のリミットの列）が前と同じこと。

**実機での確認の範囲の案（ユーザーが行う。Claude は実機を動かさない。URDF の幾何は同一なので、L1〜L3 と同じ最小限）**:
- **必須**: 読み取り専用の起動 1 回（`read_only:=true`、RViz あり）。`[diag]` と `/joint_states` が従来どおり／RViz のモデルの形・向きが実機の姿勢と一致（メッシュが全て表示される）／`ros2_control` のログに `transmission` の警告なし。
- **必須**: 書き込みモードの W0（保持のみ 10 秒）1 回と、その前後の読み値の比較（L1〜L3 と同じ。`write_<段階>_W0.log`・`_W0_after.log`）。
- **任意（推奨はしない）**: W4（MoveIt の Plan & Execute）。URDF の幾何・リミット・向きが同一で、MoveIt 本体の設定も変えないので、**省略してよい**とする案。行うなら pan を +1°、または elbow を開く向きに小さく、の 1 項目だけ（OMPL を使う。`elbow_flex` の出発点が上限を 0.00009 rad 超える件は従来どおり）。
- W1〜W3（関節ごとの動作）は、ros2_control の経路と URDF の関節名・向きが変わらないので**不要**。

### 11.6 SRDF `:45` は行ごと削除する（URDF とは別のコミット）

- 対象: `so101_new_calib.srdf:45` の `<disable_collisions link1="gripper_link" link2="elbow_flex" reason="Never"/>` の**1 行**。
- 削除でよい理由 **[事実]**: `elbow_flex` はリンクではなく関節名で、`move_group` が起動のたびに警告して、この行は**無視される**（行の効果は元から無い）。`gripper_link` との組は、すでに `:40`（`gripper_link × lower_arm_link`）と `:44`（`gripper_link × upper_arm_link`）がある。`elbow_flex` が指す意図だったリンク（`upper_arm_link` か `lower_arm_link`）の行が既にあるので、情報は失われない。
- 変更の性質: MoveIt の設定（SRDF）の変更なので**承認制**。**URDF の置き換えとは別のコミット**にする（順序は、URDF の置き換えのコミットの**後**。URDF の前後比較に SRDF の変更が混ざらないようにするため）。
- 確認: モックの `move_group` のログから `Link 'elbow_flex' is not known to URDF` が**消え**、他の警告が増えていないこと。`disable_collisions` が 15 行になること（`grep -c`）。XML として読めること（`python3 -c` の `ET.parse`）。計画（Plan）は、モックで RViz から 1 回（任意）。実機の確認は、次の実機確認（11.5）に**まとめてよい**（警告が消えるだけで、衝突判定は変わらない）。

---

## 12. description の未使用ファイルの削除の候補と理由（**削除はしない**。承認制。`git rm`、戻し方は `docs/inventory.md` に記録）

10.2 の結果（読む launch・試験・tools・CMake が無い）に基づく。**別のコミット**にする（URDF の置き換え・SRDF とは分ける）。

| # | 候補 | 理由 [事実] | 削除した場合の影響・注意 |
|---|---|---|---|
| 1 | `urdf/so101_new_calib.urdf`（今の nimiCurtis 版） | A 案で、生成物に**置き換わる**（候補ではなく、置き換えの一部） | 11.1 のとおり。戻し方を記録 |
| 2 | `urdf/so101_new_calib.urdf.xacro`（description 側） | どの launch も読まない（moveit_config の xacro が別にある）。`mode`・`joint_states_gui` の引数は誰も使わない | 無し。**残すと、A 案で `<transmission>` を持つ URDF を include して、`with_transmission.xacro` と二重になる**ので、紛らわしい |
| 3 | `urdf/so101_new_calib.ros2_control.xacro`（description 側） | moveit_config 側に同名の自前版がある。description 側は `isaac`・`gazebo`・`topic_based` 用で、動く経路は `mode:=real`（L1 で `<ros2_control>` を出さなくした） | Isaac・Gazebo を将来使うなら、タグ `stage2c-done` か削除直前のコミットから取り出す |
| 4 | `urdf/so101_new_calib_with_transmission.xacro` | description の xacro からしか include されない。A 案の URDF が `<transmission>` を持つので、**役目が重なる** | 無し |
| 5 | `config/initial_positions.yaml`、`config/joint_limits.yaml`、`config/joint_names_so_arm_urdf.yaml` | 参照は #2 の xacro の既定値（`initial_positions.yaml`）だけ。moveit_config 側に同名で別の値のものがあり、そちらが使われる（description 側は lift −1.75 など、値も違う） | #2 を消すなら不要。値が違うので、**間違って読まれる危険**もある |
| 6 | `joints_properties.xml` | オリジナル（`Simulation/SO101/joints_properties.xml`）と同一（末尾の改行以外）。MuJoCo 用で、ROS 側から読まない | 無し |
| 7 | ~~`usd/`~~ **候補から外した（ユーザーの判断 2026-11-12: Isaac Sim／Isaac Lab で使う予定があるため残す）** | （元の理由: Isaac Sim 用で参照が無い。約 23 MB、5 ファイル） | 残す。由来（どのツールで、どの URDF から作ったか）は後で調べる課題（`docs/inventory.md` に記録） |
| 8 | `.flake8` | description に Python が無いので不要 **[推測]**（lint の設定が他から参照されるか未確認） | 影響は小さい。判断は任意 |
| 9 | `package.xml` の `<depend>urdf</depend>`、`CMakeLists.txt` の `find_package(urdf)`、`install(DIRECTORY urdf/ …)`（`urdf/` が空になる場合） | 削除ではなく**編集**（package.xml・CMake の変更で承認制）。A 案の CMake の変更と同じコミットで整理する案 | `find_package(urdf)` は使っていない **[推測]** |

- 残す: `meshes/*.stl`（13 個。`SHA256SUMS` で保護）、`package.xml`、`CMakeLists.txt`、（A 案の）`third_party/SO-ARM100/`。
- `git rm` したものは `docs/inventory.md` に「削除の直前のコミット `<hash>` から取り出せる」と書く。git 管理外のものは**無い**（`git ls-files` で確認。`usd/` の 5 ファイルも**追跡されている**（`git check-ignore` で無視対象でもない）ので、`git rm` でき、削除直前のコミットから戻せる）。

---

## 13. `LICENSE.so101_ros2` と `package.xml` の `<license>` の扱いの案

**事実の整理**（5 章・7 章）: URDF・メッシュ = TheRobotStudio（Apache-2.0）。`controllers.yaml`・launch・package.xml の author・description のその他のファイル = nimiCurtis（MIT）。`<license>` は description・moveit_config が `BSD`、bringup・controller が `MIT`。

| 項目 | 案 | 理由 |
|---|---|---|
| `LICENSE.so101_ros2`（MIT、nimiCurtis） | **残す**（A 案の実施の後も）。削除の判断は、nimiCurtis 由来のファイルが無くなった後（例: `controllers.yaml` の値を自前で書き直し、package.xml の author を整理した後）。grep（`nimiCurtis`・`Copyright`・`MIT License`・`Nimrod`）で**帰属表示が必要なものが残らない**ことを確かめてから、別の 1 手で削除 | URDF とメッシュを TheRobotStudio の無変更のものに付け替えても、`controllers.yaml` などが nimiCurtis 由来のまま。MIT は著作権表示と許諾文を**コピーに残すこと**が条件 |
| `third_party/SO-ARM100/LICENSE`（Apache-2.0） | **新規に置く**（11.1）。`docs/third_party.md` に表を追加（出典・ライセンス・場所・無変更の宣言・生成物は「パスの置換のみ」） | Apache-2.0 §4(a): ライセンスのコピーを渡す。§4(b): 変更したファイルに変更の明記（生成物のコメント）。§4(c): 元の表示を残す（URDF 内の Onshape・onshape-to-robot のコメントも無変更で残る）。§4(d): NOTICE は無い |
| `so_arm101_description/package.xml` の `<license>BSD</license>` | `Apache-2.0` を含める案: `<license>Apache-2.0</license>`（URDF・メッシュ）に加えて、nimiCurtis 由来のファイルが残る間は `<license>MIT</license>` も並記（package.xml は `<license>` を複数書ける）。実際にそのパッケージに入るファイルのライセンスの正確な洗い出しは、実施時に行う | 今の `BSD` は**事実と合っていない**（どのファイルも BSD の表示を持たない。`so101_ros2` の package.xml がそう書いていたのを引き継いだ **[推測]**） |
| `so_arm101_moveit_config/package.xml` の `<license>BSD</license>` | 同様に整理（MoveIt Setup Assistant の生成物は、ひな形の `BSD`。自前の変更部分のライセンスはユーザーの判断） | ユーザーの判断が要る（自分のコードのライセンスを何にするか） |
| `so_arm101_bringup`・`so_arm101_controller` の `<license>MIT</license>` と `<author>Nimrod</author>` | author は、nimiCurtis 由来のファイルが残る間は**残す**。ライセンスは MIT のままでよい **[推測]** | 帰属の記載を勝手に消さない |
| ユーザー自身のコードのライセンス（`so_arm101_hardware`・`so_arm101_calibration`・`so_arm101_bringup` の新規部分） | **ユーザーの判断**（リポジトリのルートに LICENSE が無い **[事実]**）。決まったら、ルートに LICENSE を置く | 法的助言ではない。公開する前に決める |

- package.xml・LICENSE の変更は、URDF の置き換え・SRDF・削除とは**別のコミット**にする（承認制）。

---

## 14. 実施の段階（案）と、ユーザーに決めてほしいこと

| 段階 | 内容 | 変更するもの | 確認 |
|---|---|---|---|
| S0 | 計画の承認。ユーザーの「実機を使っていない」の返答（モックの前） | — | — |
| S1 | 基準（前）の取得: robot_description 6 通りの保存、`colcon test` の件数、（モックの基準ログ） | `~/so_arm101_check/` と `docs/inventory.md` のみ | 保存物の sha256 の記録 |
| S2 | オリジナルの取り込み（`third_party/SO-ARM100/`: URDF・LICENSE・SOURCE.md・SHA256SUMS）と、sha256 の試験。**URDF の経路は変えない** | description に新規ファイルと試験（CMake）、`docs/third_party.md` | `colcon build`・`colcon test`（試験の件数の前後）。コミット |
| S3 | 生成（CMake の置換）への切り替えと、今の `urdf/so101_new_calib.urdf` の `git rm` | description の CMake、`urdf/so101_new_calib.urdf`（削除） | 11.5 の前後比較（期待する差だけ）、`check_urdf`、`pose_margin.py`、`colcon build`・`test`。モック（11.4）。コミット |
| S4 | SRDF `:45` の削除 | SRDF（1 行） | 11.6。コミット |
| S5 | description の未使用ファイルの削除（12 章で承認されたもの） | `git rm` | 前後比較（変わらない）、`colcon build`・`test`。コミット |
| S6 | LICENSE・package.xml の整理（13 章） | package.xml・docs・LICENSE | `colcon build`。コミット |
| S7 | 実機の確認（ユーザー。11.5） | — | 結果を `docs/inventory.md` に記録 |

**ユーザーに決めてほしいこと**:
1. 11 章の計画（置き場所 `third_party/SO-ARM100/`、`meshes/` は動かさない、CMake の置換と 34 か所の検査、出所のコメントの挿入）でよいか。
2. 12 章のうち、削除する候補の範囲（#7 の `usd/` は**候補から外れた**。ほかの範囲は後で判断）。
3. 13 章の `<license>` の案（特にユーザー自身のコードのライセンス）。
4. 「実機を使っていない」の返答のタイミング（S1 のモックの基準ログの前）。
5. 11.4 の 1（ros2_control のソースの確認）を、先に Claude が行ってよいか（GitHub のソースを読むだけ。リポジトリは変えない）。

---
---

# 追記（2026-11-12、第 3 回。ros2_control のソースの確認と、A 案の S1〜S3 の実施の記録）

## 15. `<ros2_control>` の外の `<transmission>` は無視されるか（ソースの確認）

**結論 [事実]**: **無視される**（ros2_control 2.54.3 のソースで確認。モックでも確認: 17 章）。

- 読んだソース: https://github.com/ros-controls/ros2_control の **タグ `2.54.3`**（コミット `65d389596953cb4656c4216a3f0bc08a73230739`）。ブランチ `humble` の先端（`31f0b868043ea0a3a23d538e8596f4d4a3beb435`、2026-11-06）も取得したが、読んだのは 2.54.3 のタグ。ファイル `hardware_interface/src/component_parser.cpp`。
- インストール版 [事実]（`dpkg -l`）: `ros-humble-hardware-interface 2.54.3-1jammy.20261005.211947`、`ros-humble-controller-manager 2.54.3-1jammy.20261005.213325`、`ros-humble-ros2-control 2.54.3-1jammy.20261005.220306`、`ros-humble-transmission-interface 2.54.3-1jammy.20261005.212908`。**タグとインストール版のバージョンが一致**。
- 根拠（`component_parser.cpp`）:
  - `parse_control_resources_from_urdf()`（約 567 行〜）は、`robot_it->FirstChildElement("ros2_control")` と `NextSiblingElement("ros2_control")` だけをたどる。`<robot>` 直下の `<transmission>` は**見ない**。
  - `<transmission>` を読む `parse_transmission_from_xml()`（407 行）は、`parse_resource_from_xml()`（`<ros2_control>` の子を順に見るループ。547 行の `else if (kTransmissionTag == child->Name())`）からだけ呼ばれる。つまり**`<ros2_control>` の中の `<transmission>` だけ**。
  - 「Missing `<plugin>` tag of `<transmission>` element」（416 行）は、その `<ros2_control>` の中の `<transmission>` に対する検査。
  - `controller_manager/src/*.cpp` と `hardware_interface/src/resource_manager.cpp` には `transmission` の語が**無い**（grep）。
- 残る確認: robot_state_publisher（urdfdom）・MoveIt・RViz が `<transmission>` を無視すること（urdfdom は `<transmission>` を無視 **[推測]**）。モックで確認した（17 章）。RViz は未起動（実機確認で見る）。

## 16. 実施の記録（S1〜S3。1 コミット = URDF の置き換えだけ）

実施前の基準（S1）:
- `colcon test`（全 6 パッケージ）: **194 件・失敗 12**。内訳（`colcon test-result`）: description 0、moveit_config 0、controller 0、bringup 16（失敗 12。lint の copyright/flake8/pep257 と ctest の 3 件。**以前から同じ**。`inventory.md` の L1〜L4 の記録と同じ）、hardware 105（gtest 99）、calibration 73（gtest 71）。
- robot_description 6 通りを `~/so_arm101_check/urdf_before/` に保存（スクリプト `~/so_arm101_check/gen_rd6.sh`。`xacro` のみ。launch・実機は使わない）。

変更（コミットに入るもの）:
- 新規 `src/so_arm101_description/third_party/SO-ARM100/`: `so101_new_calib.urdf`（385e8d7 を `git show` で取得。無変更）、`LICENSE`（Apache-2.0）、`SOURCE.md`、`SHA256SUMS`（URDF・LICENSE・`meshes/*.stl` 13 個の 15 行。385e8d7 と一致することの記録）。
- 新規 `src/so_arm101_description/cmake/gen_urdf.cmake`（生成）、`check_checksums.cmake`・`check_generated.cmake`（試験）。
- 変更 `src/so_arm101_description/CMakeLists.txt`（生成・インストール・試験 2 件）。パッケージの外のファイルは CMake が読まない（すべて `src/so_arm101_description/` の中）。
- 削除（`git rm`）`src/so_arm101_description/urdf/so101_new_calib.urdf`（nimiCurtis 版）。**戻し方: 削除の直前のコミット `92e9a05` から取り出せる**（`git show 92e9a05:src/so_arm101_description/urdf/so101_new_calib.urdf`）。
- moveit_config の xacro・SRDF・`.setup_assistant`・launch・`tools/` は**変更なし**（生成物を同じパス・同じ名前でインストールするため）。

結果:
- 生成: `gen_urdf.cmake: wrote …/urdf/so101_new_calib.urdf (34 mesh paths replaced)`。installed: `install/so_arm101_description/share/so_arm101_description/urdf/so101_new_calib.urdf`、`third_party/SO-ARM100/` も share にインストール。
- 試験（description）: 0 → **2 件**（`urdf_original_checksums`: 15 ファイル・メッシュ 13 個 OK／`urdf_generated_matches_original`: 34 参照・13 種のメッシュ OK）。**変造の検出を `/tmp` のコピーで確認**: URDF の末尾に空白 1 つ→生成が sha256 不一致で失敗、STL に 1 行追記→チェックサムの試験が失敗、`meshes/` に余分な STL→失敗、生成物の質量を 0.147→0.148 に変更→生成物の試験が失敗。正しい状態では 2 件とも通過。
- `colcon test`（全体）: **194 → 196 件・失敗 12 → 12**（description 0 → 2、他は同じ: moveit_config 0、controller 0、bringup 16（失敗 12）、hardware 105（gtest 99）、calibration 73（gtest 71））。失敗は bringup の以前からの lint だけ。
- robot_description 6 通りの前後: 下の表。**メッシュのパス（34 か所）・リンク・関節・`<ros2_control>` のブロックは同一**。差は (a) `<transmission>` ×6 の増加（6 通りとも同じ 6 要素）、(b) `world` のコメント 2 ブロックの消失、だけ。出所のコメント 1 行は、`<?xml?>` と `<robot>` の間にあり、xacro の出力には出ない（インストールされたファイルには有る）。構造比較（許容差 0）の不一致は、6 通りとも `ONLY_B transmission` ×6 のみ。`check_urdf` は 6 通りとも成功。`tools/pose_margin.py` の URDF のリミットは前と同じ。

| 引数の組 | 前の sha256（先頭 16 桁） | 後の sha256（先頭 16 桁） |
|---|---|---|
| 1 mock + hardware:=feetech | d49d109d657d564b… | d8eebdcd27762b2b… |
| 2 mock（hardware 未指定） | d49d109d657d564b… | d8eebdcd27762b2b… |
| 3 feetech read_only:=true | 9cac4b9748b8f90f… | 9bfaceea6da94c16… |
| 4 feetech read_only:=false | ab402331d817819e… | a5160ee8aad290a8… |
| 5 同 + apply_servo_config:=true | a792d24e8c020a17… | 6bf7aa602dc5b6ad… |
| 6 同 + probe_lock_sram:=true | 4bc148cb8c48c78c… | 7dc2c92cc80f37ed… |

（1 と 2 が同一の出力なのは、`hardware` の既定が `feetech` のため。以前の 7 通りのうち description 側の `mode:=real|isaac|gazebo` の 3 通りは、description の xacro を削除しない今回は対象外。それらは `include` するファイルが変わった（transmission が増える）ので、同じ差が出る [推測]。誰も読まない）

## 17. モックでの確認（2026-11-12。ユーザーが「実機は使っていない」と返答済み。ログ `~/so_arm101_check/urdf_after_mock.log`）

- 起動: `ros2 launch so_arm101_bringup bringup.launch.py mock:=true use_rviz:=false`（`setsid` で起動。launch の本体 PID **379627**、子は同じプロセスグループ。起動前に `ps` と `ros2 node list --no-daemon` で ROS のノードが無いことを確認）。
- **transmission**: ログに `transmission` の語が **0 件**（警告・エラーなし）。`/robot_state_publisher` の `robot_description` に `<transmission>` 6 個、`package://so_arm101_description/meshes/` 34 か所、`assets/` の残り 0、`<ros2_control>` 1 個。
- **ハードウェア**: `SO101System`（`mock_components/GenericSystem`）が `active`。6 関節のコマンドインターフェースが available・claimed。
- **コントローラ 3 つ**: `joint_state_broadcaster`・`arm_controller`・`gripper_controller` がすべて `active`（spawner のログも「Configured and activated」）。
- `/joint_states`: 6 関節、位置 (0.0, −1.57, 1.57, 0.8, 0.0, 0.0)（mock の初期値）。
- `move_group`: SRDF の警告は `Link 'elbow_flex' is not known to URDF. Cannot disable/enable collisons.` の **1 件のみ**（変化なし。次の SRDF のコミットで消す）。メッシュの読み込みエラーは無い（`mesh|resource|Unable to open` の grep で 0）。ほかの WARN/ERROR は従来と同じ種類（kdl_parser の root の inertia、octomap の 3D センサー未設定、`allow_nonzero_velocity_at_trajectory_end`、controller_manager の robot_description の非推奨）。
- 停止: `kill -INT 379627` のみ。3 秒で launch が終了。**終了時に `move_group` が exit code −11**（既知。`inventory.md` 932 行・`feetech_write_check.md` 296 行と同じ）。
- 確認: `ps`（`ros2_control_node|move_group|robot_state_publisher|static_transform_publisher|spawner|ros2 launch`）は**空**。`ros2 node list --no-daemon` も**空**。`ros2 node list`（daemon 経由）には `/move_group` と `/moveit_simple_controller_manager` が**残っている**（既知の古いキャッシュ。`inventory.md` 1127 行と同じ）。**ユーザーの指示（ros2 daemon には触らない）により、`ros2 daemon stop/start` は実施していない**。10 秒待って再確認しても同じ。daemon（PID 373597、15:07 起動）はそのまま。
- 起動前（置き換え前）のモックの基準ログは取っていない（実施前に取らなかった）。代わりに、robot_description の前後の構造比較（上）と、過去の起動のログ（`inventory.md`）の記録との一致で確認した。

## 18. 補足（10.1 の daemon について）

**[事実]** 15:07 起動の `ros2 daemon`（PID 373597）の親 PID は 11609。今回の launch の親（`setsid` の後の起動元）も 11609 で、**このセッションのシェルと同じ親**。**[推測]** この daemon は、このセッションのシェル（またはその親の端末）から実行された `ros2` コマンドで自動起動されたもの。いつ・誰のコマンドかは確定できない。daemon には触っていない。

## 19. SRDF `:45` の削除（別コミット。2026-11-12）

- 変更: `so_arm101_moveit_config/config/so101_new_calib.srdf` の `<disable_collisions link1="gripper_link" link2="elbow_flex" reason="Never"/>`（`:45`）を**行ごと削除**（1 行の削除のみ。`disable_collisions` 16 → 15 行。XML として読める）。`gripper_link × lower_arm_link`（`:40`）と `gripper_link × upper_arm_link`（`:44`）は既にある。
- **前後の比較（モック。前 `~/so_arm101_check/srdf_before/`、後 `~/so_arm101_check/srdf_after/`。`/get_planning_scene` の `ALLOWED_COLLISION_MATRIX` と `/move_group` の `robot_description_semantic` を取得）**:
  - 許可衝突行列（ACM）: リンク名 8 つ・行列とも**前後で完全に同一**（許可されたペア 15 組 → 15 組）。つまり、この行は MoveIt では元から**効いていなかった**。
  - `robot_description_semantic` の差は、削除した 1 行だけ。
  - `move_group` の警告 `Link 'elbow_flex' is not known to URDF. Cannot disable/enable collisons.` は 1 件 → **0 件**。WARN/ERROR の他の行は前後で同じ（終了時の `move_group` exit code −11（既知）と、PID の違いだけ）。
  - コントローラ 3 つ（`joint_state_broadcaster`・`arm_controller`・`gripper_controller`）は前後とも `active`。
- 試験: `colcon test` 全体 **196 → 196 件・失敗 12 → 12**（description 2、moveit_config 0、controller 0、bringup 16（失敗 12。以前から）、hardware 105、calibration 73）。`colcon build` は成功。
- 停止の手順で起きたこと（**正直な記録。CLAUDE.md の規則からの逸脱が 1 回**）: 前の起動（`srdf_before`）は、確認用のシェルスクリプトの中で `setsid … &` で起動したため、launch のプロセスが SIGINT を**無視する設定を引き継いだ**（`/proc/<pid>/status` の `SigIgn` で確認）。`kill -INT` が効かなかったので、**自分が起動して PID を控えたものだけ**を対象に、(1) 子ノード 4 つ（PID 380263・380265・380267・380269）に `kill -INT`（3 秒で終了。ただし `robot_state_publisher` は launch が `respawn` で再起動した）、(2) 待機していた launch 本体（PID 380261）に **`kill -TERM`**（規則の `kill -INT` ではない）、(3) 再起動された `robot_state_publisher`（PID 380764）に `kill -INT`。その後、`ps` は空、`ros2 node list --no-daemon` も空。`pkill`・名前での停止は使っていない。次の起動（`srdf_after`）は、起動の前に Python で SIGINT を既定に戻してから `exec` する形に直し、**`kill -INT` で正常に停止**（PID 380964、`ps` 空、`--no-daemon` 空）。`~/so_arm101_check/mock_acm.sh` がそのスクリプト。daemon（PID 373597）には触っていない。


## 20. ユーザーの判断（2026-11-12。記録のみ。コード・設定は変更していない）

- **`usd/` は残す**（Isaac Sim／Isaac Lab で使う予定）。12 章の削除の候補から外した（#7）。由来（どのツールで、どの URDF から作ったか）は、後で調べる課題（`docs/inventory.md` に記録）。
- **ユーザー自身のコードのライセンスは MIT にする**。今回は実施しない。ルートの `LICENSE`（MIT）の追加と、各 `package.xml` の `<license>` の案（`third_party` の Apache-2.0、nimiCurtis 由来の MIT、LeRobot 0.4.2 由来の基準値（正解表・`third_party.md` の Apache-2.0）の扱いを含む）は、**URDF の作業が終わった後に、計画として出す**（13 章を、この判断に合わせて作り直す）。
- 12 章のほかの削除の範囲は、後でユーザーが判断する。
