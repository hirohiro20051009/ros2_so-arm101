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
| メッシュのパス | `package://so_arm101_description/meshes/xxx.stl`（全 26 か所） | `assets/xxx.stl`（相対） | **✘** | 差 ① |
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
