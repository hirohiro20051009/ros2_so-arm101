# カメラの段の計画（C1〜C4）

状態: 計画（実装・ビルド・実機は未実施）。作成日 2026-11-15。HEAD は 4be7791 の次のコミットから。

「事実」は読んで確かめたこと、「推測」は未確認の見立て。表のセルに縦線は使わない。
実機（サーボ）・カメラを動かすのはユーザーだけ。Claude Code は、オフラインの試験、ビルド、モック（ROS_DOMAIN_ID=87）、`--show-args` までを行う。

## 0. 決定済みの前提

ユーザーの方針（2026-11-15）:

- 最初にカメラ（RealSense D415）と AprilTag の起動を新しいワークスペースに入れる（C1）。そのあと実機で点を集めて Eye-to-Hand を導入する（C2 点の収集、C3 Kabsch で base_link から camera_link の TF を出す、C4 独立した確認）。
- 大きな移動は既存の `tools/ik_move_run.py`（解析 IK、MoveIt は関節の目標だけ）。一回の誤差は 1 cm 程度まで許容。
- カメラの値と FK(q_meas) の値を混ぜない。列名の接頭辞で区別する（8 章）。
- 必要最小限にする。

D1〜D6（計画の下書きで挙げた判断。すべて推奨どおりに決定）:

| # | 決定 |
|---|---|
| D1 | Eye-to-Hand は新パッケージ so_arm101_camera に置く。CLAUDE.md の 1 行の文面は提案だけ（下記）。変更は別に承認を得る |
| D2 | 全点で pitch・roll を同じにしたまま、オフラインの検討（pitch の候補ごとの届く範囲と見え方）で決める |
| D3 | 点の範囲は約 70×120×40 mm の格子 12 点（推測）から。広げるなら pitch を変える（D2 と一緒に決める） |

**D2・D3 の決定（2026-11-16、ユーザー。上の表の元の記述は残す）**: オフラインの検討（`docs/camera_pose_study.md`）の結果、D2 は pitch −30°・roll 0（実機の C2 で見え方が悪ければ −15° に切り替える。そのときは別に相談）。D3 は 70×120 mm の 2×3 を 2 層、dz 0.08 の 12 点（約 70×120×80 mm。上の「約 70×120×40 mm」は 2 層の間隔 0.04 の場合で、dz 0.08 に変えた）。中心は (0.280, 0.040, 0.100) m、点の x は 0.245 と 0.315、y は −0.020・0.040・0.100、2 層の z は 0.060 と 0.140。12 点の `--goal` つきの表は camera_pose_study.md の 1.1。
| D4 | タグ（ID 0 と ID 1）の一辺 60 mm が黒い正方形の外縁かを、ノギスで測って確かめる |
| D5 | キャリブの間はカメラを固定し、位置が変わったら C2〜C4 をやり直す |
| D6 | 深度（depth 640x480x15、align_depth）は旧起動と同じに残す |

CLAUDE.md の「最終構成」への提案（変更はしない。承認後に別のコミットで行う）:

- 追加する行: `so_arm101_camera : カメラ（RealSense D415）とAprilTagの起動、base_link → camera_link のTF、Eye-to-Hand のキャリブ（点の収集・Kabsch・確認）`
- 直す行: `so_arm101_calibration : サーボのキャリブ（脱力→中央→MIN/MAX）`（「＋AprilTagのEye-to-Hand」を外す）

## 1. 旧 apriltag_test の読み取り（値の出どころ）

旧ワークスペース `~/ros2_so101` は読むだけ。実行・import・変更はしない。

**事実: 旧の launch（`src/apriltag_test/launch/apriltag_d415.launch.py`）**

| 項目 | 場所と内容 |
|---|---|
| ノード | :18-30。package='apriltag_ros'、executable='apriltag_node'、name='apriltag'、output='screen'（apt の実体は /opt/ros/humble/lib/apriltag_ros/apriltag_node） |
| remap | :24-27。image_rect を /camera/camera/color/image_raw に、camera_info を /camera/camera/color/camera_info に（生の color 画像。rectify は使わない） |
| 設定の読み込み | :10-14、:29。パッケージの config の yaml をそのままパラメータに渡す |

**事実: 旧の設定（`src/apriltag_test/config/tags_36h11.yaml`）**

| 項目 | 場所と値 |
|---|---|
| トップのキー | :1 apriltag（ノード名） |
| image_transport | :4 raw |
| family | :6 36h11 |
| size | :8 0.06 |
| max_hamming | :10 0 |
| detector | :12-17 threads 2、decimate 2.0、blur 0.0、refine true、sharpening 0.25 |
| tag | :19-22 ids [0, 1]、frames [tag36h11_0, tag36h11_1]、sizes [0.06, 0.06] |

ID 1 は、ユーザーが接触試験のために足したもの（旧 `test/contact_test.py:13` の `TAGF = 'tag36h11_1'`）。**残す**。

**事実: そのほかの出どころ**

| 値 | 出どころ |
|---|---|
| RealSense の起動の 3 つの値（depth_module.depth_profile、rgb_camera.color_profile は 640x480x15、align_depth.enable は true） | `~/ros2_so101/test/readme.txt:30-33` |
| 光学フレーム名とタグのフレーム名 | `test/collect_cal.py:20`（camera_color_optical_frame、tag36h11_0） |
| タグの取り付けのオフセット概算（全関節 0、base_link の軸、TCP からタグ中心まで）(−0.055, 0, 0.035) m | `test/diffik_pose.py:12`（D_BASE0）、`readme.txt:45-48` |
| camera_link から光学フレームへの TF（位置 (0, 0.015, 0)、クォータニオン (−0.500, 0.499, −0.500, 0.501)） | `test/cal_to_tf.py:17-19`（今回はコードに書かず、C2 で TF から読んで記録する。6 章） |

**新しい構成に書き写すもの**（apriltag_test からファイルを移す・コピーすることはしない。値と出どころ（上の表）を新しい yaml と launch に書く）: ID 0・1、一辺 0.06、family、max_hamming、detector の 5 項目、2 つの remap。

## 2. 新しい構成

### 2.1 決まり（ユーザーと見直した）

- 外部のもの（realsense2_camera、apriltag_ros）は、コピーも改名もせず apt のまま使う。リポジトリに入れるのは、それぞれに渡す yaml だけ。
- config のファイル名は、読む相手の名前にする。各 yaml の 1 行目のコメントに「読む相手」と「手で書く／道具が書く」を書く。
- apt の例と同じ名前（tags_36h11.yaml、camera_36h11 など）は使わない。apriltag.yaml は新しく書く（例のコメント・本文は持ち込まない）。
- 純粋な計算は `src/so_arm101_camera/so_arm101_camera/`、ROS の部分の道具はリポジトリの `tools/`（既存の ik_move_run.py などと同じ）。

### 2.2 ファイルの一覧

ビルドの型は ament_cmake と ament_cmake_python（so_arm101_ik と同じ。事実: ik の CMakeLists.txt）。launch と config は `install(DIRECTORY …)` で入れる（bringup と同じ）。

| 段 | ファイル | 内容 |
|---|---|---|
| C1 | src/so_arm101_camera/package.xml、CMakeLists.txt | 依存は 2.3 |
| C1 | launch/camera.launch.py | 3 章・6 章の動作 |
| C1 | config/realsense.yaml | 読む相手は realsense2_camera（rs_launch.py の launch 引数）。手で書く。3 値（上の表） |
| C1 | config/apriltag.yaml | 読む相手は apriltag_ros の apriltag_node（parameters）。手で書く。トップのキーは apriltag（ノード名）。1 章の値（ids [0, 1]、frames [tag36h11_0, tag36h11_1]、sizes [0.06, 0.06]、size 0.06 を含む） |
| C1 | so_arm101_camera/__init__.py、camera_config.py | 純粋: yaml の読み込み、rs_launch の引数への変換、extrinsics の検査と TF の引数の作成、未宣言の引数の検査 |
| C1 | test/test_camera_config.py | 試験（11 章） |
| C2 | config/tag_mount.yaml | 読む相手は記録の道具（C2）。手で測った値を手で書く。tag_id（既定 0）と offset_at_zero_pose_base_m（全関節 0 のとき TCP からタグ中心までの base_link の軸方向の差 [m]。既定 [−0.055, 0.0, 0.035]）。取り付けが変わったらこのファイルを直すだけ |
| C2 | so_arm101_camera/tag_mount.py、record.py、points.py | 純粋: オフセット変換とタグ中心の FK、記録のループ（RecordLoop）、オフラインの点の検討 |
| C2 | tools/camera_record_run.py、tools/camera_points_run.py | ROS の部分の記録の道具（購読と標準入力だけ）、オフラインの検討（ROS 不要。ik の import だけ） |
| C2 | test/test_record.py、test_record_static.py、test_tag_mount.py、test_points.py | 試験 |
| C3 | so_arm101_camera/extrinsics.py、tools/camera_extrinsics_run.py、test/test_extrinsics.py など | Kabsch、換算、yaml の書き出し |
| C3 | config/camera_extrinsics.yaml | 読む相手は camera.launch.py（static_transform_publisher）。camera_extrinsics_run.py が書く。人は書かない。今は git に入れ、公開のときに見直す（キャリブの yaml と同じ扱い） |
| C4 | record の検証モード（tfb_ の列、hover の目標の計算）、手順書 | 新しい道具は増やさない |

C1 の時点では camera_extrinsics.yaml は存在しない（概算の値は使わない）。

### 2.3 package.xml の依存

- C1: buildtool_depend は ament_cmake、ament_cmake_python。exec_depend は realsense2_camera、apriltag_ros、tf2_ros、launch、launch_ros、ament_index_python、python3-yaml。test_depend は ament_cmake_pytest、ament_lint_auto、ament_lint_common。
- C2 以降で exec_depend に追加: rclpy、tf2_ros_py、sensor_msgs、python3-numpy、so_arm101_ik、so_arm101_description（URDF の読み取り用）。package.xml の変更なので C2 で承認を取る。
- bringup の package.xml にある usb_cam・realsense2_camera・realsense2_description の exec_depend は触らない（`dependency_audit_plan.md:22` の「過剰の候補」の件は別件。8 章でなく 2.5 で扱う）。

### 2.4 宣言する launch 引数（2 つだけ）

- `use_apriltag`: true か false、既定 false。true のときだけ apriltag のノードを起動する。
- `extrinsics_file`: 既定は share/so_arm101_camera/config/camera_extrinsics.yaml。ビルド前の候補ファイルで試す（C4 の前）ために置く。

画像サイズなどは launch の引数にしない（config から渡す）。内部パラメータはドライバの camera_info（工場の値）を使い、ファイルには持たない。

**未宣言の引数の検査: する**

- 起動前、どのノードより前に、宣言していない引数があればエラーで止める（bringup.launch.py:59-86 の `_check_launch_arguments` と同じ運用。bringup は触らないので約 25 行をコピーし、出どころをコメントに書く）。
- 理由: ① use_apritag:=true のような打ち間違いが黙って無視されると、検出が動かないことに後で気づく。② rs_launch.py の引数（depth_module.depth_profile:= など）を旧起動の癖で渡すと、config を迂回して設定が二重になる。
- 実装上の注意: bringup は `ld.get_launch_arguments()`（:231）で include 先の引数も「宣言済み」に数える。camera では rs_launch.py が約 90 個を宣言するので、同じ方法だと rs の引数が全部通る。camera では自分の 2 つ（と `launch-prefix`・`launch-prefix-filter`）だけを宣言済みとして数える。

**推測（rs_launch の警告）**: rs_launch.py:124-126 は、context.launch_configurations の中で自分が知らない名前（use_apriltag など）を黄色の警告で表示する。親の引数が見える見込み。include を `GroupAction(forwarding=False)` で包んで防ぐ案を第一にする。効くかは、実装時に起動しない評価（visit）で確かめ、できなければ C1 の実機で警告の有無を見る。警告だけで動作は変わらない。

### 2.5 新しいワークスペースに残っているもの（確認結果。2026-11-15）

確認の方法は ls・find・grep・git grep・git log だけ。

**事実**

- `~/ros2_so-arm101` の build・install・log・.git を除く全体を、名前に apriltag・realsense・camera・usb_cam を含むパスで find した結果は 0 件。build/install にも残りは無い。
- apriltag_test は、コミット 759893f（`Remove unused candidates: apriltag_test (A), …`）で git rm 済み。戻し方は、削除の直前のコミット 3a3b0c2 から取り出す（`docs/inventory.md:1795-1797`）。`inventory.md:39` は、旧ワークスペースの調査の表の行（`apriltag_test` は自作・雛形のまま、依存の宣言なし）で、削除の記録は 1794-1813 行にある。
- カメラ起動のコード（`so_arm101_bringup/launch/include/camera.launch.py` と `config/so101_cameras.yaml`・`so101_realsense2.yaml`・`so101_usb_cam.yaml`）は、コミット b95cc8c（L2）で git rm 済み（`docs/lerobot_removal_plan.md:32`）。
- `bringup_with_cameras.launch.py` も L1（7b5360b）で削除済み（`docs/bringup_lint_fix_plan.md:48`）。
- 追跡ファイルに残るカメラ系の語は、`so_arm101_bringup/package.xml:18-21` の `usb_cam`・`realsense2_camera`・`realsense2_description` の exec_depend だけ（ほかは ik_move.py:490 の文言と moveit.rviz の RViz のクラス名で、無関係）。

**未確認**: 段階 1 で「カメラ系は残す」とした記録は、grep（`カメラ系`、`カメラ.*残`）では見つからなかった。ユーザーの記憶している記録の場所があれば教えてもらう。

**計画への反映**

- so_arm101_camera の作成後に削除する段は、**今のところ無い**（新ワークスペースに残りが無いため）。旧ワークスペースでは git rm などをしない。
- bringup の 3 つの exec_depend（上）は、so_arm101_camera が realsense2_camera を宣言した後は、bringup 側では不要になる可能性がある。ただし bringup.launch.py と package.xml は変えない方針なので、**C4 のあとに別の段として、削除の一覧と理由を示して承認を得る**（git rm ではなく package.xml の編集なので、承認を得てから）。

## 3. camera.launch.py の動作

順序:

1. 最初の動作: 未宣言の引数の検査と、extrinsics_file の検査（存在すれば形式・値を検査。壊れていれば、どのノードも起動する前にエラーで止める）。
2. rs_launch.py（apt の realsense2_camera）を include する。launch_arguments は config/realsense.yaml を読んで渡す。
3. use_apriltag が true のときだけ、apriltag_ros の apriltag_node を起動する（name は apriltag、remap は 1 章の 2 つ、パラメータは config/apriltag.yaml）。
4. extrinsics_file があれば、tf2_ros の static_transform_publisher を起動する（親 base_link、子 camera_link。値は yaml から）。無ければ TF を出さず、ログに一行（「camera_extrinsics.yaml が無いので base_link から camera_link の TF は出さない」）。use_apriltag とは関係なく、この動作は同じ。

### 3.1 realsense.yaml から rs_launch.py への渡し方

**事実（rs_launch.py）**: 引数の名前は :37 `rgb_camera.color_profile`、:44 `depth_module.depth_profile`、:82 `align_depth.enable`。ほかに平らなキーの yaml を渡せる `config_file`（:28、:116-117、:153-165）がある。

**推奨: include の launch_arguments に yaml を読んで渡す**（値は文字列、bool は true・false に直す）。旧の `:=` と同じ経路になるので「同じ設定」の証明が簡単。`config_file` で渡す案は、型が経路で微妙に変わり、rs_launch の警告の仕組み（:128-132）にも引っかかる。

**旧起動と同じ設定になることの確かめ方**

1. オフライン試験: 読み込んだ引数が 3 つ（depth_module.depth_profile=640x480x15、rgb_camera.color_profile=640x480x15、align_depth.enable=true。出どころは readme.txt:30-33）と完全に一致すること。
2. オフライン試験: realsense.yaml の全キーが、インストールされた rs_launch.py の `configurable_parameters` に含まれること（綴りの間違いは rs_launch が警告を出すだけなので、ここで検出する）。realsense2_camera が無い環境では skip。
3. 実機（ユーザー、C1）: `ros2 param get /camera/camera depth_module.depth_profile` など 3 つ、camera_info の幅 640 と高さ 480、`ros2 topic hz /camera/camera/color/image_raw` が約 15 Hz。
4. `ros2 launch so_arm101_camera camera.launch.py --show-args` が、宣言した 2 つだけを表示すること。

## 4. 依存の確認

**事実（ls・cat・grep だけで確認）**

| 項目 | 結果 |
|---|---|
| realsense2_camera | 4.58.4（Apache-2.0）。rs_launch.py と realsense2_camera_node あり。realsense2_camera_msgs、realsense2_description も入っている |
| ros-humble-librealsense2 | 2.58.4（realsense2_camera が依存） |
| Intel の librealsense2 | 2.56.5（別の apt。udev の規則 60-librealsense2-udev-rules.rules はこちら） |
| apriltag_ros | 3.4.0（MIT、Christian Rauch）。/opt/ros/humble/lib/apriltag_ros/apriltag_node。apriltag 3.4.5、apriltag_msgs、camera_ros も入っている |
| tf2_ros | 0.25.24。static_transform_publisher は /opt/ros/humble/lib/tf2_ros/。tf2_ros_py も入っている |
| そのほか | launch_ros、ament_cmake_python、cv_bridge、image_transport が入っている。/etc/sysctl.d/99-ros2.conf に rmem_max と wmem_max の 67108864（readme.txt:20-22 の設定） |

**未確認（C1 の起動ログで見る）**: ros-humble-librealsense2 2.58.4 と Intel の librealsense2 2.56.5 が混ざって読み込まれないか。ドライバが起動時に出す Built with と Running with の 2 行で確認する。

**実行前に報告して承認を得るコマンド**（今は 1 つも実行していない。ls・cat・grep・find 以外）:

| コマンド | 目的 |
|---|---|
| `ros2 pkg prefix realsense2_camera`、`ros2 pkg prefix apriltag_ros` | 解決される prefix が /opt/ros/humble であることの確認 |
| `rosdep check --from-paths src --ignore-src -r` | package.xml の依存キーが解決できるか（rosdep が入っているかは未確認） |
| `ros2 run tf2_ros static_transform_publisher --help` | Humble で名前付きの引数（--x … --frame-id）が使えるかの確認。使えなければ位置引数の形（readme.txt:58-61 と同じ）にする |

## 5. ライセンスと THIRD_PARTY

- **事実**: 旧 apriltag_test はユーザーの自作（package.xml:8 の maintainer が hirok、license は TODO。`inventory.md:39`）。readme.txt もユーザーの文章。
- **事実**: apriltag_ros（MIT）と realsense2_camera（Apache-2.0）は、実行時に使う apt の依存で、リポジトリにコードを入れない（コピーも改名もしない）。rs_launch.py も include で参照する。
- **推測**: 旧 tags_36h11.yaml は、apt の apriltag_ros の cfg/tags_36h11.yaml（例）を雛形に作られた可能性がある（detector の 5 項目のキーとほとんどの値が一致）。差は、キー（`/**` と apriltag）、size（0.173 と 0.06）、threads（1 と 2）、qos_profile と pose_estimation_method の有無、tag の中身。
- **決め**: apriltag.yaml は新しく書く（例のコメント・本文は持ち込まない。キー名はノードのパラメータ名で、値は 1 章の旧の値）。THIRD_PARTY_NOTICES.md には足さない（コードを配布しないため）。`docs/third_party.md` に「実行時に使う apt パッケージ（配布しない）」として、realsense2_camera 4.58.4（Apache-2.0）と apriltag_ros 3.4.0（MIT）を 2 行足す。
- もし yaml に例のコメントや本文を写した場合は、NOTICES に MIT の帰属（第 5 節）を足す。判断はユーザー。
- 新しいコードのヘッダは、他のファイルと同じ MIT の許諾文（Copyright 2026 hiro）。

## 6. フレームの関係と換算

**事実（旧の確認と apt の仕様）**

- ドライバの根は camera_link（rs_launch.py:94 の base_frame_id と、旧の tf2_echo の確認 cal_to_tf.py:17）。
- ドライバの TF は /tf_static に出る（publish_tf は既定 true、rs_launch.py:75）。
- 旧のタグの TF の読み方は `lookup_transform(camera_color_optical_frame, tag36h11_0)`（collect_cal.py:95）。
- 新しいリポジトリの src と tools に camera_link を使う箇所は無い（git grep で 0 件）。

**TF の木（予定）**

| 親 | 子 | 出す元 |
|---|---|---|
| world | base_link | moveit_config の static_virtual_joint_tfs（既存） |
| base_link … | gripper_frame_link | robot_state_publisher（既存） |
| base_link | camera_link | 新しい static_transform_publisher。camera_extrinsics.yaml があるときだけ |
| camera_link … | camera_color_optical_frame | ドライバ |
| camera_color_optical_frame | tag36h11_0、tag36h11_1 | apriltag ノード（use_apriltag:=true のとき）。画像の header の frame_id が親になる見込み（推測） |

**換算**（Kabsch の結果から camera_link の TF）

Kabsch は C3 で、光学フレームの点を base_link の点に写す R、t を出す（p_base = R·p_opt + t）。ドライバの camera_link から光学フレームへの回転と位置を R_lo、t_lo とすると、camera_link の TF は次のとおり。

- R_bl = R · R_loᵀ
- t_bl = t − R_bl · t_lo

旧の式（cal_to_tf.py:174-180）と同じ。違いは、旧は R_lo と t_lo を定数でコードに書いていた（:17-19）こと。新は C2 の記録のときに TF から読んだ値を meta に残し、C3 はその値を使う。

**TF の二重の発行が起きないことの確認**

1. オフライン試験: camera.launch.py の TF を出すアクションが 1 つだけで、子が camera_link であること。ほかに TF を出すアクションが無いこと。
2. 実機（ユーザー）の起動前に、残りの static_transform_publisher が無いこと（手順書の ps の確認にある）。
3. 実機（ユーザー）の起動後に、/tf_static を数える（camera_link の行が 1 であること）:

   ```
   timeout 5 ros2 topic echo --no-daemon --qos-durability transient_local --qos-reliability reliable /tf_static 2>&1 | grep child_frame_id | sort | uniq -c
   ```

4. 併せて `ros2 topic info -v /tf_static` で publisher のノード名を見る（4 つの見込み: ドライバ、robot_state_publisher、static_virtual_joint_tfs、新しい TF）。`--no-daemon` が使えるかは未確認。使えなければ付けない（ros2 control list_controllers と同じ扱い）。

## 7. C2 の点の選び方

**守りの一覧（事実。ik_move.py と diff_mode.py の設定）**

| 守り | 値 | 場所 |
|---|---|---|
| 開始の測定の床 | 先端 z が 0.045 m 以上 | ik_move.py:210-219 |
| 目標の床 | z が 0.048 m 以上 | ik_move.py:176 |
| 全点（経路）の床 | 0.045 m | ik_move.py:63 |
| 関節の範囲 | 厳密な範囲と、端から 0.02 rad の余裕 | ik_move.py:167-174 |
| 特異 | abs(D) が 0.99 − 0.001 以下 | ik_move.py:179-182 |
| 半径 | 0.05 + 0.001 m 以上 | ik_move.py:184-187 |
| 肘の枝 | 今の腕と同じ枝 | ik_move.py:196 |
| 移動量の上限 | 先端 0.15 m、各関節 1.0 rad | ik_move.py:57-58、:198-206 |
| 計画の長さ | 20 s 以内 | ik_move.py:67 |

**事実（手順書）**: 出発点（床の下、z ≈ 0.033 m）からは動けない。T への移動は RViz（`large_move_real_check.md` の M1）で、最後の戻しも RViz（V8）。T の関節は (0, −22, 30, 83, 0)°。開始の拒否（START_REFUSED、終了コード 3）は確認の画面より前なので、何も動かず何も失わない（ik_move.py:665-666）。

**事実（届く範囲。`large_move_plan.md:295`、`:509`、`:563`）**: pitch を −91° のまま保つと、wrist_flex の上限 95° が先に当たる。z = 58.8 mm で x = 0.270 m、z = 80 mm で x = 0.2355 m が範囲の外。実機で通った点は T の先端 (0.177, 0, 0.059) m、M2 が (0.200, 0, 0.060)、M3' が (0.230, 0.060, 0.060)。

**推測（手計算）: pitch −91° の範囲**: 手首までの距離が L1 と L2 の和 0.251 m に近づくので、先端の高さは約 0.12 m が上限。使える範囲は x が約 0.15〜0.23 m、z が約 0.06〜0.10 m、y が ±0.06 m くらい。100 mm の立方体は入らない（D3）。

**訂正（2026-11-16。オフラインの計算。上の推測の元の記述は残す）**: pitch −91° の z の上限は約 0.12 m ではなく 0.078 m だった（格子 0.01 m 刻みの 4186 点。x は 0.10〜0.27 m）。pitch −91° では、dz 0.04 の 2 層の箱（12 点）も通らない。詳しくは `docs/camera_pose_study.md` の 3.1。D2 は pitch −30° に決まった（上の 0 章の注記）。

**推測（手計算）: 見え方**: T でのタグの中心は TCP から前へ約 36 mm、上へ約 54 mm、つまり (0.213, 0, 0.113) m ほど。タグの法線は +x（水平で前向き）。旧のカメラ位置の当て推量 (0.416, 0.331, 0.646) m（readme.txt:59）を仮に使うと、法線と視線の角度は約 72°（斜めすぎる）。同じ計算で pitch 0°（旧、collect_cal.py:31）は 36° で、旧の実測の法線 (−0.23, −0.52, −0.82)（35°）とほぼ一致するので、幾何の見立ては当たっていそう。旧の 9 点のうち見えなかった 2 点（行 2・3）は、どちらも y が約 −49 mm（事実。旧 CSV の tag_ok）。見えた点のカメラからの距離は 0.45〜0.58 m。

**方法（姿勢と点の決め方）**

1. **見え方の調査（C1 のあと。腕を動かさない）**: 読み取り専用の起動（トルク OFF、M0 と同じ `read_only:=true`）で、ユーザーが手で腕を候補の姿勢に置く。記録の道具の調査モード（何も送らない）が、検出の有無、法線と視線の角度、検出の率を表示する。
2. **オフラインの検討（純粋な関数、ROS も実機も使わない）**: 出発は T の関節。候補の点を並べ、点ごとに `ik_move.solve_goal(前の点の q_goal, goal, pitch, roll)` を呼ぶ（到着が完全という仮定で、前の目標を次の q_meas に使う）。上の守りの理由を全部出し、キャリブ用の追加の余裕（床 z が 0.058 m 以上、wrist_flex の端から 0.05 rad 以上。どちらも推測）も見る。pitch の候補（−91、−75、−60、−45、−30°）ごとに、届く範囲と、仮のカメラ位置からの角度の平均と、点の広がり（下）を表にする。カメラ位置は検討用の概算で、どこにも保存しない。
3. **並べ方**: 隣の点へ短い移動になる折り返しの順にする（空間の対角線は約 173 mm で先端の上限 150 mm を超えるので使わない）。最初の点は T の近く。
4. **実機での使い方**: オフラインの表が出す `--goal` つきのコマンドをそのまま使う（`--pitch-deg` と `--roll-deg` を明示。M2・M3' と同じ形）。`--dry-run` は最初の点を T から実機で 1 回だけ通す。2 点目以降は実行そのものが開始の前に拒否するので、別の dry-run は要らない（拒否された点は飛ばして記録に残す）。

**点の広がりの基準（3 軸すべてに広がりがあること）**

「少なくとも 2 軸で 40 mm」とはしない。点の集合を重心まわりに中心化し、特異値から各方向の広がり（RMS）を出す。**最小の広がり s_min が、下限を超えること**を条件にする。pitch −91° の範囲は高さ方向が狭いので、この条件が効く。

具体的な値の決め方（オフラインの検討の結果を見て決める。今は値を書かない）:

1. 点の集合の最小の広がりを出し、候補の集合どうしで比べる。
2. 下限は、「回転の角度誤差 ≈ 雑音の RMS ÷ (s_min × √点数)」が約 0.3°（推測）以下になる値から決める。雑音の RMS には、旧 CSV の std（0.3 mm 以下）ではなく、FK の誤差まで含めた 3 mm（推測）を使って保守的に見積もる。
3. タグが見えない点を最大 2 つ失っても（どの 2 つでも）下限を割らない集合を選ぶ。
4. 決めた値は、オフラインの検討の出力とともに手順書に書く（計画の値の改訂は別のコミット）。C3 の合否にも同じ値を使う（9 章）。

**タグが見えない姿勢の扱い**: その点は tag_seen を 0 にして記録し、Kabsch から外す。腕を動かして見つけようとはしない。余裕のため点は 12 個＋最初の点の再訪 1 個とする。最初の点が見えなければ、そこで止めて報告し、D2 に戻る。Kabsch に使う点は 6 個以上（旧の注意の基準 cal_to_tf.py:171）、4 個未満は中止（:151）。

**全点で同じ姿勢にする理由（推論）**: タグの取り付けのオフセットが少し違っていても、同じ姿勢なら FK から出したタグ中心の誤差が一定のずれになる。回転は歪まず、並進に混ざる。この一定のずれは C2 と C4 の「同じ姿勢の比較」では見えない。それを C4 の物理的な確認で見る（9 章）。

## 8. C2 の記録と道具

**値の出どころの区別（列名の接頭辞）**

| 接頭辞 | 出どころ | 例 |
|---|---|---|
| fk_ | FK(q_meas)（URDF の連鎖と tag_mount の値）。カメラは使わない | fk_tcp_x/y/z、fk_tag_x/y/z（base_link、m） |
| cam_ | カメラ（光学フレームのタグの位置の TF）。FK は使わない | cam_tag_x/y/z（光学フレーム、m）、cam_std_*、cam_ptp_*、cam_n、cam_normal_*（タグの z 軸） |
| tfb_ | C4 のみ。TF の木（キャリブ済み）から読んだ base_link のタグの位置。カメラ由来 | tfb_tag_x/y/z |

**記録する項目（1 点 1 行の CSV）**: idx、時刻、目標（入力した `--goal`。参照用）、ラベル、q_meas 5 つ（フレームごとに読んだ値の平均）、q_span（最大と最小の差）、上の列、incidence_deg（タグの法線と視線の角度）、tag_seen、still（整定）、stable（前半と後半の平均の差）。

- **整定の判定**: ik の整定と同じ基準を使う。待ちは 1.5 s のあと、0.6 s あけて 2 回読み、差が 0.003 rad 以内、時間切れは 8 s（`step_runner.py:59-62` の is_stopped、SETTLE_MIN など）。旧の collect_cal.py:86-88 より、今の実機で検証済みの基準に合わせる。
- **平均するフレーム数**: 15 フレーム（推測。旧は 10、collect_cal.py:21。15 fps で約 1 s）。stamp が違うものだけ数え、1 s 以内の新しいものに限る（旧の仕組み、:96-100）。前半と後半の平均の差が 0.3 mm 以内（旧、collect_cal.py:150）を stable とする。旧の 10 フレームの中の標準偏差は、ほとんどの点で 0.28 mm 以下（事実。旧 CSV の std）。

**meta（CSV の横の別ファイル。出どころの記録）**: URDF の sha256（`model_check.py:71` のハッシュ関数）、タグのオフセット（base_link の軸方向と、変換後のグリッパー座標）、tag_id、タグの一辺、apriltag.yaml と realsense.yaml の sha256、camera_info（K、D、幅、高さ。内部パラメータはドライバの値で、ファイルには持たない。meta は記録の控え）、camera_link から光学フレームへの TF（6 章）、フレーム数と閾値、日付。

**新しく作るか、既存を使うか**

- 腕を動かす部分は、既存の `tools/ik_move_run.py` をそのまま使う（M0〜M3' で実機確認済み）。
- 記録の道具を新しく作る。腕に何も送らず、シリアルポートも開かず、カメラも開かない。/joint_states、camera_info、TF を購読するだけ。
- 1 点ごとに、ターミナル A で ik_move_run（確認 2 回）、ターミナル B で記録（Enter で開始）。

**純粋な部分と ROS の部分（ik_move.py と ik_move_run.py と同じ形）**

| 純粋（ROS・時計・入出力なし） | ROS の部分 |
|---|---|
| so_arm101_camera/record.py の RecordLoop。判断だけをして、入出力は io オブジェクトに任せる（now、wait、読み取り、タグの読み取り、say、log_row） | tools/camera_record_run.py。ノード、TF の listener、/joint_states と camera_info の購読、標準入力の確認 |
| tag_mount.py（オフセット変換、タグ中心の FK。URDF の連鎖は ik の UrdfChain を使う。URDF は description の share/urdf/so101_new_calib.urdf。`test_model_check.py:36` と同じ） | |
| points.py（オフラインの検討） | tools/camera_points_run.py（ROS 不要。ik の import だけ） |

## 9. C3（Kabsch）と C4（独立した確認）

### 9.1 C3: tools/camera_extrinsics_run.py（ROS 不要。CSV から yaml）

流れ:

1. CSV と meta を読む。列は固定の名前で読み、旧のような列名の自動判定（cal_to_tf.py:83-89）はしない。
2. 使う行を選ぶ: tag_seen、still、stable、incidence_deg が閾値以下（60° 以下を仮置き。推測）、cam_std が 1 mm 以下（推測）。
3. Kabsch（P は cam_tag、Q は fk_tag。反射の補正つき。旧 :67-75 と同じ）。
4. 報告: RMS、点ごとの残差、1 点外し（LOO。旧 :163-168）、点の広がり（7 章の s_min と、3 軸の広がり）。
5. **スケールの診断**: 剛体の解のほかに、スケール付きの当てはめも出す。1 から離れていれば、タグの一辺の違いか内部パラメータのずれが疑われる（タグの一辺が違うと、カメラの奥行きが比例してずれる。例えば 1 mm の違いで、距離 0.8 m のとき約 13 mm のずれ。手計算）。Kabsch は剛体なので、このずれは残差に出る。
6. 6 章の式で camera_link に換算し、camera_extrinsics.yaml を作る。

yaml の中身: frame_id が base_link、child が camera_link、translation（x、y、z [m]）、rotation（クォータニオン x、y、z、w）、人が読むための rpy、出どころ（CSV 名と sha256、使った点数、RMS、LOO の最大、日付、タグのオフセット、URDF の sha256、realsense.yaml の sha256）。

**書き込みの守り**

- 既定では提案を表示するだけ。`--write` と指定したときだけ書く。
- 既存のファイルは `--force` が無いと上書きしない。
- 合格しなければ何も書かない（終了コード 2）。`--write` と `--force` を付けても、不合格なら書かない。

**合否の基準（案。数字はすべて推測。点の広がりだけは 7 章の決め方で決める）**

| 項目 | 基準 |
|---|---|
| 使った点数 | 6 以上（旧の注意 cal_to_tf.py:171 に合わせる） |
| RMS | 5 mm 以下（旧の警告の基準 :169。ユーザーの方針は一回で 1 cm までの許容） |
| LOO の最大 | 8 mm 以下 |
| スケール | 1 ± 0.01 を超えたら警告（合否は変えない）、1 ± 0.05 を超えたら不合格（J4d、2026-11-17 の決定。元の「1 ± 0.01」を直した） |
| 点の広がり | 3 軸すべてに広がりがあり、最小の広がり s_min が 7 章で決めた下限以上 |

**注記（2026-11-16、C3-a。コミット 14ce22d。上の元の記述は残す）**

- **yaml の中身は 2 キーだけ**: camera_config.py:162-165 が、キーがちょうど `translation_m` と `quaternion_xyzw` でないと拒否する。上の「yaml の中身」（frame_id・child・rpy・出どころ）は本体に入れず、**sidecar `camera_extrinsics.yaml.meta.yaml`**（launch は読まない）に入れた。本体の 1 行目は「読む相手・道具が書く」のコメント。sidecar には本体の sha256、入力の CSV と meta の sha256、使った行と使わなかった行の理由、閾値、点ごとの残差と LOO、スケールと含意するタグの一辺、s_min と特異値、rpy、identity の sha256 を入れる。
- **引数の追加**: `--output`（既定は src の `config/camera_extrinsics.yaml`。install には書かない）と `--exclude LABEL`（やり直した点の古い行などを外す。無い label は終了コード 2）。閾値は引数にしない。
- **行の選び方**: tag_seen・still・stable が 1、cam_n が 10 以上（推測）、incidence_deg が 60 以下、cam_std が各軸 1 mm 以下、数値が有限。
- **LOO は組ごと**: fk_tag が 10 mm 以内（推測）の行を 1 つの組として、組ごと外す（P01 と P13 は FK で約 7.4 mm の差で、片方を外してももう片方が残るため）。
- **診断の追加**: 含意するタグの一辺（0.06 × s）、特異値 3 つ。不合格でも、残差・LOO・スケール・広がりをすべて表示してから終了コード 2 にする。
- **書き込み**: 一時ファイルに作って `load_extrinsics` で読み戻し、新規は `os.link`、`--force` は `os.replace`。本体を最後に書く。

### 9.2 C4: 独立した確認（2 段。**どちらも必須**）

C4 は、記録の道具の検証モードと、既存の `ik_move_run.py` だけで行う。新しい道具は増やさない。

#### C4-a: カメラと FK の比較（ソフトの経路の確認）

- C2 で使っていない別の位置（内側 3〜4 点と、点の集合の外側に約 20 mm 出た 1〜2 点）で記録する。
- camera_extrinsics.yaml を使った TF の木（camera.launch.py を起動し、base_link から tag36h11_0 までの TF が通る状態）から、タグの位置を base_link で読む（列 tfb_tag_x/y/z）。fk_tag との差のベクトルを出す。
- 見つかるもの: 換算式の間違い、yaml から launch、TF の木までの経路の間違い、TF の二重の発行。
- 合否（案、推測）: 内側の点は平均 5 mm 以下、最大 10 mm 以下。外側の点は最大 15 mm 以下。yaml の値と TF の値が 1e-6 以内で一致する。

**C4-a だけでは足りない理由**: C2 と C4-a が同じ姿勢で同じ概算のタグのオフセットを使うと、オフセットの誤差は C2 で TF の並進に吸収され、C4-a では打ち消し合って見えない。回転にも混ざらないので、RMS も LOO も小さいままになる。この誤差は、カメラが見るもの（机の上のタグ）の位置を、一定のずれでずらす。これを、カメラにも FK にも頼らない定規の測定で見るのが C4-b。

#### C4-b: 定規による物理的な確認（必須）

旧の接触試験（`~/ros2_so101/test/contact_test.py`。読むだけ）を参考にする。旧の流れ: 机の上の tag36h11_1 の base_link の位置を TF から読み（:13、:22）、タグが平らか（傾き 10° 以下、:41-43）と目標の高さ（:44-46）を検査し、タグ中心の真上へ TCP を送り（:3）、最後に定規で測る（:80）。

新しい C4-b との違い:

- 目標は、大きな移動の道具（`ik_move_run.py --goal …`）で一回だけ送る。旧の最大 3 回の補正のループ（:53-79）は入れない（補正すると、測りたい誤差が消える）。
- pitch・roll は C2 と同じ値にする（旧は 0/0）。姿勢が違うとオフセットの誤差が回転して見えるので、C2 と同じ姿勢でないと意味がない。
- 床の規則を満たす高さにする（目標の床 z ≥ 0.048 m、キャリブ用の余裕を入れて 0.058 m 以上。7 章）。机の面は z ≈ 0（取り付け面と同じ高さ。事実: `inventory.md` の実測の記録）。タグが机に直置きなら、hover（タグ中心から TCP までの高さ）は 0.060〜0.080 m にする。厚さを測った台に置くなら、台の厚さを加えて、その分 hover を小さくしてもよいが、目標 z は 0.058 m 以上。

**準備（ユーザー）**

1. ID 1 のタグを机（または、厚さをノギスで測った台）に置く。平らに。タグの一辺は D4 で測った値を使う（apriltag.yaml の sizes 2 つに同じ値）。
2. TCP（gripper_frame_link）の物理的な位置に、目印をつける（先端の治具、テープの印など）。TCP は仮想の点なので、定規で測る基準を決めておく（推測: 今まで接触試験でどう決めていたかは、ユーザーに確認が要る）。
3. カメラは C2・C3 から動かさない（D5）。

**手順（骨組み。実行するのはユーザー）**

1. 検証モードで、ID 1 のタグの base_link の位置（tfb_tag）と傾きを読む。腕がタグを隠さない姿勢で読む。傾きが 10° を超えたら置き直す（旧 :43）。
2. 検証モードが、目標 (x, y, z + hover) と、そのまま使える `ik_move_run.py --goal … --pitch-deg … --roll-deg …` のコマンドを表示する（純粋な関数。目標の z が 0.058 m 未満なら、表示せず理由を出す）。
3. まず `--dry-run`。通ったら実行（確認 2 回）。
4. 止まったら（コマンド A で `STOPPED`）、ユーザーが定規で測る: 真上から、TCP の目印とタグ中心の**水平のずれ**（前後 X、左右 Y）と、**TCP の高さ**（机、または台の面から）。
5. 検証モードが、同時に記録した値と比べる。比べる値: 予測 = FK(q_meas) の TCP − tfb_tag（どちらも base_link）。測った値と予測の差が、カメラの経路と FK を合わせた誤差になる。
6. タグの位置を変えて、**少なくとも 3 か所**で繰り返す（机の上で ID 1 を動かす。推測: 3 か所なら、一定のずれ（オフセットの誤差）とばらつきを分けられる）。毎回、腕は T に近い安全な位置から送る（移動量の上限 先端 0.15 m）。

**合否の基準（案。すべて推測。定規の読みは ±2 mm 程度と見込む）**

| 項目 | 基準 |
|---|---|
| 1 か所ごとの水平の差（測った水平のずれと予測の差、ベクトルの大きさ） | 5 mm 以下（10 mm 超は不合格。ユーザーの方針は 1 cm） |
| 1 か所ごとの高さの差 | 5 mm 以下 |
| 3 か所の差のベクトルの平均（一定のずれ。オフセットの誤差の目印） | 3 mm 以下。超えたら、tag_mount のオフセットを測り直して C2 からやり直す |
| 3 か所の差のばらつき（標準偏差） | 4 mm 以下 |

差の原因の切り分けの目安: 一定のずれが大きい → オフセットの誤差か、camera_link の換算の誤差。場所によって大きく違う → スケール（タグの一辺の違い）か FK のモデル誤差。

## 10. 各段の実機の手順の骨組み

Claude が実行するのは、オフラインの試験、ビルド、モック、`--show-args` まで。実機とカメラはユーザー。

**モックでの確認（Claude が実行できる）**: モックの腕（`mock:=true`、`ROS_DOMAIN_ID=87`）に、スクラッチの合成のタグ TF 発行器（真の変換と 0.3 mm 程度の雑音を仮定し、FK から光学フレームのタグの位置を作る）をつなぎ、記録（C2）、C3、C4-a を通して、仮定した変換が戻ることを確かめる。カメラは開かない。発行器はリポジトリに入れない（スクラッチ）。launch はCLAUDE.md の「前景」の方法で起動し、停止後は ps と `ros2 node list --no-daemon` で残りを確認する。

**C1 の実機（ユーザー。腕は要らない）**

1. ps と `ros2 node list --no-daemon` が空。
2. `ros2 launch so_arm101_camera camera.launch.py`（use_apriltag は false）。確認: 3.1 の 3 項目、camera_info の D が全部 0（推測。D415 の color は歪みがほぼ 0。0 でなければ apriltag の入力の rectify を検討）、/tf_static に camera_link から光学フレームがあって base_link から camera_link は無い、ログに extrinsics が無い旨の一行、ドライバの Built with と Running with の 2 行（4 章）。
3. `use_apriltag:=true` で、ID 0・1 のタグを手で持って 0.6、0.8、1.0 m にかざし、tag36h11_0 と tag36h11_1 が出ること、距離と std（D4 で測った一辺）を見る。
4. 6 章の /tf_static の確認。
5. 停止後の ps と `ros2 node list --no-daemon` の確認。

**C1 の続き（ユーザー。腕を手で置く）**: 読み取り専用の起動（トルク OFF）で、7 章の見え方の調査をする。

**C2 の実機（ユーザー）**

- 端末は 4 つ: bringup（M1 と同じ書き込みモード、`torque_limit:=500`）、camera.launch.py（`use_apriltag:=true`）、ik_move_run、記録。
- 手順: M1 と同じに RViz で T へ、T で記録の調査（何も送らない）で検出を確認、1 点目だけ dry-run、点ごとに「移動、整定、記録」を繰り返す。
- 終わりに T へ戻し（`--joints-deg 0 -22 30 83 0`）、RViz で出発点へ戻し（V8）、停止の 2 行（`[deactivate]` と `deactivated.`）と ps を確認する。
- 所要は 1 点約 2 分で、12 点で約 25〜30 分（推測）。毎回、CSV は ls で同名が無いことを確かめる（既存の約束）。

**C3**: オフライン。ユーザーが提案を読んで `--write` で yaml を作り、colcon build、コミット。

**C4 の実機**: C4-a は C2 と同じ端末の構成で、camera.launch.py はキャリブ済みの TF つき。C4-b は C4-a の続きで、ID 1 のタグを使い、定規で測る。

**注記（2026-11-17、F5。`docs/zero_offset_calib_plan.md` の 6.3。上の手順の元の記述は残す）**: 実行の順序は、**センターキャリブ → オフセットキャリブ → カメラのキャリブ（C2・C3）**。オフセットキャリブ（lift・elbow・wrist_flex の `zero_offset_deg`。先端を机に当てた 4 姿勢。手順は `docs/zero_offset_calib_real_check.md`）で得た δ を、J5（F6）で yaml に入れてから、カメラの C2・C3 を行う（pan のずれは外部パラメータに入る）。C3 の再計算（J6）は、J5 の後に、δ を `--joint-offset` で渡すか、yaml 反映後の新しい記録で行う。センターキャリブをやり直したら、オフセットキャリブもやり直す。

## 11. colcon test の件数の見込み

**事実（`build/*/test_results` から読んだ数え方）**: 件数は pytest、lint、CTest の合計。ik は 294 + 49 + 21 = 364。bringup は pytest 34、lint 7（pep257 2、copyright 2、flake8 1、xmllint 1、lint_cmake 1）、CTest 6（5 つの lint と pytest の 1 本）で 47。新しいパッケージは、.py ごとに copyright と pep257 が 1 件ずつ、それに flake8・xmllint・lint_cmake が 1 件ずつ、CTest は lint の 5 つと pytest のファイル数。

**試験に絞る範囲**: 次の 5 つに絞る。points.py のオフラインの検討の試験は最小限（実機の安全は ik_move 側の守りと実機の dry-run が受け持つ）。

1. 守り: 記録の道具が何も送らず、シリアルポートもカメラも開かないこと（静的な検査）、`--write`・`--force`・不合格なら書かないこと。
2. 換算式: Kabsch の結果から camera_link の TF。
3. Kabsch: 既知の変換が戻ること、反射の補正。
4. 未宣言の引数の検査。
5. realsense.yaml が rs_launch の引数と一致すること。

**見込み（推測。±15%）**

| 段 | 内容の内訳 | pytest | lint | CTest | 小計 |
|---|---|---|---|---|---|
| C1 | camera_config の試験（realsense.yaml の一致 4、未宣言の引数の検査 7、extrinsics から TF の引数 5）。py は launch、__init__、camera_config、test の 4 つ | 16 | 11 | 6 | 33 |
| C2 | record 14、静的な守り 4、tag_mount 6、points 4（最小限）。py は 3 つの module と 4 つの test の 7 つ | 28 | 14 | 4 | 46 |
| C3 | Kabsch と反射、換算、スケール、広がり、書き込みの守り（`--write`・`--force`・不合格）で約 30。py は module 1 と test 2 の 3 つ | 30 | 6 | 2 | 38 |
| C4 | tfb_ の列、hover の目標の床、コマンドの文字列、傾きの検査。py は test 1 | 6 | 2 | 1 | 9 |
| 合計 | | 80 | 33 | 13 | 126 |

累計は 591 + 126 = 約 717 件（推測）。

**注記（2026-11-16、C3-a。コミット 14ce22d。上の見込みの元の記述は残す）**: C3 の見込み 38（pytest 30、lint 6、CTest 2）は少なかった。実績は camera が 218 → 285 件で、pytest +59（extrinsics 45、道具 14）、lint +6、CTest +2（.py が 3 つ、試験の登録が 2 つ）。C2 の見込み 28 に対して実績が 153 だったのと同じ傾向。全体は 809 → 876 件（C3-a のあと。ほかのパッケージは変更していない）。

**既存の 591 件が変わらないことの確かめ方（C1）**

1. `git diff --stat` に src/so_arm101_bringup と他の既存パッケージが出ない。
2. パッケージごとの件数が不変: bringup 47、calibration 73、description 2、hardware 105、moveit_config 0、ik 364。
3. `bringup.launch.py --show-args` の 21 個が前後で diff 空。
4. `~/so_arm101_check/gen_rd6.sh` の 6 通りの sha256 が `urdf_after/` と一致（`bringup_lint_fix_plan.md:248-250` の手順）。

## 12. 承認が必要な変更と、段の区切り

**承認が要るもの**

- 新しいパッケージ（so_arm101_camera 一式）、package.xml、CMakeLists.txt、launch/camera.launch.py、config の yaml、新しい tools/*.py。
- C2 での package.xml の依存の追加。
- CLAUDE.md の最終構成の 2 行（0 章の提案。別の承認）。`docs/third_party.md` の 2 行。
- bringup の 3 つの exec_depend の削除（2.5。C4 のあとの別の段）。
- 触らないもの: so_arm101_bringup、so_arm101_ik、MoveIt の設定（URDF、関節リミット、planning_frame、IK solver）。旧ワークスペース `~/ros2_so101` は読むだけ。

**段とコミット**（段ごとに colcon build、関係パッケージの colcon test、承認、コミット。push はしない）

| 段 | 内容 | コミット |
|---|---|---|
| 計画 | docs/camera_stage_plan.md | docs だけ。colcon test は省略 |
| C1-a | so_arm101_camera、launch、config 2 つ、試験、docs/third_party.md | build と test のあと |
| C1-b | docs/camera_real_check.md（手順書）。実機の結果は別の docs コミット | docs だけ |
| C2-a | tag_mount.yaml、record、points、2 つの tools、package.xml の依存、試験 | build と test のあと |
| C2-b | 実機の結果の記録 | docs だけ |
| C3-a | extrinsics.py、tools/camera_extrinsics_run.py、試験 | build と test のあと |
| C3-b | camera_extrinsics.yaml（生成物） | データだけ。生成の記録つき |
| C4 | 検証モード（C4-a、C4-b の hover の計算）、手順書、結果 | 実装は build と test のあと。結果は docs |
| 後 | bringup の exec_depend の整理（削除の一覧と理由を示して承認を得る） | 別の段 |

## 13. 未確認・保留

- 段階 1 の「カメラ系は残す」の記録の場所（2.5）。
- ros-humble-librealsense2 2.58.4 と Intel の librealsense2 2.56.5 の混在（C1 のログで見る）。
- rs_launch の警告が親の引数で出るか、`GroupAction(forwarding=False)` で防げるか（実装時に確かめる）。
- TCP の物理的な目印の決め方（C4-b の準備 2）。
- D4 のタグの一辺の実測値（ID 0・1）。
- 点の広がりの下限 s_min（7 章の決め方で、オフラインの検討の後に決める）。
- 本文の［推測］の値（閾値、フレーム数、所要時間、件数の見込み）。

**注記（2026-11-17、J4d。コミット cef7a65。上の合否の表の「スケール 1 ± 0.01」は残す）**

- スケールは、|s − 1| が 0.01 を超えたら警告（合否は変えない。`WARNING:` の行と `result: accepted (with N warning)`）、0.05 を超えたら不合格に変えた（ユーザーの決定）。警告つきの合格でも `--write` できる。sidecar の閾値は `scale_warn`・`scale_fail`、result に `warnings`。詳しくは inventory.md の「J4d」。

**注記（2026-11-19、C4 の計画の改訂。ユーザーの決定。上の 9.2 の元の記述は残す）**

- **検証モードが要る**: 9.2 の「新しい道具は増やさない」は成り立たない。`tools/camera_record_run.py` に検証モード、`record.py` に純粋な関数、`test_record.py` に試験を足す（CMakeLists.txt・package.xml・launch・URDF は変えない）。手順書は `docs/camera_c4_real_check.md`。
- **δ を二重にかけない**: F6a 以降、/joint_states の読みにはドライバが δ を足した値が出る。C4 の FK は読みをそのまま入れる（δ の引数を持たない）。`camera_extrinsics_run.py --joint-offset` が δ を足すのは、補正前の C2 の CSV を計算し直すときだけで、C4 の記録には使わない。試験で、入口に δ の引数が無いこと、二重にかけると約 18 mm 外れることを見る。
- **C4-b の比較は 2 つ**: (a) 定規で測った ID 1（台座の決めた角から x・y。机に印）と tfb_tag1 の差（カメラの確認）。(b) 定規で測った固定あごの先端と ID 1 の相対から出した物理の TCP と、FK(読み)・FK(reference) の差（A・B・δ の判定。カメラは使わない）。台座の角と base_link の関係は URDF から出し、手順書に図と式で書く。角は C4-0 の出力（台座の外形）を見てユーザーが決める。
- **hover の目標は、定規で測った ID 1 の位置から出す**。tfb_tag1 は (a) の比較にだけ使う（カメラの誤差が目標に入らないように）。
- **A・B の判定**: |FK(読み) − FK(reference)| が 4 mm 以上の点だけで行い、その点で物理の TCP がどちらに近いかの平均で見る。点は 3〜4 か所、少なくとも 2 か所は T のような重力の負荷が大きい姿勢。4 mm 以上の点が 2 か所未満なら「判定不能」。FK(reference) は `/arm_controller/controller_state` の reference を購読して使い、ik_move の q_goal との差も記録する。lift は推定値と残差を記録するだけで、yaml には書かない。
- **測り方**: TCP の目印は固定あごの先端。水平の相対は、三角定規（直角）を机に立てて固定あごの先端を机の面に落とし、ID 1 の中心との X・Y を読む。高さは ID 1 の面から先端まで定規を垂直に立てて読む。台座の取り付け面と机の面の高さの差、ID 1 の厚さも定規で測る。
- **タグの一辺（D4）を測る段を入れる**: 黒い枠の外側の一辺を、ID 0 と ID 1 で縦横 2 回ずつ測って平均する。値を apriltag.yaml に反映するか、C3-b の外部パラメータの見直しが要るかは、別の承認・別の判断。
- **点の作り方**: 印は 1 か所が望ましい。案 A（同じ ID 1 の上で hover の高さと前後のずれを変える）と案 B（印を点ごとに変える）を、C4-0 のオフラインの見積りで比べて決める。
- **新しい基準**: 止める条件と判定は、F6 のあとの基準（RViz の終点の差 0.04 rad、elbow の大きい側 +0.05、gripper ±0.01、出発点の pan・lift 0.05 rad、物差しは固定あご）に合わせる。
- **δ・外部パラメータ**: **δ は C4 では、A でも B でも書き換えない**（follower_calibration.yaml・camera_extrinsics.yaml・sidecar は変えない）。B のときは T の付近を差分 IK で詰める。lift は推定値と残差を記録するだけ。δ の書き換えが必要とユーザーが判断したときだけ、別の計画と承認で J6（`--joint-offset`、`--write --force`、sidecar の `provisional_note` の書き直し）を行う。
- **含めないもの**: bringup の exec_depend の整理（C4 のあとの別の段）。

**注記（2026-11-19、C4-0 の結果と C4 の計画の確定。ユーザーの決定。9.2 と上の注記の元の記述は残す。詳しくは `inventory.md` の「C4-0」）**

- **pitch・印・点**: C4-b は pitch −91°（T と同じ向き。roll 0）、案 A（印は 1 か所）、4 点。9.2 の「pitch・roll は C2 と同じ値」は、床 z 0.058 m 以上の hover に pitch −30° では届かない（C4-0）ので、(b) の A・B の判定と (a) のカメラの確認については外す。印は base_link の (0.20, 0)（後ろの辺から 0.221 m、y = 0）。4 点は、印を中心に前後のずれと高さを変えた組で、実装のあとの計算で決め、手順書に書く。
- **台座の基準**: x は後ろの辺（base_link の x = −0.0211）から前へ、y は後ろの 2 つの角 (−0.0211, ±0.0430) の中点の線（y = 0）から左右へ。base_link の x = 後ろの辺からの x − 0.0211。
- **定規の目印**: 固定あごの先端のパッドの、ベースから最も遠い縁（前の縁）の中央（REF_POINT。gripper_frame の +z 0.0063 m。pitch −91° では固定あごの最低点）。水平は三角定規（直角）を机に立てて先端を机の面に落とし、ID 1 の中心との X・Y を読む。高さは ID 1 の面から先端まで、定規を垂直に立てて読む。
- **高さの基準と符号（record.py の関数のコメント・手順書の測り方と同じ向き）**: Δ = 机の面の高さ − 取り付け面の高さ（**机の面が高いとき正**）、t = ID 1 の厚さ。ID 1 の面の base_link の z = −0.0024 + Δ + t。取り付け面からの高さ H = base_link の z + 0.0024（F6 の「机からの高さ」と同じ基準。Δ = 0 のとき）。ID 1 の面からの高さ = H − Δ − t。予測は base_link の z と H の両方を書く。**C4-b の定規は ID 1 の面から測るので ID 1 の面からの高さと、F6 のように机から測るときは H と比べる。**
- **A・B の判定と δ の位置づけ**: |FK(読み) − FK(reference)| が 4 mm 以上の点だけで、物理の先端がどちらに近いかの平均で判定する（2 点未満は不能）。点は 4 か所で、T のような重力の負荷が大きい姿勢が 2 か所以上。**C4 の主な結果は A・B の判定と (a) のカメラの確認**。δ_lift・δ_elbow の推定値は参考値で、共通のずれ（Δ・t・x の基準を各 ±1〜2 mm）による不確かさ（約 ±0.5〜1°）を添える。
- **(a) の比較は腕を退けた状態だけ**: hover の 4 点では ID 1 があごに隠れる。(a)（定規で測った ID 1 と tfb_tag1）は `--verify-survey`（腕を退けた状態）の値だけを使う。`--verify-record` は tfb_tag1 を判定に使わない。記録するなら TF の時刻の古さも残す。
- **FK の入力**: /joint_states の読みをそのまま FK に入れる（δ は二重にかけない）。reference は /arm_controller/controller_state の reference。ik_move の q_goal との差も記録する。
- **F6 の値の基準**: F6 の 54.89・49.91・53.86 mm は取り付け面からの高さ（H）。F6 の結論は変わらない（inventory.md の C4-0 の 4）。
