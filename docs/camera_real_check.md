# 実機確認手順: カメラの段 C1（`camera.launch.py`。腕は使わない）

**2026-11-16 にユーザーが実施し、すべて合格。** 結果は `docs/inventory.md` の「C1 の実機の確認（2026-11-16）」。実機で直した箇所は、各段に「2026-11-16 の実機で直した」と書いた（元の記述は残してある）。

計画: `docs/camera_stage_plan.md`（決定 D1〜D6 は 0 章、C1 の骨組みは 10 章、フレームは 6 章、依存は 4 章）。実装: コミット `6cd918c`（C1-a）と `d2b497b`（クォータニオンの正規化）。記録: `docs/inventory.md` の「`so_arm101_camera` の C1-a」。

> **この手順書のコマンドは、Claude Code が一つも実行していない（未実行）。** Claude が実行したのは、C1-a のオフラインの試験（`colcon test`）と `--show-args` までで、**カメラ（RealSense）も launch も開いていない**。**この手順書はユーザーだけが実行する。** 書いた内容は、apt のファイルとリポジトリのファイルを読んで確かめた（確かめた場所は各段に `ファイル:行` で書く）。読んで確かめられなかったことは「見込み」と書く。
>
> **腕（サーボ）は使わない。** bringup は起動しない。シリアルポートは開かない。カメラ（USB）だけを使う。

この段で確かめること: (1) タグの一辺の実測（D4）、(2) カメラのトピックと設定（640x480、約 15 fps）、(3) AprilTag の検出（ID 0・1）、(4) extrinsics が無いとき `base_link` から `camera_link` の TF が出ないこと、存在しないパスを渡すと起動前に止まること、(5) inventory の未確認の 2 点（rs_launch の警告、librealsense2 の版の混在）。

## 共通の約束

| 約束 | 内容 |
|---|---|
| 実行するのはユーザーだけ | launch の端末（ログを流す。操作しない）と操作用の端末の 2 つ。止めるのは Ctrl-C だけ（`kill -9`・`pkill`・`killall` はしない） |
| 新しい端末 | **conda を activate していない**端末。準備の 3 行は下の「ユーザーがすること」。**`ROS_DOMAIN_ID` は設定しない（87 はモック専用）** |
| `--no-daemon` | `ros2 topic`・`ros2 node`・`ros2 param` には付ける。付けて `unrecognized arguments` になったら、付けずに実行し、その旨を報告する（未確認。計画 6 章と同じ扱い）。**2026-11-16 の実機で直した。`ros2 topic hz` は `--no-daemon` を受け付けない**（`unrecognized arguments`、exit=2）。`hz` には付けない。ほかのコマンドは付けて通った |
| ログ | `tee -i` と `; echo "exit=${PIPESTATUS[0]}"`。保存先は `~/so_arm101_check/camera_<日付>_C1/` |
| 同名のファイル | **書く前に `ls` で同名が無いことを確かめる**（`No such file or directory` が出るのが期待どおり）。`tee` は上書きする |
| コードと設定 | **この段では何も変えない。** タグの一辺が 0.06 m と違っても `config/apriltag.yaml` は変えず、報告だけ（D4） |

## ユーザーがすること（最初に 1 回）

1. 新しい端末で、準備の 3 行を実行する（conda なし）:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_so-arm101/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
2. **ros2 daemon を起動し直す**（cyclonedds を設定したこの端末で。別の RMW の daemon が残っている場合に備えて、止めてから起動する。ユーザーが行う）:

   ```bash
   ros2 daemon stop; ros2 daemon start; sleep 3
   ros2 daemon status
   ```
3. 前回の起動の残りが無いこと:

   ```bash
   ps aux | grep -E "realsense2_camera_node|apriltag_node|static_transform_publisher|ros2_control_node|move_group|robot_state_pub|rviz2|spawner" | grep -v -E "grep|gvfsd"
   ros2 node list --no-daemon
   ```

   どちらも空であること。残っていたら、ここで止めて報告（`static_transform_publisher` が残っていると、TF が二重になる。計画 6 章）。
4. 保存先を作り、カメラが USB で見えること（`<日付>` は実行する日。例: `camera_20261116_C1`）:

   ```bash
   mkdir -p ~/so_arm101_check/camera_<日付>_C1
   cd ~/ros2_so-arm101
   ls /dev/video* | head -3
   ```

   カメラを USB 3 のポートに挿す。**カメラが挿さっていないと、ドライバは起きるまで待ち続ける**（`wait_for_device_timeout` の既定は −1。`rs_launch.py:92`）。そのときは Ctrl-C で止めて報告する。

## 止め方（先に読む）

| 状況 | 操作 |
|---|---|
| launch の端末（C1-1、C1-2） | Ctrl-C を 1 回。腕は無いので、落ちるものは無い。ドライバの終了に数秒かかることがある。**2 回目の Ctrl-C は、10 秒待っても止まらないときだけ** |
| 操作用の端末の `ros2 topic …` | Ctrl-C |
| 止まらないとき | `kill -9`・`pkill`・`killall` はしない。報告して指示を待つ。最後の手段は USB ケーブルを抜くこと（ドライバのログに切断が出る） |

停止のたびに、下の「後始末」を行う。

## 終了コードと表示

| 場面 | 見込み |
|---|---|
| launch を Ctrl-C で止めた | `exit=0`、または 130 など。どれかを記録する（C1-a ではこの値を確かめていない） |
| C1-3 の存在しないパス | **0 以外**（Python の例外で止まる。値は 1 の見込み。記録する）。**2026-11-16 の実機で直した。実機の値は 1**（元は「値は 1 の見込み」） |
| `timeout N ros2 …` | 時間切れは `exit=124`（期待どおり） |

## C1-0. タグの一辺の実測（D4。launch なし）

AprilTag の一辺 `size` は `config/apriltag.yaml:7` と `:18` の 0.06 m。これが「黒い正方形の外縁」の長さであることを、実物で確かめる（計画 0 章 D4）。

1. **ID 0 と ID 1 の両方**について、ノギスで黒い正方形の外縁から外縁までを測る。**縦と横の 2 方向**を、0.1 mm の単位で（白い縁は含めない）。
2. 4 つの値（ID 0 の縦・横、ID 1 の縦・横）を、報告に書く。
3. `0.060 m` と違っても、**この段では yaml を変えない**。報告して、直すかは別に決める。

併せて、起動前の引数の確認（ノードは起動しない）:

```bash
ros2 launch so_arm101_camera camera.launch.py --show-args
```

- 合格: 引数が **`use_apriltag` と `extrinsics_file` の 2 つだけ**（`camera.launch.py:79-84`）。rs_launch の引数（約 90 個）は出ない。

## C1-1. `use_apriltag:=false`（既定）で起動

launch の端末で:

```bash
ls ~/so_arm101_check/camera_<日付>_C1/C1-1_launch.log
ros2 launch so_arm101_camera camera.launch.py 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C1/C1-1_launch.log; echo "exit=${PIPESTATUS[0]}"
```

起動したら 10 秒ほど待つ。

### C1-1a. ログ（inventory の未確認の 2 点）

別の端末（準備の 3 行を実行した端末）で、ログを読む:

```bash
grep -n "Warning: Parameter" ~/so_arm101_check/camera_<日付>_C1/C1-1_launch.log
grep -n "Built with\|Running with" ~/so_arm101_check/camera_<日付>_C1/C1-1_launch.log
grep -n "TF は出さない" ~/so_arm101_check/camera_<日付>_C1/C1-1_launch.log
grep -n -i "error\|fatal\|failed\|No RealSense" ~/so_arm101_check/camera_<日付>_C1/C1-1_launch.log
```

- **rs_launch の警告**（未確認の 1 点目）: 警告の文言は `Warning: Parameter '<名前>' is not supported.`（`rs_launch.py:126`）。`camera.launch.py:54-60` の `GroupAction(forwarding=False)` で、`use_apriltag` と `extrinsics_file` が rs_launch に渡らず、警告が出ない見込み（起動しない評価では未確認）。
  - **合格: 警告の行が 0 件。**
  - 出たら（`use_apriltag`・`extrinsics_file` など）: 失敗ではない。警告だけで動作は変わらない。**どの名前が出たかを報告**（直すかは別に決める）。
  - `Parameter '…' in config file is not supported`（`rs_launch.py:132`）が出たら、`config/realsense.yaml` のキーの綴りの間違い。報告。
- **librealsense2 の版**（未確認の 2 点目）: ドライバが 1 行に出す `Built with LibRealSense v<版> Running with LibRealSense v<版>`（文言は `librealsense2_camera.so` の文字列。`/opt/ros/humble/lib/librealsense2_camera.so`）。入っているのは apt の `ros-humble-librealsense2 2.58.4` と、Intel の `librealsense2 2.56.5`（`dpkg -l` の表示。計画 4 章）。
  - **合格: `Built with` と `Running with` の版が同じで、2.58.4。**
  - 版が違う（`Running with` が 2.56.5 など）なら、混在。**中止はしない。ログ（`C1-1_launch.log`）を保存して報告する**（直さない）。
  - 読み込まれたライブラリの確認（実行中のドライバの PID を `ps` で読む。`kill` はしない）:

    ```bash
    ps aux | grep realsense2_camera_node | grep -v grep
    awk '{print $6}' /proc/<PID>/maps | grep -i realsense | sort -u
    ```

    表示されたパスに `/opt/ros/humble/lib/…librealsense2…` と `/usr/lib/x86_64-linux-gnu/librealsense2…` の両方が出るなら混在。パスを報告。
- **extrinsics が無い旨の 1 行**: `…/camera_extrinsics.yaml が無いので base_link から camera_link の TF は出さない (no extrinsics file: …)`（`camera_config.py:209-212`。`camera.launch.py:67-68` の `LogInfo`）。**合格: 1 行ある。**
- エラー・`No RealSense`（`librealsense2_camera.so` の文字列）が無い。
- **USB の種類（報告する項目。合格・不合格にはしない）**: ドライバは起動時に `Device USB type: <値>` の行を出す見込み（`Device USB type: ` は `librealsense2_camera.so` の文字列で確かめた。`<値>` の形、たとえば `3.2` は未確認なので**見込み**）。次で読み、行をそのまま報告する（USB 3 か USB 2 か）:

  ```bash
  grep -n "USB type" ~/so_arm101_check/camera_<日付>_C1/C1-1_launch.log
  ```

  行が無ければ「無かった」と報告する。USB 2 のときは、15 fps が出ない・画像の設定が通らない可能性があるので、C1-1c の結果と一緒に報告する。

### C1-1b. ノードとトピック

```bash
ros2 node list --no-daemon
ros2 topic list --no-daemon | tee ~/so_arm101_check/camera_<日付>_C1/C1-1_topics.txt
```

- 見込み: ノード `/camera/camera`（`rs_launch.py:23-24` の `camera_name` と `camera_namespace` の既定がどちらも `camera`）。`apriltag_node` は無い（`use_apriltag` が false）。
- 見込み: カラー画像のトピック `/camera/camera/color/image_raw` と `/camera/camera/color/camera_info`（`camera_config.py:43-44` の remap の元と同じ。旧の起動と同じ名前）。深度のトピック名（`depth/…`、`aligned_depth_to_color/…`）は、文字列だけでは確かめられなかった。**`topic list` の出力をそのまま報告**する。
- **合格: 上の 2 つのカラーのトピックがある。`/detections` は無い。**

### C1-1c. 設定（640x480、15 fps）

```bash
ros2 param get --no-daemon /camera/camera depth_module.depth_profile
ros2 param get --no-daemon /camera/camera rgb_camera.color_profile
ros2 param get --no-daemon /camera/camera align_depth.enable
timeout 5 ros2 topic echo --no-daemon --once /camera/camera/color/camera_info 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C1/C1-1_camera_info.txt; echo "exit=${PIPESTATUS[0]}"
timeout 15 ros2 topic hz /camera/camera/color/image_raw 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C1/C1-1_hz.txt; echo "exit=${PIPESTATUS[0]}"
```

（2 つのファイルは、書く前に `ls` で同名が無いことを確かめる。）

> **2026-11-16 の実機で直した。** 元は `timeout 15 ros2 topic hz --no-daemon /camera/camera/color/image_raw`。`ros2 topic hz` は `--no-daemon` を受け付けず、`unrecognized arguments`（exit=2）になった。`--no-daemon` を外した形に直した（上のコマンド）。

- 値の出どころ: `config/realsense.yaml:22-24`（`rs_launch.py:44`、`:37`、`:82` の引数）。
- **合格:**
  - `depth_module.depth_profile` が `640x480x15`、`rgb_camera.color_profile` が `640x480x15`、`align_depth.enable` が `True`。
  - `camera_info` の `width: 640`、`height: 480`。
  - `ros2 topic hz` の `average rate` が **約 15 Hz（14〜16）**（この範囲は推測）。
- **`camera_info` の `d`（ひずみの係数）がすべて 0 でなければ、報告する**（`distortion_model` と `d` の値をそのまま）。apriltag には `image_raw` をそのまま `image_rect` として渡しているため（`camera_config.py:43-44`）、`d` が 0 でないと、タグの位置にひずみの分の誤差が乗る。**合格・不合格にはしない**（D415 のカラーは 0 に近い見込み）。
- `ros2 param get` が値を返さない・形が違うときは、表示をそのまま報告。

### C1-1d. TF（ドライバの根が `camera_link`。`base_link` から `camera_link` の TF が出ないこと）

後から購読しても受け取れるように、`/tf_static` は `transient_local` で購読する。`ros2 topic echo --help`（Humble）で、`--qos-durability {system_default,transient_local,volatile,unknown}`、`--qos-reliability`、`--no-daemon`、`--once` があることを確かめた（`--qos-profile` の既定は `sensor_data`、`--qos-durability` と `--qos-reliability` の既定は「出版者に自動で合わせる」）。`PYTHONUNBUFFERED=1` は、`timeout` が止めたときに出力が失われないようにするため。

```bash
ls ~/so_arm101_check/camera_<日付>_C1/C1-1_tf_static_pairs.txt
ls ~/so_arm101_check/camera_<日付>_C1/C1-1_tf_static_raw.txt
timeout 10 ros2 topic echo --no-daemon --once --qos-durability transient_local --qos-reliability reliable /tf_static \
  > ~/so_arm101_check/camera_<日付>_C1/C1-1_tf_static_raw.txt 2>&1
grep frame_id ~/so_arm101_check/camera_<日付>_C1/C1-1_tf_static_raw.txt | paste - - | sort | uniq -c \
  | tee ~/so_arm101_check/camera_<日付>_C1/C1-1_tf_static_pairs.txt
ros2 topic info -v --no-daemon /tf_static
ros2 node list --no-daemon
ps aux | grep -E "static_transform_publisher" | grep -v -E "grep|gvfsd"
timeout 10 ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame 2>&1 | head -20
```

> **2026-11-16 の実機で直した。** 元は `PYTHONUNBUFFERED=1 timeout 5 ros2 topic echo --no-daemon --qos-durability transient_local --qos-reliability reliable /tf_static 2>&1 | grep frame_id | paste - - | sort | uniq -c`（`--once` なし、パイプで直接受ける形）。この形では `timeout` が止めたときに出力が失われ、表が空（`Terminated` だけ）になった。上のように `--once` を付け、いったんファイル（`C1-1_tf_static_raw.txt`）に受けてから `grep` する形で通った（表は 4 行）。

`paste - -` は、`frame_id`（親。`header` の中）と `child_frame_id`（子）の 2 行を 1 行にする。1 行が 1 本の TF（親、子の順）。

- 見込み: `camera_link` が親の行が複数あり、`camera_color_optical_frame` が子の行がある（`base_realsense_node.h:89` の `OPTICAL_FRAME_ID` は `<camera_name>_<ストリーム名>_optical_frame`）。
- **合格（2 つ。C1-1d の合格の基準はこれだけ）:**
  1. **`ros2 node list --no-daemon` と `ps` に、`static_transform_publisher`（`camera.launch.py:70-72` が出す TF のノード。ノード名は `base_to_camera_link_tf`）が無い。**
  2. **`/tf_static` に `child_frame_id: camera_link` が無い**（上の表で、`camera_link` が子として出る行が 0 件）。
- （C1 では腕を起動しないので、「`base_link` が出ない」は何もしなくても成り立つ。基準にしない。）

### C1-1e. ドライバの根のフレーム（C3 のために。合格の基準に入れる）

C3 で `base_link` から `camera_link` の TF を出すので、**ドライバの根が `camera_link` であること**が前提になる。

- **合格: `camera_color_optical_frame` から親をたどると、根が `camera_link` になる**（C1-1d の表で `camera_link` → … → `camera_color_optical_frame` と続く）。または、上の `tf2_echo camera_link camera_color_optical_frame` が `Translation` と `Rotation` を出す（`Could not transform` や `frame does not exist` が出続けない）。
- **違う名前だったら（`link` など）、中止はしない。** 表（`C1-1_tf_static_pairs.txt`）をそのまま報告する。根の名前の根拠: `rs_launch.py:94` の `base_frame_id` の既定は `link` で、`camera_link` になる経路（`camera_name` を前に付ける）はバイナリの文字列だけでは確かめられていない。`camera_link` は旧の確認（計画 6 章。旧の `tf2_echo`）による。
- `ros2 topic info -v` の publisher は、ドライバ（`camera`）だけの見込み（腕を起動していないので、`robot_state_publisher` などは無い）。

### C1-1 の止め方

launch の端末で Ctrl-C を 1 回。`exit=` の値を記録する。そのあと、**この次の段の前に、下の「後始末」を行う。**

## C1-2. `use_apriltag:=true` で ID 0・1 を検出

タグ（ID 0 と ID 1）を印刷した板を、手で持つ（またはカメラの前に固定する）。**腕は無い。**

```bash
ls ~/so_arm101_check/camera_<日付>_C1/C1-2_launch.log
ros2 launch so_arm101_camera camera.launch.py use_apriltag:=true 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C1/C1-2_launch.log; echo "exit=${PIPESTATUS[0]}"
```

### C1-2a. 起動の確認

別の端末で（10 秒ほど待ってから）:

```bash
grep -n "Warning: Parameter" ~/so_arm101_check/camera_<日付>_C1/C1-2_launch.log
grep -n "Built with\|Running with" ~/so_arm101_check/camera_<日付>_C1/C1-2_launch.log
ros2 node list --no-daemon
ros2 topic list --no-daemon | grep -E "detections|tf"
```

- ノードに `/apriltag` が増える（`camera.launch.py:63-66` の `name='apriltag'`）。トピックに `/detections` がある（`libAprilTagNode.so` の文字列にある `detections`。名前空間なしのノードの相対名）。
- C1-1a と同じ 2 点（警告 0 件、`Built with` と `Running with` が同じ版）。

### C1-2b. タグを見せて、検出を確かめる

タグを、**ID 0 と ID 1 の両方が画像に入る**ように持つ（カメラから約 0.6 m）。

```bash
ls ~/so_arm101_check/camera_<日付>_C1/C1-2_detections.txt
timeout 10 ros2 topic echo --no-daemon --once /detections 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C1/C1-2_detections.txt; echo "exit=${PIPESTATUS[0]}"
```

- 見込み: `AprilTagDetectionArray` に、`family: 36h11` で `id: 0` と `id: 1` の 2 件。`decision_margin` と `hamming`（`max_hamming: 0` なので `hamming: 0`。`config/apriltag.yaml:8`）を見る。
- **合格: `id: 0` と `id: 1` が検出される。`hamming` が 0。**
- 検出が出ない（`--once` が時間切れで `exit=124`）なら、タグを近づける・照明を変えて再度試す。それでも出なければ、中止して報告（`image_rect` の remap が合っていない可能性。`camera_config.py:43-44`）。

### C1-2c. タグの TF

```bash
timeout 12 ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11_0 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C1/C1-2_tf_tag0.txt
timeout 12 ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11_1 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C1/C1-2_tf_tag1.txt
```

（書く前に `ls` で同名が無いことを確かめる。`tf2_echo` は止まらないので、`timeout 12` で切る。）

> **2026-11-16 の実機で直した。** 元は `tf2_echo ... | head -40 | tee ...`（`head -40` で切る、または Ctrl-C）。実機では `timeout 12` を付けた形で実行した。`head` で切ると exit=120 になった。

- **巻き尺は光軸に沿って当てる。タグを画像の中央に、正面向き（光軸に垂直）に置くと、z と巻き尺を直接比べられる**（C1-2d の置き方）。2026-11-16 の最初の 2 枚（タグが約 38° 傾いていた）は、巻き尺の当て方（光軸に沿ってか、斜めにまっすぐか）が区別できず、z との直接の比較にならなかった（ID 0 は巻き尺 710 mm に対し z 0.702、ID 1 は 750 mm に対し 0.745。ユーザーの報告）。

- フレーム名は `config/apriltag.yaml:17`（`tag36h11_0`、`tag36h11_1`）。親は、画像の `header` の `frame_id`（ドライバのカラーの光学フレーム `camera_color_optical_frame` の見込み。**推測**。旧の読み方は `lookup_transform(camera_color_optical_frame, tag36h11_0)`。計画 6 章）。
- **合格: 2 つのタグの TF が出る**（`Translation` が出る。`Could not transform` や `frame does not exist` が出続けない）。
- **合格: `Translation` の z（光学フレームの前方の距離）が、巻き尺の距離と 2 cm 以内で合う**（**2 cm は推測の値**。根拠のある値ではない）。巻き尺は、**カメラの前面から、タグの面まで**当てる。光学フレームの原点は、前面より数 mm 奥にある見込み（**[推測]**。D415 の資料は確かめていない）なので、z は巻き尺の値より数 mm 大きく出る見込み。測った距離と z の値を併せて報告する。**（2026-11-16 の実機で直した。実測は z が巻き尺より 5〜8 mm 小さく、この見込みは検証できなかった。上の注を参照。）**

### C1-2d. 距離を変えて見る（0.6、0.8、1.0 m）

タグ（ID 0）を、カメラから約 0.6、0.8、1.0 m に静止して持ち、それぞれ次を実行する（距離は巻き尺で測り、ファイル名に入れる）:

```bash
ls ~/so_arm101_check/camera_<日付>_C1/C1-2_tag0_<距離>m.txt
timeout 12 ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11_0 2>&1 \
  | tee ~/so_arm101_check/camera_<日付>_C1/C1-2_tag0_<距離>m.txt; echo "exit=${PIPESTATUS[0]}"
```

- 見込み: 10 回ほどの出力が溜まる。**合格の基準は決めない**（計画 10 章 C1 の 3 項目は「距離と std を見る」）。ログの置き場を報告し、`Translation` の z の平均と標準偏差は Claude が後で読む。
- **一辺の実測（C1-0）が 0.06 m と違うとき**: 距離の読みが一辺の比だけずれる（ずれの大きさは、実測した一辺を 0.06 で割った値に比例する見込み）。その旨を併せて報告。**yaml は変えない。**
- 検出が途切れる距離（例: 1.0 m で出ない）があれば、その距離を報告。

### C1-2 の止め方

launch の端末で Ctrl-C を 1 回。`exit=` の値を記録する。そのあと「後始末」。

## C1-3. extrinsics の扱い（`base_link` から `camera_link` の TF）

C1-1d で、ファイルが無い既定のパスのとき TF が出ないことは確かめた（`camera_config.py:201-204`）。ここでは、**既定以外の存在しないパスを明示すると、どのノードも起動する前にエラーで止まる**ことを確かめる（`camera_config.py:201-203`、`camera.launch.py:47-48`）。**カメラは開かれない見込み**（`_start` は、検査のあとに rs_launch を include する）。

```bash
ls ~/so_arm101_check/camera_<日付>_C1/C1-3_launch.log
ls /tmp/so_arm101_no_such_extrinsics.yaml
ros2 launch so_arm101_camera camera.launch.py extrinsics_file:=/tmp/so_arm101_no_such_extrinsics.yaml 2>&1 \
  | tee -i ~/so_arm101_check/camera_<日付>_C1/C1-3_launch.log; echo "exit=${PIPESTATUS[0]}"
```

（2 つめの `ls` は、`/tmp/so_arm101_no_such_extrinsics.yaml` が**無い**ことの確認。`No such file or directory` が出るのが期待どおり。あったら別の名前にする。）

- 見込みの表示: `ExtrinsicsError: extrinsics_file /tmp/so_arm101_no_such_extrinsics.yaml does not exist`（`camera_config.py:203`）を含む例外の表示（`Caught exception in launch` の形。**文言は未確認**）。**2026-11-16 の実機で直した。実機の表示は `[ERROR] [launch]: Caught exception in launch (see debug for traceback): extrinsics_file /tmp/so_arm101_no_such_extrinsics.yaml does not exist`、`exit=1`。ドライバは起動しなかった。**（元は上の「見込みの表示」で、`ExtrinsicsError:` の接頭辞が付く形を見込んでいた。実機の表示には `ExtrinsicsError:` の語は出ていない。）
- **合格:**
  - `exit=` が 0 以外。
  - 上の `does not exist` の文言が出る。
  - **`Launching as Normal ROS Node`（`rs_launch.py:154`）が出ない。`Built with` が出ない。** ドライバが起動していない。
  - 自然に終わる（Ctrl-C は要らない）。
- 終わったあとに、ノードが残っていない（`ros2 node list --no-daemon` が空）。

## 後始末（C1-1、C1-2、C1-3 のたびに）

launch の端末が止まったあと、**数秒待ってから**:

```bash
ps aux | grep -E "realsense2_camera_node|apriltag_node|static_transform_publisher|ros2_control_node|move_group|robot_state_pub|rviz2|spawner" | grep -v -E "grep|gvfsd"
ros2 node list --no-daemon
```

- **合格: どちらも空。**
- `ros2 node list --no-daemon` にだけ残り、`ps` が空なら、数秒待ってもう一度確認する。それでも残るなら、報告して指示を待つ。**ros2 daemon には触らない**（`stop`・`start` をしない）。
- `ps` にドライバが残るときは、`kill -9`・`pkill`・`killall` をしない。報告して指示を待つ（最後の手段: USB ケーブルを抜く）。
- 次の段に進む前に、`ls /dev/video*` でカメラが見えることを確かめる（ドライバが止まったあとにカメラが見えなくなったら報告）。

## 合格の基準の一覧

| 段 | 合格の基準 |
|---|---|
| 準備 | ps と node list が空。保存先ができた。カメラが見える |
| C1-0 | 引数が use_apriltag と extrinsics_file の 2 つだけ。ID 0 と ID 1 の一辺の実測値（縦・横の 4 つ）を報告（0.06 m と違っても yaml は変えない） |
| C1-1a | rs_launch の警告が 0 件。Built with と Running with が同じ版（2.58.4。違えば中止せずログを保存して報告）。extrinsics が無い旨の 1 行がある。エラーが無い。USB の種類の行を報告（合否なし） |
| C1-1b | ノード /camera/camera がある。apriltag と /detections は無い。カラーの 2 トピックがある |
| C1-1c | depth_profile と color_profile が 640x480x15、align_depth.enable が True。camera_info が幅 640、高さ 480。image_raw が約 15 Hz（14〜16）。d が全部 0 でなければ報告（合否なし） |
| C1-1d | node list と ps に static_transform_publisher が無い。/tf_static に child_frame_id が camera_link の行が 0 件 |
| C1-1e | camera_color_optical_frame の根が camera_link（違う名前なら中止せず表を報告） |
| C1-2 | /apriltag ノードと /detections がある。id 0 と id 1 が検出され hamming が 0。tag36h11_0 と tag36h11_1 の TF が出る。z が巻き尺（カメラの前面からタグの面）と 2 cm 以内（推測の値）。0.6、0.8、1.0 m のログを保存（基準なし） |
| C1-3 | exit が 0 以外。does not exist の文言が出る。ドライバが起動しない。自然に終わる |
| 後始末 | 各段の後で ps が空、node list が空 |

## 中止の基準（どの段でも）

次のときは止めて（Ctrl-C）、報告する。再試行しない。

- ドライバのログに、エラー・`FATAL`・USB の切断・`No RealSense devices were found`（`librealsense2_camera.so` の文字列）が出る。
- C1-0 の `--show-args` に、2 つ以外の引数が出る。
- C1-1d で `static_transform_publisher` が見つかる、または `child_frame_id: camera_link` が出る（extrinsics が無いはずなのに TF が出ている。TF の二重の発行や、既存のファイルの取り違え）。`/tf_static` を、`ps` の `static_transform_publisher` と一緒に報告。
- C1-3 で、ドライバ（`Launching as Normal ROS Node`）が起動してしまう（起動前の検査が効いていない）。
- 後始末で、`ps` のプロセスが残り、数秒待っても消えない。
- Ctrl-C を 10 秒待っても launch が止まらない。

## 報告してほしいもの

- C1-0 のノギスの 4 つの値。
- ログ 3 本（`C1-1_launch.log`、`C1-2_launch.log`、`C1-3_launch.log`）と、`C1-1_*.txt`、`C1-2_*.txt` の置き場所と `exit=` の値。
- 次の 5 点の結果: rs_launch の警告の有無（名前）、`Built with` と `Running with` の版、トピック名の一覧（深度の名前）、camera_info の `d`、`tf2_echo` の親のフレーム名（`camera_color_optical_frame` のとおりか）。
