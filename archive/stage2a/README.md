# archive/stage2a

段階2aで bringup に統合したため使わなくなったファイル。削除はせずここに移動した（元のパスは下記）。

| ファイル | 元の場所 (src/ 以下) | 理由 |
|---|---|---|
| bringup/launch/so101_robot.launch.py | so_arm101_bringup/launch/ | `bringup.launch.py` (mock:=false) に置換 |
| bringup/launch/include/follower.launch.py | so_arm101_bringup/launch/include/ | `/follower` 名前空間と TF プレフィックスを廃止。bringup に統合 |
| bringup/rviz/robot_display*.rviz | so_arm101_bringup/rviz/ | `/follower/robot_description` と TF prefix `follower` 前提。MoveIt の RViz を使う |
| controller/launch/*.launch.py | so_arm101_controller/launch/ | `/follower` namespace の cm と spawner。bringup の単一 ros2_control_node に統合 |
| controller/config/so101_controllers.yaml | so_arm101_controller/config/ | 統合後の `controllers.yaml`（ルート名前空間）の元。比較用に原本を保存 |
| description/launch/{rsp,display,display_robot}.launch.py | so_arm101_description/launch/ | `follower` namespace / frame_prefix 前提。bringup に統合 |
| description/rviz/display.rviz | so_arm101_description/rviz/ | `follower` TF prefix 前提 |
| moveit_config/launch/demo.launch.py | so_arm101_moveit_config/launch/ | 起動入口を `so_arm101_bringup/launch/bringup.launch.py` に一本化 |
