   # ros_so_arm101 プロジェクトルール
   ## 目的
   SO-ARM101をROS 2 Humble + MoveIt2で制御する。LeRobotに依存しない。
   ## 参照元（読み取り専用・絶対に変更しない）
   - ~/ros2_so101/src （旧ワークスペース。GitHub由来の寄せ集め）
   - ~/ros2_so101/test （自作の解析IK・Diff-IK・キャリブ用スクリプト）
   ## ルール
   - 日本語で報告する
   - ファイルの削除はしない。不要なものは archive/ へ移動し、理由を記録する
   - 作業は段階ごとに計画を出し、承認を得てから実行する
   - 1段階ごとに colcon build が通ることを確認し、git commit する
   - 実機（サーボ）を動かすコマンドは実行しない。必要なら私に依頼する
   - 起動したものを止めた後は、ps と `ros2 node list` の両方で残りがないことを確認する。`ros2 node list` だけに残り、ps が空なら、`ros2 daemon stop; ros2 daemon start` の後にもう一度確認する
   - MoveIt本体の設定（URDF・関節リミット・planning_frame・IK solver）は勝手に変えない
   ## 最終構成（目標）
   - so_arm101_description / so_arm101_moveit_config
   - so_arm101_hardware : ros2_control SystemInterface（Feetechドライバ、use_mock_hardwareで切替）
   - so_arm101_calibration : サーボのキャリブ（脱力→中央→MIN/MAX）＋AprilTagのEye-to-Hand
   - so_arm101_ik : 解析IK + Diff-IK
   - so_arm101_bridge : rosbridge経由で座標・姿勢コマンドを受けるインターフェース
   - so_arm101_bringup : mock:=true/false で切り替えるランチャー一式
   ## 将来の拡張（今は実装しないが、設計で妨げない）
    - リーダーアーム: so_arm101_hardware を名前空間（/leader）で複数起動できるようにする。トルクOFF・読み取り専用モードを持たせる。キャリブはフォロワーと同じ仕組みを使う
    - 模倣学習: 関節角・画像は rosbag2 で記録できればよい