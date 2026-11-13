   # ros_so_arm101 プロジェクトルール
   ## 目的
   SO-ARM101をROS 2 Humble + MoveIt2で制御する。LeRobotに依存しない。
   ## 参照元（読み取り専用・絶対に変更しない）
   - ~/ros2_so101/src （旧ワークスペース。GitHub由来の寄せ集め）
   - ~/ros2_so101/test （自作の解析IK・Diff-IK・キャリブ用スクリプト）
   ## ルール
   - 日本語で報告する
   - 不要なものの削除は git rm で行う（archive/ には移さない）。削除のときは、戻すためのタグかコミットを docs/inventory.md に書く（例: 「削除の直前のコミット <hash> から取り出せる」）。承認なしに削除しない（削除の一覧と理由を示し、承認を得てから実行する）
   - git 管理外のファイル（.gitignore の対象など）は git rm の対象外で、タグからも戻せない。削除や書き換えの前に、元の内容を docs/inventory.md に控える
   - build/ と install/ を消すのはクリーンビルドのときだけで、直前に報告する。log/ は消さない
   - 作業は段階ごとに計画を出し、承認を得てから実行する
   - 1段階ごとに colcon build が通ることを確認し、git commit する
   - 実機（サーボ）を動かすコマンドは実行しない。必要なら私に依頼する
   - 起動したものを止めた後は、ps と `ros2 node list --no-daemon` の両方で残りがないことを確認する。`--no-daemon` だけに残り、ps が空なら、数秒待ってもう一度確認し、それでも残るなら報告して指示を待つ。ros2 daemon には触らない（stop・start をしない）。`--no-daemon` が使えるコマンドには必ず付け、daemon を自動で起動させない
   - MoveIt本体の設定（URDF・関節リミット・planning_frame・IK solver）は勝手に変えない
   - pkill -f や killall など、名前・パターンでプロセスを止めない（ユーザーが実機の launch を動かしていることがあり、止めるとトルクが切れて腕が落ちる）。止めるのは、自分で起動したプロセスを、控えた PID で kill -INT するだけ
   - コミットの前に、関係するパッケージの colcon test を実行し、失敗があればコミットの前に報告する
   - 作業ツリーやインデックスや履歴を変える git の操作（stash・checkout・reset・clean・restore・amend・rebase など）は、push の前でも事前に承認を得る。承認されたコミットのための git add・git commit と、承認された削除の git rm は除く。確認のためのコマンド（status・diff・log・ps・test の結果表示など）に、これらの操作をつなげない（確認は確認だけで 1 つのコマンドにする）
   - launch（モックなど）は「前景」で起動する。前景とは、`&` を付けず、Bash の run_in_background で `exec ros2 launch ...` として動かすこと（`&` の子は、非対話の bash が SIGINT を無視する設定にするので、`kill -INT` で止まらない）。PID は exec の前に自分で控え、起動の直後に /proc/<PID>/status の SigIgn に SIGINT（0x2）が立っていないことを確認する。止めるのは、控えた PID への `kill -INT` だけ。モックは ROS_DOMAIN_ID=87 と RMW_IMPLEMENTATION=rmw_cyclonedds_cpp で動かす
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