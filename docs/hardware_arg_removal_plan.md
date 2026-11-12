# launch の `hardware` 引数の削除の計画（2026-11-12。**計画のみ。コード・xacro・launch・手順書は変更していない**。実機・ポートには触れていない）

> **更新（2026-11-12、ユーザーの指示）: 採用案は 7 章の D 案**（`hardware` の宣言・mappings・xacro の `hardware` を削除し、`bringup.launch.py` に「宣言していない引数は起動前にエラー」の検査を入れる。`hardware` に限らない）。1〜6 章は、A・B・C 案を比べたときの検討記録（事実の確認は有効。案 B の推奨は D 案に置き換わった）。

表記: **[事実]** = ソース・ファイル・ログ・コマンドで確認したこと。**[推測]** = 確認していないこと。

## 1. 現状 [事実]

`hardware` 引数は、`feetech` だけが使える状態（LeRobot 経路は L1〜L2 で削除済み）。

| 場所 | 内容 |
|---|---|
| `so_arm101_bringup/launch/bringup.launch.py` | docstring（:5-12）、`_require_feetech`（:33-42。`hardware != "feetech"` なら `RuntimeError`。`mock:=true` でも検査）、`LaunchConfiguration("hardware")`（:48）、`robot_description` の mappings に `"hardware": hardware`（:64）、`DeclareLaunchArgument("hardware", …)`（:89 付近）、検査の呼び出し（:150 付近）、`mock`・`read_only`・`calibration_file` の description 内の「hardware:=feetech」（:84・:102・:108） |
| `so_arm101_moveit_config/config/so101_new_calib.urdf.xacro` | `<xacro:arg name="hardware" default="feetech"/>`（:7）、マクロへ `hardware="$(arg hardware)"`（:28）、コメント（`feetech のみ`） |
| `so_arm101_moveit_config/config/so101_new_calib.ros2_control.xacro` | マクロ引数 `hardware:=feetech`（:3）、`xacro.fatal`（:4-6。feetech 以外で xacro の展開が rc=2 で失敗）、`xacro:if value="${hardware == 'feetech'}"`（:16）。`use_fake_hardware`（ros2_control の慣例の名前）は**別物で残す** |
| 他の launch（`rsp`・`move_group` など） | `hardware` を使わない |
| `tools/pose_margin.py` | `hardware` を渡さない（`use_fake_hardware:=true` のみ） |
| 試験 | `hardware` 引数を試験するものは無い（bringup は lint のみ） |

## 2. 古いコマンドで `hardware:=…` を付けたときの挙動（引数を削除した後）

### 2.1 `ros2 launch`（ソースで確認）[事実]
- `ros2launch`（0.19.14）`api/api.py:167-175` は、起動する launch ファイルを `IncludeLaunchDescription(launch_arguments=parse_launch_arguments(…))` で include する。`name:=value` の形でなければ別のエラーになるが、形が合っていれば名前は何でもよい。
- `launch`（1.0.15）`actions/include_launch_description.py:48-52` のドキュメントと `:179-184` の実装: **宣言されていない引数は、エラーにも警告にもならず、そのまま Launch Configuration として設定される**（`SetLaunchConfiguration`）。エラーになるのは、その逆（宣言された引数に値が無く、既定も無い）だけ。
- したがって、引数 `hardware` の宣言を消すと、**`hardware:=feetech` も `hardware:=lerobot` も `hardware:=なんでも` も、無言で受け付けられ、効果は無い**。
- **危険な点**: 今は `hardware:=lerobot`（古いコマンド）は、起動前に分かりやすいエラーで止まる（`inventory.md` 1703 行で確認済み）。引数を単に消すと、古い `hardware:=lerobot …` のコマンドが**エラー無しで、feetech のドライバで起動する**。`read_only:=false` が付いた古いコマンドなら、**書き込みモード（トルク ON）で起動する**。LeRobot のつもりで実行して、別のドライバがトルクを入れるのは避けたい。

### 2.2 `xacro`（実際に実行して確認）[事実]
- 宣言していない引数（例 `foo:=bar`）を `xacro` に渡しても、**rc=0・出力に警告なし**（無視される）。
- 今（引数が宣言されている状態）の `hardware:=lerobot` は、rc=2 で失敗する（`error: hardware:=lerobot is not supported …`）。引数を消すと、これも無言で通る。
- xacro を手で直接実行するのは、`tools/pose_margin.py` と、確認用のスクリプト（`~/so_arm101_check/gen_rd6.sh`。`hardware:=feetech` を渡している。git 管理外）だけ。

### 2.3 ドキュメント側 [事実]
- 手順書の起動コマンド（`ros2 launch … hardware:=feetech …`）が、`feetech_write_check.md`（:40・59・121・137・145・160・195・237）、`feetech_servo_config_check.md`（:36・49）、`feetech_readonly_check.md`（:81）にある。ユーザーはこれをコピーして実行するので、**引数を消しても、付いたままのコマンドは動く（無視される）**。ユーザーの bash の履歴にも `hardware:=feetech` 付きの起動が残っている。

## 3. 案

| 案 | 内容 | 長所 | 短所 |
|---|---|---|---|
| A | `hardware` を launch と xacro から**単に削除**。古い引数は無言で無視 | 最小。コードが一番単純 | **古い `hardware:=lerobot` が無言で feetech として起動**（2.1 の危険）。`read_only:=false` 付きなら書き込みモード |
| **B（推奨）** | launch の `hardware` の**宣言と mappings だけ削除**し、**`OpaqueFunction` の検査を、`context.launch_configurations` を見る形に変えて残す**（宣言していなくても、渡された引数は `launch_configurations` に入る。2.1）。`hardware` が**渡されていなければ何もしない**／`feetech` なら**非推奨の警告を 1 行出して続行**／それ以外は**今と同じエラーで起動前に止める**。xacro は `hardware` の引数・マクロ引数・`xacro.fatal`・`if` を削除 | 古い `hardware:=lerobot` が、今と同じく**エラーで止まる**（危険を避ける）。古い `hardware:=feetech` は警告つきで動く。launch の外から見える引数の一覧（`--show-args`）から `hardware` が消える | 検査の関数は残る（最初の数か月の移行用。後で消してよい）。xacro を手で直接実行した場合の `hardware:=lerobot` は、無言で通る（実際の影響は小さい。xacro は launch の中でしか使わない） |
| C | 今のまま（宣言を残す） | 変更なし | 「値が 1 つだけの引数」が残る。説明が冗長 |

**推奨は B。** 理由: 2.1 の危険（LeRobot のつもりのコマンドが、エラー無しで、トルクを入れるドライバとして起動する）を、小さな検査 1 つで避けられる。警告つきで受けるのは `feetech` だけ（既存の手順書・履歴のコマンドは動く）。

### 3.1 B の実装の骨子（実装は承認後）
1. `bringup.launch.py`: `DeclareLaunchArgument("hardware", …)` と `hardware = LaunchConfiguration("hardware")`、mappings の `"hardware"` を削除。`_require_feetech` を `_check_legacy_hardware_arg` に改め、`context.launch_configurations.get("hardware")` を見る（`None` なら何もしない。`"feetech"` なら `LogInfo` か `warnings` で「`hardware` 引数は削除した。無視する」と 1 行。それ以外は今と同じ `RuntimeError`（メッセージは今のまま。タグ `stage2c-done` を案内））。docstring・description の「hardware:=feetech」の文言を直す（`mock:=false` = 実機、など）。
2. xacro 2 つ: `hardware` の引数・マクロ引数・`xacro.fatal`・`xacro:if hardware == 'feetech'` を削除（`use_fake_hardware` は残す）。`<plugin>so_arm101_hardware/Sts3215System</plugin>` は `use_fake_hardware=false` のとき常に出す。
3. 試験: 検査の関数を `pytest` で試す（`LaunchContext` に `launch_configurations["hardware"]` を入れて、`None`・`"feetech"`・`"lerobot"` の 3 通り）。bringup は今は lint のみ（16 件・失敗 12。以前から）。`pytest` を足すと件数が増えるので、前後の件数を記録する（失敗 12 は変えない）。
4. `hardware` を使わない `tools/pose_margin.py`・他の launch は変更なし。

### 3.2 確認（実装の後）
- **オフライン**（ROS のノードは起動しない）: ① `gen_rd6.sh` の 6 通りを、`hardware:=feetech` 付きと**無しの両方**で生成し、**現在の保存物（`~/so_arm101_check/urdf_after/`）と sha256 が同一**であること（引数の削除だけなので出力は変わらないはず）。② `colcon build`、`colcon test`（件数の前後。bringup・moveit_config・description・hardware・calibration）。
- **launch の挙動**（起動前に終わるので、ROS のノードは起動しない。SIGINT を既定に戻すラッパーを使い、PID を控える）: `mock:=true hardware:=lerobot` → 今と同じエラーで exit code 1（ノードは起動しない）。`mock:=true hardware:=feetech` → 警告 1 行で通常どおり起動（モックの確認と同じ: コントローラ 3 つ active）→ `kill -INT`。`hardware` 無し → 警告なし。`ros2 launch … --show-args` に `hardware` が出ないこと。
- **実機**（ユーザー）: `robot_description` が同一なので、実機の確認は**読み取り専用の起動 1 回**（`hardware` 無しの起動。`[diag]` 6/6 と `/joint_states`）で足りる案。W0 以降は不要（ドライバ・xacro の出力が同じ）。
- コミット: コード（launch・xacro・試験）は 1 コミット。手順書の更新（4 章）は別のコミット。

## 4. 手順書・記録の `hardware:=feetech` の扱い

方針: **実行する手順書は直す。過去の記録は書き換えない**（これまでの docs の「元の記録は書き換えていない。追記する」の運用に合わせる）。

| 区分 | 場所（[事実] grep の件数。`hardware:=` は全 docs で約 70 か所） | 扱い |
|---|---|---|
| **A. 実行する手順書の起動コマンド**（ユーザーがコピーして使う） | `feetech_write_check.md` :40・59・121・137・145・160・195・237（8 コマンド）、`feetech_servo_config_check.md` :36・49、`feetech_readonly_check.md` :81 | **`hardware:=feetech` のトークンを削除**（他の引数は変えない）。実装のコミットの**後**の、手順書だけのコミットで行う（コードと手順書の食い違いを避けるため） |
| **B. 手順書の説明文** | `feetech_write_check.md` :3・:6、`feetech_servo_config_check.md` :3、`feetech_readonly_check.md` :1・:6・:97・:194（見出しや文中の「hardware:=feetech」） | 「`hardware:=feetech`」を「書き込みモード（`read_only:=false`）」「読み取り専用の起動」などに言い換え（意味は変えない）。見出しの変更は目次の参照に注意 |
| **C. `feetech_write_check.md` 6 行目（古い LeRobot の記述）** | 現在の文言（:6）: 「起動の既定（段階2b-4 以降）: `hardware` は `feetech`、`read_only` は `true`。書き込みモード（トルク ON）は `read_only:=false` を明示したときだけ。**LeRobot 経路は `hardware:=lerobot` を明示したときだけ。**この手順書の起動コマンドは両方を明示してあるので、既定の変更で意味は変わらない。」 | **事実の誤り**（LeRobot 経路は削除済み。`hardware:=lerobot` はエラー。起動コマンドが「両方を明示」というのも、:40・:145 は `read_only` を書いていない（読み取り専用の既定）ので正確でない）。**修正案**: 「起動の既定: `read_only` は `true`。**書き込みモード（トルク ON）は `read_only:=false` を明示したときだけ**。この手順書の書き込みモードの起動コマンドは `read_only:=false` を明示してあり、`read_only` を書いていないコマンド（W0 の前後の読み取り専用の確認など）は読み取り専用で起動する。LeRobot 経路と `hardware` 引数は削除した（タグ `stage2c-done` に残る。`hardware:=…` を付けた古いコマンドは、`feetech` なら警告つきで動き、それ以外は起動前にエラーで止まる）。」あわせて :3 の「対象: `hardware:=feetech read_only:=false`」→「対象: 書き込みモード（`read_only:=false`）」 |
| **D. すでに「歴史的」と明記された手順** | `feetech_readonly_check.md`（LeRobot 側の手順 0.5・1・4・2.5・3.5）、`feetech_servo_config_check.md` :24・:31（`hardware:=lerobot` の起動） | **変更しない**（再現できないと冒頭に書いてある）。`hardware:=lerobot` のコマンドは**そのまま**（歴史的な記録。コピーしても今は起動前にエラーで止まる） |
| **E. 過去の記録** | `inventory.md`（約 29 か所）、`lerobot_removal_plan.md`（11）、`stage2c_plan.md`（1）、`original_urdf_plan.md`（5） | **書き換えない**。`inventory.md` に「`hardware` 引数の削除」の節を**追記**し、「以前の記録の `hardware:=feetech` は、今は付けても無視される（警告）／`lerobot` はエラー」と読み替えを書く |
| **F. コード内の文言** | `bringup.launch.py` の docstring・description、xacro のコメント | 実装のコミットで直す（3.1 の 1・2） |
| **G. docs の外** | `~/so_arm101_check/gen_rd6.sh`（git 管理外）、ユーザーの bash 履歴 | `gen_rd6.sh` は、実装の後に、`hardware:=feetech` 付き・無しの両方で生成するよう直す（git 管理外。直す前の内容は `docs/inventory.md` に控える規則。短いので全文を控える）。bash 履歴は触らない |

## 5. 段階（承認後）

| 段階 | 内容 | 確認 |
|---|---|---|
| H1 | 計画の承認と、案（B）の確認 | — |
| H2 | 基準の取得: `colcon test` の件数、`urdf_after/` の sha256 を基準として使う（すでにある）、`gen_rd6.sh` の全文を `inventory.md` に控える | — |
| H3 | コード（launch・xacro）と試験の追加。B の挙動の確認（3.2）。**コミット 1** | `colcon build`・`colcon test`（件数の前後）、6 通りの sha256 が同一、launch の挙動 3 通り（ROS のノードの起動は 1 通りだけ。`kill -INT` で停止、`ps` と `ros2 node list --no-daemon` で確認） |
| H4 | 手順書（A・B・C）の更新と、`inventory.md` の追記（E）。**コミット 2**（docs のみ） | 変更した行の一覧を示す |
| H5 | 実機の確認（ユーザー。読み取り専用の起動 1 回） | 結果を `inventory.md` に記録 |

## 6. ユーザーに決めてほしいこと
1. 案 A / **B** / C のどれか（推奨は B）。
2. B の警告の出し方（`LogInfo` の 1 行か、`warnings`）と、移行期間（いつ検査を消すか。例: 次の大きな変更で、または半年後）。
3. 手順書の更新は、実装と別のコミットでよいか（推奨）。
4. 実機の確認は読み取り専用の起動 1 回でよいか。


---

## 7. D 案（採用案。2026-11-12 のユーザーの指示）

### 7.1 内容
1. `bringup.launch.py` の `hardware` の宣言と mappings、`so101_new_calib.urdf.xacro` と `so101_new_calib.ros2_control.xacro` の `hardware`（引数・マクロ引数・`xacro.fatal`・`xacro:if hardware == 'feetech'`）を削除（3.1 の 2 と同じ。`use_fake_hardware` は残す）。`_require_feetech` は削除。
2. `bringup.launch.py` に**「宣言していない引数が渡されたら、起動前にエラーで止める」検査**を入れる（`hardware` に限らない）。宣言・検査・ノードの順を保つ（検査はすべての宣言の後、最初のノードの前）。
3. エラーの文面（案。日本語と英語を 1 行ずつ）:
   - 「未宣言の引数: `foo`（もしかして: `food_x`?）。宣言されている引数: `mock`, `serial_port`, …（名前順の全一覧）」
   - 打ち間違いの候補は `difflib.get_close_matches`（既定のしきい値 0.6）で最大 2 つ。
   - `hardware` が含まれるときは追加で: 「`hardware` 引数は削除しました（LeRobot 経路の削除に伴う。タグ `stage2c-done`）。外して実行してください」。`hardware:=feetech` も同じ（値によらず）。
   - 例外は `RuntimeError`（今の `_require_feetech` と同じ。`ros2 launch` が `Caught exception in launch` で終了コード 1 にする。ノードは起動しない）。

### 7.2 「未宣言」を誤って判定しない（設計とその根拠。**[事実]** はソースの確認）
- **判定の対象は `context.argv`（利用者が打った引数）だけ**。`context.launch_configurations`（内部の設定が入る）は**見ない**。
  - **[事実]** `ros2launch/api/api.py:159-168`: `LaunchService(argv=launch_file_arguments)`。`launch_file_arguments` は、利用者が打った `name:=value` と、`--launch-prefix`・`--launch-prefix-filter` を指定したときの `launch-prefix:=…`・`launch-prefix-filter:=…`（`api.py:154-160`）だけ。
  - **[事実]** `launch/launch_service.py:77` と `launch_context.py:76-78`: `context.argv` はその `argv` をそのまま返す。
  - **[事実]** include した launch ファイルが足す設定（`IncludeLaunchDescription` の `SetLaunchConfiguration`、`DeclareLaunchArgument` の既定、`OpaqueFunction` の中で足される値）は **`launch_configurations` に入るだけで `argv` には入らない**（`include_launch_description.py:179-184`、`declare_launch_argument.py:205-214`）。したがって、これらを未宣言と誤判定しない。
  - 判定は `name:=value` の形の要素だけ（`:=` を含まないものは無視。`ros2 launch` が先に形式エラーにする: `api.py:127-138`）。
- **「宣言されている引数」は、そのファイルが宣言したものに、include した launch ファイルが宣言したもの（通り抜ける引数）を足したもの**:
  - **[事実]** 宣言が `launch_configurations` に既にある名前は、`DeclareLaunchArgument` が既定で上書きしない（`declare_launch_argument.py:205`）。つまり、`bringup.launch.py` に渡した `monitor_dynamics:=false` や `rviz_config:=…` は、include した `move_group.launch.py`・`moveit_rviz.launch.py` に**通り抜けて効く**（今も使える。**これを未宣言と誤判定すると、動いている使い方を壊す**）。
  - **[事実]** `LaunchDescription.get_launch_arguments()` は、include を辿った宣言を返す。実際に今の `bringup.launch.py` で実行したところ **22 個**（自分の宣言 `mock`・`hardware`・`serial_port`・`calibration_file`・`read_only`・`acceleration`・`torque_limit`・`apply_servo_config`・`probe_lock_sram`・`use_rviz`・`db` の 11 個と、include した launch の `debug`・`allow_trajectory_execution`・`publish_monitored_planning_scene`・`monitor_dynamics`・`capabilities`・`disable_capabilities`・`rviz_config`・`moveit_warehouse_database_path`・`reset`・`moveit_warehouse_port`・`moveit_warehouse_host` の 11 個）。`hardware` を消すと 21 個。条件付きで include される `moveit_rviz`・`warehouse_db` の引数も入っている。
  - 実装: 名前の一覧は、`generate_launch_description()` の最後（すべての宣言と include を足した後）に `ld.get_launch_arguments()` から作り、検査の関数（`OpaqueFunction(args=[…])`）に渡す。**手で書いた一覧は持たない**（include 先の引数が増減しても自動で追従する）。
  - 常に許す名前: `launch-prefix`、`launch-prefix-filter`（`ros2 launch` の option が足す。`api.py:154-160`）。
- **限界（[推測]・明記）**: ほかの launch ファイルが `bringup.launch.py` を `IncludeLaunchDescription(launch_arguments=…)` で include する場合、その引数は `argv` に入らないので、検査の対象外（誤判定はしない。検査が効かないだけ）。今のリポジトリに、そういう launch は無い（grep で確認）。

### 7.3 誤判定しないことの確認（ソースと実行）
1. **ソース**（上の 7.2。実装の後に、実際の行を `inventory.md` に記録）。
2. **単体試験（`pytest`。ノードは起動しない）**:
   - `bringup.launch.py` を `importlib` で読み込み、`generate_launch_description()` を呼ぶ（ノードは起動しない）。宣言の名前の集合 `D` を `ld.get_launch_arguments()` から得る。
   - **`D` の全要素**について、`LaunchContext(argv=[f"{name}:=x"])` で検査を呼び、**エラーにならない**こと（`D` が 21 個であること、`hardware` が含まれないこと、`monitor_dynamics`・`rviz_config` が含まれることも確認）。`launch-prefix:=x`・`launch-prefix-filter:=x` も通る。`argv=[]` も通る。
   - 未宣言 1 つ（`foo:=bar`）→ `RuntimeError`。文面に `foo`、宣言の一覧（`mock` を含む）がある。
   - `hardware:=feetech`・`hardware:=lerobot` → `RuntimeError`。文面に「`hardware` 引数は削除しました」「`stage2c-done`」がある。
   - 打ち間違い: `read_olny:=false` → 文面に `read_only`、`use_rvis:=false` → `use_rviz`。`Mock:=true`（大文字。名前は区別される）→ エラーで、候補に `mock`。
   - **内部の設定の誤判定がないこと**: 最小の `LaunchDescription`（`SetLaunchConfiguration("internal_x", "1")`、`DeclareLaunchArgument("inner_arg", default_value="v")` を持つ入れ子の `IncludeLaunchDescription`、`OpaqueFunction` が `context.launch_configurations["added_by_opaque"]` を足す、そのあとに検査）を `LaunchService(argv=[])` で実行して、**終了コード 0**。同じ記述を `argv=["foo:=bar"]` で実行して終了コード 1。ノードは起動しない。
3. **実行（`ros2 launch`。起動前に止まるものはノードが起動しない）**: 7.5 の表。
4. **`ros2 launch so_arm101_bringup bringup.launch.py --show-args`** の出力に `hardware` が無く、21 個あること（検査は走らない）。

### 7.4 対象の launch は `bringup.launch.py` だけでよいか
- **[事実]** 利用者が直接起動するのは `so_arm101_bringup/launch/bringup.launch.py` だけ。docs の `ros2 launch` のコマンド 18 件（`so_arm101_bringup bringup.launch.py` のみ。そのほかの 6+6+4+2+1 件は、削除済みの旧パッケージ（`so101_*`・`apriltag_test`）と `realsense2_camera`）。ユーザーの bash 履歴の最近の 8 件のうち `ros2 launch` は `so_arm101_bringup bringup.launch.py`（56 件）。
- `so_arm101_moveit_config/launch/*.launch.py`（8 個）は、`bringup.launch.py` が include するもの（`static_virtual_joint_tfs`・`spawn_controllers`・`move_group`・`moveit_rviz`・`warehouse_db`）か、手順書に出てこないもの（`rsp`・`servo`・`setup_assistant`）で、**`hardware` を使わない**。直接は起動しない。`so_arm101_description/launch/`・`so_arm101_controller/launch/` は空。
- **結論**: 検査は `bringup.launch.py` だけでよい。moveit_config の launch に直接 `hardware:=…` を付けた場合は無言で無視される（実際には起きていない。起きても `hardware` は元から無効だった）。
- **注意（影響）**: D 案では `hardware:=feetech` もエラーになる（値によらず）。手順書・履歴・コピーしたコマンドの `hardware:=feetech` は、**手順書の更新（7.6）をするまで動かない**。エラーの文面が「外して実行してください」と教える。手順書の更新は、実装のコミットの直後に行う（8 章）。

### 7.5 実装後の launch の挙動の確認（起動前に止まるか・起動するか。ノードの起動は 3 件だけ）
SIGINT を既定に戻すラッパー（`inventory.md` の `kill -TERM` の件の修正。`SigIgn` の確認つき）で起動し、PID を控え、`kill -INT` だけで止める。止めた後は `ps` と `ros2 node list --no-daemon`（daemon には触らない）。ログは `~/so_arm101_check/hwarg_*.log`。

| # | 引数 | 期待 | ノード |
|---|---|---|---|
| 1 | `mock:=true use_rviz:=false`（引数は宣言済みのみ。RViz を開かないために `use_rviz:=false`。引数なしそのままは RViz が開くだけの違い） | 検査を通って起動。コントローラ 3 つ active | **起動する**（`kill -INT`） |
| 2 | 通り抜ける引数: `mock:=true use_rviz:=false monitor_dynamics:=false` | **誤判定しない**（検査を通って起動） | **起動する**（`kill -INT`） |
| 3 | 未宣言 1 つ: `mock:=true use_rviz:=false foo:=bar` | 起動前に `RuntimeError`、終了コード 1。文面に `foo` と宣言の一覧 | 起動しない |
| 4 | `mock:=true use_rviz:=false hardware:=feetech` | 起動前にエラー。「`hardware` 引数は削除しました…外して実行してください」 | 起動しない |
| 5 | `mock:=true use_rviz:=false hardware:=lerobot` | 同上（今と同じく止まる。文面は新しいもの） | 起動しない |
| 6 | 打ち間違い: `mock:=true use_rviz:=false read_olny:=false` | エラー。「もしかして: `read_only`」 | 起動しない |
| 7 | 打ち間違い: `mock:=true use_rvis:=false` | エラー。「もしかして: `use_rviz`」（**RViz の既定が true のままにならない**ことの確認でもある） | 起動しない |
| 8 | 宣言済みの引数の値の誤り: `mock:=true use_rviz:=false read_only:=flase` | 今の挙動（`choices` のエラー）のまま。検査の対象外（宣言済み） | 起動しない |
| 9 | `--show-args` | `hardware` が一覧に無い | 起動しない |

- 3〜9 は、各ログに `process started` が**無い**こと、`ps` に ROS のプロセスが**無い**ことを確認する（起動前に止まったことの証拠）。
- `mock:=false`（実機のポートを開く）の試験は**しない**（実機の確認はユーザー）。

### 7.6 手順書の更新は実装と別のコミット（ユーザーの指示）
- 4 章の扱いのとおり（A: 実行コマンドの `hardware:=feetech` を削除、B: 説明文の言い換え、C: `feetech_write_check.md` 6 行目の修正、D: 「歴史的」な手順は変更しない、E: 過去の記録は書き換えず、`inventory.md` に読み替えを追記）。**11 コマンド**: `feetech_write_check.md`（:40・59・121・137・145・160・195・237）、`feetech_servo_config_check.md`（:36・49）、`feetech_readonly_check.md`（:81）。
- 読み替え（E の追記）: 「過去の記録の `hardware:=feetech` は、削除した（D 案）。今は**付けるとエラー**になる（`hardware` 引数は削除した）。`hardware:=lerobot` も同様」。
- 6 行目の修正案は 4 章 C のとおり（ただし D 案なので「付けた古いコマンドは警告つきで動く」ではなく「**付けると起動前にエラーで止まる（外して実行する）**」に直す）。

### 7.7 試験・lint の件数（`so_arm101_bringup`。実装のコミットの前に `colcon test` を実行して前後を比べる）
- **[事実] 今の状態（16 件・失敗 12）の内訳**（xunit の中身を読んだ）: `copyright` 1 件（`launch/bringup.launch.py`: copyright の表示なし）、`flake8` 4 件（`bringup.launch.py` の E501（99 文字超）が 100・121・181・182 行）、`pep257` 4 件（`bringup.launch.py` の D400・D415 が 2 か所ずつ: 1 行目のモジュールの docstring と、34 行目の `_require_feetech` の docstring。どちらも 1 行目が「。」で終わる）、`lint_cmake` 1 件・`xmllint` 1 件（成功）、CTest のまとめ 5 件（`copyright`・`flake8`・`pep257` の 3 つが失敗）。失敗 12 = 1 + 4 + 4 + 3。**失敗は `bringup.launch.py` 1 ファイルの既存の lint 違反**（違反 1 つ = 1 件）。
- **したがって `bringup.launch.py` を直すと、件数が動く**（違反の数がそのまま件数になるため）。予想:
  - `_require_feetech`（docstring が D400・D415）を削除 → **pep257 の失敗が 4 → 2**（−2 件）。新しい検査の関数の docstring は、1 行目を `.` で終える（違反を足さない）。
  - 新しい行は 99 文字以内（E501 を足さない）。既存の E501 の 4 行（`calibration_file` の行、`torque_limit` の description、`moveit_rviz`・`warehouse_db` の include）は、**直さない**（今回の範囲外。行番号は動く）。
  - モジュールの docstring（1 行目が「。」で終わる）は、文言を直しても D400・D415 の 2 件は残る（今回は直さない）。
  - 予想: lint のみで **16 件・失敗 12 → 14 件・失敗 10**（pep257 −2 件。増えるのは、後述の試験の分だけ）。
- **試験の追加**（承認が要る: `package.xml` の `<test_depend>ament_cmake_pytest</test_depend>`、`CMakeLists.txt` の `find_package(ament_cmake_pytest)` と `ament_add_pytest_test`、新規 `test/test_launch_arg_check.py`）: 7.3 の 2 の試験。件数は、`pytest` の関数の数（約 10。実装で確定）と、CTest のまとめ 1 件、新しいファイルの `copyright` の 1 件（MIT のヘッダを付けて成功させる。ユーザーの「自分のコードは MIT」の判断と同じ）が増える。**新しいファイルは flake8・pep257 にも通す**（違反を足さない）。
- **報告の形**: 前（16・12）→ 後を、種別ごと（`copyright`・`flake8`・`pep257`・`pytest`・CTest）の表にし、増減の理由（pep257 の −2 は削除した関数の docstring、+N は新しい試験）を書く。失敗が **増えた**場合は、コミットの前に報告する。
- ほかのパッケージ（`so_arm101_description`・`so_arm101_moveit_config`・`so_arm101_hardware`・`so_arm101_calibration`・`so_arm101_controller`）の `colcon test` も全体で実行し、件数の前後（今: 全体 196 件・失敗 12。description 2、moveit_config 0、controller 0、hardware 105、calibration 73）を報告する（moveit_config の xacro を変えるため、moveit_config と bringup は特に）。

### 7.8 robot_description の前後の確認
- `~/so_arm101_check/gen_rd6.sh` の 6 通りで、**現在の保存物 `~/so_arm101_check/urdf_after/` と sha256 が一致**すること（引数 `hardware` を消しても出力は変わらないはず）。確認は 2 通り: `hardware:=feetech` を**付けて**（xacro は宣言していない引数を無言で無視するため、結果が変わらないこと）と、**付けずに**。
- `gen_rd6.sh`（git 管理外）は、実装の後に、`hardware:=feetech` を外した版に直す。**直す前の全文を `docs/inventory.md` に控える**（CLAUDE.md の規則: git 管理外の書き換えの前に元の内容を控える）。
- `tools/pose_margin.py` の出力（URDF のリミット）が前と同じこと。

### 7.9 実機の確認（ユーザー）
- 読み取り専用の起動 1 回（`mock:=false`、`hardware` なし。`[diag]` 6/6・コントローラ 3 つ active・カウンタ 0）。`robot_description` が同一なので、W0 以降は不要。

## 8. 段階（D 案）

| 段階 | 内容 | 確認 |
|---|---|---|
| D1 | 計画の承認 | — |
| D2 | 基準の取得: `colcon test` の件数（特に bringup の種別ごとの内訳）、`gen_rd6.sh` の全文を `inventory.md` に控える | — |
| D3 | 実装: launch・xacro・試験（`package.xml`・`CMakeLists.txt` の変更を含む）。7.3 の単体試験、7.5 の launch の挙動、7.8 の sha256。**コミット 1** | `colcon build`・`colcon test`（7.7 の表）、6 通りの sha256 一致、launch の挙動 9 通り |
| D4 | 手順書の更新（7.6）と `inventory.md` の読み替えの追記。**コミット 2**（docs のみ。D3 の直後） | 変更した行の一覧 |
| D5 | 実機の確認（ユーザー。読み取り専用の起動 1 回） | 結果を `inventory.md` に記録 |

## 9. ユーザーに決めてほしいこと（D 案）
1. `package.xml`・`CMakeLists.txt`（bringup）に `ament_cmake_pytest` と試験を足してよいか（承認制。足さない場合、7.3 の 2 の単体試験は無し、確認は 7.5 の実行だけ）。
2. 既存の lint 違反（E501 ×4、D400・D415 ×2 の 1 か所、copyright）は今回は直さない（pep257 の 2 件は削除する関数の分だけ減る）でよいか。
3. エラーの文面（日本語と英語の併記か、日本語のみか）。
4. 7.5 の launch の挙動の確認で、ノードを起動する 2 件（#1・#2）と `--show-args` をしてよいか（ユーザーが実機を使っていないときに実施。起動の前に確認を求める）。


## 10. 実施の記録（D3。コミット 1。2026-11-12）
実装した（`bringup.launch.py`・moveit_config の xacro 2 つ・bringup の `CMakeLists.txt`/`package.xml`・新規 `test/test_launch_arg_check.py`）。確認の結果（試験の件数 196 → 231・失敗 12 → 10（pep257 の 2 件は「消滅」）、bringup 16 → 51、pytest 34 件、6 通りの robot_description の sha256 一致、launch の挙動 9 通り、`mock:=true` の 2 回の起動と `kill -INT` での停止、`ps` 空）は `docs/inventory.md` の「`hardware` 引数の削除（D 案）の実装」に書いた。pytest が登録されなかった原因（`~/.local` の anyio プラグインとシステムの pytest 6.2.5 の非互換）と対処（`PYTEST_DISABLE_PLUGIN_AUTOLOAD`）も同じ場所。**手順書の更新（7.6・D4）はコミット 2**。
