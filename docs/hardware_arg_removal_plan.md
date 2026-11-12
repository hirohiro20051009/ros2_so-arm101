# launch の `hardware` 引数の削除の計画（2026-11-12。**計画のみ。コード・xacro・launch・手順書は変更していない**。実機・ポートには触れていない）

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
