# bringup.launch.py の既存の lint 違反 10 件を直す計画

作成: 2026-11-12 / **調査と計画のみ（実装しない。承認待ち）** / 基準: HEAD `6f245da`（この計画のコミットの前）

- 調査の方法: `colcon test --packages-select so_arm101_bringup` の実行（lint とオフラインの pytest だけ。ノードもシリアルポートも使わない）、`git log --follow`、`git show`、上流（nimiCurtis/so101_ros2）の取得（読み取り）、行の比較。リポジトリのファイルは、この文書以外は変えていない。
- 凡例: **[事実]** = 実際に確かめた、**[推測]** = 確かめていない。
- 場所: 新しいファイル（`inventory.md` には、残課題 3 から指す 1 行だけを足す）。

## 1. 要点

1. 違反は **`launch/bringup.launch.py` の 7 件**（copyright 1、flake8 E501 ×4、pep257 D400・D415）と、それが原因で失敗する CTest のまとめ 3 件（copyright・flake8・pep257）。`colcon test-result` は 51 件・失敗 10。**`test_launch_arg_check.py`・`CMakeLists.txt`・`package.xml` に違反はない。**
2. 直しは**すべて「見た目」の変更**（ヘッダのコメント追加、docstring の 1 行目、改行の位置）。動作・引数・戻り値は変えない。
3. **由来: `bringup.launch.py` は新規に書いたファイルで、nimiCurtis のコードは残っていない**（4 章）。Copyright 行は **`Copyright 2026 hirok` だけ**を推奨。`THIRD_PARTY_NOTICES.md` は**変更不要**（理由は 4.4）。
4. 実機の読み取り専用の確認は**不要**を推奨（7 章）。

## 2. 違反 10 件の一覧（実際の出力）

`colcon test`（bringup、51 件）の `ament_copyright`・`ament_flake8`・`ament_pep257` の出力と、`colcon test-result --verbose` から。**[事実]**

| # | ツール | ファイル:行 | コード | 内容 |
|---|---|---|---|---|
| 1 | ament_copyright | `launch/bringup.launch.py`（先頭） | — | `could not find copyright notice` |
| 2 | ament_flake8 | `launch/bringup.launch.py:115:100` | E501 | `line too long (104 > 99 characters)`。`get_package_share_directory("so_arm101_hardware"), "config", "follower_calibration.yaml"` |
| 3 | ament_flake8 | `launch/bringup.launch.py:136:100` | E501 | 115 > 99。`torque_limit` の `description=`（`"書き込みモードの Torque_Limit。shoulder_pan と wrist_roll だけに効く（lift / elbow / wrist_flex は常に 1000、"`） |
| 4 | ament_flake8 | `launch/bringup.launch.py:193:100` | E501 | 107 > 99。`ld.add_action(include("moveit_rviz.launch.py", condition=IfCondition(LaunchConfiguration("use_rviz"))))` |
| 5 | ament_flake8 | `launch/bringup.launch.py:194:100` | E501 | 102 > 99。`ld.add_action(include("warehouse_db.launch.py", condition=IfCondition(LaunchConfiguration("db"))))` |
| 6 | ament_pep257 | `launch/bringup.launch.py:1` | D400 | `First line should end with a period (not '。')` |
| 7 | ament_pep257 | `launch/bringup.launch.py:1` | D415 | `First line should end with a period, question mark, or exclamation point (not '。')` |
| 8 | CTest | `copyright` | — | #1 のまとめ |
| 9 | CTest | `flake8` | — | #2〜#5 のまとめ（`4 errors`） |
| 10 | CTest | `pep257` | — | #6・#7 のまとめ（`2 errors`） |

- 失敗の件数: 7（#1〜#7）+ 3（#8〜#10）= **10**。`Summary: 51 tests, 0 errors, 10 failures, 0 skipped`。
- 通っているもの: `test_launch_arg_check`（pytest、34 件）、`lint_cmake`、`xmllint`、`test_launch_arg_check.py` の copyright・flake8・pep257。

## 3. 由来の確認（依頼 2）

### 3.1 `git log --follow` の経緯 [事実]

| コミット | 日付 | 内容 |
|---|---|---|
| `3bc04f8` | 2026-11-05 | **最初のコミット（`--diff-filter=A`）**。Stage 2a-2「unified bringup launcher」。**新規ファイルとして追加**（92 行。`git mv` や rename の検出ではない） |
| `1e9e968` | 2026-11-06 | 2b-2: 読み取り専用の Feetech ドライバの導入 |
| `ff91fd6` | 2026-11-10 | 2b-3b: 書き込みモード（`torque_limit` など） |
| `6b9d483` | 2026-11-11 | 2b-4: 既定を `hardware=feetech` |
| `7b5360b` | 2026-11-12 | L1: LeRobot 経路と `bringup_with_cameras` の削除 |
| `3a3b0c2` | 2026-11-12 | L4: 文言 |
| `35ebba4` | 2026-11-12 | `hardware` 引数の削除と、未宣言の引数の検査（D 案） |
| `1fe9e34` | 2026-11-12 | E1: `controllers.yaml` の移動（パス 3 行） |

- 作者はすべて `hirohiro20051009`。現在 199 行。

### 3.2 上流との比較 [事実]（読んだ版を併記）

- **上流**: `nimiCurtis/so101_ros2` の **HEAD `b50872d2ba03a3d67fb9a9a7b59af1e72c7040b3`** の `so101_bringup/launch/so101_robot.launch.py`（91 行）を `raw.githubusercontent.com` から取得して読んだ。旧ワークスペースの `~/ros2_so101/src/so101_ros2/so101_bringup/launch/so101_robot.launch.py` と**取得したものは `diff` が空（同一）**だった。以下の「77 行」は、空行を除いた行の数で、ファイルの行数（91）とは別。
- **比較の方法**: 空行を除き、各行の前後の空白を除いて、ファイル同士の「同じ行の集合」の大きさを数えた。
  - 現在の `bringup.launch.py`（空行を除いて、重複を除くと 134 種類の行）のうち、`so101_robot.launch.py` と同じ行は **10 行**。`follower.launch.py` と 8 行、`so101_teleoperate.launch.py` と 10 行、`so101_record.launch.py` と 6 行など、**ほかの上流の launch でも 4〜10 行で、差がない**。
  - 同じ 10 行は、`import os`、`from launch import LaunchDescription`、`from launch.conditions import IfCondition`、`def generate_launch_description():`、閉じ括弧 `)` など、**launch ファイルで誰が書いても出る行**だけ。上流に固有の式・名前・引数・コメントは**一致していない**。
  - 最初のコミットの版（3bc04f8、64 種類の行）でも、上流と同じ行は 4〜8 行（import と閉じ括弧）。
- **骨組みの出所は MoveIt Setup Assistant の `demo.launch.py`**: 旧ワークスペースから取り込んだ `so_arm101_moveit_config/launch/demo.launch.py`（タグ `stage2a-done` の `archive/stage2a/moveit_config/launch/demo.launch.py`、94 行）と同じ行は **現在 21 行・最初の版 22 行**。内訳は `MoveItConfigsBuilder("so101_new_calib", package_name="so_arm101_moveit_config")`、`DeclareBooleanLaunchArg("use_rviz", default_value=True)`、`DeclareBooleanLaunchArg("db", default_value=False)`、`executable="ros2_control_node"` など、**Setup Assistant の雛形の骨組み**。この `demo.launch.py` には**著作権ヘッダがない**（Setup Assistant の生成物。`THIRD_PARTY_NOTICES.md` の 4 章に書いてある BSD-3-Clause の雛形）。
- **結論 [事実と判断]**:
  - nimiCurtis の `so101_robot.launch.py` **由来の部分は残っていない**（固有の行が 0）。最初のコミットの時点でも、固有の行は 0。
  - 骨組みに MoveIt Setup Assistant の雛形（BSD-3-Clause）が残っている。ただし「雛形と同じ行」は API 呼び出しの形の数行で、**著作権表示を求められる量ではない** [判断]。`THIRD_PARTY_NOTICES.md` の 4 章が、すでに Setup Assistant の雛形について書いている。
  - **[推測]** 上流の `so101_robot.launch.py` を読んで構成の着想を得た可能性は否定できないが、コードは写していない（行が一致しない）。

### 3.3 比較の限界 [事実]

- 比較は「行の一致」で、**意味の類似**（同じ構成を別の書き方で書いたもの）は見ていない。
- 取得したのは HEAD の 1 版だけで、上流の履歴の古い版（2025 年の途中の版）は読んでいない。旧ワークスペースの複製（`d8869be`、2026-11-05 時点の自分のリポジトリ）とは一致した。

## 4. Copyright 行と THIRD_PARTY_NOTICES.md の提案

### 4.1 Copyright 行

- **推奨: `# Copyright 2026 hirok` のみ**。nimiCurtis は**併記しない**。
- 理由: 3 章のとおり、上流のコードが残っていない。併記すると「上流のコードを含む」と読めて、事実と合わない。
- `test_launch_arg_check.py` の既存のヘッダ（`Copyright 2026 hirok`）、ルートの `LICENSE`（`Copyright (c) 2026 hiro`）と**同じ表記に揃える**。
- **決めてほしいこと（項目 A）**: 上記の `hirok`（試験ファイルと同じ）でよいか。ルートの `LICENSE` は `hiro` なので、表記が 2 通りある（今回の変更の範囲外だが、気づいたので書く）。

### 4.2 もし「併記」にするなら（採らない案）

- 2 行目に `# Copyright 2025 nimiCurtis` を足す。ただし 3.2 の結論と合わず、`THIRD_PARTY_NOTICES.md` の 2 章のファイル一覧にも `bringup.launch.py` を足す必要が出る。

### 4.3 ヘッダの文面（依頼 3）

`ament_copyright` が通る形は、すでに `test/test_launch_arg_check.py`（このリポジトリで通っているもの）と同じ。**MIT の全文**（`test_launch_arg_check.py` の 1〜19 行目。このヘッダで `ament_copyright` が通っている [事実]）:

```python
# Copyright 2026 hirok
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
```

- 実装では、その 19 行を**そのままコピー**する（手で打たない）。
- 置く場所は**ファイルの先頭、モジュールの docstring の前**。コメントなので、docstring は引き続き「モジュールの最初の文」で、`pep257` の対象のまま。
- 19 行が先頭に入るので、**以降の行番号が 19 ずれる**（`inventory.md` などの `bringup.launch.py:行` の参照に影響する。実装のときに grep で確認し、必要なら直す）。

### 4.4 THIRD_PARTY_NOTICES.md の変更の要否

- **変更不要**（推奨）。`bringup.launch.py` は 2 章（nimiCurtis）の一覧にないままで、事実と合っている。上流のコードが残っていないので、一覧に足さない。
- 4 章（MoveIt の Setup Assistant の雛形）は、すでに「ソースコードは含まない」「生成物に著作権ヘッダがない」と書いてあり、`bringup.launch.py` の骨組みの出所と矛盾しない。
- 変更するのは、4.1 で「併記」を選んだときだけ。

## 5. 各違反の変更前後（依頼 4）

**動作を変える変更は入れない。** 文字列リテラルは、改行で分けても**連結の結果が 1 文字も変わらない**ようにする（隣り合う文字列リテラルの連結）。

### 5.1 #1・#8 copyright

- 前: ヘッダなし。後: 4.3 の 19 行を先頭に追加。

### 5.2 #6・#7・#10 pep257 D400・D415（モジュールの docstring の 1 行目）

現状（形は D213: `"""` の次の行から始まる。D212 は既定で対象外。2 行目以降は空行の次）:

```python
"""
SO-ARM101 統合ランチャー（ルート名前空間）。

  mock:=true ...
```

- 原因: 1 行目が `。` で終わっていて、`.` `?` `!` のどれでもない。D400 は ASCII のピリオドだけを見る。
- **案 1（推奨）**: 1 行目を**英語の 1 文（ピリオドで終わる）**にして、日本語の説明は**空行の次の本文**に残す。

  ```python
  """
  Unified launcher for the SO-ARM101 (root namespace).

  SO-ARM101 統合ランチャー（ルート名前空間）。
  （以下、現在の本文を変えない）
  ```

  - 日本語の 1 行は本文の最初に残す（情報は減らさない）。D205（要約と本文の間に空行）、D213 は保つ。
- 案 2: 日本語の 1 行目の末尾の `。` を `.` に変える（`ランチャー（ルート名前空間）.`）。日本語の文章に ASCII のピリオドが混ざる。最小の変更だが、読みにくい。
- 案 3: 末尾の `。` の後に `.` を足す（`。.`）。避ける。
- **決めてほしいこと（項目 B）**: 案 1（英語の要約 1 行を足す）を推奨。docstring は実行時に使われない（`__doc__` を参照するコードはない [事実]: `grep -rn "__doc__" src/so_arm101_bringup --include=*.py` で 0 件を確認済み）。

### 5.3 #2 E501（`calibration_file` の default_value、115 行目）

前（104 桁）:

```python
            default_value=os.path.join(
                get_package_share_directory("so_arm101_hardware"), "config", "follower_calibration.yaml"
            ),
```

後（引数ごとに 1 行。結果のパスは同じ）:

```python
            default_value=os.path.join(
                get_package_share_directory("so_arm101_hardware"),
                "config",
                "follower_calibration.yaml",
            ),
```

### 5.4 #3 E501（`torque_limit` の `description`、136 行目）

前（2 つの隣り合う文字列リテラルの 1 つ目が 115 桁）:

```python
            description="書き込みモードの Torque_Limit。shoulder_pan と wrist_roll だけに効く（lift / elbow / wrist_flex は常に 1000、"
            "gripper は min(値, 500)）。W1〜W2 の初回は 500",
```

後（リテラルを 3 つに分ける。**連結の結果は同じ**）:

```python
            description="書き込みモードの Torque_Limit。shoulder_pan と wrist_roll だけに効く"
            "（lift / elbow / wrist_flex は常に 1000、"
            "gripper は min(値, 500)）。W1〜W2 の初回は 500",
```

- 各行の桁数は 99 以下（1 行目は、`description="` の 12 桁の字下げ + 約 58 桁。全角は flake8 で 1 文字 = 1 桁と数える [事実: 既存の違反が 115 桁と数えられている点から、文字数で数えている]）。実装のときに flake8 で確認する。
- 確認: `--show-args` の `torque_limit` の説明が**変更前と文字列として一致**すること（下の 6.3）。

### 5.5 #4・#5 E501（`moveit_rviz`・`warehouse_db` の include、193・194 行目）

前:

```python
    ld.add_action(include("moveit_rviz.launch.py", condition=IfCondition(LaunchConfiguration("use_rviz"))))
    ld.add_action(include("warehouse_db.launch.py", condition=IfCondition(LaunchConfiguration("db"))))
```

後（引数ごとに改行。`include(...)` の引数と `add_action` の順序・個数は同じ）:

```python
    ld.add_action(
        include(
            "moveit_rviz.launch.py",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    ld.add_action(
        include(
            "warehouse_db.launch.py",
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )
```

- `ld.add_action` を呼ぶ**順序は変えない**（`ld.get_launch_arguments()` の順序に影響しうるため）。

### 5.6 触らないもの

- `_check_launch_arguments`、`declared_names`、`TimerAction(period=3.0, ...)`、引数の既定値・`choices`・`description`（文字列の中身）、`include` の定義。
- `test_launch_arg_check.py`（違反なし）、`CMakeLists.txt`（`PYTEST_DISABLE_PLUGIN_AUTOLOAD` はそのまま。`docs/inventory.md` の残課題 1 の決定）。
- **MoveIt 本体の設定（URDF・関節リミット・planning_frame・IK solver）には触れない。**

## 6. 確認の方法（依頼 5）

実装の前に**基準を取り**、実装の後に前後を比べる。

### 6.1 colcon（オフライン）

1. 基準（今回取得済み）: bringup **51 件・失敗 10**（pytest 34 件は全件成功。残り 17 件が lint と CTest のまとめ）。
2. 実装後: `colcon build --packages-select so_arm101_bringup`、`colcon test --packages-select so_arm101_bringup`、`colcon test-result --verbose`。**期待は 51 件・失敗 0**。
   - **総数が 51 から変わったら止めて報告する**（件数が変わるのは、検査が増減した、またはテストが収集されなくなった、のどちらか）。
   - 失敗が 0 にならないときは、残った違反を報告し、コミットしない。
3. CLAUDE.md の規則どおり、**コミットの前に colcon test を実行**して結果を報告する。bringup 以外のパッケージの試験は、このファイル 1 つの見た目の変更では影響しない [推測]。ただし全体も 1 回実行し、前後の件数を比べる（今の全体の件数は、実装の前に取り直して基準にする）。

### 6.2 `--show-args`（起動しない）

- `ros2 launch so_arm101_bringup bringup.launch.py --show-args` の**宣言された引数が 21 個**で、実装の前後で**名前・既定値・説明が一致**すること（前後の出力を `diff`。`torque_limit` と `calibration_file` の説明・既定値は、改行を変えた箇所なので特に見る）。
- 検査は走らない（`--show-args` は launch を読み込むだけ）。

### 6.3 robot_description の sha256（6 通り）

- `~/so_arm101_check/gen_rd6.sh` を実行し、`~/so_arm101_check/urdf_after/` の保存物と**sha256 が一致**すること（基準値は 6 通りとも保存済み。`.urdf` の先頭 16 桁: 1・2 `d8eebdcd27762b2b`、3 `9bfaceea6da94c16`、4 `a5160ee8aad290a8`、5 `6bf7aa602dc5b6ad`、6 `7dc2c92cc80f37ed`。`.err` は 6 つとも `37d255df752f8d3d`）。
- 注意 [事実]: `gen_rd6.sh` は **xacro を直接実行**するので、`bringup.launch.py` を通らない。**launch の変更では sha256 が変わりようがない**。そのため、この確認は「xacro の側を壊していない」ことの確認にとどまる。launch が作る `robot_description` そのものの確認は、**launch ファイルを Python で読み込んで `MoveItConfigsBuilder(...).robot_description(mappings=...)` の結果を比べる**方法がある（実装の前後で同じスクリプトを実行し、文字列の sha256 を比べる。ノードは起動しない）。**推奨**: 前後の両方を実施する（後者は一時ディレクトリの小さなスクリプトで、リポジトリのファイルは変えない）。

### 6.4 未宣言の引数の検査（ノードは起動しない）

- 既存の pytest（`test_launch_arg_check.py`、34 件）が、この検査を試験している。これが 34 件すべて通ること。
- 実際の launch でも確認する: `ros2 launch so_arm101_bringup bringup.launch.py mock:=true use_rviz:=false hardware:=x` が **exit 1** で、`process started` が **0 行**であること（`docs/hardware_arg_removal_plan.md` の確認 #3・#4 と同じ。SIGINT を既定に戻すラッパーを使い、PID を控える。止めた後は `ps` と `ros2 node list` の両方で残りがないことを確認する）。
- 宣言済みの引数だけで `--show-args` が通ること（6.2）。

### 6.5 モック（`mock:=true`）で 3 コントローラ active

- **起動する前に、ユーザーが実機（サーボ・launch）を使っていないと答えてから行う。** 答えがあるまで起動しない。
- 手順: `mock:=true use_rviz:=false` で起動（PID を控える）→ `ros2 control list_controllers` で `joint_state_broadcaster`・`arm_controller`・`gripper_controller` の 3 つが `active` → `kill -INT <控えた PID>` → `ps` と `ros2 node list` で残りがないことを確認（`ros2 node list` だけに残り `ps` が空なら `ros2 daemon stop; ros2 daemon start` の後に再確認）。
- `pkill -f`・`killall` は使わない（実機の launch を止めて、腕が落ちるのを避ける）。
- 起動のログに `[ERROR]` が増えていないこと（基準: 実装の前のモックの起動ログも同じ手順で 1 回取り、前後を比べる。これは前の計画でも行っているので、既存のログ `~/so_arm101_check/` の最新のモックのものを基準にしてもよい）。

## 7. 実機の読み取り専用の確認の要否（依頼 6）

**不要**を提案する。

- 変更は**コメントの追加・docstring の 1 行目・改行の位置**だけで、Python の構文木（AST）が**変わらない**（docstring の 1 行目と、コメント以外）。実行時の値（引数の既定値、`description` の文字列、`add_action` の順序）は、6 章のオフラインの確認で一致を確かめる。
- さらに強い確認として、**実装の前後で `ast.dump` を比べる**方法がある（docstring を除き、同一であること）。これは**一時ディレクトリのスクリプトで行い、リポジトリは変えない**。推奨: 6 章に加える。これで「動作が変わらない」ことが、実機なしで機械的に示せる。
- 実機に関わる部分（`Sts3215System`、xacro、`controllers.yaml`、`robot_description` の中身）は変更しない。`robot_description` の sha256 の一致（6.3）で、ドライバに渡る引数が同じことを確かめる。
- 比較: 前の計画（`controllers_yaml_move_plan.md`）は `controllers.yaml` の**実行時に読まれる設定**を動かしたので、実機の読み取り専用の確認を行った。今回は、**実行時に読まれる値が同じ**ことを AST と sha256 で示せるので、実機の確認は省ける。
- 例外: 6.1〜6.5 のどれかが**一致しない**、または AST に差が出たときは、原因を調べて報告し、実機の確認の要否を改めて相談する。

## 8. 実施の段階とコミット

| 段階 | 内容 | 確認 | コミット |
|---|---|---|---|
| H1 | 基準の取得: 全体の `colcon test` の件数、`--show-args` の出力（`~/so_arm101_check/` に保存）、launch 経由の `robot_description` の sha256、`ast.dump` の保存、モックの起動（ユーザーの確認の後） | — | なし |
| H2 | `bringup.launch.py` の変更（5 章。ヘッダ・docstring・E501 の 4 か所） | 6.1〜6.4（オフライン）、`ast.dump` の一致 | — |
| H3 | モックの起動（6.5。ユーザーが実機を使っていないと答えた後） | 3 コントローラ active、残りがない | — |
| H4 | `inventory.md` に結果を記録（残課題 3 を「解消」に更新）。違反が消えたことと、**「解消」か「消滅」か**（今回は**本当に解消**。違反を消したのではなく、直した）を明記 | — | **コミット 1**（`colcon test` の結果を報告した上で） |

- 段階ごとに計画を出し、承認を得てから実行する（CLAUDE.md）。
- `build/`・`install/` は消さない。`log/` は消さない。削除するファイルはない（`git rm` なし）。
- `docs/inventory.md` の `bringup.launch.py:行` の参照は、ヘッダの追加（+19 行）でずれる。実装のときに grep して、ずれた参照を直す（記録の書き換えなので、直す一覧を先に示す）。

## 9. 決めてほしいこと（項目・推奨）

| 項目 | 内容 | 推奨 |
|---|---|---|
| A | Copyright 行 | `# Copyright 2026 hirok` のみ。nimiCurtis は併記しない。`THIRD_PARTY_NOTICES.md` は変更しない |
| B | docstring の 1 行目（D400・D415） | 案 1: 英語の要約 1 行を足し、日本語の 1 行は本文に残す |
| C | 実機の読み取り専用の確認 | 不要（AST の一致と sha256 の一致で代える） |
| D | 確認の追加 | `ast.dump` の前後比較と、launch 経由の `robot_description` の sha256 を足す |
| E | モックの起動 | 実機を使っていないと答えてから実施 |

## 10. 事実と推測の区別

- **事実**: 違反 10 件の出力、`git log --follow` の経緯、上流 HEAD `b50872d` の `so101_robot.launch.py` の取得、行の一致の数、`gen_rd6.sh` と `urdf_after/` の sha256、`ament_copyright` の出力。
- **推測 / 判断**: 上流の構成から着想を得た可能性、Setup Assistant の雛形の一致が「著作権表示を要する量ではない」という判断、bringup 以外のパッケージの試験が影響を受けないこと、`__doc__` を参照するコードが bringup の外（ほかのパッケージや launch の機構）にもないこと。
