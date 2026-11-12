# ライセンスの計画（2026-11-12。調査と計画のみ。LICENSE・package.xml・ソースのヘッダは変更していない）

法的助言ではない。ユーザーの決定（前提）と、調べた事実・推測を分けて書く。**[事実]** = コマンドやファイルで確認した、**[推測]** = 根拠が弱い。

## 0. 前提（ユーザーの決定）と、この文書の置き場所

- ユーザー自身のコードのライセンスは **MIT**。今の `<license>BSD</license>`（description・moveit_config）は実態に合わないので直す。
- 外部由来の帰属は消さない: `third_party/SO-ARM100`（TheRobotStudio、Apache-2.0）、nimiCurtis/so101_ros2 由来（MIT。`LICENSE.so101_ros2` は由来のファイルが残る間は残す）、`[diag]` の基準値・黄金表（LeRobot 0.4.2 由来、Apache-2.0）。
- maintainer は全パッケージ `hiro`。bringup・controller・description・hardware には `<author>Nimrod</author>`。

**置き場所の提案: 新しいファイル `docs/license_plan.md`（この文書）にする。** 理由: `original_urdf_plan.md` 13 章は URDF の置き換えの時系列の記録の一部で、範囲が description・moveit_config・LeRobot 由来の値に広がった今は、1 か所で読めるほうが実施と確認がしやすい。13 章には、この文書への 1 行の案内だけを足した（本文は変えていない）。

## 1. 由来の一覧（git 管理下 120 ファイル）

調べ方: `git log --follow` の最初のコミット（Stage 1 = `29ff6bf` は旧ワークスペースからの取り込み、2026-11-05）、ファイル中の記述、`docs/inventory.md`・`docs/original_urdf_plan.md`（5・7 章）・`docs/third_party.md` の記録、旧ワークスペース `~/ros2_so101/src`（読み取りのみ）。

| 由来 | ファイル（パッケージごと） | 根拠 |
|---|---|---|
| **自作** | **hardware**: `src/`・`include/`・`test/`（`.cpp`/`.hpp` 19 本）、`config/follower_calibration.yaml`、`so_arm101_hardware_plugins.xml`、`test/data/follower_calibration_fixture_20260907.yaml`（自分の校正の数値）、`CMakeLists.txt`（骨組みは雛形で、中身は書き換え）。**calibration**: 全部（`.cpp`/`.hpp` 17 本、`CMakeLists.txt`、`package.xml`）。**bringup**: `launch/bringup.launch.py`（`3bc04f8` で新規作成）、`test/test_launch_arg_check.py`（`35ebba4`）。**description**: `cmake/*.cmake` 3 本（`56e6111` で新規）。**ルート**: `tools/pose_margin.py`・`tools/send_joint_delta.py`、`calibration_records/`（実機の測定記録 4 本）、`docs/`（11 本）、`CLAUDE.md`、`.gitignore` | [事実] 最初のコミットが 2026-11-05 以降の自分のコミット（`1e9e968`、`1af9412`、`3bc04f8` ほか）。hardware・calibration は旧ワークスペースに無く、nimiCurtis の `so101_hardware_interface`（Python の LeRobot ブリッジ）とは別実装（ブリッジは `git rm` 済み） |
| **自作（ひな形あり）** | **hardware / calibration / bringup の `package.xml`・`CMakeLists.txt`**: 骨組みは `ros2 pkg create` か nimiCurtis の骨組み。bringup の `CMakeLists.txt`・`package.xml` は `29ff6bf` が最初（nimiCurtis の `so101_bringup` の改名）で、`package.xml` に `<author>Nimrod</author>` が残る | [事実] `git log --follow` が `29ff6bf`、`<author>` の記載。ファイルの中身がどこまで nimiCurtis のままかは [推測]（`package.xml`・`CMakeLists.txt` は短い定型文で、独創性は低い） |
| **nimiCurtis（MIT）** | **controller**: `config/controllers.yaml`（冒頭のコメントに「旧 `so101_controllers.yaml` の /follower ブロック。値は全て引き継ぎ」。`3bc04f8` で整理した構成）、`CMakeLists.txt`・`package.xml`（`29ff6bf`、元の maintainer は Nimrod）。**description**: `package.xml`（author）、`CMakeLists.txt`（`29ff6bf` が元。今は URDF 生成・試験を足して書き換え済み）、`usd/` 5 本。**bringup**: `package.xml`・`CMakeLists.txt`（上の行と同じ） | [事実] LICENSE の著作権者（Copyright (c) 2025 nimiCurtis）、`<author>`、`controllers.yaml` のコメント、`29ff6bf`（旧 `so101_*` の取り込み）。**`usd/` は [推測]**（TheRobotStudio の `Simulation/SO101` に無く、nimiCurtis の `so101_description/usd` に同名のディレクトリがある。内容の一致と作成ツールは未確認。`original_urdf_plan.md` の課題に記録済み）。`bringup.launch.py` が nimiCurtis の `so101_robot.launch.py`・`follower.launch.py`（`archive/stage2a` を経て削除済み）を**参考にした部分があるか**は [推測]（コードの流用は確認していない。`3bc04f8` の統合版） |
| **TheRobotStudio（Apache-2.0）** | `description/third_party/SO-ARM100/`（`so101_new_calib.urdf`、`LICENSE`、`SOURCE.md`、`SHA256SUMS`）、`description/meshes/*.stl` 13 本（`SHA256SUMS` で原本と一致を固定） | [事実] `SOURCE.md`（385e8d7、バイト単位で同一）、sha256 の照合試験 `urdf_original_checksums`。`SOURCE.md` と `SHA256SUMS` はこのリポジトリで作った記録（自作） |
| **LeRobot 0.4.2 由来の値（Apache-2.0）** | `hardware/test/data/conversion_golden.csv`・`unnormalize_golden.csv`（`_normalize`/`_unnormalize` の出力の数値表。ヘッダ 2 行は自作）。`hardware/src/servo_config.cpp`・`include/servo_config.hpp`（`[diag]` の基準値 = `configure()` が書く値）。`include/calibration.hpp`・`src/calibration.cpp`（変換式の再実装）。`include/sts3215_bus.hpp`（レジスタ表・符号ビット）。`test/test_conversion.cpp`（黄金表を使う） | [事実] ファイル先頭の記述（`LeRobot 0.4.2 (Apache-2.0, Copyright 2024 The HuggingFace Inc. team)`、4 本。grep の結果）と `docs/third_party.md` の対応表。**コードそのものはコピーしていない**（式・数値・仕様を C++ で再実装）と記録されている。ただし、数値表・レジスタ表が著作物にあたるかは [推測]（事実・仕様は一般に著作権の対象外だが、保守的に帰属は残す、というユーザーの方針に従う） |
| **MoveIt Setup Assistant の生成物** | **moveit_config**: `.setup_assistant`（`generated_timestamp` = 2026-04-13）、`launch/*.launch.py` 7 本（雛形のまま）、`config/*.yaml`・`moveit.rviz`・`*.srdf`・`*.xacro`、`CMakeLists.txt`、`package.xml`（description・`<license>BSD</license>`・`<url>` が雛形） | [事実] `29ff6bf` で取り込み。旧 `so101_moveit_config` の maintainer・author は `hirok <chimay_beer@outlook.jp>` = **ユーザー自身が生成したもの**（nimiCurtis のものではない。nimiCurtis の clone に moveit_config は無い）。雛形の launch 7 本はヘッダ無し |
| **自作の変更を加えた生成物** | moveit_config の `servo.yaml`（日本語コメント。MoveIt Servo のサンプルが元か）、`so101_new_calib.ros2_control.xacro`（`mode` 切替など。nimiCurtis の `so101_new_calib.ros2_control.xacro` が元の可能性）、`moveit_controllers.yaml`・`ros2_controllers.yaml`・`joint_limits.yaml`・`so101_new_calib.srdf`・`so101_new_calib.urdf.xacro` | 履歴が複数のコミット（自分の変更）。**`servo.yaml`・`ros2_control.xacro` の元は [推測]**（`29ff6bf` 以前の履歴は無く、`git log` からは分からない。旧 `~/ros2_so101/src` と nimiCurtis の clone `/tmp/orig_nimi/repo`（`/tmp` なので消えうる）で突き合わせれば確かめられる。実施の段階で行う） |
| **そのほか（外部のものを含まない）** | `src/so_arm101_hardware/test/data/*`（上の 2 表を除く）、`LICENSE.so101_ros2`（nimiCurtis の MIT 全文）、`third_party/SO-ARM100/LICENSE`（Apache-2.0 全文） | — |

**git 管理外**: `setup_so101.sh`（`.gitignore` の対象、272 バイト）はライセンスの対象外。内容は今回の計画に関わらない（書き換えない）。

## 2. 今あるライセンス関係のファイルと記述

**ファイル**（`git ls-files` で確認）: `LICENSE.so101_ros2`（ルート。MIT、Copyright (c) 2025 nimiCurtis）、`src/so_arm101_description/third_party/SO-ARM100/LICENSE`（Apache-2.0）、`.../SOURCE.md`（出典の記録）。**ルートの `LICENSE` は無い**。`NOTICE*` は**無い**。ただし上流（SO-ARM100）にも NOTICE は無い（`SOURCE.md`・7 章）。LeRobot 0.4.2 は、インストール済みのもの（`site-packages/lerobot-0.4.2.dist-info/licenses/`）に `LICENSE` だけで `NOTICE` は無い **[事実（インストール済みの配布物）。リポジトリ側の NOTICE の有無は未確認]**。

**package.xml の `<license>`**（grep）:

| パッケージ | `<license>` | `<author>` | `<maintainer>` |
|---|---|---|---|
| so_arm101_bringup | MIT | Nimrod | hiro |
| so_arm101_calibration | MIT | なし | hiro |
| so_arm101_controller | MIT | Nimrod | hiro |
| so_arm101_description | **BSD** | Nimrod | hiro |
| so_arm101_hardware | MIT | Nimrod | hiro |
| so_arm101_moveit_config | **BSD** | hirok | hiro |

**ソースのヘッダ**（`.py`/`.cpp`/`.hpp` は `src` に 45 本。`git grep` と `ament_copyright` の結果）:

| ヘッダ | 件数 | 場所 |
|---|---|---|
| `// Copyright 2026 so_arm101 authors. MIT License.`（1 行） | **36** | hardware 19、calibration 17 |
| MIT の全文つき（`# Copyright 2026 hirok` + 許諾文） | **1** | bringup `test/test_launch_arg_check.py` |
| ヘッダ無し | **8** | bringup `bringup.launch.py`、moveit_config `launch/*.launch.py` 7 本 |
| Apache-2.0・HuggingFace への言及（本文中の出典の記述） | 6 本（`calibration.hpp`、`servo_config.hpp`、`sts3215_bus.hpp`、`test_conversion.cpp`、黄金表 CSV 2 本） | hardware |
| （`.cpp`/`.hpp` 以外）`CMakeLists.txt`・`gen_urdf.cmake` に Apache-2.0 の言及 | 2 | description（URDF の出典の記述） |

`SPDX-License-Identifier` は **0 件**。nimiCurtis・Nimrod の言及は、`<author>` の 4 つ（bringup・controller・description・hardware）と `LICENSE.so101_ros2` だけ（`src`・`tools` の grep）。

## 3. 案

### 3a. ルートの `LICENSE`（MIT）

文面（標準の MIT。1 行目の著作権表示だけをユーザーが決める）:

```
MIT License

Copyright (c) <年> <著作権者>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

**著作権表示の案（ユーザーが選ぶ）**:

| 案 | 表示 | 向く場合 | 注意 |
|---|---|---|---|
| **A（推奨）** | `Copyright (c) 2026 hiro` | package.xml の maintainer（`hiro`）と揃う。既存のヘッダ（`so_arm101 authors`）と矛盾しない（個人が著作権者で、プロジェクトの総称がヘッダ） | 個人の呼び名。本名にしたいなら案 C |
| B | `Copyright (c) 2026 so_arm101 authors` | 既存の 36 のヘッダと**同一**（ヘッダの書き換えが不要）。共同作者が増えても書き換えない | 法的な主体が曖昧（個人名が出ない）。nimiCurtis の MIT のように人名を書く慣習とは違う |
| C | `Copyright (c) 2026 <本名>` | 公開・配布するときに主体を明確にしたい | 本名の公開はユーザーの判断。ヘッダの `hirok`（`test_launch_arg_check.py`）とも揃える必要 |

年: **2026**（最初のコミット `29ff6bf` = 2026-11-05。moveit_config の生成は 2026-04-13 だが、リポジトリでの公開・取り込みは 2026）。複数年にするなら `2026-<最終年>`（`2026-2027` のように、変更のある年を伸ばす運用）。ヘッダの `hirok`（`test_launch_arg_check.py`）の表記は、決まった名前に合わせる（別の手）。

### 3b. package.xml の `<license>`

package.xml のスキーマ（format 3）は `<license>` を複数書ける **[確認済み]**（`ament_xmllint` で、description の package.xml の写しに `MIT` と `Apache-2.0` を並べたものが `valid`）。意味は「**複数のライセンスのコードがこのパッケージに含まれる**」で、デュアルライセンス（どちらか選べる）の意味ではない（ROS の慣習。REP 149 の記述の解釈 **[推測]**）。識別子は SPDX（`MIT`、`Apache-2.0`）にそろえる（`BSD` は SPDX にない）。

| パッケージ | 案 | 理由 |
|---|---|---|
| so_arm101_hardware | `MIT` | 自作。LeRobot 由来の値は Apache-2.0 だが、コードはコピーしていない（3c）。帰属は NOTICE 相当の文書（3c）とヘッダの出典で行う |
| so_arm101_calibration | `MIT` | 自作 |
| so_arm101_bringup | `MIT`（変更なし） | 自作 + nimiCurtis（MIT）。**同じ MIT なので `<license>` は 1 つでよい**。nimiCurtis の帰属は `LICENSE.so101_ros2` と `<author>` で残す |
| so_arm101_controller | `MIT`（変更なし） | 同上 |
| so_arm101_description | `MIT` + `Apache-2.0`（2 行。`BSD` を置き換え） | `MIT`: 自作の cmake・nimiCurtis 由来の usd・`package.xml` など。`Apache-2.0`: `third_party/SO-ARM100`（URDF）・`meshes/`（TheRobotStudio）。実態を表す。`<license>` を `Apache-2.0` だけにすると、自作・nimiCurtis の部分が表せない |
| so_arm101_moveit_config | `MIT`（`BSD` を置き換え） | 自作の変更 + ユーザー自身が生成した設定（Setup Assistant の雛形の `BSD` は、雛形が package.xml に入れる既定値で、雛形の launch にヘッダは無い **[推測]**）。MoveIt 本体（BSD-3-Clause）のコードは含まない。**迷う点**: `.setup_assistant`・雛形の `launch/*.launch.py` を「MoveIt の BSD の雛形」として `BSD-3-Clause` も並べるか（A: `MIT` だけ（推奨。雛形の出力に MoveIt の著作権表示は無い **[事実: grep で 0 件]**）、B: `MIT` + `BSD-3-Clause`） |

`<author>Nimrod</author>` と `<maintainer>hiro</maintainer>` は、そのまま。

### 3c. Apache-2.0 の帰属の扱い

一般論（法的助言ではない）: Apache-2.0 §4(a) LICENSE の写しを渡す、(b) 変更したファイルに変更の明記、(c) 元の表示を残す、(d) **上流に NOTICE があれば**その該当部分を渡す。

| 対象 | NOTICE は要るか | 案 |
|---|---|---|
| `third_party/SO-ARM100`（URDF・メッシュ） | **要らない**（上流に NOTICE が無い **[事実]**） | 今のまま: `LICENSE`（写し）・`SOURCE.md`・`SHA256SUMS` を `third_party/SO-ARM100/` に置き、`install(DIRECTORY third_party …)` でパッケージと一緒にインストールされる **[事実: CMakeLists]**。URDF は無変更、生成物は XML コメント 1 行で出典と変更（パスの置換）を明記済み（§4(b)） |
| LeRobot 0.4.2 由来の値（黄金表・基準値・変換式・レジスタ表） | **要らない**（インストール済みの配布物に NOTICE は無い **[事実]**。リポジトリ側は未確認） | 案 A（推奨・最小）: ルートに `THIRD_PARTY_NOTICES.md` を 1 つ置き（**`NOTICE` という名前は避ける**。Apache の NOTICE は上流の NOTICE を転記するものという意味を持つため）、出典（プロジェクト・バージョン・Copyright 2024 The HuggingFace Inc. team・Apache-2.0・URL）と、対象ファイルの一覧（1 章の表）を書く。ヘッダの出典の記述は残す。**Apache-2.0 の全文の写しは新しく作らない**（コードをコピーしていないため。`third_party/SO-ARM100/LICENSE` に写しがある）。案 B（保守的）: 案 A に加えて `hardware/test/data/` などに `LICENSE.lerobot`（Apache-2.0 全文）を置く。数値表を著作物と見るなら案 B。どちらでもコードは MIT のまま |
| `docs/third_party.md` | — | 今のまま残す（開発者向けの詳細な対応表）。`THIRD_PARTY_NOTICES.md` は、配布物に付ける短い版 |

### 3d. nimiCurtis 由来の MIT の扱い

- **`LICENSE.so101_ros2` の置き場所**: 案 A（推奨）: **ルートのまま**（今の名前。`docs`・`inventory.md` が参照している）。案 B: description に移す（`third_party/so101_ros2/LICENSE` + `SOURCE.md`）。`install(DIRECTORY third_party …)` でインストールされる利点があるが、対象は controller・bringup・description の 3 パッケージにまたがるので、ルートが素直。
- **どのファイルが対象かの示し方**: `THIRD_PARTY_NOTICES.md` に表を置く（3c と同じファイル）。対象（1 章）: `so_arm101_controller/config/controllers.yaml`、`so_arm101_controller|bringup|description` の `package.xml`・`CMakeLists.txt`（骨組み）、`so_arm101_description/usd/`（**[推測]**。確認できたら表の「推測」を外す）。`controllers.yaml` の冒頭コメントにも 1 行（`値の出典: nimiCurtis/so101_ros2（MIT。LICENSE.so101_ros2）`）を足す案（**実施は別承認**）。
- **削除の条件（今は削除しない）**: `controllers.yaml` の値の書き直し、`package.xml`・`CMakeLists.txt` の骨組みの整理、`usd/` の扱い（作り直すか、由来を確かめるか削除するか）が済み、grep（`nimiCurtis`・`Nimrod`・`Copyright`）で帰属が必要なものが無くなったことを確かめたあと、別の 1 手で `git rm`（CLAUDE.md の規則どおり、承認を得て、戻すコミットを `inventory.md` に書く）。

### 3e. ソースのヘッダの著作権表示

| 案 | 内容 | 長所 | 短所 |
|---|---|---|---|
| **E1（推奨）** | **ヘッダは増やさない。** 36 本の 1 行ヘッダは現状のまま（3a の案 B を選べば完全に一致。案 A・C なら著作権者の名前が違うので、後で一括置換するかを決める）。ルートの `LICENSE` と package.xml でライセンスを示す | 差分が最小。ament_copyright を走らせない 5 パッケージは件数が変わらない | ヘッダが無い 8 本は無いまま。1 行ヘッダは `ament_copyright` の基準では「ライセンス不明」（下の 4 章） |
| E2 | 自作の `.cpp`/`.hpp`/`.py` 全部（約 45 本 − 生成物 7 本 ≒ 38 本）に `Copyright` 1 行 + `SPDX-License-Identifier: MIT` を入れる | 機械で読める。短い | `ament_copyright`（Humble）は SPDX を**認識しない**（確認済み: `license=<unknown>` で失敗）ので、bringup の lint では失敗のまま |
| E3 | 自作のうち lint を走らせる **bringup だけ**、MIT の全文のヘッダ（`test_launch_arg_check.py` と同じ。約 20 行）を `bringup.launch.py` に足す | `ament_copyright` が通る（確認済み）。bringup の既存の copyright 失敗 1 件が消える | bringup の lint の件数が動く（4 章）。moveit_config の雛形 7 本は対象外（自作ではない） |
| E4 | 全部に MIT の全文のヘッダ | `ament_copyright` が全部通る | 約 40 本に 20 行ずつ。差分が大きい。ノイズ |

推奨は **E1**。bringup の lint の失敗の解消（E3）は、ライセンスの整理とは別の目的なので、やるなら別の手（承認制）。

## 4. colcon test への影響

**確かめたもの**（コマンドを実行。実機・ポート・ノードは使っていない）:

- **ament_copyright を走らせるパッケージは、今は bringup だけ [事実]**。`CMakeLists.txt` に `ament_lint_auto`・`ament_lint_auto_find_test_dependencies()` と `package.xml` の `<test_depend>ament_lint_common` があるのは bringup。controller は両方ともコメントアウト、description は `ament_lint_auto` を呼ぶが `package.xml` に lint の `test_depend` が無いので試験が 0 件（`colcon test-result` の description は 2 件 = 自前の cmake 試験のみ）、hardware・calibration・moveit_config は lint を使わない。
- `ament_copyright src`（手動で全 src を走らせた）: **48 ファイル中 47 件が失敗**。通るのは MIT の全文を持つ `test_launch_arg_check.py` だけ。失敗の内訳: ヘッダ無し 11（`.py`・`CMakeLists.txt` を含む）、ライセンス不明 36（**`// Copyright 2026 so_arm101 authors. MIT License.` の 1 行は「不明」扱い**）。bringup だけを走らせたときは 2 ファイル中 1 件失敗（`bringup.launch.py`）で、`inventory.md` の記録（copyright 1/1 失敗）と一致。
- 試し書き（scratchpad の一時ファイル。リポジトリは変更していない）: `SPDX-License-Identifier: MIT` だけのヘッダ、「Licensed under the Apache License…」の途中までのヘッダ、1 行のヘッダ → すべて `license=<unknown>` で失敗。`ament_copyright` が通るのは、**認識される許諾文の全文**（MIT・Apache-2.0・BSD など。`--list-licenses`）だけ。
- `ament_xmllint`: 6 つの package.xml は現状 `valid`（全 `src/*/package.xml`）。複数 `<license>` を並べた写し（scratchpad）も `valid`。**`<license>` の値の中身を検査する lint は無い**（`ament_xmllint` はスキーマだけ）。

**推測**（実施前に実測で確かめる）:

- `LICENSE`・`THIRD_PARTY_NOTICES.md` の追加は、colcon test の件数・結果に**影響しない**（`ament_copyright` はソースのヘッダだけを見て、LICENSE ファイルを読まない。ビルドの入力でもない）。
- package.xml の `<license>` の変更（3b）は、bringup の xmllint 1 件（現状成功）が成功のまま。他は lint を走らせないので件数は変わらない。**変更は package.xml なので、実施前に承認が必要**（5 章）。
- 3e の E1 なら、件数・失敗とも変わらない。E3 を選ぶと、bringup の copyright は 1 件失敗 → 成功（失敗数 −1。**bringup の失敗 10 → 9、件数は同じ**）。ヘッダは `#` コメントなので flake8（E501 は 80 字以内のヘッダなら出ない）・pep257（docstring より前のコメントは対象外）には影響しない見込み。実施後に前後の件数を比べる。
- 既存の bringup の copyright 失敗 1 件（`bringup.launch.py` のヘッダ無し）は、**ライセンスの整理だけでは解消しない**（E1）。解消するのは E3 か E4 のみ。1 行ヘッダを E2 のように増やしても解消しない（確認済み）。
- 今後、hardware・calibration で `ament_lint_auto` を有効にすると、1 行ヘッダの 36 本が失敗する（今は走らない）。有効にするなら、そのときにヘッダを全文にするか `set(ament_cmake_copyright_FOUND TRUE)` で copyright だけ除く（別の判断）。

## 5. 変更の種類（実施の順序と承認）

| # | 変更 | 種類 | 承認 |
|---|---|---|---|
| 1 | `docs/license_plan.md`（この文書）、`inventory.md` への記録 | docs のみ | 今回（実施済み） |
| 2 | ルートに `LICENSE`（MIT）を追加。著作権者・年はユーザーが決める（3a） | 新規ファイル（ビルドに無関係） | 承認（著作権者・年を指定） |
| 3 | `THIRD_PARTY_NOTICES.md` を追加（3c・3d）。`LICENSE.so101_ros2` は**残す** | 新規ファイル | 承認（案 A / B の選択） |
| 4 | **各 `package.xml` の `<license>` を変更**（description: `BSD` → `MIT` + `Apache-2.0`、moveit_config: `BSD` → `MIT`（または `MIT` + `BSD-3-Clause`）） | **package.xml の変更。実装前に承認が必要。** 変更後に `colcon build`・`colcon test`（bringup の xmllint）を確認 | **必須の承認**（変更前後の値を示す） |
| 5 | `controllers.yaml` などへの出典コメントの追加（3d） | 設定ファイルのコメントのみ。値は変えない | 承認 |
| 6 | ソースのヘッダの変更（E2〜E4 を選んだ場合） | ソースの変更。件数が動く | 承認（E1 なら無し） |
| 7 | 1 行ヘッダの著作権者の一括置換（3a で案 A・C を選んだ場合） | 36 本のソース（コメントのみ） | 承認 |
| 8 | `LICENSE.so101_ros2` の削除 | `git rm`（3d の条件が満たされてから。別の 1 手） | 承認 + `inventory.md` に戻すコミットを記録 |

各手は、コミットを分ける（CLAUDE.md の規則: 1 段階ごとに `colcon build` が通ること、コミット前に関係パッケージの `colcon test`、失敗は先に報告）。LICENSE・docs だけの手（2・3）は `colcon test` を省略できる。**4・5・6・7 は、ビルドを通して件数を前後で比べる**。

## 6. ユーザーに決めてほしいこと

1. 著作権表示（3a の A / B / C、年）。
2. moveit_config の `<license>`（`MIT` のみ / `MIT` + `BSD-3-Clause`）。
3. LeRobot 由来の値の帰属（3c の案 A: 表示のみ / B: Apache-2.0 全文の写しも置く）。
4. ヘッダ（3e の E1〜E4。bringup の copyright の失敗 1 件をこの機会に直すか = E3）。
5. `LICENSE.so101_ros2` の置き場所（ルートのまま / description に移す）。
6. `usd/` の由来の確認を、ライセンスの実施の前にやるか後にやるか（旧ワークスペースと nimiCurtis の clone との突き合わせ。`servo.yaml`・`ros2_control.xacro` の元の確認も同じ作業）。

## 7. 確認できた範囲・できなかった範囲

- 確認できた: 1〜2 章の件数と記述（`git ls-files`・grep）、`ament_copyright`・`ament_xmllint` の挙動（手動実行。ノード・実機は使っていない）、LeRobot 0.4.2（インストール済み）の `LICENSE` に NOTICE 無し、SO-ARM100 に NOTICE 無し。
- 確認できなかった: nimiCurtis 版の個々のファイル（`usd/`・`servo.yaml`・`ros2_control.xacro`）との一致、onshape-to-robot（Rhoban）・Open Duck Mini（apirrone）のライセンス（SO-ARM100 の URDF の生成・物性値の出典。`original_urdf_plan.md` 7 章）、LeRobot の GitHub リポジトリ側の NOTICE（インストール済みの配布物の範囲のみ）、`colcon test` の実測（今回は docs の変更だけなので実行していない。5 章の変更のときに実測する）。
