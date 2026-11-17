# オフセットキャリブ（zero-offset calibration。机当て floor touch）の計画: lift・elbow・wrist_flex の 0 のずれ zero_offset_deg を、先端を机に当てた姿勢の関節角から決める

2026-11-17。**計画だけ**。コード・yaml・URDF・ビルド・モック・実機は触っていない（読むことだけで作った。読み取りは ls・cat・grep・sed・wc のみ。3 章のオフラインの検討の Python 8 回だけ、承認を得て実行した）。実装は承認後。

| 2 つのキャリブの違い | センターキャリブ（center calibration） | オフセットキャリブ（zero-offset calibration。本書） |
|---|---|---|
| 何を決めるか | 各関節の Min・Max・center（0 rad の位置の tick。center は Min と Max の中央）と、EEPROM の Homing_Offset | center から URDF の 0 までの角度のずれ zero_offset_deg（lift・elbow・wrist_flex。度） |
| 使う道具 | `so_arm101_calib measure` と `apply`（C++。サーボの EEPROM を読み書きする） | `zero_offset_record_run.py`（記録）と `zero_offset_fit_run.py`（当てはめ）。読み取り専用で、サーボにもファイルにも書かない |
| 決める方法 | 腕を手で中央の姿勢と可動範囲の両端に動かして測る | 先端を机に当てた 4 姿勢（各 2 回）の関節角から、当てはめる |
| yaml での置き場所 | `center`・`min`・`max`・`eeprom`（`apply` が書く） | `zero_offset_deg`（J5 のコミットで手で貼る。`apply` は値を持ち越す） |
| 実行の順番 | 1 番目 | 2 番目（センターキャリブの後、カメラのキャリブの前） |
| やり直し | 関節の交換・再組み立てなど | **センターキャリブをやり直したら、オフセットキャリブもやり直す**（center が変わると 0 の意味が変わる） |

- 前提: `docs/joint_offset_plan.md`（9.1 が決定）、`docs/inventory.md` の「C3 の dry 実行と elbow のずれの調査」と J1〜J4d、`docs/camera_stage_plan.md`。HEAD は 0551eaf。
- 書き方: **事実**（コード・docs・ログで確かめたもの）と **推測**（見立て・概算・閾値）を分ける。数値の閾値はすべて推測で、実機の後に見直す。表のセルに縦線は使わない。
- 記号: **δ** = zero_offset_deg（度）。真の角度 = 測った角度 + δ（J1 の符号。`joint_offset_plan.md` 2.1）。**h** = 机の高さ（base_link の z、m）。**ε** = 先端の軸と机の角度（90° が真下向き、小さいほど浅い）。**向き付きで、折り返さない**（F4b）: 90° 未満が前傾（先端が手首より前）、90° 超が後傾。95° と 85° は区別される。**実物の ε（expected）** = 読んだ角度に `--expected-offset` の値を足した姿勢の ε（記録の案内にだけ使う）。**読んだ値の ε（read）** = `/joint_states` の角度そのものの ε（記録される値で、fit が使う）。目標の姿勢（0.1 の 1）は実物の ε と x で持つ。**x** = base_link の x（gripper_frame の +z 方向 6.3 mm の基準点。`reach_scan` と同じ定義）。**q_state** = `/joint_states` の値（yaml に δ があれば補正後）。**q_raw0** = yaml の δ を引き戻した値。

## 0. 要約

### 0.1 決定（ユーザー、2026-11-17）

1. **姿勢**: 傾き 90・70・50° の 4 姿勢 × 各 2 回（記録 8 回）。姿勢と目標（mm、許容 ±10 mm、傾きは全姿勢 ±5°。向き付きの角度）: P1 は 90° で x 100、P2 は 90° で x 300、P3 は 70° で x 150、P4 は 50° で x 250。**目標は実物（真の角度）で持つ**（F4b、ユーザーの決定。reach_scan と design_study の ε が真の角度だったため）。yaml に δ がまだ無い間は、道具に `--expected-offset NAME=DEG`（初回は `elbow_flex=-6.3`）を渡し、実物 = 読んだ角度 + DEG と見込んで案内する。表示・目標の判定・ε_min の判定だけに使い、記録する角度と fit には使わない。F6 で yaml に δ を入れた後は渡さない。
2. **モデル**: 固定あご（gripper_link）の最低点の z = h。90° は両あごの差が実物で小さく、**あごを閉じた状態では動くあごが先に机に当たることはない**（ユーザーが実物で確認、F4b）。使う。90° の点の σ の上乗せの**既定は 0**（F4b で 1 mm から変更。選べる引数は残す）。モデルの jaw_diff（真下で動くあごが 1.26 mm 低い）との食い違いは未確認（8 章）。精度の見込みは design_study_3 の 90・70・50° の値（上乗せを入れた値なので保守側）を使う。
3. **ε_min = 45°**（守りとして残す）。
4. **h は事前分布つき**（平均 −2.4 mm、σ_h 1 mm。腕と机が同じ面のとき）。違う台なら高さの差を測って事前分布の平均を直す（手順書 7.5）。
5. **合否の基準**は 4 章（すべて推測の値。実機の後に見直す）。
6. **wrist の貼る行**は、δ_wrist の絶対値が 1.5° 以上で sd が 1.0° 以下のときだけ出す。それ以外は「wrist は 0 のまま（理由: 90° の当たり方の 1 mm で約 0.7° の偏りが出るため）」と表示する。
7. **yaml は道具が書かない**。表示した貼る行（0.1° 刻み）を、J5 のコミットで手で入れる。
8. **置き場所**: 純粋な計算は `so_arm101_calibration`（センターキャリブと一緒に実行する道具なので）。道具は `tools/`。5.1。

### 0.2 呼び名と道具の名前

- 呼び名を固定する: 日本語は「センターキャリブ」と「オフセットキャリブ」、英語は center calibration と zero-offset calibration。docs・ログ・CSV・yaml のコメントはこの呼び名にそろえる（「関節のキャリブ」「校正」とは書かない）。
- 道具の名前（現在の名前は floor_touch。机当ての方法を表すが、何を決めるかが分からない）。**改名（決定、ユーザー、2026-11-17。9.1）。実施は F1**:

| 現在の名前（本書） | 改名案 | 理由 |
|---|---|---|
| `docs/floor_touch_plan.md`、`docs/floor_touch_real_check.md` | `docs/zero_offset_calib_plan.md`、`docs/zero_offset_calib_real_check.md` | 題名で何のキャリブか分かる |
| `tools/floor_touch_run.py`、`tools/floor_touch_fit.py` | `tools/zero_offset_record_run.py`、`tools/zero_offset_fit_run.py` | camera の `camera_record_run.py`・`camera_extrinsics_run.py` と同じ命名。center の `so_arm101_calib` と混ざらない |
| `so_arm101_calibration/floor_touch.py` | `so_arm101_calibration/zero_offset.py`（と `zero_offset_record.py`） | モジュール名で zero_offset_deg と結びつく |
| CSV `floor_touch_YYYYMMDD.csv`、材料の置き場 `~/so_arm101_check/floor_touch_YYYYMMDD/` | `zero_offset_YYYYMMDD.csv`、`~/so_arm101_check/zero_offset_YYYYMMDD/` | 同上 |

  決定: 改名する。F1 で `git mv docs/floor_touch_plan.md docs/zero_offset_calib_plan.md`（作業ツリーを変える git の操作。F1 の承認の項目）と本書の全置換を 1 回で行う。`docs/floor_touch_real_check.md`・`tools/`・モジュール・CSV はまだ無いので、新しい名前で作る（`git mv` は計画書の 1 件だけ）。**既存の材料 `~/so_arm101_check/floor_touch_20261117/`（git 管理外。3.4.6 の sha256）は動かさず、名前も変えない**（本書の 3 章に出る名前はそのまま）。新しい記録の置き場は `~/so_arm101_check/zero_offset_YYYYMMDD/`。F1 で本文を全置換した（この節の旧名の記述と、既存の材料のパスだけが floor_touch のまま）。
- 道具の最初の表示: 起動時の最初の行に `[zero-offset calibration] zero_offset_record_run: records touch poses for zero_offset_deg (NOT the center calibration: center/min/max are decided by so_arm101_calib measure/apply). Read-only: nothing is written.`。fit は最初の行に `[zero-offset calibration] zero_offset_fit_run: result for zero_offset_deg (display only; the yaml is not written)`。最後の判定の行にも `[zero-offset calibration]` を付ける。
- CSV の最初の行は `# zero-offset calibration (floor touch). Not the center calibration.`（`#` で始まる行は読み込みで飛ばす。試験 1 件）。meta に `calibration_kind: zero_offset`。fit は `calibration_kind` が違う meta を拒否する。
- yaml のコメント案（J5 で手で入れるとき。`zero_offset_deg` を入れる関節の行の上、`joints:` の直前）:

```
# zero_offset_deg (degrees, per joint) = result of the zero-offset calibration (floor touch; docs/zero_offset_calib_plan.md).
#   true angle = direction*(raw-center)*360/angle_ticks + zero_offset_deg.
#   NOT the center calibration (center/min/max are decided by so_arm101_calib measure/apply).
#   Redo the zero-offset calibration whenever the center calibration is redone.
```

  `so_arm101_calib apply` は yaml のコメントを固定の文面で作り直す（`apply.cpp:57-59`）ので、上のコメントは次の apply で消える（値は J3 の持ち越しで残る）。**決定（ユーザー、9.1）**: apply が作る yaml の先頭のコメント（固定の文面）に、次の 2 行を足す（C++ の小さな変更。段 F1b）:

```
# zero_offset_deg は オフセットキャリブ（zero-offset calibration。docs/zero_offset_calib_plan.md）の値。
# center・min・max・eeprom などは センターキャリブ（so_arm101_calib measure・apply）の値。
```

  足す場所は `apply.cpp:58-59` の 2 行目（`違うと起動を拒否する）。`）の直後、`schema_version:` の前。コメントだけで、yaml の中身（キー・値・順序）は変わらない。F6 で本番の yaml に手で入れるコメント（上の 4 行）の先頭にも、この 2 行を同じ文面で入れる（apply がやり直しても同じ文面になる）。`inventory.md` の J5 の節に控える。
- 既存の docs の呼び名: **決定（ユーザー、9.1）** 実装の段 F5 で、`so_arm101_calibration_check.md` と `joint_offset_plan.md` の冒頭に用語の注記を足す（本文は直さない。J4d と同じ注記のやり方）。文面の案: 「この文書の『キャリブ』『校正』『再校正』はセンターキャリブ（center calibration）を指す。オフセットキャリブ（zero-offset calibration）は `docs/zero_offset_calib_plan.md`。」`camera_stage_plan.md` には 6.3 の注記（手順の順序）を足す。

### 0.3 要点

1. 道具は 2 つ。`tools/zero_offset_record_run.py`（記録。`/joint_states` を購読するだけ。読み取り専用の起動で使う）と `tools/zero_offset_fit_run.py`（オフラインの当てはめ。CSV から δ を出し、yaml に貼る行を**表示するだけ**）。純粋な計算は `so_arm101_calibration` の Python モジュール（5.1）。
2. モデル: 1 回の記録 = 「その姿勢で固定あごの一番低い頂点の z = 机の高さ h」という 1 本の式。未知数は δ_lift・δ_elbow・δ_wrist と h の 4 つ（h は事前分布つき）。pan は面内の回転なので式に効かず、roll は高さに効かない（記録だけする）。
3. **最大の注意**: 先端の傾きが一定の点だけだと、wrist の列と h の列が同じになり、δ_wrist と h を分けられない。傾きを変えた点（3 種類）が要る。3 章の結果: 4 姿勢で sd は lift 0.32、elbow 0.51、wrist 0.69°（当て方 1 mm、σ_h 1 mm）。90° の点が 1 mm ずれると wrist に約 0.7° の偏りが出る。
4. 4 姿勢 + h の事前分布 = 5 本の方程式 − 未知数 4 = 余り 1。3 姿勢は余り 0 で、残差の検査も 1 姿勢ずつ外す検査もできないので不採用（3.4.4）。
5. 合否は 4 章。yaml は道具が書かない（6.1）。合格した関節の貼る行（0.1° 刻み）と現在の yaml との差を表示する。
6. 試験は主な守りに絞り、pytest 約 82・gtest +2・lint 最大 +9（7.1）。コミットは F0〜F6（F1b を含む。7.4）。J5 は「δ = −6.3° で始めて物差しで確定」から「この道具で測った値を yaml に入れる」に変わる（6.3）。
7. 実機のコマンドは私は実行しない。実機の手順書は 7.5 の骨組み（F5 で書く）。

## 1. 事実（読んだもの）

- 換算: 真の角度 = direction × (raw − center) × 360 / angle_ticks + `zero_offset_deg`（`joint_offset_plan.md` 2.1、3.1）。`follower_calibration.yaml` に `zero_offset_deg` はまだ無い（全関節 0）。center は lift 2023.5、elbow 1969.5、wrist_flex 1991（`follower_calibration.yaml:17-19`）。読み込みの上限は 10°（J1）。
- `/joint_states` を購読するだけのツールの前例: `tools/camera_record_run.py`（記録）、`tools/pose_margin.py`（オフライン）。記録の識別（yaml の sha256・`zero_offset_deg`・URDF の sha256）は `so_arm101_camera/record.py` の `make_identity`（`:277`）と `read_zero_offsets`（`:287`）が持つ。
- 一番低い頂点の計算の前例（スクラッチ。読むだけ）: `~/so_arm101_check/camera_20261116_C3/lowest_point.py`。URDF（install の `so101_new_calib.urdf`）の visual のメッシュ（STL。バイナリ）の全頂点を FK で base_link に移し、z の最小を取る。床 = −0.0024 m（取り付け面）。固定あごは gripper_link の `wrist_roll_follower_so101_v1.stl`（と `sts3215_03a_v1.stl`）。
- 出発点の物差し: 一番低い点が机から約 29 mm（±3 mm 程度。ユーザー）。補正なしの予測 4.8 mm、elbow −6.31° で 31.3 mm、1° あたり約 4.3 mm（`inventory.md` の C3 の節の 3）。
- 既存の δ の推定（独立の相手）: カメラの当てはめ (g) は lift −0.14 ± 1.76、elbow −6.31 ± 0.38、wrist_flex −6.44 ± 3.45、roll −0.57 ± 3.70 度（線形化。概算）。物差しの elbow は −5.77（−5.07〜−6.47）。
- 読み取り専用の起動: `ros2 launch so_arm101_bringup bringup.launch.py mock:=false read_only:=true use_rviz:=false`（`docs/camera_c2_real_check.md:208`）。ユーザーが実施。
- `so_arm101_calibration`（C++。`ament_cmake`）: `CMakeLists.txt` は `ament_cmake`・`so_arm101_hardware`・`ament_index_cpp`・`yaml-cpp` を探し、ライブラリ `so_arm101_calib_core` と実行ファイル `so_arm101_calib`（`ros2 run so_arm101_calibration so_arm101_calib`）を作る。試験は gtest だけ（`ament_cmake_gtest`）。`package.xml` の依存は hardware・ament_index_cpp・yaml-cpp。Python は無い。`include/so_arm101_calibration/` はヘッダ用で、Python のパッケージ名と同名のディレクトリを別に（パッケージ直下に `so_arm101_calibration/`）作っても衝突しない（推測。F2 のビルドで確認）。
- camera・ik の `CMakeLists.txt`: `ament_cmake_python` と `ament_python_install_package(${PROJECT_NAME})`、試験は `ament_add_pytest_test` を 1 本ずつ登録（camera の `:21-37`）。ik・description・hardware はどれも calibration に依存していない（依存の向きは 5.1）。
- camera の道具の終了コードの前例: 0 合格、2 不合格または拒否（`camera_extrinsics_run.py` の docstring）。

## 2. モデルと測定の式

### 2.1 1 回の記録の式

- 記録 i の関節角 q_i（lift・elbow・wrist_flex。pan・roll は FK に入れるが未知数ではない。グリッパーは固定あごのモデルに効かない）。δ = (δ1, δ2, δ3) と h のもとで、予測の高さ z_i(δ) = **固定あご（gripper_link の visual のメッシュ）の全頂点の base_link の z の最小**（姿勢は q_raw0,i + δ）。残差 r_i = z_i(δ) − h。
- 当てはめは重みつきの最小二乗: Σ (r_i / σ_i)² + ((h − h0) / σ_h)²。未知数 4、h0 = −2.4 mm、σ_h = 1 mm（既定）。σ_i = 1.0 mm（当て方のばらつき）。90° の記録の上乗せは既定 0（F4b。設計の検討 3.4.3 は √(1.0² + 1.0²) = 1.41 mm で計算したので、その sd は保守側）。
- q_raw0 = q_state − δ_old（δ_old は、記録のとき動いていたドライバの yaml の `zero_offset_deg`。meta に入れる）。結果は**絶対値**の δ として出し、現在の yaml との差も出す。δ_old = 0 の yaml（今）では q_raw0 = q_state。
- 動くあごと手首のメッシュはモデルに入れない。動くあごが先に机に当たる姿勢は無い前提（90° は両あごの差が 1 mm 以下という確認で扱う）。
- 数値の扱い: ヤコビアンは有限差分（刻み 1e-4 rad）、ガウス・ニュートンで 6 回程度。メッシュの最小は区分的に滑らかで、一番低い頂点が切り替わる所で折れる（3.4.2: 直線からのずれは 2° 動かして最大 0.14 mm）。
- 余り（自由度）: 4 姿勢 + h の事前分布 1 行 = 5 本 − 未知数 4 = 1。記録 8 回のうち、同じ姿勢の 2 回目が追加の 4 自由度（当て方のばらつきだけを見る）。

### 2.2 ヤコビアンの見立て（3.4 で確かめた）

- 面内の腕では、δ_j が高さを動かす量は「関節 j から先端までの水平距離」d_j（rad あたり）。h の列は定数（−1）。
- 傾き一定の点では d_wrist がほぼ定数 → h の列と平行（3.4.1: 45° のみでは決まらない）。傾きを変えれば分かれる（3.4.2: 80° + 45° などで wrist sd 0.6〜1.7°）。
- 90° の点は d_wrist が小さく、wrist の情報は 70° と 50° の点から来る。90° の点は x を大きく変える（100 と 300 mm）役割で、lift と elbow を分ける。
- 未モデルの系統誤差: 先端の位置のずれ（メッシュと実物の差、3D プリントの寸法、すり減り）。3.4.2: 両あごが工具軸方向に 1 mm ずれると elbow +0.41、wrist −0.40°（h 固定・事前分布つき）。90° の点が 1 mm ずれると wrist に約 +0.7°（3.4.3）。

### 2.3 当て方の下限（ε_min = 45°）

- 道具は記録の時に、姿勢から ε を FK で出して表示し、ε < ε_min（45°）なら記録しない（行を書かない）。**記録のときの ε は実物の ε（読んだ角度 + `--expected-offset`）。fit のときの検査（ε_min と傾き 3 群）は読んだ値の ε のまま**（F4b、ユーザーの確認: 全点が同じだけずれるので 3 群の間隔は変わらない。ε_min は fit のほうが |δ| だけ甘くなる）。fit の表示に、meta の expected offset を 1 行出す。決定（ユーザー）: 45° を守りとして残す。4 姿勢の傾き 90・70・50° はどれも 45° 以上で、目標の傾きの許容 ±5° でも 45° 以上に収まる。
- 理由: 45° 未満は先端の側面や角で当たりやすく、3.4 の使える範囲（x 220 以上）も狭い。30° は届かない（3.4.1）。

## 3. オフラインの検討（スクラッチ。実機なし。完了。結果は 3.4）

### 3.1 目的と入力

- 目的: 姿勢の組と点数を決める（届くか、3 関節と h が分かれるか、当て方のばらつきでの σ）。
- 入力（すべて読むだけ）: install の URDF と meshes、`follower_calibration.yaml`（中心・範囲）。出発点（R1）の関節角は `lowest_point.py` の値をコードに写した。
- 出力先: `~/so_arm101_check/floor_touch_20261117/`（git 管理外。リポジトリには書かない）。ログと CSV だけ。sha256 は 3.4.6。
- 仮の δ の真値（合成データ用）: 1 回目は (0, −6.3, −3)°、2 回目以降は (0, −6.3, 0)°。h = −2.4 mm。

### 3.2 方法の要約

- S1（`reach_scan*.py`）: 面内の 3 関節の逆問題を数値で解き、(傾き, x) ごとに届くか（URDF のリミットと yaml の測定範囲）、固定あごと動くあごの最低点の差、手首との差を出す。x は 100〜330 mm を 10 mm 刻み、pan は 0。
- S2・S3（`design_study*.py`）: 点の組ごとのヤコビアン、条件数、相関、δ の標準偏差（当て方のばらつき 0.5・1・2 mm、h の扱い 4 通り）、当て直し、x の範囲、非線形の戻り、Monte Carlo。
- `nuisance_study*.py`: あごのずれによる δ の偏り、頂点の切り替わり。

### 3.3 実行の記録（Python は 9 回、各 1 回）

reach_scan、design_study、nuisance_study（1 回目）、reach_scan_2、design_study_2、nuisance_study_2（2 回目）、design_study_3（4〜6 姿勢）、design_study_4（3 姿勢）、design_study_5（85・70・50° の 4 姿勢。F4b の前、3.4.7）。承認を得て、両方の setup.bash を source し、`python3 <絶対パス> > <script>.log 2>&1` で実行した。ネットワーク・シリアル・カメラは使っていない。乱数の種は固定。経緯と結果は 3.4。

### 3.4 結果（2026-11-17。実行した事実と、そこからの推測。実機・シリアル・カメラ・launch は使っていない）

「事実」はログに出た値、「推測」は私の読み。仮の真の δ とモデルの前提は各節に書く。スクリプトは `~/so_arm101_check/floor_touch_20261117/`（git 管理外）。すべて終了コード 0。Python の実行は次の 9 回だけ（各 1 回）: reach_scan、design_study、nuisance_study、reach_scan_2、design_study_2、nuisance_study_2、design_study_3、design_study_4、design_study_5。

#### 3.4.1 1 回目（reach_scan・design_study・nuisance_study。ログは `*.log`、CSV は `reach_points.csv`・`design_sets.csv`）

- R1 の自己確認（事実）: 補正なしで一番低い頂点 4.8 mm（固定あご）、基準点の x 196.22 mm。`lowest_point.log` と一致。軸の符号は +1。
- 仮の δ は (0, −6.3, −3)°。wrist の −3° は裏づけのある値ではなく、昨日の当てはめ（−6.44 ± 3.45°）のおよそ半分を「大きめの値」として置いた仮定（推測）。2 回目で 0 に直した（物差しで elbow だけが合ったため）。
- 判定の誤り（事実）: 「他リンクと 3 mm 以上」の 1 つの条件で、真下（90°）と後傾を、動くあごが先に当たる姿勢として除外した。使えたのは前傾 75°（x 120〜330）、60°（x 170〜330）、45°（x 220〜330）だけ。前傾 30° は URDF の elbow 上限（1.69 rad）を超えて届かない。後傾は動くあごが先に当たるか、リミット外。
- design_study の中心の計算は、真下を含む組が全部スキップされ、組 B（45° だけ）しか出なかった。nuisance_study の組の部分も全部スキップ。頂点の切り替わりの部分（前傾 45・60・75°）だけが有効だった。
- 組 B の h 自由の値は**無効**（私のスクリプトの誤り）: `pinv`（rcond 1e-13）がほぼ特異な方向を落とし、wrist の sd が 0.06〜0.23° と見かけ上小さく出た。h と wrist の相関は +1.00 で、実際は決まらない。
- h の事前分布つきが固定と同じ値になったのも**誤り**（私のスクリプトの誤り）: h の真値が事前分布の平均に等しいという前提の「ばらつき」だけを計算していて、事前分布の誤差が入っていなかった。
- 45° のみでは elbow と wrist が分かれない（elbow と wrist の相関 −0.99。h を固定しても wrist の sd 約 5〜10°）。傾きを混ぜる必要がある（事実。1 回目の組 B）。

#### 3.4.2 2 回目（`reach_scan_2.py`・`design_study_2.py`・`nuisance_study_2.py`。ログ・CSV は `*_2.*`）

直した点（事実）:
- 傾きを前傾 90° から 45° まで 5° 刻みに。後傾は省いた。あごの判定を、他リンクとの 3 mm（手首）と、固定あごと動くあごの最低点の差（jaw_diff。使える条件は固定あごが 1.0 mm 以上低い。推測の値）に分けた。
- `pinv` をやめ、特異に近い設計は「UNDETERMINED（条件数・最小特異値。条件数 1e5 超）」と表示する。
- 事前分布つきは、事前分布の誤差を含む全体の共分散にした。順序の検査（sd は h 自由 ≥ prior2 ≥ prior1 ≥ 固定）は全組で失敗 0。Monte Carlo では真の h を事前分布から引く。
- 点の組は、使える傾きから自動で作る。組の作成に失敗したら理由を出して終了コード 3。

使える傾きと x の範囲（事実。仮の δ は (0, −6.3, 0)°）:

| 傾き | 使える x（mm） | 点数 | 固定あごが低い量（mm） | 使えない範囲 |
|---|---|---|---|---|
| 90° | なし | 0 | −1.26 | 動くあごが低い。x 310 以上は解なし |
| 85° | なし | 0 | +0.67 | 1.0 mm 未満 |
| 80° | 110〜330 | 23 | 2.60 | x 100 は URDF リミット外 |
| 75° | 120〜330 | 22 | 4.53 | x 100〜110 は URDF リミット外 |
| 70° | 140〜330 | 20 | 6.45 | x 100〜130 は URDF リミット外 |
| 65° | 150〜330 | 19 | 8.32 | x 100〜140 は URDF リミット外 |
| 60° | 170〜330 | 17 | 10.15 | x 100〜160 は URDF リミット外 |
| 55° | 190〜330 | 15 | 11.92 | x 100〜170 は URDF リミット外、180 は yaml 範囲外 |
| 50° | 210〜330 | 13 | 13.42 | x 100〜150、190 は URDF リミット外、160〜180 は解なし、200 は yaml 範囲外 |
| 45° | 220〜330 | 12 | 15.22 | x 100〜160、200 は URDF リミット外、170〜190 は解なし、210 は yaml 範囲外 |

jaw_diff は傾きを浅くするほど大きくなる（動くあごが上がる）。

組ごとの δ の標準偏差（事実。当て方のばらつき 1 mm、n=8、度。lift, elbow, wrist）:

| 組 | 内容 | h 自由 | prior2（σ_h 2 mm） | prior1（σ_h 1 mm） | h 固定 |
|---|---|---|---|---|---|
| A | 80° のみ | 決まらない | 0.31, 2.85, 23.9 | 0.31, 2.85, 23.2 | 0.31, 2.85, 23.0 |
| B | 45° のみ | 決まらない | 0.78, 5.83, 9.74 | 0.78, 5.83, 9.70 | 0.78, 5.83, 9.68 |
| C | 80° + 45° | 0.35, 3.00, 1.69 | 0.35, 0.92, 0.72 | 0.35, 0.61, 0.61 | 0.35, 0.44, 0.57 |
| D | 80° + 60° + 45° | 0.32, 2.03, 1.40 | 0.30, 0.83, 0.84 | 0.30, 0.55, 0.75 | 0.30, 0.40, 0.72 |
| E | 75° + 45° | 0.36, 3.13, 1.61 | 0.36, 0.88, 0.76 | 0.36, 0.59, 0.69 | 0.36, 0.44, 0.67 |
| F | 80・75・60・45° | 0.26, 2.83, 1.63 | 0.25, 0.80, 0.79 | 0.24, 0.50, 0.72 | 0.24, 0.34, 0.70 |
| G | 80° + 60° | 0.33, 2.25, 1.96 | 0.31, 0.88, 1.12 | 0.31, 0.59, 1.00 | 0.30, 0.43, 0.95 |
| H | 60° + 45° | 0.49, 2.70, 1.32 | 0.45, 0.87, 1.27 | 0.45, 0.71, 1.27 | 0.45, 0.65, 1.27 |

- 0.5 mm と 2 mm の値は 1 mm の値のほぼ 1/2 と 2 倍（ログに全部）。相関（組 C、h 自由、n=8）: elbow と wrist −0.96、h と wrist +0.94。h 固定: lift と elbow −0.81、elbow と wrist −0.49。
- 点数の効き（組 C、h 固定、1 mm）: n=6、8、10、12、16 で elbow の sd 0.47、0.44、0.41、0.39、0.35。
- 当て直し（組 C、h 固定、1 mm、姿勢ごとの系統誤差 0 / 0.5 / 1 mm）: 8 姿勢 × 1 回 (0.35, 0.44, 0.57) / (0.40, 0.50, 0.64) / (0.50, 0.63, 0.81)、6 姿勢 × 2 回 (0.26, 0.33, 0.46) / (0.32, 0.41, 0.57) / (0.45, 0.58, 0.80)。姿勢の数が少なすぎる（4 姿勢 × 2〜3 回、h 自由）と elbow が 10° 超に悪化して決まらない。
- x の範囲を狭めたとき（組 C、n=8、h 固定）: 幅 40 mm で (2.44, 3.73, 2.03)、80 mm で (0.96, 1.50, 1.00)、120 mm で (0.73, 1.16, 0.87)、200 mm で (0.56, 0.83, 0.68)、400 mm で (0.35, 0.44, 0.57)。幅が 120 mm 以下だと h 固定でも elbow が 1° 前後以上になる。
- ノイズなしの非線形の戻り: 3 通りの h の扱いで δ = (0, −6.3, 0)° に戻った（最大の残差 0.0001 mm）。δ_old の引き戻しは差 0。
- 線形と Monte Carlo（100 回、1 mm、n=10）: 組 C h 自由 MC sd (0.31, 3.34, 1.95) 対 線形 (0.33, 2.90, 1.62)（偏り elbow −0.85°）、組 C prior2 (0.33, 0.91, 0.75) 対 (0.33, 0.92, 0.67)、組 F h 自由 (0.25, 1.73, 1.24) 対 (0.26, 1.82, 1.27)、組 F prior2 (0.26, 0.71, 0.85) 対 (0.25, 0.79, 0.78)。よく一致。
- あご 1 mm のずれによる δ の偏り（事実。両あごを工具座標でずらした。1 mm の値）: 工具軸方向は h 固定・prior2 で (lift, elbow, wrist) = (0.00, +0.41, −0.40)°、h 自由では h に吸収され (0.00, −0.00, −0.18)°（h が +1.04 mm 動く）。工具 x 方向は wrist +0.33°。工具 y 方向は −0.02°。傾きの組によらずほぼ同じ。
- 頂点の切り替わり（事実）: 高さの直線からのずれは最大 0.009 mm（0.5° 動かしたとき）と 0.14 mm（2° のとき）。最小の頂点は傾き 65・60・45° で 0.5° の動きの半数で切り替わる。最小から 0.1 mm 以内に約 84〜96 個、0.5 mm 以内に約 400〜460 個の頂点がある（辺で当たる形）。最小の頂点は gripper_frame でおよそ (+3.4〜5.0, 0, +5.4〜6.3) mm。

#### 3.4.3 design_study_3（4・5・6 姿勢、各 2 回。90・70・50° と、比較の 80・60・45°）

ユーザーの追加の判断（実機で確認）: 真下で両あごの先の高さの差は実物で 1 mm 以下。90° を使える傾きに入れる（モデルは固定あごの最低点のまま。90° の点だけ σ を 1 mm 上乗せ。二乗和で √2 = 1.41 mm とした。「2 mm」と読む場合も別に計算）。
- 探索: 3 種類の傾きを各 1 姿勢以上含む全組み合わせを、x の 20 mm 刻みで総当たりし、最良を 10 mm 刻みで 1 姿勢ずつ動かして仕上げた。目的関数は elbow と wrist の sd の大きい方（h の事前分布 1 mm、当て方のばらつき 1 mm）。

選ばれた組（90・70・50°。x は mm。sd は度）:

| 姿勢数 | 選ばれた組 | sd lift / elbow / wrist（σ_h 1 mm） | σ_h 2 mm | 系統誤差 0.5 mm | 90° が 2 mm のとき |
|---|---|---|---|---|---|
| 4 | 90°@100、90°@300、70°@150、50°@250 | 0.32 / 0.51 / 0.69 | 0.33 / 0.81 / 0.79 | 0.37 / 0.56 / 0.79 | 0.41 / 0.53 / 0.83 |
| 5 | 90°@290、90°@300、70°@140、50°@270、50°@280 | 0.28 / 0.53 / 0.62 | 0.29 / 0.80 / 0.71 | 0.33 / 0.60 / 0.72 | 0.31 / 0.53 / 0.73 |
| 6 | 90°@100、90°@290、90°@300、70°@150、50°@260、50°@270 | 0.26 / 0.50 / 0.55 | 0.27 / 0.80 / 0.67 | 0.30 / 0.54 / 0.63 | 0.32 / 0.52 / 0.66 |

比較（80・60・45°）:

| 姿勢数 | 選ばれた組 | sd lift / elbow / wrist（σ_h 1 mm） | σ_h 2 mm | 系統誤差 0.5 mm |
|---|---|---|---|---|
| 4 | 80°@120、80°@330、60°@250、45°@260 | 0.27 / 0.53 / 0.63 | 0.27 / 0.86 / 0.68 | 0.33 / 0.59 / 0.76 |
| 5 | 80°@120、80°@330、60°@170、45°@250、45°@260 | 0.23 / 0.44 / 0.54 | 0.24 / 0.69 / 0.58 | 0.29 / 0.49 / 0.66 |
| 6 | 80°@120、80°@320、80°@330、60°@170、45°@260、45°@270 | 0.20 / 0.44 / 0.50 | 0.21 / 0.69 / 0.54 | 0.24 / 0.49 / 0.60 |

- 目的関数の最良値（度）: 4 姿勢 A 0.690 対 D 0.628、5 姿勢 0.619 対 0.542、6 姿勢 0.554 対 0.498。D の方が約 0.06〜0.08° 良い。ただし D の 80° は固定あごが動くあごより 2.6 mm 低い傾きで、A の 90° は動くあごが 1.26 mm 低い（実物の差は 1 mm 以下という判断に頼る）。
- 1 回外したときの変化の見込み（最悪の 1 回、lift / elbow / wrist、度。A）: 4 姿勢 0.25 / 0.23 / 0.40、5 姿勢 0.22 / 0.38 / 0.27、6 姿勢 0.13 / 0.23 / 0.18。
- 90° の点が +1 mm ずれたときの偏り（線形、度。lift, elbow, wrist）: 4 姿勢 −0.13 / −0.16 / +0.68、5 姿勢 −0.20 / −0.06 / +0.54、6 姿勢 −0.13 / −0.16 / +0.65。90° の誤差が最も効くのは wrist（約 0.7°）。
- 相関（A、4 姿勢、σ_h 1 mm）: lift と elbow −0.38、elbow と wrist −0.60、elbow と h −0.75、wrist と h +0.34。
- 線形と Monte Carlo（100 回。90° の点を +1 mm、真の h は事前分布から引く）: 4 姿勢の平均のずれ (−0.14, −0.21, +0.69)（線形の偏り (−0.13, −0.16, +0.68)）、MC sd (0.37, 0.50, 0.69) 対 線形 (0.32, 0.51, 0.69)。5 姿勢 (−0.19, −0.08, +0.59) 対 (−0.20, −0.06, +0.54)、MC (0.29, 0.57, 0.68) 対 線形 (0.28, 0.53, 0.62)。6 姿勢 (−0.13, −0.19, +0.63) 対 (−0.13, −0.16, +0.65)、MC (0.24, 0.50, 0.58) 対 線形 (0.26, 0.50, 0.55)。線形の予測と非線形の結果はよく一致した。

#### 3.4.4 design_study_4（3 姿勢、各 2 回。不採用）

- 余り（自由度）が 0（事実）: 6 回の当てはめは、異なる姿勢 3 つ + h の事前分布 1 行 = 方程式 4 本に対し未知数 4 つ。姿勢に由来する余りは 0。追加の 3 自由度は同じ姿勢の繰り返しから来るだけで、姿勢ごとの誤差（メッシュと実物の差、あごの当たる点、モデルの誤り）は δ と h に吸収され、残差に出ない。**残差によるモデルの検査も、1 姿勢ずつ外す検査もできない**（外すと 3 本で未知数 4 つ、決まらない）。
- 結果（σ_h 1 mm、1 mm）: 90・70・50°（90°@300、70°@140、50°@270）は sd (0.32, 0.55, 0.81)、目的関数 0.807。80・60・45°（80°@310、60°@170、45°@280）は (0.31, 0.55, 0.80)、0.795。系統誤差 0.5 mm を足すと wrist が 0.95〜0.97° で基準 1.0° に近い。Monte Carlo は線形と一致（A: 平均のずれ (−0.24, +0.01, +0.47) 対 線形の偏り (−0.20, −0.06, +0.51)）。
- 4 姿勢との差は小さい（目的関数で約 0.12° 悪いだけ）が、検査ができないため**不採用**。

#### 3.4.5 決定（ユーザー、2026-11-17）

- **4 姿勢（各 2 回、計 8 回）に決める。** 傾きは 90・70・50° を第一候補にする。選ばれた x（A、4 姿勢）: 90°@100、90°@300、70°@150、50°@250（目標の x。当てる位置の許容は ±10 mm）。4 姿勢 + h の事前分布の 5 本の方程式 − 未知数 4 = 余り 1（残差の検査と、1 回外した検査ができる）。
- h の扱い: 事前分布つき（σ_h 1 mm を主、2 mm を参考）。h 自由は決まらない（sd 約 2〜3°）。
- ε_min は 45°（守りとして残す）。90° は固定あごのモデルのまま使う。（当初は 90° の記録だけ σ に 1 mm を二乗和で上乗せしたが、F4b で既定 0 に変えた。0.1 の 2。）
- 決定の反映: 0 章（0.1）、2.3、4 章、5 章、6 章、7 章、8 章、9 章に反映済み（2026-11-17 の改訂。元の記述は残していない）。

#### 3.4.6 材料の sha256（`~/so_arm101_check/floor_touch_20261117/`。git 管理外。`__pycache__` の `.pyc` も同じ場所）

| ファイル | sha256 |
|---|---|
| reach_scan.py | 4304460a69baa473ece71fc2773b0ff49af3f88e05919256b1e03d36dfa0f021 |
| design_study.py | 6d2800788561e0870bcbc4bbd32cc2870d8c4d83fb290c0f49796c8945d6d274 |
| nuisance_study.py | 40e27ea426ad38c53524ed59c2a41ffdd105f767475757282beadd93e15ee937 |
| reach_scan.log | 7ae96de633b3ddf6179a1874b35f16c1613bf0a3b6ffdbc4a8433f4ce7a077a9 |
| design_study.log | ee10062e43897d6b641f4db642a00d964e0da82e0b09665bf1eaaa00ecdd2c89 |
| nuisance_study.log | c8b84d244dd137d1247732aefe2bf76ce6e42a3206d7a02f14e2c3fcf8ed3002 |
| reach_points.csv | a072527dd765c2d7a9b4a5114db3783215a43191e2abeb6697d5e5fa558a2479 |
| design_sets.csv | 342553b81d7728c118ef1fe52455a0a7b03a2df7ffea44115d369f58b5090890 |
| reach_scan_2.py | c891c87be32a3f7763927053c49eba60514682cb6ef6e03022531f1657e503b1 |
| design_study_2.py | e0b7eec94b34d6da68f96303902d60147fb3d4a515ccfe5ceceaea6884c63790 |
| nuisance_study_2.py | 438b3e08f94af8c3a346139db2f87d0f98ec877fa0f2a2d726b8babaf7abe164 |
| reach_scan_2.log | 6b8deb43c7b91ef5515719a0b3e1f6149c3319ea7c10c64407755fd7904f4cf1 |
| design_study_2.log | 457283cc56f30a6e772fd938cf1b6494d6475c13853594fee5f9e26cefd616dc |
| nuisance_study_2.log | 39300af972cd5b239c5a2ca0b90e93014f9ec0dc85d89dbeab4096041e5fc2f3 |
| reach_points_2.csv | 4854858f02e64dbee90d6824cb7718760dd421fee86aa0ecef8ceb4143bc641b |
| design_sets_2.csv | 8c779db1e10681c07dcdf7087ce3904b93d8c3f2806af2fc1d3794c385a37ae7 |
| design_study_3.py | 9ad74384927c355ccb04e51be6fae57e02ac929798c32a2c1e89f366c32c28b8 |
| design_study_3.log | 77f50dd2af4b6916d89c2f8b7f5931112e9bdf54760fdee5454318517017f04b |
| design_study_4.py | f7d3f8d9de9f19f95fe8fe4b25074192f390f332d4f795e2e34bb0810f803b3e |
| design_study_4.log | 81150a2c2368ad641ddb600f4a7eb955d10d70fed4c0ab035a3b88598a449c75 |
| design_study_5.py | eb830c8dc744b0c4c60f54bad4aeca3ccdfe3bf04abaa0f85c59094e7b9d0228 |
| design_study_5.log | 176e72d4e4435506535ac5170e738f032aa3cbca1ee2bc0ade092bcb20db29be |
| `__pycache__/reach_scan.cpython-310.pyc` | 3ba2803ae055a4e742d3e8eea2f7124b38dcb1ee5ae8afcf933243ddec152ecd |
| `__pycache__/design_study.cpython-310.pyc` | 18f401395cc5e071ec0c8d845c4dde3e7830b8c0cce47ee4af564dba107e927c |
| `__pycache__/reach_scan_2.cpython-310.pyc` | 8a445fd6e6fac02a01f4c90391426977be4f43b585369811a99619ef90800d70 |
| `__pycache__/design_study_2.cpython-310.pyc` | 76cba74d4e235cc2586abcc791a6e795d2b4d5c80564d23d31f657a7eff8fbb8 |

- 注: `design_study_3`・`design_study_4`・`nuisance_study_2` の `.pyc` はできていない（スクリプトとして直接実行し、import されなかったため）。
- 注: 補助の sed スクリプト `/tmp/claude-1000/ed.sh`（reach_scan_2.py のファイル名と δ の置換だけ。実行済み）を `/tmp` 直下に作った（scratchpad でない）。リポジトリ・材料には影響しない。
- 注: `design_study_5` も import されていない（直接実行）ので `.pyc` は無い。`reach_scan_2` の `.pyc` は design_study_5 が import したが、sha256 は変わっていない（上の値のまま）。
- 進め方の記録: 確認のコマンドに Python は混ぜていない。実行は上の 9 回だけ。

#### 3.4.7 design_study_5（85・70・50° の 4 姿勢を固定。検討の記録。F4b の前に 1 回実行）

目的（ユーザー）: 真下の 2 姿勢を実物 85°（固定あごが 0.67 mm 低い）にしたときの精度を、design_study_3 の 90・70・50° と並べる。design_study_3.py のコピー。姿勢を固定（85°@100、85°@300、70°@150、50°@250。各 2 回）。σ_h 1 mm。85° の点に σ の上乗せはしない（B）。比較は 90° の σ 1.41 mm の A。終了コード 0、スキップなし。ログ `design_study_5.log`。

| 項目 | B: 85・70・50°（σ 1.0） | A: 90・70・50°（90° の σ 1.41） |
|---|---|---|
| sd lift / elbow / wrist（σ_h 1 mm） | 0.25 / 0.48 / 0.65 | 0.32 / 0.51 / 0.69 |
| sd（σ_h 2 mm） | 0.25 / 0.80 / 0.75 | 0.33 / 0.81 / 0.79 |
| 姿勢ごとの系統誤差 0.5 mm | 0.31 / 0.52 / 0.79 | 0.37 / 0.56 / 0.79 |
| 先頭の点の σ を 2 mm にしたとき | 0.43 / 0.53 / 0.93 | 0.41 / 0.53 / 0.83 |
| 相関 elbow と wrist | −0.58 | −0.60 |
| 相関 elbow と h | −0.81 | −0.75 |
| 相関 wrist と h | +0.34 | +0.34 |
| 1 点外した変化の最悪 lift / elbow / wrist | 0.19 / 0.16 / 0.43 | 0.25 / 0.23 / 0.40 |
| 先頭の点が +1 mm ずれたときの偏り（線形） | −0.10 / −0.25 / +0.83 | −0.13 / −0.16 / +0.68 |
| 同じ偏り（Monte Carlo 100 回の平均のずれ） | −0.09 / −0.30 / +0.82 | −0.15 / −0.07 / +0.60 |
| Monte Carlo の sd | 0.29 / 0.48 / 0.65 | 0.32 / 0.53 / 0.65 |

- 事実: sd は B が 0.03〜0.07° 良い。ただし σ の条件が違う（B は 1.0、A は 1.41）。1 mm ずれの偏りは B のほうが大きい（wrist +0.83° 対 +0.68°）。
- 決定（ユーザー、F4b）: 目標は 90・70・50°（実物）にする。**精度の見込みは A（design_study_3）の値を使う**（上乗せを入れた値なので保守側）。design_study_5（85°）は検討の記録として残す。

## 4. 合否の基準（すべて推測の値。実機の後に見直す。ユーザーの決定、2026-11-17）

当てはめの前に「姿勢の検査」、当てはめのあとに「結果の検査」。

| 項目 | 基準（推測） | 不合格のとき |
|---|---|---|
| 姿勢の数と記録の数 | 姿勢 4 以上、記録 8 以上（各姿勢 2 回） | 拒否（終了コード 2） |
| 傾きの種類 | ε が 10° 以上離れた 3 群（各群に 1 姿勢以上） | 拒否（wrist と h が分かれない） |
| x の広がり | 全記録で最大 − 最小が 150 mm 以上 | 拒否 |
| 姿勢の ε | すべて ε_min（45°）以上 | 記録の時点で拒否（行を書かない）。CSV に有っても fit は拒否 |
| 条件数（列を正規化。h の事前分布の行を含む 4 列） | 1e3 以下（3.4 の事前分布つきの組では 10〜60 台）。超えたら「決まらない」と条件数・最小特異値を表示 | 拒否 |
| 残差 RMS | 1.5 mm 以下 | 不合格 |
| 残差の最大 | 3 mm 以下 | 不合格 |
| δ の標準偏差（モデル上の値と、残差から出した値の大きい方） | lift 0.5° 以下、elbow 1.0° 以下、wrist 1.0° 以下 | 不合格（wrist だけ 1.0° 超のときは、wrist の貼る行を出さない。下の wrist の規則） |
| 1 回外したときの δ の変化（記録を 1 回ずつ外す） | lift 0.5° 以下、elbow 1.0° 以下、wrist 1.0° 以下 | 不合格（外れ値の疑い） |
| 同じ姿勢の 2 回の差 | 2 mm 以下 | 警告（当て方のばらつきの見積りとして表示） |
| δ の絶対値 | 10° 以下（yaml の上限。`joint_offset_plan.md` 2.1） | 不合格 |
| h の推定値 | 事前分布の平均 −2.4 mm から 3 mm 以内 | 警告。5 mm 超は不合格 |
| 現在のモデルでの高さ（記録の時） | 机の高さから 60 mm 以内 | 警告（空中か、δ が大きく外れている） |

- 余り 1（2.1）なので、「1 回外す」検査は記録の 1 回ずつ（同じ姿勢のもう 1 回が残る）。1 姿勢ずつ外す検査は、残りが 3 姿勢 + 事前分布で余り 0 になり、決まらない場合があるので行わない。
- **wrist の貼る行の規則**: δ_wrist の絶対値が 1.5° 以上で、sd が 1.0° 以下のときだけ貼る行を出す。それ以外は貼る行を出さず、「wrist は 0 のまま（理由: 90° の当たり方の 1 mm で約 0.7° の偏りが出るため）」と表示する（1.5° は推測の値）。
- wrist を 0 のままにするとき、lift と elbow の貼る行は、**δ_wrist = 0 に固定して当てはめ直した値**にする（**決定（ユーザー、9.1）**。理由（推測）: wrist を自由にした当てはめの elbow は wrist と −0.6 の相関があり、wrist を 0 にする貼り方とは整合しない）。両方の値（自由にした当てはめと、wrist = 0 に固定した当てはめ）と、**固定した当てはめの sd（lift・elbow）**も表示し、貼る行は固定した方。固定した当てはめの合否（残差・sd・1 回外した変化）も 4 章の基準で判定する。wrist の規則で貼る行を出すとき（1.5° 以上かつ sd 1.0° 以下）は、3 関節を自由にした値を貼る。
- 終了コード（camera の前例にそろえる）: 0 合格、2 不合格または拒否（貼る行を出さない）、1 引数・読み込みの誤り。何も書かない。表示は不合格でも全項目を出してから終了する。
- 警告の扱いは J4d と同じ（合格のまま、「accepted (with N warning)」と表示し、meta に記録）。
- **F2 の実装で決めたこと（推測の値。実機の後に見直す）**: (1) 「90° の記録」（σ に 1 mm を上乗せ）は、状態の傾き ε が 80° 以上のもの（当初の 85° では、δ の分だけ状態の ε がずれたとき外れるため）。(2) 傾き 3 群と ε_min の検査は、記録した状態の ε（表示と同じ値）で行う（当てはめた δ は使わない）。(3) wrist は、規則（1.5° 以上・sd 1.0° 以下）と、1 回外した変化 1.0° 以下と、10° 以内のすべてを満たすときだけ貼る。満たさないときは 0 のまま（不合格ではない）で、lift・elbow は wrist = 0 に固定して当てはめ直した値を、その fit の残差・sd・1 回外した変化で判定する。(4) 終了コード 2 のときは、貼る行を 1 行も出さず、関節ごとに「書かない（理由）」を出す。
- **F4b で変えたこと**: (1) 目標の姿勢は実物（真の角度）で持ち、`--expected-offset` を案内の表示・目標の判定・ε_min の判定にだけ使う（記録する q と fit は読んだ値だけ。meta に値を記録）。(2) ε は折り返さない向き付きの角度（`tool_state` が pan 角から前方を出す）。(3) 90° の記録の σ の上乗せの既定は 0。上の (1) の「80° 以上」の規則は、上乗せを引数で選んだときだけ働く。(4) 許容は全姿勢 ±5°。

## 5. 置き場所・記録の形式・終了コード

### 5.1 置き場所（ユーザーの決定: so_arm101_calibration）

| 部品 | 置き場所 | 理由 |
|---|---|---|
| 純粋な計算（メッシュの読み、固定あごの最小の z、モデル、当てはめ、統計、姿勢の検査、合否、貼る行の文字列） | `src/so_arm101_calibration/so_arm101_calibration/zero_offset.py`（Python）。記録の形式と識別は `zero_offset_record.py` に分ける | センターキャリブと一緒に実行する道具で、`so_arm101_calib`（センターキャリブ）と同じパッケージに置く |
| 記録の道具（購読、端末の表示、Enter、CSV と meta） | `tools/zero_offset_record_run.py` | camera_record_run.py と同じ作り。ROS に依存するのはこの道具だけ |
| 当てはめの道具（CSV。表示だけ） | `tools/zero_offset_fit_run.py` | camera_extrinsics_run.py と同じ作り。ROS を使わない |

- **`CMakeLists.txt` の変更点**（実装の範囲の報告に書いて承認を得てから。F2）:
  - `find_package(ament_cmake_python REQUIRED)` と `ament_python_install_package(${PROJECT_NAME})` を足す。パッケージ直下に `so_arm101_calibration/`（`__init__.py` と上の 2 モジュール）を作る。`include/so_arm101_calibration/`（C++ のヘッダ）とは別のディレクトリで、衝突しない（F2 のビルドで確認）。
  - `BUILD_TESTING` の中に `find_package(ament_cmake_pytest REQUIRED)` と、試験ごとの `ament_add_pytest_test`（camera と同じく `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`）。
  - 既存の C++ のターゲットと gtest は変えない。
  - lint: `ament_lint_auto` は使わない（calibration の C++ に標準の著作権ヘッダーが無く、`ament_copyright` と C++ の lint が落ちるため。推測。変えるのは範囲外）。Python 用の `ament_flake8()` と `ament_pep257()` だけを明示で足す案（F2。承認の項目。9.2）。
  - 具体（行の追加。F2 の時点。F3・F4 は `ament_add_pytest_test` の行だけが増える）:

```
find_package(ament_cmake_python REQUIRED)          # find_package(ament_cmake REQUIRED) の次の行
ament_python_install_package(${PROJECT_NAME})      # install(DIRECTORY include/ ...) の次の行（コメント: Pure Python library）
# BUILD_TESTING の中（find_package(ament_cmake_gtest REQUIRED) と gtest の登録は変えない。その後ろに足す）:
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_flake8 REQUIRED)          # 9.2 の承認が出たときだけ
find_package(ament_cmake_pep257 REQUIRED)          # 同上
ament_flake8()                                     # 同上
ament_pep257()                                     # 同上
set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
ament_add_pytest_test(test_zero_offset test/test_zero_offset.py ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)                  # F2
ament_add_pytest_test(test_zero_offset_static test/test_zero_offset_static.py ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)    # F2
ament_add_pytest_test(test_zero_offset_record test/test_zero_offset_record.py ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)    # F3
ament_add_pytest_test(test_zero_offset_record_static test/test_zero_offset_record_static.py ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)  # F3
ament_add_pytest_test(test_zero_offset_fit_tool test/test_zero_offset_fit_tool.py ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)  # F4
```

  - C++ の変更（F1b。CMake は変えない）は 5.1 の外の `src/apply.cpp` だけ（0.2）。
- **`package.xml` の変更点**（同じく承認を得てから）:
  - F2 で足す行（既存の `ament_cmake`・depend・`ament_cmake_gtest` の行は変えない）:
    - `<buildtool_depend>ament_cmake_python</buildtool_depend>`（`<buildtool_depend>ament_cmake</buildtool_depend>` の次）。
    - `<exec_depend>so_arm101_ik</exec_depend>`（`UrdfChain`）、`<exec_depend>so_arm101_description</exec_depend>`（URDF とメッシュ）、`<exec_depend>python3-numpy</exec_depend>`、`<exec_depend>python3-yaml</exec_depend>`。
    - `<test_depend>ament_cmake_pytest</test_depend>`、`<test_depend>ament_index_python</test_depend>`。lint を採るとき（9.2）は `<test_depend>ament_cmake_flake8</test_depend>`、`<test_depend>ament_cmake_pep257</test_depend>`。
  - F3 で足す行: `<exec_depend>rclpy</exec_depend>`、`<exec_depend>sensor_msgs</exec_depend>`（記録の道具）。F4 は変更なし。
  - `<description>`（F2）に、センターキャリブ（EEPROM の measure・apply）に加えてオフセットキャリブ（zero_offset_deg の記録と当てはめ）を持つことを書く。
- **依存の向き**: calibration → hardware（今のまま。ビルド時）、calibration → ik・description（新しい。実行時）。ik・description・hardware は calibration に依存していない（`package.xml` で確認）ので、循環しない。camera は calibration に依存せず、calibration も camera を import しない。
- **記録の識別の関数**（**決定（ユーザー、9.1）: calibration 側に重複して持つ**）: camera の `record.py` の `make_identity`・`read_zero_offsets` は import しない（calibration が camera に依存するのを避ける）。`zero_offset_record.py` に同じ規則の小さな関数（sha256、`zero_offset_deg` の読み取り: 有限・10° 以内・gripper は不可、center の読み取り）を持つ。重複は約 30 行。規則の同一性は、同じ入力に対する出力が camera と同じであることを、試験で確かめる（`test_record.py` の `read_zero_offsets` の入力と期待値を、calibration の試験に写す。camera は import しない）。重複をなくす（共通のライブラリに移す）のは、必要になったときに別の承認で。
- **道具の起動の形**: `tools/` のスクリプトを `python3 tools/zero_offset_record_run.py ...` で動かす（両方の setup.bash を source）。`ros2 run so_arm101_calibration ...` にしない理由: ik・camera の道具がすべて `tools/` で、`ros2 run` にするには CMake に `install(PROGRAMS ...)` と ROS 用のスクリプトの形が要り、変更が増える。後で必要になれば別の承認で足す。センターキャリブ（`ros2 run so_arm101_calibration so_arm101_calib`）との違いは、起動の最初の行（0.2）で分かる。
- 試験の見込みの変化: 試験は camera でなく **calibration の pytest** に入る（calibration は 82 → 約 137。camera は 370 のまま）。`colcon test` は calibration を対象にする（7.1）。
- ik に置く案は、yaml と識別を読む結合が ik に入るので採らない。camera に置く案（前回の推奨）は、ユーザーの決定で採らない（センターキャリブと一緒に実行する道具なので）。

### 5.2 記録の道具（`zero_offset_record_run.py`）

- 起動: `python3 tools/zero_offset_record_run.py --csv PATH [--expected-offset NAME=DEG ...] [--calibration-yaml ...] [--h-prior-mm -2.4] [--h-sigma-mm 1.0]`（初回は `--expected-offset elbow_flex=-6.3`。書式と検査は camera の `--joint-offset` と同じ: 5 つの角度関節、上限 10°、gripper 不可、同じ関節の重複は拒否。起動の直後に値を 1 行表示。案内の表示・目標の判定・ε_min の判定だけに使い、記録する角度と fit には使わない）。購読は `/joint_states` だけ。トルクもサーボも書かない（`ros2 launch` は読み取り専用の起動をユーザーが行う）。最初の行は 0.2 のとおり。
- 目標の姿勢（**実物の値**。姿勢ごと。x は mm、許容 ±10 mm、ε は全姿勢 ±5°。向き付きの角度なので 95° 側も区別する）:

| 姿勢 | 傾き ε | x | 記録 |
|---|---|---|---|
| P1 | 90° | 100 | 2 回（P1#1、P1#2） |
| P2 | 90° | 300 | 2 回 |
| P3 | 70° | 150 | 2 回 |
| P4 | 50° | 250 | 2 回 |

- 表示（更新は 5 Hz 程度）: 現在の x、ε（実物の値。括弧で読んだ値の ε も並べる）、目標との差（Δx mm、Δε °）、範囲に入ったかの印、関節の動き（最近 1 s の最大の幅 ° と静止の印）、現在のモデルでの高さ（記録の前の参考。0 に近いほど当たっているはず。警告の判定に使う）。目標は姿勢の順に自動で進める（`--target P3` で選べる）。
- 記録: Enter で、静止（幅 0.2° 以下、推測。サーボの 1 tick は 0.088°）のときだけ、直近 N サンプル（既定 20、約 1 s）の平均を 1 行にする。ε < ε_min（45°）は拒否（メッセージを出して書かない）。目標の範囲外でも記録はできる（確認を 1 回要求。範囲の外は fit の姿勢の検査が評価する）。グリッパーは開かない（記録した gripper の値を表示するだけ。モデルには使わない）。
- 同じ姿勢の 2 回目は、一度先端を机から離してからもう一度当てる。label の末尾の連番（`P3#1`、`P3#2`）で区別する。
- 暫定の当てはめ: 記録が 6 回以上になったら、記録のたびに fit の純粋関数を呼び、δ と sd を表示（書かない。表示だけ）。止め時の目安にする（合格するのは 4 姿勢 8 回がそろってから）。

### 5.3 記録の形式

- CSV（1 行 1 回の記録。**q・x_mm・eps_deg は読んだ値、x_expected_mm・eps_expected_deg は実物の見込みの値**。後者は案内の確認用で、fit は読まない）: 最初に `# zero-offset calibration (floor touch). Not the center calibration.`（コメント行。読み込みで飛ばす）。label、姿勢、時刻、q_state の 6 関節（rad。平均）、各関節の幅（°）、サンプル数、x、ε、pan、モデルの高さ（記録の時の δ_old のもと）、静止・警告の印。数値は有効桁を camera の `_num` と同じに。
- meta（`<csv>.meta.yaml`。camera の `build_meta` と同じ作り。**1 つの CSV は 1 つの識別**。違う識別の行は追記を拒否）:
  - `calibration_kind: zero_offset`、`follower_calibration_sha256`、`zero_offset_deg`（全関節。δ_old。camera と同じ制限）、`urdf_sha256`、使ったメッシュ（gripper_link の 2 つ）の sha256、`h_prior_mm`（−2.4）、`h_sigma_mm`（1.0）、ε_min（45）、`sigma_90_extra_mm`（0）、`expected_offset_deg`（5 関節。識別には入れない。1 つの CSV は 1 つの値で、違う値での追記は拒否）、道具の版。
  - 記録の時の `/joint_states` が yaml の δ_old のもとのものかは、道具からは確かめられない（ドライバが読んだ yaml と同じかは、起動のログの「δ ≠ 0 の関節の INFO」で**ユーザーが確認**）。識別には install の yaml の sha256 を使う（既定。camera_record_run と同じ）。
- fit の入力は CSV と meta。meta が無い・`calibration_kind` が違う・識別が違う行がある・URDF の sha256 が今と違う、は拒否。

### 5.4 当てはめの道具（`zero_offset_fit_run.py`）

- 引数: `CSV`、`--meta`、`--exclude LABEL`、`--sigma-contact MM`（既定 1.0）、`--sigma-90-extra MM`（既定は meta の値、なければ 0。90° の記録だけ二乗和で上乗せ）、`--h-prior-mm`・`--h-sigma-mm`（meta の値が既定）、`--calibration-yaml`（識別の確認と δ_old の読み取り用。読むだけ）。**yaml を書く引数は無い**（6.1）。
- 表示: 最初の行（0.2）、使った記録・外した記録、記録ごとの残差（mm）と ε と x、δ ± sd（絶対値と、現在の yaml との差）、h、相関（lift・elbow・wrist・h）、1 回ずつ外したときの δ の変化、同じ姿勢の 2 回の差、条件数、各項目の合否、最終の判定。合格した関節の「yaml に貼る行」（0.1° 刻み）と、合格しなかった関節の「書かない（理由）」。wrist の規則と δ_wrist = 0 の再当てはめ（4 章）。
- 終了コード: 0 合格、2 不合格または拒否、1 引数・読み込みの誤り。

## 6. yaml には書かない、J5 との関係

### 6.1 yaml には書かない（ユーザーの決定、2026-11-17）

- 道具（`zero_offset_fit_run.py`）は yaml を**書かない**。`--write`・`--output`・`--joints`・`--force` は設けない（読むだけ。yaml のパスは識別の確認のために読む `--calibration-yaml` だけ）。
- 道具が表示するもの（合格した関節だけ）: yaml に貼る行（関節名と `zero_offset_deg` を 0.1° 刻みに丸めた値。例: `elbow_flex: ... zero_offset_deg: -6.3` の差し込み部分だけ）と、現在の yaml の δ_old との差（絶対値と差）。合格しなかった関節（wrist の規則を含む）は、貼る行を**出さず**、「書かない（理由）」と明記する。
- yaml は J5 のコミットで私が手で直す（承認を得てから）。0.2 のコメントを付ける。値は読み込みの検査（J1。有限・10° 以内・gripper は不可）と、起動の INFO（J2。δ ≠ 0 の関節だけ）で確かめる。元の yaml の内容（該当行と sha256）は J5 のとき `inventory.md` に控える。
- 既存の δ との関係: 結果は絶対値で、現在の yaml の δ_old との差も出す。δ_old ≠ 0 の yaml で記録したデータも、同じ式で絶対値が出る。貼るのは絶対値（置き換え。足し算ではない）。差が 10° を超える・符号が逆で大きいなどは警告。
- yaml を直したあとは colcon build（install への反映）とドライバの再起動が要る（J5 の手順）。
- センターキャリブとの関係: J3 の持ち越し（`apply` は `zero_offset_deg` をそのまま持ち越す）に従う。センターキャリブをやり直したら、オフセットキャリブをやり直す（`joint_offset_plan.md` 2.3）。記録の識別に yaml の sha256（center を含む）が入っているので、yaml が変わった後の fit は、前の記録を拒否する（5.3）。

### 6.2 当てはめ後の確認の実行（閉ループ）

- δ を yaml に入れてドライバを再起動したあと、同じ手順をもう一度（4 姿勢 × 1 回でよい、推測）。当てはめの結果が「δ ≒ 0（sd の範囲内）」なら、yaml の値が正しい。道具は δ_old ≠ 0 でも正しく動くので、追加のコードは要らない。

### 6.3 J5 との関係（ユーザーの決定、2026-11-17）

- 決定: J5 の「δ = −6.3° で始めて、物差しで確定」は、「この道具で測った値を yaml に入れる」に置き換える。lift・elbow・wrist_flex が対象（wrist は 4 章の規則）。pan と roll は 0 のまま（pan はカメラの外部パラメータに吸収。roll は高さに効かない）。
- 物差し（出発点の机との間隔）は、確定の根拠でなく**独立の確認**として残す（安い。δ を入れたあとの予測と ±3 mm の比較。ユーザー決定）。
- 独立の比較: カメラの (g)（elbow −6.31 ± 0.38）と物差し（−5.77）との一致を結果のときに表にする。一致しない場合は、メッシュと実物の差、90° の当たり方、机の傾きを疑う（7.5）。
- `joint_offset_plan.md` の直す所（実装の段 F5 で、元の記述は残して注記を足す。J4d と同じやり方。別の承認）:
  - 0 要約の 4（δ の値は決めない）、5.1 の表の「確定」の記述、5.2 の 1・5.4（−6.3 で始める）、9.1 の項目 3、8.4 の J5 の行（「値はユーザーが決めたもの」→「オフセットキャリブの結果」）に注記。
  - 7.4（実機の確認の段 R0〜R3）に、オフセットキャリブの記録と確認の実行を足す。
  - 2.3（再校正のときの補正値の意味）に、「センターキャリブをやり直したらオフセットキャリブをやり直す」を足す。
  - 10 章（未確認）に、本書の 8 章の未確認を参照として足す。
  - 冒頭に、用語の注記（0.2）。
- `camera_stage_plan.md`: C3 の再計算（J6）は、J5 の後に、オフセットキャリブの δ で `--joint-offset`（または yaml 反映後の新しい記録）で行う。手順の順序（センターキャリブ → オフセットキャリブ → カメラ）を 11 章の近くに注記。
- 実行の順序（運用）: センターキャリブ → オフセットキャリブ（lift・elbow・wrist の δ。表示された行を J5 で手で yaml に入れる）→ カメラのキャリブ（C2・C3。pan のずれは外部パラメータに入る）。

## 7. 試験の見込み、守りを外す確認、実機の手順書の骨組み、コミットの区切り

### 7.1 試験の見込み（推測。展開後。C3 と同じ方針で、主な守りに絞る。9.1 の回答を反映して見直した）

事実: 今の件数は 1002（bringup 47、calibration 82、description 2、hardware 137、moveit_config 0、ik 364、camera 370。ユーザーの報告）。注: `build/` の xml の testcase を数えると calibration は 80（gtest 27 + 53）、camera は 357（pytest 326 + lint 31）で、報告の 82・370 と一致しない（colcon の数え方の差と思う。未確認）。以下は**増える件数**で書き、実装のとき `colcon test-result` の増分で比べる。calibration に Python の試験と C++ の試験（F1b）が加わる。camera の過去の実績は見込みの数倍に出たので、守りを下の表に限り、パラメータ化の展開を抑える。

**段ごとの増分（展開後。CTest は ctest の項目数、pytest・gtest・lint は testcase の数）**

| 段 | pytest | gtest | lint（flake8 1 と pep257 は py ファイルごとに 1） | CTest の項目 |
|---|---|---|---|---|
| F1（docs） | 0 | 0 | 0 | 0 |
| F1b（C++ のコメント 2 行） | 0 | +2 | 0 | 0（既存の `test_measure_apply` の中） |
| F2（`zero_offset.py`） | 約 +40 | 0 | +5（flake8 1、pep257 4。lint を採るとき） | +4（pytest 2、flake8、pep257） |
| F3（`zero_offset_record.py`・記録の道具） | 約 +27 | 0 | +3（pep257 3） | +2 |
| F4（fit の道具） | 約 +15 | 0 | +1（pep257 1） | +1 |
| 合計 | 約 +82（70〜95） | +2 | +9（lint を採らなければ 0） | +7（lint を採らなければ +5。今の 2 から 9 または 7） |

- 全体の見込み: calibration は 82 → 約 175（lint なしで 約 166）、全体は 1002 → 約 1095（lint なしで 約 1086）。camera・ik・hardware・bringup・description・moveit_config は不変。
- lint の内訳: ament_copyright は使わない（C++ の著作権の 1 行ヘッダーが標準に合わない。推測）。cpplint・uncrustify・cppcheck・lint_cmake・xmllint も使わない（C++ と CMake は今まで lint 対象外）。

**pytest の内訳**（F2〜F4。守りごと）

| 段 | 守り | 内容（展開後） | 見込み（件） |
|---|---|---|---|
| F2 | モデルと当てはめの計算の正しさ | 合成データの戻り（δ の組 3 通り × 姿勢の組 2 通り = 6）、δ の符号（1）、h の符号（1）、q_raw0 の変換（δ_old ≠ 0 でも絶対値が同じ。2）、最小の頂点が固定あごのメッシュ全体の最小であること（合成の小さな STL。2）、90° の σ の上乗せ（1）、σ と 1 回外した変化が式と合う（2）、貼る行の丸め・書式・δ_old との差（3）、wrist 固定の再当てはめ（値 2、sd の表示 1。3） | 約 21 |
| F2 | 合否の判定 | 姿勢の数・記録の数（境目の両側。2）、傾き 3 群（2）、x の広がり 150 mm（2）、残差 RMS と最大（2）、1 回外した変化（1）、δ の絶対値 10°（1）、wrist の貼る行（出す、δ が 1.5° 未満、sd が 1.0° 超、境目。4）、終了コード 0 と 2 の判定（2） | 約 16 |
| F2 | 静的な試験（ROS を起動しない） | 第三者の import がすべて package.xml に宣言されている（1）、行が 99 字以下（1）、純粋なモジュールが rclpy・serial を import しない（1） | 約 3 |
| F3 | 記録の拒否 | 静止していない（1）、ε < ε_min（境目の両側。2）、識別の違いで追記を拒否（yaml の sha256、δ_old、URDF の sha256、`calibration_kind` の 4 通り。4）、CSV と meta の形式（コメント行を飛ばすことを含む。2）、元の CSV を書き換えない（1） | 約 10 |
| F3 | 識別の関数が camera と同じ出力（item 5） | `test_record.py` の `read_zero_offsets` の入力と期待値を写す: 既定 0（1）、elbow −6.3（1）、上限 10 で通る（1）、上限超・非有限・bool・文字列の拒否（4）、gripper 0 は通り 1.0 は拒否（2）、ファイル無し・壊れた yaml・角度でない関節の拒否（3）。camera は import せず、期待値は試験に書き写す | 約 12 |
| F3 | 記録の道具の静的な試験（ROS を起動しない） | `/joint_states` を購読するだけ（1）、発行・サービス・書き込み・プロセスを起動する名前が無い（1）、最初の行に `[zero-offset calibration]`（1）、引数がちょうど期待どおり（1）、ROS を使わない検査が `rclpy.init` の前（1） | 約 5 |
| F4 | fit の拒否と表示 | meta が無い（1）、識別の違い（1）、URDF の sha256 の違い（1）、`calibration_kind` の違い（1）、除外した記録の扱い（1）、`--calibration-yaml` が識別と違うときの拒否（1）、終了コード 0・2・1（3）、不合格でも全項目を出す（1）、wrist の規則と固定の再当てはめの表示（2） | 約 12 |
| F4 | fit の静的な試験 | `--write`・`--output`・`--joints`・`--force` が無い（1）、yaml を書く関数を呼ばない（1）、最初の行に `[zero-offset calibration]`（1） | 約 3 |
| F1b（gtest） | apply のコメント 2 行 | apply が作る yaml の先頭に 2 行が有る（1）、コメントを除いた yaml の中身（キー・値・順序）は変わらない（1） | +2（C++。pytest でない） |
| 合計 | | pytest 約 82（F2 40、F3 27、F4 15）、gtest +2、lint 最大 +9 | 約 93。calibration は 82 → 約 175、全体は 1002 → 約 1095 |
- 変えないパッケージの件数は不変（bringup 47、description 2、hardware 137、moveit_config 0、ik 364、camera 370）。`git diff --stat` に出るのは calibration だけ（と tools/、docs）。
- **ビルドと colcon test の範囲**（各段。両方の setup.bash を source。失敗があればコミットの前に報告。docs だけの段は省略して報告に書く）:

| 段 | colcon build | colcon test |
|---|---|---|
| F1 | 省略（docs と `git mv` だけ。コードは変わらない） | 省略 |
| F1b | `--packages-select so_arm101_calibration` | calibration と hardware（gtest 2 本の増分。hardware は不変の確認） |
| F2 | 同上（`ament_cmake_python` と lint の新しい設定が通るか。`so_arm101_calibration/` と `include/so_arm101_calibration/` が衝突しないことをここで確認） | calibration |
| F3 | 同上 | calibration |
| F4 | 同上 | 全パッケージ（1002 + 増分と一致するか。camera 370・ik 364・hardware 137・bringup 47・description 2 は不変） |
| F5 | 省略（docs だけ） | 省略 |
| F6 | 実機の後。yaml の変更のあと `--packages-select so_arm101_hardware so_arm101_calibration` | hardware と calibration（本番の yaml の検査） |
- 試験に入れないもの（報告する）: 実機の挙動、メッシュと実物の差、表示の見た目（端末の整形）、購読の通信そのもの。

### 7.2 守りを外す確認（スクラッチのコピーで。別の build・install。tools/ もコピー。リポジトリは変えない。外す → 落ちる試験を確かめる）

| # | 外し方 | 落ちるべき試験 |
|---|---|---|
| 1 | δ の符号を逆（q + δ を q − δ） | 合成データの戻り、δ の符号 |
| 2 | q_raw0 の変換をやめる（δ_old を引かない） | δ_old ≠ 0 の試験 |
| 3 | h の符号を逆 | 合成データの戻り、h の符号 |
| 4 | 固定あごのメッシュの最小でなく、1 つの先端の点の z を使う | 最小の頂点の試験、合成データの戻り |
| 5 | 90° の記録の σ の上乗せを外す | 90° の σ の試験 |
| 6 | 記録の ε_min の拒否を外す | 記録の拒否（ε） |
| 7 | 記録の静止の検査を外す | 記録の拒否（静止） |
| 8 | 追記の識別の検査を外す | 記録の拒否（識別の 4 通り） |
| 9 | fit が meta 無しを受け入れる | fit の拒否（meta） |
| 10 | fit の識別（yaml の sha256・δ_old・`calibration_kind`）の検査を外す | fit の拒否（識別） |
| 11 | fit の URDF の sha256 の検査を外す | fit の拒否（URDF） |
| 12 | 傾き 3 群の検査を外す | 合否（傾き） |
| 13 | 姿勢の数・記録の数の検査を外す | 合否（数） |
| 14 | x の広がりの検査を外す | 合否（x） |
| 15 | 1 回外した変化の基準を外す | 合否（外し） |
| 16 | wrist の規則を外す（1.5° 未満でも貼る行を出す） | 合否（wrist の貼る行） |
| 17 | fit に yaml を書く引数・関数を足す（`--write` など） | 静的な試験（引数と呼び出しが無いこと） |
| 18 | 記録の道具が `/joint_states` 以外に発行する（トルク・サーボの書き込み系を足す） | 静的な試験 |
| 19 | apply のコメント 2 行を外す（F1b。C++ をスクラッチでビルド） | gtest（2 行の有無） |
| 20 | apply のコメントを足すときに yaml の中身（キー・値・順序）まで変える | gtest（中身が同じ） |
| 21 | 識別の関数の δ の上限 10° の検査を外す（F3） | 識別の同一性（上限超の拒否） |
| 22 | 識別の関数の gripper の δ 不可の検査を外す（F3） | 識別の同一性（gripper） |
| 23 | wrist を 0 のままにするとき、固定した再当てはめでなく自由にした値を貼る行にする（F2） | wrist 固定の再当てはめ |
| 24 | py ファイルに 100 字の行を 1 行足す（lint を採るとき） | flake8、静的な試験の 99 字 |

- 守りを外した結果、落ちる試験が無いものは、試験を足すか、その守りを試験の対象外と報告する。

### 7.3 元に戻す方法

- 道具は何も書かない（yaml もサーボも）。コードは該当のコミットを戻す（`git revert` は承認を得てから）。J5 で yaml に貼った値を戻すときは、`inventory.md` に控えた元の行（または `zero_offset_deg` を消す）に戻す。calibration の CMake・package.xml の変更も、F2 のコミットを戻せば元に戻る。

### 7.4 コミットの区切り（案。各段で colcon build が通ること。承認を得てから）

| 段 | 内容 | 試験 | 実機 |
|---|---|---|---|
| F0 | 本書（計画書）と 3 章の結果の記録。docs だけ。**済み（c54930f）** | 省略 | 不要 |
| F1 | 計画書の `git mv`（`floor_touch_plan.md` → `zero_offset_calib_plan.md`）と本書の全置換、9.2 の回答の反映（本書の今回の変更を含む）。docs だけ | 省略 | 不要 |
| F1b | C++: `src/apply.cpp` の yaml ヘッダーのコメントに 2 行（0.2）。`test_measure_apply.cpp` に gtest 2 件。CMake は変えない | calibration・hardware | 不要（実機を開かない。偽サーボの試験だけ） |
| F2 | calibration: `CMakeLists.txt`・`package.xml`（5.1）、`so_arm101_calibration/__init__.py`、`zero_offset.py`（メッシュ・モデル・当てはめ・統計・姿勢の検査・合否・貼る行）。試験 | calibration | 不要 |
| F3 | `zero_offset_record.py`（記録の形式・識別。camera と同じ規則の重複）と `tools/zero_offset_record_run.py`。`package.xml` に rclpy・sensor_msgs。試験 | calibration | 不要（ROS は起動しない試験） |
| F4 | `tools/zero_offset_fit_run.py`（表示だけ。yaml は書かない）。試験 | calibration（と全パッケージの不変の確認） | 不要 |
| F4b | 目標の姿勢を実物の値に（`--expected-offset`、向き付きの ε、許容 ±5°、90° の σ の上乗せの既定 0）。`zero_offset.py`・`zero_offset_record.py`・両方の tools・試験・本書。pytest +39（calibration 206 → 245。守りを外す確認 13 件。ベースラインの scratch でも ProductionYaml の gtest は環境の差で落ちる） | calibration | 不要 |
| F5 | docs: 実機の手順書 `docs/zero_offset_calib_real_check.md`、`joint_offset_plan.md` と `so_arm101_calibration_check.md` の冒頭の用語の注記（0.2）、`camera_stage_plan.md` の順序の注記と `joint_offset_plan.md` の直す所（6.3）、`inventory.md` | docs のみ | 不要 |
| F6（J5） | ユーザーが実機で記録 → オフラインで fit → 合格した関節の貼る行を、承認を得て私が yaml に手で入れる（0.2 のコメントつき。元の行と sha256 を inventory に控える）→ build → R0〜R3 と確認の実行 | hardware・calibration（本番の yaml の検査）。J1 の読み込みの検査と J2 の起動の INFO で確かめる | 要 |

- F0〜F5 は yaml を変えないので実機の挙動は今と同じ（F1b は apply が作る yaml のコメントだけ。apply を実行しない限り何も起きない）。J5（F6）だけが挙動を変える。

### 7.5 実機の手順書の骨組み（F5 で `docs/zero_offset_calib_real_check.md` に書く。ユーザーが実施。私は実機を動かさない）

1. 準備: センターキャリブが済んでいること。yaml の δ_old を確認（今は無し）。URDF・メッシュ・yaml の sha256 の控え。**机の高さの確認**: 腕の取り付け面と机が同じ面か。同じなら h の事前分布は平均 −2.4 mm、σ_h 1 mm（既定）。違う台なら高さの差を物差しで測り、`--h-prior-mm` を測った値に直す（σ_h 1 mm は測定の精度による。測れなければ 2 mm）。
2. 読み取り専用の起動（`read_only:=true`。トルク OFF）。**安全**: トルク OFF なので腕は自重で落ちる。片手で腕を支える。机にそっと当てる（押し付けない。たわみが誤差になる）。グリッパーは開かない。起動のログの「δ ≠ 0 の関節の INFO」と yaml の δ_old の一致を確認。
3. 記録: `zero_offset_record_run.py --expected-offset elbow_flex=-6.3` を起動（最初の行に `[zero-offset calibration]`。初回だけ。yaml に δ を入れた後は渡さない）。目標は実物の値。P1（90°、x 100）、P2（90°、x 300）、P3（70°、x 150）、P4（50°、x 250）を順に、各 2 回。各姿勢で、表示の Δx・Δε が範囲（±10 mm、±5°）に入るまで手で動かし、先端を机に当て、静止したら Enter。2 回目は一度離してもう一度当てる。**90° は、固定あごの先が机に当たっていること（動くあごが先に当たっていないこと）を目で確認する**。
4. 終了: 止めるときは自分で起動したプロセスを PID で kill -INT（pkill 禁止）。ps と `ros2 node list --no-daemon` の両方で残りがないことを確認。
5. オフラインの当てはめ: `zero_offset_fit_run.py CSV`（表示だけ）。結果をユーザーと私で確認。合格した関節の「貼る行」を、J5 のコミットで私が yaml に手で入れる（承認を得てから）。
6. 反映: colcon build → 起動のログで δ の INFO を確認 → 出発点の物差し（予測との比較。独立の確認）→ 確認の実行（6.2）→ R0〜R3（`joint_offset_plan.md` 7.4）。
7. 記録: 材料を `~/so_arm101_check/` の新しいディレクトリに置き、sha256 を `inventory.md` に。

## 8. 事実と推測、未確認

### 事実（読んだもの、計算したもの）
- 1 章と 3.4（オフラインの検討の結果）に挙げたもの。4 姿勢（90・70・50°）の sd は lift 0.32、elbow 0.51、wrist 0.69°（当て方 1 mm、σ_h 1 mm、90° の σ 1.41 mm）。3 姿勢は余り 0 で検査ができない。

### 推測（見立て。値は実機の後に見直す）
- F4b: 精度の見込みは design_study_3 の 90・70・50° の値（90° の σ 1.41 mm の上乗せを入れた保守側）を使う。design_study_5（85°。σ 1.0 mm）は検討の記録（3.4.7）。
- 4 章の閾値すべて、静止の幅 0.2°、許容 ±10 mm・±5°、条件数の上限 1e3、wrist の 1.5°、90° の σ の上乗せの形（二乗和）。
- STS3215 の出力軸のエンコーダの読みにはバックラッシュが乗らない（要確認）。
- `so_arm101_calibration/` を Python のパッケージ名にしても、`include/so_arm101_calibration/` と衝突しない（F2 のビルドで確認）。

### 未確認（計画の段階で確かめていないこと）
- 90° の姿勢（x 100 と 300 mm）が実機で取れるか（ベースとの干渉、関節の範囲。reach_scan は URDF のリミットと yaml の範囲だけを見ている）。
- 90° で、あごを閉じたとき動くあごが先に机に当たらないか（ユーザーが実物で確認した。モデルの jaw_diff は真下で動くあごが 1.26 mm 低いので、**食い違い。原因は未確認**: メッシュと実物の差か、閉じたときのグリッパーの角度の差 [推測]）。
- メッシュ（STL）の先端の形と実物の差（先端の長さ・丸み・すり減り）。3.4 のあごのずれの偏りは見積もったが、実物との比較は無い。
- 机と取り付け面が同じ面か（ユーザーが測れる）。机が水平か（pan を変えた点が無いので、残差では見えない）。
- トルク OFF で腕を机に当てたときの、関節の読みの再現性（同じ姿勢の 2 回で測る）。重力による lift のたわみや歯車のたわみは、押す力で変わる。
- 識別: 記録の時にドライバが読んでいた yaml と、道具が見る install の yaml が同じか（道具では確かめられない。ユーザーの確認）。
- `ros2_controllers.yaml` の目標の許容（constraints）は `joint_offset_plan.md` の未確認のまま（本書には無関係）。

## 9. 決めたこと、確認したいこと

### 9.1 決定（2026-11-17、ユーザー）

| 項目 | 決定 | 反映 |
|---|---|---|
| 姿勢 | 90・70・50° の 4 姿勢 × 各 2 回（記録 8 回）。P1 90° x 100、P2 90° x 300、P3 70° x 150、P4 50° x 250（mm。許容 ±10 mm、傾きは全姿勢 ±5°、向き付き）。**目標は実物（真の角度）の値。初回は `--expected-offset elbow_flex=-6.3` で案内**（F4b） | 0.1、5.2、7.5 |
| 90° | あごを閉じた状態では、動くあごが先に机に当たることはない（ユーザーが実物で確認、F4b）。使う。モデルは固定あごの最低点のまま。90° の点の σ の上乗せの既定は 0（選べる）。モデルの jaw_diff との食い違いは未確認（8 章）。精度の見込みは design_study_3 の 90・70・50° の値（保守側） | 2.1、5.3 |
| ε_min | 45°（守りとして残す） | 2.3 |
| h | 事前分布つき、σ_h 1 mm（腕と机が同じ面のとき。違う台なら測り直す） | 2.1、7.5 |
| 合否の基準 | 4 章のとおり（推測の値。実機の後に見直す） | 4 |
| wrist の貼る行 | δ_wrist の絶対値が 1.5° 以上かつ sd が 1.0° 以下のときだけ。それ以外は「wrist は 0 のまま（理由: 90° の当たり方の 1 mm で約 0.7° の偏りが出るため）」 | 4、5.4 |
| yaml | 道具は書かない。J5 で手で直す | 6.1 |
| 目標の座標と expected-offset（F4b） | 目標は実物（真の角度）。`--expected-offset`（既定なし。camera の `--joint-offset` と同じ書式と検査）は案内の表示・目標の判定・ε_min の判定だけ。記録する q と fit には使わない。meta に記録。初回は elbow_flex=-6.3、F6 の後は渡さない | 0.1、5.2 |
| ε（F4b） | 折り返さない向き付きの角度。3 群の検査もこの角度。fit の ε の検査は読んだ値のまま（全点が同じだけずれる）。fit の表示に meta の offset を 1 行 | 2.3、5.2 |
| 許容と 90° の σ（F4b） | 許容は全姿勢 ±5°。90° の点の σ の上乗せの既定は 0 | 0.1、4 |
| CSV と meta（F4b） | CSV に x_expected_mm と eps_expected_deg の 2 列、meta に expected_offset_deg を残す | 5.3 |
| 置き場所 | 純粋な計算は so_arm101_calibration（センターキャリブと一緒に実行する道具なので）。CMake・package.xml の変更は承認を得てから | 5.1 |
| 当て直し | 各姿勢 2 回 | 5.2 |
| 物差し | 独立の確認として残す | 6.3 |
| 呼び名 | センターキャリブ（center calibration）とオフセットキャリブ（zero-offset calibration） | 題名、0.2 |
| `/tmp/claude-1000/ed.sh` | そのまま残してよい | なし |
| 改名（9.2 の 1。2026-11-17） | `zero_offset_*` に改名する（docs・tools・モジュール・CSV・新しい材料のディレクトリ）。既存の材料 `~/so_arm101_check/floor_touch_20261117/` は動かさない | 0.2、7.4 の F1 |
| wrist を 0 のままにするとき（9.2 の 2） | lift・elbow の貼る行は、wrist = 0 に固定して当てはめ直した値（その sd も表示） | 4、7.1 |
| yaml のコメント（9.2 の 3） | apply が作る yaml の先頭のコメントに 2 行（zero_offset_deg はオフセットキャリブの値。center・min・max などはセンターキャリブの値）。C++ の小さな変更（段 F1b） | 0.2、7.4 |
| 既存 docs の用語の注記（9.2 の 4） | `so_arm101_calibration_check.md` と `joint_offset_plan.md` の冒頭に足す（F5） | 0.2、7.4 |
| 識別の関数（9.2 の 5） | calibration 側に重複して持つ。camera と同じ出力を試験で確かめる | 5.1、7.1 |

### 9.2 確認したいこと（実装の前の承認の項目）

| 項目 | 私の推奨 | 理由 |
|---|---|---|
| 1. F1 の `git mv docs/floor_touch_plan.md docs/zero_offset_calib_plan.md`（作業ツリーとインデックスを変える git の操作） | 承認する（1 件だけ） | 改名は決定済み。実装前に 1 回で行う |
| 2. F2 の `CMakeLists.txt`・`package.xml` の変更（5.1 の具体） | 承認する | camera・ik と同じ形 |
| 3. lint: `ament_flake8()` と `ament_pep257()` だけを明示で足す（`ament_lint_auto` は使わない） | 足す（+9 件） | camera・ik と同じ水準を保てる。足さなければ CMake・package.xml の行が 4 行減り、件数は 9 減る |
| 4. F1b の C++ の変更（`apply.cpp` のコメント 2 行と gtest 2 件） | 承認する | 決定済み。yaml の中身は変えない |
| 5. 守りを外す確認をスクラッチで行うときの C++ のビルド（F1b の 19・20） | 承認する（別の build・install。ユーザーが実機を使っていない間に実行） | ユーザーが実機を使っている間は、ビルドとモックを実行しない |
