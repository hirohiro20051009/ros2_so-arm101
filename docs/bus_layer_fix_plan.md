# バス層の「古い応答の取り違え」の修正計画（計画のみ。実装は承認の後）

作成: 2026-11-11。コードは変えていない。実機・シリアルポートは使っていない（コードの読解と、偽サーボ・実機ログの確認だけ）。
**[事実]** = コード（行番号）・ログ・実行で確認したこと。**[推測]** = 未確認。
関連: `docs/inventory.md` の「残課題: バス層の古い応答の取り違え」（調査）、「試験の固定ファイル化」（不安定な試験の頻度の測定）。
ファイルの略称: `bus` = `src/so_arm101_hardware/src/sts3215_bus.cpp`、`system` = `…/sts3215_system.cpp`、`fake` = `src/so_arm101_hardware/test/fake_servo.hpp`。

## 1. 問題（要約）
- **[事実]** バスは、要求の直前（`begin_request`、bus:98-103）と失敗の直後（`fail`、bus:114）に受信バッファを捨てる。**捨てた後に届いたバイトは、どこでも検査されず、次の読み取りに入る**。
- **[事実]** ステータスパケットに、どの要求への応答かを示す項目は無い（ID・長さ・チェックサムだけ）。検査（bus:118-185）は、ヘッダ、ID・LEN、チェックサム、ID 一致、ERR、データ長の順。
- **[事実]** SYNC_READ（bus:208-239）が途中の ID で失敗して返ると、残りの ID の応答はまだバスを流れていて、`fail` の捨てる処理の**後**に着く。**直後に次の要求を送る**と（即時の再試行）、その要求は残りの応答を先に受け取る。
- **[事実]** 即時に再試行する箇所（失敗の直後に次の要求を送る）: ① ドライバの周期の読み取り（`system:443-456`。失敗したら、無条件で `read_raw_positions` を 1 回やり直す）、② 1 Hz の監視の SYNC_READ（`system:907-910`。失敗したら 1 回やり直す）、③ `so_arm101_calib` の `measure` の `read_present`（`measure.cpp`。通信エラーを最大 3 回再試行）、④ `so_arm101_calib` の EEPROM の読み戻し（`eeprom_ops.cpp:14-22`。単独の READ の再試行）。待ってから再試行する箇所: ⑤ 起動時の読み戻し `read_reg_retry`（`system:536-553`。5 ms 待つ）。
- **[事実]** `IdMismatch` は通信エラーではない（`sts3215_bus.hpp:116-119`）。呼び出し側（ops:19、`system:541`）は再試行しない。SYNC_READ だけが、先に来た ID が「これから来る ID」なら `Timeout` に読み替える（bus:225-232）。
- **[事実]** 不安定な試験 `Measure.CommErrorsWhileRecordingAreTolerated`（`test_measure_apply.cpp`）の失敗は `timeout: no answer from id 1 (id 4 answered first)`。この経路と合っている。頻度（測定、2026-11-11）: ビルド済みバイナリの単独実行を連続 40 回で 31 回失敗、`test_measure_apply` 全体を連続 15 回で 3 回失敗、`colcon test` 3 回で 0 回。
- **[事実]** 偽サーボは、SYNC_READ の応答を、1 つのスレッドで続けて `write` するだけで、タイミングを制御できない（`fake` の `handle`・`read_reply`・`reply`）。競合の起きやすさは、pty とスレッドのスケジューリング次第。

### 実機での頻度（事実）
- 書き込みモードの運転（W0〜W4、最終確認）の `deactivated.` 行の読み取りの合計は約 22.2 万回（1083 + 22106 + 16200 + 38348 + 131540 + 13087）で、**`retried=0`、`failed_cycles=0`**。起動時の読み戻しの再試行も `startup_read_retries=0`（W0 の 2 回の失敗の後）。
- `measure` の記録中の SYNC_READ: C1b 3170 回、C1c 3991 回と 2838 回で、**失敗 0**（計 9999 回）。
- したがって、実機の通信エラー自体がまれで、この競合が実機で起きた記録は無い。**[推測]** 修正の主な価値は、（1）試験を決定的にすること、（2）通信エラーが起きたとき（USB のつまり、電源の揺れなど）の再試行を確実にすること。

## 2. 修正案

### 案 A（基本）: 失敗したら、バスが静かになるまで待ってから捨てる
- 中身: `SerialPort` に `drain_until_quiet(quiet, max_total)` を足す。`poll` で入力を待ち、バイトが来たら読み捨てて、`quiet`（例 2 ms）の間、新しいバイトが来なければ終わる。全体の上限は `max_total`（例 10 ms。1 つの応答のタイムアウトと同じ）。
- 適用先: `Sts3215Bus::fail()`（bus:105-116）の `port_.flush_input()` を、通信エラー（Timeout・BadFrame・BadChecksum・IdMismatch）のときは `drain_until_quiet` に置き換える。`begin_request` の `flush_input` は残す（安い）。`PortError`（書き込めない）は、待たない。
- 値の根拠: 1 Mbps で 1 フレーム（FF FF ID LEN ERR D0 D1 CHK = 8 バイト）は約 80 µs、6 関節で約 0.5 ms（`Return_Delay_Time` は 0。診断で確認済み）。USB CDC（`/dev/ttyACM0`）の遅れは数 ms 程度（**[推測]** 実測していない）。`quiet` の既定は **2 ms**、上限 **10 ms**。ros2_control のハードウェアパラメータ `drain_quiet_ms`（既定 2。**0 で従来どおり**＝捨てるだけ）にして、実機で A の有無を再ビルドなしで比べられるようにする。
- 費用: **正常な経路には影響しない**（失敗したときだけ最大 2〜12 ms）。失敗したサイクルは 20 ms を超えることがあるが、失敗自体がまれ。
- 限界（事実）: タイムアウト（10 ms）より後に届く遅れた応答は、`quiet` の窓の外にあると捨てられない。そのような遅れは、実機で観測されていない。

### 案 B（併用）: SYNC_READ が途中で失敗したら、残りの ID の応答を読み切る
- 中身: `sync_read`（bus:222-237）で、失敗したときに、まだ受け取っていない ID の数（`ids.size() - (i+1)`。先に来た ID を読み替える場合は、その ID より後の数）だけ、短いタイムアウト（例 2 ms。合計の上限 10 ms）で `receive_status` と同じ形のフレームを読み捨てる。早く揃えば待たない。
- 効果: 残りが揃っていれば、A の `quiet` の窓を待たずに終わる（A の `quiet` は、B の後は 0.5 ms に短くできる）。A が「何も来なくなる」を待つのに対して、B は「来るはずのものが来た」を確かめる。
- 注意: 応答しない ID（落ちたサーボ）がいると、その分は待つ（短いタイムアウトで打ち切る）。B は SYNC_READ だけ（単独の READ は 1 フレームしか返さない）。

### 案 C（保険）: EEPROM の書き込みの読み戻しを 2 回続けて同じ値で一致させる
- 中身: `so_arm101_calibration` の `write_calib_verified`（ops:58-76）で、待った後の読み戻しを**独立に 2 回**行い、2 回とも期待値のときだけ一致とする。違えば（片方だけ一致でも）失敗にして、今までどおり戻す（`apply`）/ Lock=1 に戻す（`restore`）。最終の状態表（`restore.cpp:168-187`、`apply.cpp:270-287`）の読み取りは、2 回読んで一致したものだけを表示する（`backup` は既に 2 回読んでいる）。
- ドライバ側（起動時の `verify_reg`、`system:572-590`）は、A・B で足りるので、今は変えない（C は書き込み・起動の回数が増えるため。必要なら後で）。
- 必要性は、5 章（「誤って一致」の経路）による。

### 案 D（観測）: 取り違えの数を数えて出す
- 中身: `Sts3215Bus` に数を持つ（`flushed_requests`・`flushed_bytes`（`begin_request` で捨てた数）、`drained_bytes`（A・B で捨てた数）、`id_mismatch`、`answered_first`（読み替えた回数）、`timeouts`、`bad_checksum`）。アクセサを足す。
- 出力: ドライバの `deactivated.` の行（`system:393-399`）の末尾に `bus: flushed_bytes=… drained_bytes=… id_mismatch=… answered_first=… timeouts=… bad_checksum=…`。`measure` の `記録:` の行にも同じ項目（`measure.cpp`）。`apply`/`restore` の終わり（失敗したときだけ）にも。
- 目的: 修正の効果と、W0 のような `checksum mismatch` の切り分け（`flushed_before_request > 0`＝古いバイトがあった、0＝配線・USB の化け）を、実機で見られるようにする。

## 3. 偽サーボに「応答を遅らせる」機能（設計）
今の偽サーボ（`fake`）は、要求を受けると、同じスレッドで応答を続けて `write` する（遅れが無い）。競合を再現・検査できるように、次を足す（**試験用の道具の変更だけ**。ドライバは変えない）。
- **応答のスケジューラ**: `reply()`（`fake`）が、すぐに `write` する代わりに、`{due = now + 遅れ, 連番, バイト列}` をキューに入れ、同じループ（`poll` のタイムアウトを「次の due までの時間」に縮める）で、due が来た順に `write` する。連番で、同じ due の順序を保つ。要求のパースは遅れない。
- **API**（すべて `std::mutex` 内。時間は `steady_clock`）:
  - `set_reply_delay_us(id, us)`: その ID の全応答を、要求を受けた時刻から `us` 遅らせる（既定 0）。
  - `set_frame_spacing_us(us)`: SYNC_READ で、応答を `us` 間隔で並べる（実機の 1 Mbps・8 バイト＝約 80 µs を模す。既定 0）。
  - `delay_next_replies(id, addr, count, us)`: その (ID, アドレス) への次の `count` 回の READ 応答だけを遅らせる（タイムアウトより遅れて着く応答の再現。`us > タイムアウト`）。
  - `reply_log()`: 応答を `write` した時刻と (id, addr, 壊したか) の記録（試験の確認用）。
- **競合の決定的な再現**: `corrupt_read_replies(1, 56, 1)`（id 1 の応答を壊す）と `set_reply_delay_us(id, 3000)`（id 2〜6 を 3 ms 遅らせる）にする。ホストは id 1 の壊れた応答で失敗して捨てる（時刻 0）。id 2〜6 の応答は 3 ms 後（**捨てた後**）に着き、即時の再試行が必ず取り違える。pty のスケジューリングに依存しない。
- 既存の機能（`corrupt_read_replies`・`drop_read_replies` など）は、そのまま使える（スケジューラの前で判定する）。

## 4. 試験の計画（偽サーボ。実機は使わない。`colcon test`）
修正の前に、次の試験を書いて、**今のコードで落ちること**を確認する（再現の記録）。修正の後に通ることを確認する。試験は無効にしない。
1. `StaleRepliesAfterAFailedSyncReadAreDiscarded`: 上の再現の構成で、`sync_read` の即時の再試行が成功する（A・B）。`IdMismatch`・`answered_first` が出ない。D の `drained_bytes` が残りの応答のバイト数と一致する。
2. `ImmediateRetryAfterTimeoutWithLateReplies`: `delay_next_replies` でタイムアウトより遅れて着く応答。A の `quiet` の窓（2 ms）の中に着けば捨てられ、外に着いたら（限界）取り違えが起きうることを、**期待どおりの失敗として**確認する（限界の記録。この試験は、`quiet` の値を変えたときの挙動の固定）。
3. `SingleReadDoesNotAcceptAnOldFrameOfTheSameIdAfterAFailedSyncRead`: SYNC_READ（Present_Position、2 バイト）が失敗した直後に、同じ ID の別の 2 バイトのレジスタの単独 READ（`read_u16`）をしたとき、古い応答を受け取らない（A・B）。**今のコードでは受け取る**ことを先に確認する（SYNC_READ → 単独 READ の経路。今の呼び出しにはこの並びは無いが、将来のために固定）。
4. `EepromReadBackNeverMatchesOnAStaleReply`（C）: 偽サーボで、書き込みの後の読み戻しに古い値（書く前の値）が 1 回混ざる構成で、読み戻しが「一致」にならない（失敗になり、戻す）。
5. `Measure.CommErrorsWhileRecordingAreTolerated` を、遅れの機能で決定的にする（既存の試験は変えず、遅れの有無を引数にした同じ内容の試験を足す）。**100 回連続で通ること**を確認する。
6. 回帰: 正常な経路で、読み取りの回数と間隔が変わらない（既存の `DisplayDoesNotChangeTheReadCadence` と同じ形で、A・B ありなしを比べる）。

### `Measure.CommErrorsWhileRecordingAreTolerated` は、この修正で安定するか
- **[事実]** 失敗の原因は、この競合（メッセージが一致）。**[推測]** A・B で、失敗した試行が必ず残りの応答を捨てるので、`corrupt_read_replies(1, 56, 3)`（3 回壊す）→ 4 回目で成功、の予定どおりに進む（再試行の回数を余分に使わない）。**安定する見込み**。A・B の実装後に、100 回連続の試験で確認する（確認できなければ、安定したと書かない）。
- 偽サーボの遅れの機能が入ると、今は pty の偶然に頼っている競合が、決定的に再現・検査できる。

## 5. 「誤って一致」と判定される経路があるか（EEPROM の書き込みの読み戻し・起動時の読み戻し）
- **SYNC_READ → SYNC_READ の再試行**: 受け取らない（**事実**）。再試行の最初に期待する ID は ids[0]、古い応答は ids[i+1..]（重複しない）なので、最初のスロットで必ず `IdMismatch`（→ 読み替えで Timeout）。取り違えた値は使われない（失敗が返り、`out` は使われない）。仮に最初の 1 フレームが同じ ID（遅れた応答）でも、2 つ目のスロットで ID がずれて失敗する（全部そろわないと成功にならない）。
- **単独の READ（EEPROM の読み戻し、起動時の `verify_reg`）**: 1 フレームしか見ないので、「同じ ID・同じデータ長・正しいチェックサム」の古い応答は受け取られる（**事実**。bus:118-185 に区別する項目が無い）。ただし、古い応答が残る条件は、次の 2 つだけ（**事実**＋**推測**）:
  1. 失敗した SYNC_READ の直後（残りの応答）。今の呼び出しには、「SYNC_READ（失敗）→ 即時に単独の READ」の並びは無い（`measure` は SYNC_READ だけ、`apply`/`restore` は単独の READ だけ、ドライバの周期の失敗は次の周期まで 20 ms 空く）。
  2. 単独の READ が**タイムアウトし、その後に遅れて**応答が着いたとき（10 ms 以上遅れる。**[推測]** 実機では観測されていない）。
- **誤って「一致」になる条件**: 古い応答の**値**が、期待値と同じで、かつ実際の値が違うとき。
  - EEPROM の書き込みの読み戻し: レジスタの値の履歴は「旧 → 新」の一方向（書く前の値は旧、書いた後は新）。**新**を示す古い応答は、サーボがそのとき新だったことを意味するので、今も新（EEPROM に書かれた）。**旧**を示す古い応答は、期待値（新）と違うので「不一致」＝失敗側。**したがって、「実際は旧なのに新と判断する」経路は、実用上無い**（**事実**として、上の履歴の性質から。同じ値を書く `--rewrite-same` は、旧も新も同じ値で、一致しても実際も同じ）。
  - 起動時の `verify_reg`: Goal_Position・Acceleration・Torque_Limit・Lock は、ドライバが書いたあとは自分で変わらないので、同じ（一方向）。**例外**: `Torque_Enable`（サーボの保護で 1 → 0 に自分で変わりうる）。起動時に 1 を読み戻す（`system:781-788`）ときに、数 ms 前の古い「1」を受け取ると、その間に 0 になっていても「一致」と判断する（**[推測]**。窓は数 ms で、遅れた応答が必要。実用上の確率は極めて低い）。周期の監視（1 Hz の SYNC_READ、`system:907-924`）は、スロットのずれで検出される。
- **別のレジスタの値がたまたま同じ**: 同じ関節の Homing_Offset・Min・Max（測定値は互いに違う）、Present_Position と Goal_Position（起動時に Goal を Present に合わせて書くので近い値）。**[推測]** 起動時に Goal_Position を書いた後、「古い Present_Position の応答（同じ ID・2 バイト）」を Goal_Position の読み戻しとして受け取ると、値が近くて「一致」しうる。ただし、これも「SYNC_READ（失敗）→ 単独の READ」の並びが前提で、起動時は、初期位置の SYNC_READ が成功した後に書き込む（`system:766-775`）ので、起きない（SYNC_READ が失敗すると起動を中止する）。
- **結論**: 今の呼び出しの並びでは、**誤って「一致」となる実用的な経路は見つかっていない**。理論上の穴（遅れた応答、`Torque_Enable`）は残る。そのため、**案 C を含める**（EEPROM の読み戻しだけ。安い保険。遅れた 1 フレームは 2 回の独立した読み取りの両方には入らない）。ドライバ側（`verify_reg`）は、A・B で古い応答を捨てるので、案 C は付けない（必要になったら別途）。

## 6. 起動時の `checksum mismatch`（W0 の 2 回）との関係
- **[事実]**（`docs/inventory.md` 段階2b-3c W0）: Goal_Position の読み戻しで `checksum mismatch`（2 回目は id=1、3 回目は id=5）。タイムアウトではなく、チェックサム不一致。メッセージの ID は**受信した ID**（bus:159）で、期待の ID と同じ。受信バイト列は当時取れていない（`tx= rx= flushed_before_request=` を出す版の前）。その後の起動は `startup_read_retries=0` が続いている。
- **[事実]** その時点までの起動手順で、直前の読み取りはすべて成功している（初期位置の SYNC_READ、Lock・Acceleration・Torque_Limit の読み戻し）。**失敗した SYNC_READ の残りの応答が無いので、今回の競合の前提が成り立たない**。
- **[事実]** 古い**完全な**フレームは、チェックサムが正しいので、`checksum mismatch` にならない（別の ID なら `IdMismatch`、同じ ID なら受け取られる）。
- **結論（事実から）**: **W0 の `checksum mismatch` は、この競合（古い応答の取り違え）では説明できない**。したがって、A・B・C を入れても、W0 の失敗が直ることは期待できない（**直るとは言わない**）。
- **[推測]** 説明の候補（どれも未確認）: 部分的なフレームの残り（直前のフレームの末尾が捨てた後に届き、ヘッダ探索 bus:124-135 が途中から始まる。ただし受信した ID が期待と一致する偶然が要る）、配線・USB・サーボ電源によるバイトの化け。
- **D の役割**: W0 のような失敗が起きたとき、`flushed_before_request > 0`（古いバイトがあった）か 0（配線・USB の化け）かを、ログだけで切り分けられるようにする。W0 の原因の特定は、D のカウンタと、次に起きたときの `rx=` の記録を待つ。

## 7. 実機での確認の手順（C4 のドライバの確認と合わせる）
前提: 実装と偽サーボの試験が通り、コミットされている（C4 の前）。**実機のコマンドは、ユーザーが実行する**（Claude は実行しない）。
1. **読み取り専用の運転（腕は触らない。トルク OFF）**: `bringup.launch.py mock:=false` を 10 分ほど動かして、Ctrl-C で止める（約 3 万回の読み取り）。`deactivated.` の行で、**`retried=0`、`failed_cycles=0`、`bus:` の `drained_bytes=0`、`id_mismatch=0`**（健全なバスでは、捨てるものが無い＝A・B は何もしない）。正常な経路が遅くなっていないこと: 同じ時間での `reads` の数が、修正前（22 万回の記録、約 50 Hz）と同じ。
2. **C4 の W0（書き込みモードの起動）を、起動と停止を 5 回繰り返す**（W0 の失敗は 4 回中 2 回だった。今は 0 が続いている）。各回の `startup_read_retries` と `[diag]`・`bus:` の行を記録する。`checksum mismatch` が出たら、`rx=` と `flushed_before_request=` の行を全部報告する（6 章）。
3. **C4 の W1〜W4**（手順書 `docs/feetech_write_check.md`）を、通常どおり行う。各回の `deactivated.` の `retried`・`failed_cycles`・`monitors`・`bus:` を記録する。**合格の基準**: 今までの記録（`retried=0`、`failed_cycles=0`、`rate_limited=0`、`port_contention=0`）を満たし、`bus:` の `id_mismatch`・`answered_first` が 0（健全なバスでは 0 のはず。0 でなければ、報告して原因を見る）。
4. `measure`（C1c と同じ手順）の記録の `記録:` の行に、`bus:` の項目が出る。失敗 0、間隔が従来（約 21.7 ms）と同じ。
5. **A の有無の比較（問題が出たとき）**: ハードウェアパラメータ `drain_quiet_ms:=0` で A を無効にして、同じ運転で `retried`・`failed_cycles` を比べる（再ビルド不要）。
6. **実機でのエラーの注入はしない**（バスのバイトを壊す・止めるのは危険）。エラー時の挙動は、偽サーボの試験（4 章）で確かめる。
- **C4 との合わせ方**: C4 は書き込みモードの W 系の確認のやり直し（手順書は C3 の後に直す）。上の 2〜3 は、C4 の W0〜W4 の記録に、`bus:` の項目と起動の繰り返し（5 回）を足すだけ。1（読み取り専用の運転）は C4 の前に、腕を動かさずに行える（C4 の前提のドライバの確認）。

## 8. 実装の順序（段階ごとに `colcon build`・`colcon test`・偽サーボでの確認・コミット・inventory に記録）
1. 偽サーボの遅れの機能（3 章）と、競合の再現の試験（4 章の 1・3・5）。**今のコードで落ちることを確認して記録する**（この段階では、落ちる試験を含めてコミットしない。修正と同じ段階にする）。
2. 案 A・B（`SerialPort::drain_until_quiet`、`Sts3215Bus::fail`・`sync_read`、パラメータ `drain_quiet_ms`）と、試験が通ること。`Measure.CommErrorsWhileRecordingAreTolerated` の 100 回連続。
3. 案 D（カウンタとログの行）と、手順書（`feetech_write_check.md` のログの読み方、`so_arm101_calibration_check.md` の `記録:` の行）の追記。
4. 案 C（`so_arm101_calibration` の読み戻し）と、試験（4 章の 4）。
5. 実機での確認（7 章。ユーザー）。結果を inventory に記録する。
- 変えないもの: タイムアウト（10 ms）、再試行の回数、`IdMismatch` を再試行しない方針、プロトコル、書き込みの手順（`apply`/`restore` の順序）、ドライバの変換・診断・安全の仕組み。
- リスク: ドライバの読み取りの失敗の経路（`fail`）を変えるので、C4 の前に偽サーボの試験と、読み取り専用の運転（7 章の 1）で確かめる。`drain_quiet_ms:=0` で従来の動作に戻せる。
