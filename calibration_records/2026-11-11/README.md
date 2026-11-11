# calibration_records/2026-11-11 — サーボキャリブレーション（段階2c）の記録

SO-ARM101（フォロワー）の EEPROM（Homing_Offset・Min/Max_Position_Limit）を、ROS のツール（`so_arm101_calib`）で測った値にそろえたときの記録。
`src/so_arm101_hardware/config/follower_calibration.yaml` の `source` が、ここのファイルを指す。手順と結果は `docs/so_arm101_calibration_check.md` と `docs/inventory.md`。

| ファイル | 内容 | 使い道 | sha256 |
|---|---|---|---|
| `eeprom_backup_20261111-163729.json` | C3 の前（旧校正）の EEPROM 全 40 バイト（6 関節）。2 回読んで一致を確認して保存したもの | **`so_arm101_calib restore --backup … --write` の元**（旧校正の値に戻す）。C0〜C3 の「旧」の値 | `729b5015274e7bcd820321b88f19712821fb100dc475c806ce0ba7e1fc7cb88f` |
| `measure_C1c_1.json` | 改善したツール（`so_arm101_calib 0.2.0`、`schema_version` 2）で測った測定ファイル（C1c の 1 回目。全関節の検査 OK） | **C3 で EEPROM と yaml に書いた値の元**（`apply --measure`） | `cfc0f592d1022ddbb22c32bcd472ba8f921c79fb907f2db6ea1bd34ee86e6f47` |
| `eeprom_after_C3.json` | C3 の後（12 V を入れ直した後）の EEPROM の読み出し（`backup` 読み取り専用。2 回読んで一致） | C3 で書いた値が残っていることの記録（新校正の値） | `60b635dbb1406f9bfaac604d81cd984cae4efd8e827054f56c5407b9ad6ec52f` |

原本は `~/so_arm101_check/`（リポジトリの外）。ここは複製（sha256 は原本と同じ）。測定ファイルの中の `backup_file` は、測定したときの原本のパスのまま（チェックサム付きなので書き換えない）。

## 値（Homing_Offset / Min / Max）
| 関節 | 旧（バックアップ） | 新（`measure_C1c_1.json`、C3 の後） |
|---|---|---|
| shoulder_pan | 135 / 743 / 3452 | 144 / 724 / 3454 |
| shoulder_lift | 112 / 842 / 3216 | 117 / 836 / 3211 |
| elbow_flex | 122 / 862 / 3088 | 119 / 849 / 3090 |
| wrist_flex | 89 / 889 / 3185 | 143 / 808 / 3174 |
| wrist_roll | 1164 / 164 / 3965 | 1196 / 100 / 3943 |
| gripper | 107 / 1900 / 3385 | 547 / 1454 / 2944 |

## 元に戻すとき
1. EEPROM: `so_arm101_calib restore --port /dev/ttyACM0 --backup calibration_records/2026-11-11/eeprom_backup_20261111-163729.json --write --eeprom-wait-ms 50`（違う項目だけ書く）。
2. yaml: この記録のコミットを `git revert`（履歴は書き換えない）。旧 yaml は、このコミットの 1 つ前（`1e9e968` で入れた内容）。
3. 全関節の dry-run（`restore --backup …`、`--write` なし）が「同じ」で、読み取り専用の起動で 6/6 MATCH になるまで、書き込みモードは起動しない。
