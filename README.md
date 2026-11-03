# LED マトリクス砂時計

ESP32 + AE-HT16K33 + 8×8 LED マトリクス 2 枚を使用した砂時計。

## スケッチ

- ht16k33_dual_test ←ht16k33とLEDマトリクスのテスト
- hourglass_button ←スイッチで 1 粒ずつ落ちる
- hourglass_tilt ←傾きで上下が入れ替わる（LSM6DSV16X）
- check_AE-LSM6DSV16X ←加速度センサーの確認

## 必要なライブラリ

- Adafruit LED Backpack
- Adafruit GFX

## 配線

[PINOUT.md](PINOUT.md)

## 使用部品

| 部品 | 型番 | 資料 |
|---|---|---|
| マイコン | ESP32 | |
| LEDドライバー | AE-HT16K33（秋月 M-11246） | [AE-HT16K33.pdf](AE-HT16K33.pdf) |
| 8*8ドットマトリクスLED ×2 | KEM-12088BW | [KEM-12088BW.jpg](KEM-12088BW.jpg) |
| 加速度センサー（IMU） | AE-LSM6DSV16X | |
| タクトスイッチ | | |
| ユニバーサル基板 | 26×21穴以上 | |

## 注意

- IMU（AE-LSM6DSV16X）は「6A」と書いてあるところの上の部分を短絡する。
