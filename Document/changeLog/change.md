# EducarSpiconLib v1 変更点

更新日: 2026-11-10

## 概要

Arduino IDE 2.x系、ESP32 Arduino（Espressif Systems）3.x系に対応するため、LEDC（PWM）関連の関数を3.x系のAPIへ移行した。あわせてライブラリのバージョンを alpha.1.1 から v1 に変更した。

## 対応環境

| 項目 | 変更前 | 変更後 |
|---|---|---|
| Arduino IDE | 1.x系 | 2.x系 |
| ESP32 Arduino（esp32 by Espressif Systems） | 2.x系 | 3.x系 |

## ファイル名の変更

フォルダ: `ProgEx01_wheelRotatePwm_Code/`

| 変更前 | 変更後 |
|---|---|
| `EducarSpiconLib_alpha.1.1.h` | `EducarSpiconLib_v1.h` |
| `EducarSpiconLib_alpha.1.1.cpp` | `EducarSpiconLib_v1.cpp` |

旧ファイルは残していない。Arduinoはスケッチフォルダ内の.cppをすべてコンパイルするため、旧ファイルが残っていると同じ関数が二重に定義されてエラーになる。

## LEDC（PWM）関連の変更

対象ファイル: `EducarSpiconLib_v1.cpp`

### `robotSetup()`

`ledcSetup()` と `ledcAttachPin()` を `ledcAttachChannel()` に置き換えた。周波数（`LEDC_SERVO_FREQ`）、分解能（`LEDC_TIMER_BIT`）、チャンネル割り当ては変更していない。

変更前:

```cpp
  //--【LEDCの設定】
  ledcSetup(LEDC_CHANNEL_0, LEDC_SERVO_FREQ, LEDC_TIMER_BIT);  //左車輪用LEDC_CHANNEL_0の設定
  ledcSetup(LEDC_CHANNEL_1, LEDC_SERVO_FREQ, LEDC_TIMER_BIT);  //右車輪用LEDC_CHANNEL_1の設定

  //--【LEDCとpwm用pinの接続】
  ledcAttachPin(PWMA_PIN, LEDC_CHANNEL_0);      //LEDC_CHANNEL_0をPWMA_PINへ出力 
  ledcAttachPin(PWMB_PIN, LEDC_CHANNEL_1);      //LEDC_CHANNEL_1をPWMB_PINへ出力
```

変更後:

```cpp
  //--【LEDCの設定とpwm用pinの接続】(ESP32 Arduino 3.x系)
  ledcAttachChannel(PWMA_PIN, LEDC_SERVO_FREQ, LEDC_TIMER_BIT, LEDC_CHANNEL_0);  //左車輪用LEDC_CHANNEL_0を設定しPWMA_PINへ出力
  ledcAttachChannel(PWMB_PIN, LEDC_SERVO_FREQ, LEDC_TIMER_BIT, LEDC_CHANNEL_1);  //右車輪用LEDC_CHANNEL_1を設定しPWMB_PINへ出力
```

### `wh_motorOutputL()` / `wh_motorOutputR()`

`ledcWrite()` を `ledcWriteChannel()` に置き換えた。3.x系の `ledcWrite()` は第1引数にピン番号を取るため、チャンネル番号を指定して出力する `ledcWriteChannel()` を使う。

| 関数 | 変更前 | 変更後 |
|---|---|---|
| `wh_motorOutputL()` | `ledcWrite(LEDC_CHANNEL_0,duty);` | `ledcWriteChannel(LEDC_CHANNEL_0,duty);` |
| `wh_motorOutputR()` | `ledcWrite(LEDC_CHANNEL_1,duty);` | `ledcWriteChannel(LEDC_CHANNEL_1,duty);` |

## その他の変更

- `EducarSpiconLib_v1.h` と `EducarSpiconLib_v1.cpp` の冒頭の更新履歴に v1 の記載を追加した。
- `EducarSpiconLib_v1.cpp` のヘッダのインクルードを `#include "EducarSpiconLib_v1.h"` に変更した。
- `ProgEx01_wheelRotatePwm_Code.ino` は、ファイル名の変更に合わせてインクルードの1行のみ変更した。

```cpp
// 変更前
#include "EducarSpiconLib_alpha.1.1.h"    //ロボット制御用ヘッダファイル 
// 変更後
#include "EducarSpiconLib_v1.h"    //ロボット制御用ヘッダファイル 
```

## 変更していない部分

- LEDC API の移行以外の制御処理
- ピン設定、チャンネル割り当て、PWM周波数、分解能
- `.ino` の動作内容

## 確認事項

- Arduino IDE 2.x で「esp32 by Espressif Systems」3.x系を選択してコンパイルできること
- 左右の車輪が変更前と同じように回転すること
