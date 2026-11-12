# サーボ駆動関数（servoOutput）仕様

## 目的

ESP32 Arduino 3.x系で、手首用とグリッパー用のサーボモータを制御する。サーボごとに設定値を持ち、入力された制御用dutyを関数内でPWMパルス幅とLEDC出力値に変換する。

## 対象ファイル

Arduinoスケッチのフォルダ名とメインファイル名は次のとおりとする。Arduino IDEで開くときは、この`.ino`ファイルを開く。

- フォルダ：`ProgEx021_servoRotateDuty_Code`
- スケッチ：`ProgEx021_servoRotateDuty_Code.ino`
- ライブラリヘッダ：`EducarSpiconLib_v1.h`
- ライブラリ実装：`EducarSpiconLib_v1.cpp`

## 対象関数

| 関数名 | 制御対象 |
|---|---|
| `wrist_servoOutput()` | 手首サーボ |
| `gripper_servoOutput()` | グリッパーサーボ |

既存ライブラリの書き方に合わせ、関数はヘッダファイルで宣言し、`.cpp`ファイルで定義する。

## 関数仕様

- 引数：角度制御用duty（整数の割合、0～100%）。
- `50`を中立位置とする。引数はサーボのパルス幅（µs）ではない。
- 戻り値：`void`
- 手首用はグローバル構造体`my_wrist`、グリッパー用は`my_gripper`を使用する。両方とも`Arm`構造体で、`duty`に位置指令、`pwm`に変換後のパルス幅（µs）を保持する。構造体はスケッチ（`.ino`）側で定義し、ライブラリ（`.cpp`）側では`extern`宣言して参照する。
- 関数内で引数を各サーボの入力範囲に収め、入力dutyをパルス幅（µs）に変換する。
- 範囲補正後のdutyを対応する構造体の`.duty`に、計算したパルス幅を`.pwm`に格納する。
- 変換したパルス幅をLEDCのデューティ値に変換し、対応するサーボのチャンネルへ出力する。
- C言語初心者にも読みやすい`if`文と段階的な計算を用いる。`constrain()`やビットシフトは使わない。

## サーボごとの設定

各サーボのパラメータは、それぞれ`#define`で定義する。

- GPIOピン番号
- LEDCチャンネル番号
- PWM周波数
- LEDC分解能（bit数）と、その分解能での最大デューティ値
- 入力dutyの最小値、最大値、中立値
- サーボ信号の最小・最大パルス幅（µs）

設定値の例（GPIOとパルス幅は実機に合わせて確認する）：

```cpp
#define WRIST_SERVO_PIN              16
#define WRIST_SERVO_CHANNEL          2
#define WRIST_SERVO_FREQ              50
#define WRIST_SERVO_TIMER_BIT         16
#define WRIST_SERVO_LEDC_MAX_DUTY  65535
#define WRIST_SERVO_MIN_DUTY           0
#define WRIST_SERVO_MAX_DUTY         100
#define WRIST_SERVO_NEUTRAL_DUTY      50
#define WRIST_SERVO_MIN_US           1000
#define WRIST_SERVO_NEUTRAL_US       1500
#define WRIST_SERVO_MAX_US           2000

#define GRIPPER_SERVO_PIN              17
#define GRIPPER_SERVO_CHANNEL           3
#define GRIPPER_SERVO_FREQ              50
#define GRIPPER_SERVO_TIMER_BIT         16
#define GRIPPER_SERVO_LEDC_MAX_DUTY  65535
#define GRIPPER_SERVO_MIN_DUTY           0
#define GRIPPER_SERVO_MAX_DUTY         100
#define GRIPPER_SERVO_NEUTRAL_DUTY      50
#define GRIPPER_SERVO_MIN_US           1000
#define GRIPPER_SERVO_NEUTRAL_US     1500
#define GRIPPER_SERVO_MAX_US         2000

#define SERVO_PERIOD_US              20000
```

50 Hzの場合、PWMの1周期は20,000 µsである。上記の例では入力duty 0～100をパルス幅1,000～2,000 µsに対応させ、50は中立パルス幅1,500 µsに変換する。サーボによって必要な範囲は異なるため、各サーボの中立値を含めて実機仕様に合わせて設定する。

## 初期化

サーボごとに、ピン・周波数・分解能・LEDCチャンネルを指定して`ledcAttachChannel()`を呼び出す。既存の車輪用チャンネル0・1と重複しないチャンネルを割り当て、サーボ出力関数を呼び出す前に初期化する。初期化が失敗した場合は、失敗を明示して適切に処理する。

## コーディング例

スケッチ（`.ino`）側で構造体を一度だけ定義し、ライブラリ（`.cpp`）側では次のように参照する。

```cpp
// .ino側
Arm my_wrist;
Arm my_gripper;

// .cpp側
extern Arm my_wrist;
extern Arm my_gripper;
```

各出力関数は、対応する構造体の`duty`を引数として受け取り、範囲補正後の値と変換したパルス幅を同じ構造体に保存する。パルス幅は中立dutyより小さい範囲と大きい範囲に分けて計算し、50が中立パルス幅になるようにする。

処理の流れを示す手首用関数の例：

```cpp
void wrist_servoOutput(int duty) {
  int pwm = 0;
  int dutyRange = 0;
  int pulseRange = 0;
  uint32_t ledcDuty = 0;

  if (duty < WRIST_SERVO_MIN_DUTY) {
    duty = WRIST_SERVO_MIN_DUTY;
  }
  if (duty > WRIST_SERVO_MAX_DUTY) {
    duty = WRIST_SERVO_MAX_DUTY;
  }
  my_wrist.duty = duty;

  if (duty < WRIST_SERVO_NEUTRAL_DUTY) {
    dutyRange = WRIST_SERVO_NEUTRAL_DUTY - WRIST_SERVO_MIN_DUTY;
    pulseRange = WRIST_SERVO_NEUTRAL_US - WRIST_SERVO_MIN_US;
    pwm = WRIST_SERVO_MIN_US;
    pwm = pwm + pulseRange * (duty - WRIST_SERVO_MIN_DUTY) / dutyRange;
  } else {
    dutyRange = WRIST_SERVO_MAX_DUTY - WRIST_SERVO_NEUTRAL_DUTY;
    pulseRange = WRIST_SERVO_MAX_US - WRIST_SERVO_NEUTRAL_US;
    pwm = WRIST_SERVO_NEUTRAL_US;
    pwm = pwm + pulseRange * (duty - WRIST_SERVO_NEUTRAL_DUTY) / dutyRange;
  }
  my_wrist.pwm = pwm;

  ledcDuty = (uint32_t)pwm * WRIST_SERVO_LEDC_MAX_DUTY;
  ledcDuty = ledcDuty / SERVO_PERIOD_US;
  ledcWriteChannel(WRIST_SERVO_CHANNEL, ledcDuty);
}
```

`gripper_servoOutput()`も同じ計算手順で実装し、`my_gripper`および`GRIPPER_SERVO_...`の設定値を使用する。

## 呼び出し例

スケッチでは構造体の`duty`に0～100の位置指令を設定し、その値を出力関数へ渡す。タイマー処理など、PWMを更新する箇所から関数を呼び出す。

```cpp
my_wrist.duty = 50;
my_gripper.duty = 50;

wrist_servoOutput(my_wrist.duty);
gripper_servoOutput(my_gripper.duty);
```

制御値`0`は設定最小側、`50`は中立、`100`は設定最大側を指令する。

## 使い方

1. サーボの信号線をヘッダで定義したGPIOへ接続する。サーボの電源とGNDは「注意事項」に従って接続する。
2. `ProgEx021_servoRotateDuty_Code.ino`で`EducarSpiconLib_v1.h`をインクルードし、`Arm`構造体をグローバル領域で一度だけ宣言する。
3. `setup()`でシリアル通信を開始してから`robotSetup()`を呼び出す。ここで車輪とサーボのLEDCを初期化する。初期化後、サーボの初期位置として`.duty`に`50`を設定する。
4. 周期実行する`timerloop()`から、構造体の`.duty`を引数にして2つのサーボ出力関数を呼び出す。
5. `loop()`で`.duty`に0～100の指令値を設定する。次のタイマー呼び出しで出力が更新される。

```cpp
#include "EducarSpiconLib_v1.h"
#include <Ticker.h>

Arm my_wrist;
Arm my_gripper;

Ticker timerLoop;
void timerloop();

void setup() {
  Serial.begin(115200);
  robotSetup();

  my_wrist.duty = 50;
  my_gripper.duty = 50;

  timerLoop.attach_ms(SAMP_TIMEms, timerloop);
}

void loop() {
  my_wrist.duty = 25;     // 手首を中立より最小側へ
  my_gripper.duty = 75;   // グリッパーを中立より最大側へ
  delay(1000);
}

void timerloop() {
  wrist_servoOutput(my_wrist.duty);
  gripper_servoOutput(my_gripper.duty);
}
```

`.duty`を変更するだけではPWM出力は更新されないため、`timerloop()`などから各出力関数を継続して呼び出す。実際のプロジェクトに車輪制御がある場合は、既存の`timerloop()`にサーボ出力関数の呼び出しを追加する。

## 注意事項

- `duty`は角度（度）でもLEDCへ直接渡すデューティ値でもなく、位置指令用の0～100の割合である。50を中立位置として扱う。
- `Arm my_wrist;`と`Arm my_gripper;`はスケッチ側のグローバル領域で一度だけ定義する。ライブラリ側は`extern`で参照し、出力関数にはそれぞれの`.duty`を渡す。
- 設定値は`MIN_DUTY < NEUTRAL_DUTY < MAX_DUTY`および`MIN_US <= NEUTRAL_US <= MAX_US`を満たすこと。各範囲が0にならないようにする。
- 実際の中立角度や端位置はサーボ・機構ごとに異なる。設定範囲は小さく試しながら調整し、機構に無理な力がかからないようにする。
- ESP32-DevKitCのGPIO16を手首サーボ、GPIO17をグリッパーサーボの信号出力に使用する仕様とする。搭載モジュールがWROVER系などPSRAM搭載モデルの場合、GPIO16・17がPSRAMに使われることがあるため、実機のモジュール型番とピン配置図で使用可能か確認する。
- GPIO16・17が使用できることを確認し、車輪やエンコーダなどの既存用途と重複しないピンを選ぶ。
- サーボには電流容量に余裕のある電源を使用する。外部電源を使う場合は、ESP32と電源のGNDを共通にする。
- サーボ用PWM周波数は50 Hzの設定例を使い、車輪用PWM（2,500 Hz）を流用しない。
