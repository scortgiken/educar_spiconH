# EducarSpiconLib_v1

ESP32を使った差動二輪ロボットの制御演習用プログラム集です。車輪エンコーダの読み取り、速度PI制御、走行状態の計測、シリアル／Wi-Fi操作、および手首・グリッパーサーボの出力例を含みます。

## 動作環境

- Arduino IDE 2.x
- ESP32 Arduino（Espressif Systems）3.x系
- RotaryEncoderライブラリ

セットアップ手順は[環境構築手順](Document/Installation/installationSetupManual.md)を参照してください。各プログラムは、それぞれのスケッチフォルダー内にある`.ino`ファイルをArduino IDEで開いて使用します。

## フォルダー構成

| フォルダー／ファイル | 内容 |
|---|---|
| `ProgEx01_wheelRotatePwm_Code/` | 左右車輪へPWM指令を出す基本例 |
| `ProgEx02_servoRotateDuty_Code/` | 車輪と2つのサーボを動かす例 |
| `Prog05_contVfOmegaSerial_PI/` | シリアル入力による前進・旋回速度PI制御 |
| `Prog06_contVfOmegaWifi_PI/` | WebUIから操作する前進・旋回速度PI制御 |
| `ProgCheck_contHeadingSteeringVel/` | 前進・旋回速度と距離・角度の計測確認 |
| `Document/requirement.md` | ESP32 Arduino 3.x系への移行要件 |
| `Document/changeLog/change.md` | 前バージョンからの変更内容 |
| `Document/Iinstallation/` | Arduino IDE、ESP32環境、ライブラリの設定手順 |

各プログラムフォルダーには、使用する`EducarSpiconLib_v1.h`と`EducarSpiconLib_v1.cpp`が含まれます。`Prog06_contVfOmegaWifi_PI/`には、Wi-FiとWebUIを扱う`EducarUiWifi1.0.cpp`もあります。フォルダーごとにスケッチとして開き、別のプログラムの`.ino`やライブラリ実装と混在させないでください。

## 前バージョンからの主な変更点

旧バージョン（alpha.1.1）から、Arduino IDE 2.xおよびESP32 Arduino 3.x系に対応しました。

- LEDC初期化を`ledcSetup()`／`ledcAttachPin()`から`ledcAttachChannel()`へ移行。
- PWM出力を`ledcWrite()`から`ledcWriteChannel()`へ移行。左右車輪のチャンネル割り当て、周波数、分解能は維持。
- ライブラリのファイル名を`EducarSpiconLib_alpha.1.1.h/.cpp`から`EducarSpiconLib_v1.h/.cpp`へ変更。
- 手首・グリッパー用の`wrist_servoOutput()`と`gripper_servoOutput()`を追加。`Arm`構造体の位置指令duty（0～100）をパルス幅に変換して出力し、50を中立位置として扱います。

## 各プログラム

### `ProgEx01_wheelRotatePwm_Code/`

左右の車輪へ正転・停止・逆転のPWM指令を順番に出す基本演習です。車輪モーター出力の確認に使います。

### `ProgEx02_servoRotateDuty_Code/`

左右の車輪に加えて、手首とグリッパーのサーボを動かす演習です。サーボ指令を最小側、中央、最大側に切り替えます。スケッチファイル名は`ProgEx021_servoRotateDuty_Code.ino`です。サーボの設定・配線上の注意は[servoOutput_spec.md](ProgEx02_servoRotateDuty_Code/servoOutput_spec.md)を参照してください。

### `Prog05_contVfOmegaSerial_PI/`

シリアルモニターから前進速度`vf_d`と旋回速度`dphi_d`を入力し、左右車輪の速度PI制御で走行させる演習です。入力形式は`01 vf_d dphi_d`です。

### `Prog06_contVfOmegaWifi_PI/`

Wi-Fi経由のWebUIからロボットを操作し、前進・旋回速度を左右車輪の速度PI制御で実現する演習です。使用前に`EducarUiWifi1.0.cpp`のWi-Fi設定を確認・変更してください。

### `ProgCheck_contHeadingSteeringVel/`

一定距離の前進と旋回を行い、前進速度・距離および旋回速度・角度の計測値をシリアルモニターに表示する確認用プログラムです。

## 使用上の注意

- 各プログラムはロボット実機のピン配置やモーター・サーボに合わせて設定を確認してから使用してください。
- サーボのピン、PWM設定、パルス幅は実機に合わせて確認してください。サーボ用電源やGPIOの注意事項は[サーボ出力仕様](ProgEx02_servoRotateDuty_Code/servoOutput_spec.md)を参照してください。
- `Prog06_contVfOmegaWifi_PI/`のWi-Fi設定はソース内で管理されています。共有・公開する場合は、個人や組織の認証情報を含めないようにしてください。

<img width="1840" height="1272" alt="image" src="https://github.com/user-attachments/assets/38d0f6ce-e9ba-49fa-a564-81e9ee446968" />
