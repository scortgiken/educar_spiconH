#　要求事項
- Arduino IDEを2.x系へ変更
- Esp32ライブラリ espressif system 3.x系へ変更
- ほかのコードは変更せず、ledcSetup()関係の関数を3.x系へ変更



# 変更点

## ESP32 Arduino 3.x系への対応

Arduino IDEを2.x系、ESP32のEspressif Arduino環境を3.x系に変更する。

### LEDC（PWM）関連

対象ファイル: `ProgEx01_wheelRotatePwm_Code/EducarSpiconLib_alpha.1.1.cpp`

- `robotSetup()`内の`ledcSetup()`と`ledcAttachPin()`を`ledcAttachChannel()`に置き換える。
- 左車輪は`PWMA_PIN`と`LEDC_CHANNEL_0`、右車輪は`PWMB_PIN`と`LEDC_CHANNEL_1`を引き続き割り当てる。
- PWM周波数（`LEDC_SERVO_FREQ`）と分解能（`LEDC_TIMER_BIT`）は維持する。
- 左右のモーター出力関数では、`ledcWrite()`を`ledcWriteChannel()`に置き換え、各チャンネルへデューティ値を出力する。

### 変更対象外

上記のLEDC API移行以外の制御処理、ピン設定、チャンネル割り当て、`.ino`の動作内容は変更しない。


# 環境変更手順

## Arduino IDE 2.x系への変更

1. Arduino公式サイトからArduino IDE 2.x系をダウンロードし、インストールする。
2. Arduino IDEを起動し、既存のスケッチと設定が利用できることを確認する。
- download (https://www.arduino.cc/en/software/#ide)
![alt text](image.png)
- インストール
![alt text](image-1.png)
- 同意する
![alt text](image-2.png)
- 現在のユーザのみにインストール
![alt text](image-3.png)
- インストール先（デフォルト）
![alt text](image-4.png)
- インストール
![alt text](image-5.png)
- インストール完了
![alt text](image-6.png)
- Arduino起動
    - USBドライバーインストール
    - Updates are available for... LATER
![alt text](image-7.png)

## ESP32 Arduino（Espressif Systems）3.x系への変更

1. Arduino IDEで「ファイル」>「基本設定」を開く。
![alt text](image-8.png)
2. 「追加のボードマネージャのURL」に次のURLを追加する。すでにURLが登録されている場合は、既存のURLを残して追加する。
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   ![alt text](image-9.png)
3. 「ツール」>「ボード」>「ボードマネージャ」を開く。
![alt text](image-10.png)
4. `esp32`を検索し、提供元が「Espressif Systems」のボードパッケージから3.x系のバージョンを選んでインストールする。
![alt text](image-11.png)
5. 「ツール」>「ボード」で使用するESP32ボードを選択し、必要に応じてシリアルポートを選択する。
![alt text](image-12.png)
6. 「ファイル」>「開く」でinoファイルを選択する
![alt text](image-13.png)
7. 「→」を選択してコンパイル＆ダウンロードを開始する。
![alt text](image-14.png)
8. 「Hard resettting via RTS pin...」が表示されたら成功
![alt text](image-15.png)
## Rotary Encoder Libraryのインストール
9. https://github.com/mathertel/RotaryEncoder?utm_source=chatgpt.comから RotaryEncoder-master.zip ファイルをダウンロード。 
![alt text](image-16.png)
10. 「スケッチ→ライブラリをインストール」で、RotaryEncoder-master.zip を選択してインストール。
![alt text](image-17.png)
![alt text](image-18.png)
11. 「Document→Arduino→libraries」の中にRotary Encoder libraryがインストールされているか確認
![alt text](image-19.png)