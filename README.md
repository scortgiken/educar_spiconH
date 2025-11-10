# educar_spiconH
教育用ロボットカーシリーズ。 車輪型移動ロボットのオドメトリを用いた自己位置推定、定速走行制御などの演習が可能。
## 本体
- EduCar_spicon_v1.0
- EduCar_spicon_v2.0: ハンド取り付け穴追加 
- Educar_spicon_v2.1: ライントレース用センサ取り付け穴追加


## ソフトウェア
- Educar-Spicon_HMM23-B_alpha.1: HMM23-B基板用ソフトウェア(EducarSpiconLib_alpha.1)
- Educar-Spicon_HMM25-B_alpha.1: HMM25-B基板用ソフトウェア(EducarSpiconLib_alpha.2)

## Howto start
- Prog06_contVfOmegaWifi_PIをダウンロード
- フォルダ内のProg06_contVfOmegaWifi_PI.inoをArduino IDEで開く(RotaryEncoder ライブラリが必要)
- ESP32に書き込む(IDEの設定は今後掲載）
- iphoneなどでwifi-->ESP****に接続
- webbrowerで、「192.168.0.3」 へに接続
- 走行用GUIで走行
