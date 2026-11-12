/* -----------------------------------------------------------------------------
 *  Prog_pidWheeltraj_struct.ino:
 *  各車輪を定速で回転するプログラム演習
 *                                            2023.06.27  by jeong
 *                                            (230712_OG)追加
--------------------------------------------------------------------------------*/
#include <Wire.h>                       
#include <Ticker.h>
#include "EducarSpiconLib_v1.h"    //ロボット制御用ヘッダファイル

Ticker timerLoop;
void timerloop();                  //タイマー割込み timerloop関数
Ticker timerLoopMonitor;
void timerloopMonitor();                 //状態変数表示のためのtimerloop関数

//--Struct 宣言
Wheel my_lw;                      //左車輪構造体
Wheel my_rw;                      //右車輪構造体
DiffWheelRobot my_rb;             //05-1:ロボットカーの構造体
IMU my_IMU;                       //IMU構造体(230626_OG)


//--グローバル変数
int motionNum = 1;                //動作順番制御変数(case文の変数)

///////////SETUP///////////////////////////////
void setup(){
  Wire.begin();                  //(230712_OG)
  
  Serial.begin(115200);          //　Serial_1との通信速度(PCとの通信 115200 bpsに設定 
  
  robotSetup();                  //　ロボット制御用セットアップ関数 robotのGIO等の設定

  delay(5000);                   // 5s後に制御開始
  
  timerLoop.attach_ms(SAMP_TIMEms,timerloop);  //timerloop関数をSAMP_TIMEmsで設定
  timerLoopMonitor.attach_ms(Disp_TIMEms,timerloopMonitor);  //timerloop関数をSAMP_TIMEmsで設定
                 
}


//////////ユーザプログラム/////////////////////////////
void loop() {
 
 my_lw.duty = 20;
 my_rw.duty = 20;
 delay(3000);
 
 my_lw.duty = 0;
 my_rw.duty = 0;
 delay(1000);
 
 my_lw.duty = -20;
 my_rw.duty = -20;
 delay(3000);

 my_lw.duty = 0;
 my_rw.duty = 0;
 delay(1000);
}  //loop()


///////////Timerloop //////////////////////////////////////////
void timerloop(){
  //------一定時間に呼び出される関数。正確な時間間隔で実行する必要があるコードを書く
  
  wh_motorOutputL(my_lw.duty);                //左車輪モータへpwm出力
  wh_motorOutputR(my_rw.duty);                //右車輪モータへpwm出力
  
 }


///---serialで表示---
void timerloopMonitor(){
             
  Serial.print("  dutyL:");  Serial.print(my_lw.duty);      //左アクチュエータのduty比
  Serial.print("  dutyR:");  Serial.print(my_rw.duty);      //右アクチュエータのduty比
  Serial.println();
  
}
///-----











 
