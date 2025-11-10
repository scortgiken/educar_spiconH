/* -----------------------------------------------------------------------------
 *  Prog06_contVfOmegaWifi_PI.ino:
 *  Wifi接続でロボットを定速走行制御(PI制御)
 *  ロボット側IP: 192.168.0.3
 *                                            20240420: jeong
--------------------------------------------------------------------------------*/
#include <Wire.h>                  
#include <Ticker.h>
#include "EducarSpiconLib_alpha.1.h"    //ロボット制御用ヘッダファイル 

extern void serverSetup();              //wifiセットアップ
extern void wifiLoop();                 //wifiループ

Ticker timerLoop;                       //timerLoop宣言
void timerloop();                       //タイマー割込み timerloop関数
Ticker timerLoop1;
void timerloop1();                 //状態変数表示のためのtimerloop関数

//--Struct 宣言
Wheel my_lw;                      //左車輪構造体
Wheel my_rw;                      //右車輪構造体
DiffWheelRobot my_rb;             //ロボットカーの構造体
IMU my_IMU;                       //IMU構造体

///////////SETUP///////////////////////////////
void setup(){
  
  Wire.begin();                  //I2c通信開始
  Serial.begin(115200);          //　Serial_1との通信速度(PCとの通信 115200 bpsに設定 
  robotSetup();                  //　ロボット制御用セットアップ関数 robotのGIO等の設定
  serverSetup();                 //  wifi interface セットアップ

  timerLoop.attach_ms(SAMP_TIMEms,timerloop);  //timerloop関数をSAMP_TIMEmsで設定
  timerLoop1.attach_ms(Disp_TIMEms,timerloop1);  //timerloop関数をSAMP_TIMEmsで設定
  //動作コードはここに書く
                  
}


//////////ユーザプログラム/////////////////////////////
void loop() {
  wifiLoop();   // wifi通信の更新
  delay(10);
}


///////////Timerloop //////////////////////////////////////////
//------一定時間に呼び出される関数。正確な時間間隔で実行する必要があるコードを書く
void timerloop(){

//------------------robotの状態計測-------------
  wh_calAngleL();                       //左車輪用角度計測 
  wh_calAngleR();                       //右車輪用角度計測
  wh_calAngVelL();
  wh_calAngVelR();
  rb_calFowardVelocity();               //ロボットの前進速度を計算
  rb_calSteeringVelocity();             //ロボットの旋回速度を計算
  rb_calFowardDistance();               //ロボットの前進速度を積分して距離を計算
  rb_calSteeringRadian();               //ロボットの旋回速度を積分して角度を計算

  sen_cal_IMU();                       //IMUセンサ計測
  sen_cal_angle();                     //IMUセンサ姿勢角計算

//----------------robotの状態取得-------------  
  my_lw.newAng = wh_getAngleL();          //左車輪の角度を取得
  my_rw.newAng = wh_getAngleR();          //右車輪の角度を取得
  my_lw.newAngVel = wh_getAngVelL();      //左車輪の角速度を取得
  my_rw.newAngVel = wh_getAngVelR();      //右車輪の角速度を取得
    
  my_rb.vf = rb_getFowardVelocity();      //ロボットの前方移動速度を取得
  my_rb.dphi = rb_getSteeringVelocity();  //ロボットの旋回速度を取得
  my_rb.pf = rb_getFowardDistance();      //ロボットの前方移動距離を取得
  my_rb.phi = rb_getSteeringRadian();     //ロボットの旋回角度を取得

  my_IMU.angle_pitch = sen_get_angle_pitch();  //ピッチ角の取得関数を実行
  my_IMU.angle_roll = sen_get_angle_roll();    //ロール角の取得関数を実行                 
  my_IMU.ay = sen_get_ay();                    //進行方向加速度の取得関数を実行
  my_IMU.az = sen_get_az();                    //鉛直方向加速度の取得関数を実行
  
//-----------------軌道生成
  rb_calDiffWheelGoalAngularVelocity(my_rb.vf_d, my_rb.dphi_d);      //vf_dとdphi_dで目標車輪角速度を計算   
  my_lw.angVel_d = wh_getGoalAngVelL()*Rad2Deg;                      //左車輪の目標車輪角速度(deg/s)の取得
  my_rw.angVel_d = wh_getGoalAngVelR()*Rad2Deg;                      //右車輪の目標車輪角速度(deg/s)の取得

//------------------PI制御
  my_lw.duty=wh_piAngVelL(my_lw.angVel_d, 0.1, 0.05);                 //左車輪のPI速度制御入力の取得
  my_rw.duty=wh_piAngVelR(my_rw.angVel_d, 0.1, 0.05);                 //右車輪のPI速度制御入力の取得

 
//------------------モータドライベーへの出力   
  wh_motorOutputL(my_lw.duty);                //左車輪モータへpwm出力
  wh_motorOutputR(my_rw.duty);                //右車輪モータへpwm出力

 }


void timerloop1(){

  //--【ロボットの前進移動距離と旋回角度を表示」              
   Serial.print("  vf:");  Serial.print(my_rb.vf);                  //05-2:ロボットの前進距離を表示
  Serial.print("  dphi:");  Serial.print(my_rb.dphi*Rad2Deg);      //05-2:ロボットの旋回角度を表示
  Serial.print("  angveld:");  Serial.print(my_lw.angVel_d);       //05-2:ロボットの旋回角度を表示
  Serial.print("  Pf:");  Serial.print(my_rb.pf);                  //05-2:ロボットの前進距離を表示
  Serial.print("  phi:");  Serial.print(my_rb.phi*Rad2Deg);       //05-2:ロボットの旋回角度を表示
  Serial.println();

}
///-----












 
