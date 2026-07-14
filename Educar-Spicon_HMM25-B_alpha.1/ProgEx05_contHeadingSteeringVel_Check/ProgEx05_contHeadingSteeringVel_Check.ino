/* -----------------------------------------------------------------------------
 *  ProgEx05_calHeadingDistVel_code.ino:
 *  前方移動速度、距離の計測プログラム演習
 *                                            2024.06.27  by jeong                           
--------------------------------------------------------------------------------*/
#include <Wire.h>                       
#include <Ticker.h>
#include "EducarSpiconLib_alpha.1.1.h"    //ロボット制御用ヘッダファイル 

Ticker timerLoop;
void timerloop();                  //タイマー割込み timerloop関数
Ticker timerLoopMonitor;
void timerloopMonitor();                 //状態変数表示のためのtimerloop関数

//--Struct 宣言
Wheel my_lw;                      //左車輪構造体
Wheel my_rw;                      //右車輪構造体
DiffWheelRobot my_rb;             //05-1:ロボットカーの構造体
IMU my_IMU;                       //IMU構造体(230626_OG)


//---制御flag
float p_val=0.2;                //車輪速度p制御ゲイン
float i_val=0.01;                //車輪速度I制御
int motionNum = 1;              //動作順番制御変数(case文の変数)

///////////SETUP///////////////////////////////
void setup(){
  Wire.begin();                  //
  Serial.begin(115200);          //　Serial_1との通信速度(PCとの通信 115200 bpsに設定 
  
  robotSetup();                  //　ロボット制御用セットアップ関数 robotのGIO等の設定

  Serial.println("Turn On Motor Switch. Robot is moving in 5 seconds !!!");
  delay(5000); 
  
  timerLoop.attach_ms(SAMP_TIMEms,timerloop);  //timerloop関数をSAMP_TIMEmsで設定
  timerLoopMonitor.attach_ms(Disp_TIMEms,timerloopMonitor);  //timerloop関数をSAMP_TIMEmsで設定
                 
}


//////////ユーザプログラム/////////////////////////////
//------- プログラムループ(繰り返し実行) この中に動作プログラムを書く
void loop() {

  switch(motionNum){

    case 1: 
      my_rb.vf_d = 0.05;
      my_rb.dphi_d = 0*Deg2Rad;  
      if(my_rb.pf >= 0.1) motionNum = 2;
      break;
      
    case 2: 
      my_rb.vf_d = 0;
      my_rb.dphi_d = 0*Deg2Rad;
      delay(3000);
      motionNum = 3;
     break;

    case 3: 
      my_rb.vf_d = 0;
      my_rb.dphi_d = 60*Deg2Rad;  
      if(my_rb.phi*Rad2Deg >= 30) motionNum = 4;
     break;

    case 4:
      my_rb.vf_d = 0;
      my_rb.dphi_d = 0*Deg2Rad;
      break;

   default: break;
    
  }
  
 delay(10);
}  //loop()


///////////Timerloop //////////////////////////////////////////
//------一定時間に呼び出される関数。正確な時間間隔で実行する必要があるコードを書く
void timerloop(){

//------------------robotの状態計測-------------
  wh_calAngleL();                       //左車輪用角度計測 
  wh_calAngleR();                       //右車輪用角度計測
  wh_calAngVelL();                      //左車輪用角速度計測
  wh_calAngVelR();                      //右車輪用角速度計測
  rb_calFowardVelocity();               //ロボットの前進速度計測
  rb_calFowardDistance();               //ロボットの前進速度を積分して移動距離を計算する
  rb_calSteeringVelocity();             //ロボットの旋回速度を計算
  rb_calSteeringRadian();               //ロボットの旋回速度を積分して角度を計算
  
//----------------robotの状態取得-------------  プログラムで使う必要があれば使う。
  my_lw.newAng = wh_getAngleL();         //左車輪用角度取得
  my_rw.newAng = wh_getAngleR();         //右車輪用角度取得
  my_lw.newAngVel = wh_getAngVelL();     //左車輪用角速度取得
  my_rw.newAngVel = wh_getAngVelR();     //右車輪用角速度取得
  my_rb.vf = rb_getFowardVelocity();     //ロボットの前方移動速度を取得
  my_rb.pf = rb_getFowardDistance();      //ロボットの前方移動距離を取得
  my_rb.dphi = rb_getSteeringVelocity();  //ロボットの旋回速度を取得
  my_rb.phi = rb_getSteeringRadian();     //ロボットの旋回角度を取得

//-----------------軌道生成
  rb_calDiffWheelGoalAngularVelocity(my_rb.vf_d, my_rb.dphi_d);      //vf_dとdphi_dで目標車輪角速度を計算   
  my_lw.angVel_d = wh_getGoalAngVelL()*Rad2Deg;                      //左車輪の目標車輪角速度(deg/s)の取得
  my_rw.angVel_d = wh_getGoalAngVelR()*Rad2Deg;                      //右車輪の目標車輪角速度(deg/s)の取得

//---------------モータ制御------------------------
  my_lw.duty=wh_piAngVelL(my_lw.angVel_d, p_val, i_val);             
  my_rw.duty=wh_piAngVelR(my_rw.angVel_d, p_val, i_val);
 
//------------------モータドライベーへの出力   
  wh_motorOutputL(my_lw.duty);                //左車輪モータへpwm出力
  wh_motorOutputR(my_rw.duty);                //右車輪モータへpwm出力 
 }

///---serialで表示---
void timerloopMonitor(){        
  Serial.print("  vf:");  Serial.print(my_rb.vf);                  //05-2:ロボットの前進距離を表示
  Serial.print("  Pf:");  Serial.print(my_rb.pf);                  //05-2:ロボットの前進距離を表示
  Serial.print("  dphi:");  Serial.print(my_rb.dphi*Rad2Deg);      //05-2:ロボットの旋回角度を表示
  Serial.print("  phi:");  Serial.print(my_rb.phi*Rad2Deg);       //05-2:ロボットの旋回角度を表示
  Serial.println();
}
///-----



 
