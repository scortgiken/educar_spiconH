/* -----------------------------------------------------------------------------
 *  Prog_pidWheeltraj_struct.ino:
 *  各車輪を定速で回転するプログラム演習
 *                                            2023.06.27  by jeong
 *                                            (230712_OG)追加
--------------------------------------------------------------------------------*/
#include <Wire.h>                       
#include <Ticker.h>
#include "EducarSpiconLib_alpha.1.h"    //ロボット制御用ヘッダファイル 

Ticker timerLoop;
void timerloop();                  //タイマー割込み timerloop関数
Ticker timerLoop1;
void timerloop1();                 //状態変数表示のためのtimerloop関数

//--Struct 宣言
Wheel my_lw;                      //左車輪構造体
Wheel my_rw;                      //右車輪構造体
DiffWheelRobot my_rb;             //05-1:ロボットカーの構造体
IMU my_IMU;                       //IMU構造体(230626_OG)

///////////SETUP///////////////////////////////
void setup(){
  Wire.begin();                  //(230712_OG)
  
  Serial.begin(115200);          //　Serial_1との通信速度(PCとの通信 115200 bpsに設定 
  
  robotSetup();                  //　ロボット制御用セットアップ関数 robotのGIO等の設定

  //serial command 操作法
  Serial.println("----------------------------------------------------------------------");
  Serial.println("Input (OP,vf_d[m/s], dphi_d[deg/s] command via serial and type enter");
  Serial.println("ex) my_rb.vf_d= 0.05 m/s, my_rb.dphi_d =30deg/s --> 01 0.05 30 ");
  Serial.println("    Max(0.08 m/s, 60deg/s) ");
  Serial.println("----------------------------------------------------------------------");
  while(1){ 
    if(serialCommandInterface()) break;   
  }
  Serial.println("Driving Start in 3s !!");
  delay(3000);

  timerLoop.attach_ms(SAMP_TIMEms,timerloop);  //timerloop関数をSAMP_TIMEmsで設定
  timerLoop1.attach_ms(Disp_TIMEms,timerloop1);  //timerloop関数をSAMP_TIMEmsで設定
                 
}


//////////ユーザプログラム/////////////////////////////
//------- プログラムループ(繰り返し実行) この中に動作プログラムを書く
void loop() {
 serialCommandInterface();
 delay(10);
}  //loop()


///////////Timerloop //////////////////////////////////////////
//------一定時間に呼び出される関数。正確な時間間隔で実行する必要があるコードを書く
void timerloop(){

//------------------robotの状態計測-------------
  wh_calAngleL();                       //04:左車輪用角度計測 
  wh_calAngleR();                       //04:右車輪用角度計測
  wh_calAngVelL();
  wh_calAngVelR();
  rb_calFowardVelocity();               //05-1: ロボットの前進速度を計算
  rb_calSteeringVelocity();             //05-1: ロボットの旋回速度を計算
  rb_calFowardDistance();               //05-2: ロボットの前進速度を積分して距離を計算する。
  rb_calSteeringRadian();               //05-2: ロボットの旋回速度を積分して角度を計算する。

  sen_cal_IMU();                       //IMUセンサ計測(230712_OG)
  sen_cal_angle();                     //IMUセンサ姿勢角計算(230712_OG)


//----------------robotの状態取得-------------  プログラムで使う必要があれば使う。
  my_lw.newAng = wh_getAngleL();       
  my_rw.newAng = wh_getAngleR();       
  my_lw.newAngVel = wh_getAngVelL(); 
  my_rw.newAngVel = wh_getAngVelR();  
    
  my_rb.vf = rb_getFowardVelocity();      //05-1:ロボットの前進速度を取得
  my_rb.dphi = rb_getSteeringVelocity();  //05-1:ロボットの旋回速度を取得
  my_rb.pf = rb_getFowardDistance();      //05-2:ロボットの前進距離を取得
  my_rb.phi = rb_getSteeringRadian();     //05-2:ロボットの旋回角度を取得

  my_IMU.angle_pitch = sen_get_angle_pitch();  //ピッチ角の取得関数を実行(230712_OG)
  my_IMU.angle_roll = sen_get_angle_roll();    //ロール角の取得関数を実行
  my_IMU.ay = sen_get_ay();                    //進行方向加速度の取得関数を実行(230712_OG)
  my_IMU.az = sen_get_az();                    //鉛直方向加速度の取得関数を実行
  
//-----------------軌道生成
  rb_calDiffWheelGoalAngularVelocity(my_rb.vf_d, my_rb.dphi_d);      //05-3: vf_dとdphi_dで目標車輪角速度を計算   
  my_lw.angVel_d = wh_getGoalAngVelL()*Rad2Deg;                      //05-3: 左車輪の目標車輪角速度(deg/s)の取得
  my_rw.angVel_d = wh_getGoalAngVelR()*Rad2Deg;                      //05-3: 右車輪の目標車輪角速度(deg/s)の取得

//------------------PID制御
  my_lw.duty=wh_piAngVelL(my_lw.angVel_d, 0.2, 0.1);            
  my_rw.duty=wh_piAngVelR(my_rw.angVel_d, 0.2, 0.1);

  
//------------------モータドライベーへの出力   
  wh_motorOutputL(my_lw.duty);                //左車輪モータへpwm出力
  wh_motorOutputR(my_rw.duty);                //右車輪モータへpwm出力
  
 }


///---serialで表示---
void timerloop1(){

  //--【ロボットの前進速度と旋回速度を表示」              
  Serial.print("  vf:");  Serial.print(my_rb.vf);                  //ロボットの前進距離を表示
  Serial.print("  dphi:");  Serial.print(my_rb.dphi*Rad2Deg);      //ロボットの旋回角度を表示
  Serial.print("  angveld:");  Serial.print(my_lw.angVel_d);       //ロボットの旋回角度を表示
  Serial.print("  Pf:");  Serial.print(my_rb.pf);                  //ロボットの前進距離を表示
  Serial.print("  phi:");  Serial.print(my_rb.phi*Rad2Deg);        //ロボットの旋回角度を表示
  Serial.println();
  
}
///-----


////////////Copy and Paste //////////////////////////文字列を分割
int split(String data, char delimiter, String *dst){
    int index = 0;
    int arraySize = (sizeof(data)/sizeof((data)[0]));  
    int datalength = data.length();
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == delimiter ) {
            index++;
            if ( index > (arraySize - 1)) return -1;
        }
        else dst[index] += tmp;
    }
    return (index + 1);
}
///////////////////////////////////////////

//----------------------------------------------
int serialCommandInterface(){
  
  String cmds[10] = {"\0"};                        // String Command Buffer
  String strCmd = Serial.readStringUntil('\n');    // Read string from serial until '\n'   
  int index = split(strCmd,' ',cmds);              // Split command
  //float vf_d_temp;
  //float dphi_d_temp;

   //OP:"01" means 'setGoalOne'
   if(cmds[0]=="01"){       
         float vf_d_temp = cmds[1].toFloat(); 
         if(vf_d_temp > -0.08 && vf_d_temp < 0.08 ) my_rb.vf_d = vf_d_temp;
         else { my_rb.vf_d=my_rb.vf_d; Serial.print(" speed over (under 0.08 m/s)"); Serial.println();}
         float dphi_d_temp = cmds[2].toFloat();
         if(dphi_d_temp > -90 && dphi_d_temp < 90) my_rb.dphi_d = dphi_d_temp*Deg2Rad;
         else { my_rb.dphi_d=my_rb.dphi_d; Serial.print(" speed over (under 60deg/s) "); Serial.println(); }
//       my_rb.vf_d =  cmds[1].toFloat(); 
  //      my_rb.dphi_d = cmds[2].toFloat()*Deg2Rad;    
        if(index==3) return 1;  
     }
   else return 0;    
}










 
