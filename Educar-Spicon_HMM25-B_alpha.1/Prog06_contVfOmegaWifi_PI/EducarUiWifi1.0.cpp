/*  2026/01/12  EducarUiWifi1.0 :  interface_Wifi.cppをベースに変更。
 *   　　　　　　　　　　　　　　　　　　速度変更ボタン追加。選択ボタンのみ色変更機能追加
 *  
 */

#include <WiFi.h>
#include "EducarSpiconLib_alpha.1.h"

extern DiffWheelRobot my_rb;
void clientWebDisp();

// ================= WiFi設定 =================
const char* ssid     = "esp32ap-wifi";
const char* password = "1234";

const IPAddress ip(192,168,0,3);
const IPAddress subnet(255,255,255,0);

WiFiServer server(80);
WiFiClient client;

// ================= ロボット制御 初期化=================
float vf_d_val   = 0.05;              // m/s
float dphi_d_val = 50 * Deg2Rad;      // rad/s

// ================= 速度状態 =================

enum VelSel {
  VEL_005,
  VEL_01,
  VEL_02
};

VelSel currentVel = VEL_005;   // 初期速度

// ===== 動作状態 =====
enum MoveSel {
  MOVE_FO,
  MOVE_BA,
  MOVE_LE,
  MOVE_RI,
  MOVE_ST
};

MoveSel currentMove = MOVE_ST;

// ================= HTML生成 =================
String makeHTML() {


  String a005 = "";
  String a01  = "";
  String a02  = "";

  if (currentVel == VEL_005) a005 = " active";
  if (currentVel == VEL_01)  a01  = " active";
  if (currentVel == VEL_02)  a02  = " active";

// ---- 動作ボタン状態 ----
  String fo="", ba="", le="", ri="", st="";
  if (currentMove == MOVE_FO) fo = " active";
  if (currentMove == MOVE_BA) ba = " active";
  if (currentMove == MOVE_LE) le = " active";
  if (currentMove == MOVE_RI) ri = " active";
  if (currentMove == MOVE_ST) st = " active";


  String html =
    "<!DOCTYPE html><html lang='ja'><head><meta charset='UTF-8'>"
    "<style>"
    "input{margin:10px;width:250px;font-size:35pt;}"
    "input.dir{background:#ddd;color:black;}"
    "input.dir.active{background:#1e90ff;color:white;}"   /* 青 */  
    "input.vel{background:#ddd;color:black;}"
    "input.vel.active{background:orange;color:white;}"
    "div{font-size:20pt;color:red;text-align:center;width:900px;height:380px;}"
    "</style>"
    "<title>WiFi_Car Controller</title></head>"
    "<body><center><div><p>MechaRobo Controller</p>"
    "<form method='get'>"

    "<input type='submit' name='fo' value='前' class='dir" + fo + "'><br>"
    "<input type='submit' name='le' value='左' class='dir" + le + "'>"
    "<input type='submit' name='st' value='停止' class='dir" + st +"'>"
    "<input type='submit' name='ri' value='右' class='dir" + ri + "'><br>"
    "<input type='submit' name='ba' value='後' class='dir" + ba + "'><br>"

    "<input type='submit' name='v005' value='0.05m/s' class='vel" + a005 + "'>"
    "<input type='submit' name='v01'  value='0.1m/s'  class='vel" + a01  + "'>"
    "<input type='submit' name='v02'  value='0.2m/s'  class='vel" + a02  + "'>"

    "</form></div></center></body></html>";

  return html;
}

// ================= AP起動 =================
void serverSetup() {

  Serial.println();
  Serial.println("Setting AP...");

  WiFi.softAP(ssid, password);
  delay(500);
  WiFi.softAPConfig(ip, ip, subnet);

  Serial.print("SSID : ");
  Serial.println(ssid);
  Serial.print("AP IP address : ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

// ================= WiFiループ =================
void wifiLoop() {
  client = server.available();
  if (client) clientWebDisp();
}

// ================= Web処理 =================
void clientWebDisp() {

  Serial.println("New Client.");
  String currentLine = "";

  while (client.connected()) {
    if (client.available()) {

      char c = client.read();
      Serial.write(c);

      if (c == '\n') {

        // ヘッダ終了
        if (currentLine.length() == 0) {

          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.print(makeHTML());
          client.println();
          break;
        }
        currentLine = "";

      } else if (c != '\r') {
        currentLine += c;
      }

      // ===== 移動指令 =====
      if (currentLine.endsWith("GET /?fo")) {
        my_rb.vf_d = vf_d_val;
        my_rb.dphi_d = 0;
        currentMove = MOVE_FO;
      }
      if (currentLine.endsWith("GET /?ba")) {
        my_rb.vf_d = -vf_d_val;
        my_rb.dphi_d = 0;
        currentMove = MOVE_BA;  
      }
      if (currentLine.endsWith("GET /?le")) {
        my_rb.dphi_d = dphi_d_val;
        currentMove = MOVE_LE;
      }
      if (currentLine.endsWith("GET /?ri")) {
        my_rb.dphi_d = -dphi_d_val;
        currentMove = MOVE_RI;
      }
      if (currentLine.endsWith("GET /?st")) {
        my_rb.vf_d = 0;
        my_rb.dphi_d = 0;
        currentMove = MOVE_ST;
      }

      // ===== 速度選択 =====
      if (currentLine.endsWith("GET /?v005")) {
        vf_d_val = 0.05;
        currentVel = VEL_005;
      }
      if (currentLine.endsWith("GET /?v01")) {
        vf_d_val = 0.1;
        currentVel = VEL_01;
      }
      if (currentLine.endsWith("GET /?v02")) {
        vf_d_val = 0.2;
        currentVel = VEL_02;
      }
           
      
    }
  }

  client.stop();
  Serial.println("Client Disconnected.");
}




 
