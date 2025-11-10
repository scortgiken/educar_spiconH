#include <WiFi.h>               //wifi用header fileをインクルード
#include "EducarSpiconLib_alpha.1.h"  


extern DiffWheelRobot my_rb;             //05-1:ロボットカーの構造体

void clientWebDisp();

//network設定  Gui用html script
const char* ssid     = "esp32ap-wifi";      //アクセスポイントのSSID
const char* password = "1234";     //アクセスポイントのパスワード
const IPAddress ip(192,168,0,3);
const IPAddress subnet(255,255,255,0);
WiFiServer server(80);
WiFiClient client;          // WiFiClinet object
//border:groove 40px orange;
//=================================================================
const char html[] =
"<!DOCTYPE html><html lang='ja'><head><meta charset='UTF-8'>\
<style>input {margin:10px;width:200px; font-size:35pt}\
div {font-size:20pt;color:red;text-align:center;width:800px; height:380px; }</style>\
<title>WiFi_Car Controller</title></head>\
<body><center><div><p>MechaRobo Controller</p>\
<form method='get'>\
<input type='submit' name='fo' value='前' /><br>\
<input type='submit' name='le' value='左' />\
<input type='submit' name='st' value='停止' />\
<input type='submit' name='ri' value='右' /><br>\
<input type='submit' name='ba' value='後' />\
</form></div> ";

//======================================================================
float vf_d_val = 0.05;           //m/s  0.1 m/sはノイズの影響で暴走する可能性あり。
float dphi_d_val = 30*Deg2Rad;   //rad/s

//---serverSetup関数
void serverSetup(){

   Serial.println();
   Serial.print("Setting AP......");
   delay(100);
   WiFi.softAP(ssid, password);
   delay(500);
   WiFi.softAPConfig(ip,ip,subnet);

   IPAddress myIP = WiFi.softAPIP(); 
 
   Serial.print("SSID :");
   Serial.println(ssid);
   Serial.print("AP IP sddress ");
   Serial.println(myIP);

   server.begin();
  
}


//-- プログラムループ
void wifiLoop() {

  client = server.available();    // clientからの接続確認
  if(client) clientWebDisp();     // clientから接続があったら、html表示

 }


//---Clinet WebDisplay 関数 
void clientWebDisp(){
 
      Serial.println("New Client.");
      String currentLine = "";
      while (client.connected()) {
           if (client.available()) {
              char c = client.read();
              Serial.write(c);
               if (c == '\n') {
                  if (currentLine.length() == 0) {
                     client.println("HTTP/1.1 200 OK");
                     client.println("Content-type:text/html");  // html header
                     client.println();
                     client.print(html);                        // html body                     
                     client.println("</center></body></html>"); 
                     client.println();
                     break;
                  } else {
                     currentLine = "";
                  }
               } else if (c != '\r') {
                  currentLine += c;
               }

            if (currentLine.endsWith("GET /?fo")) {
                  my_rb.vf_d = vf_d_val;      
                  my_rb.dphi_d = 0;              
            }
            if (currentLine.endsWith("GET /?le")) {
                  my_rb.dphi_d = dphi_d_val; 
            }
            if (currentLine.endsWith("GET /?ri")) {
                  my_rb.dphi_d = -dphi_d_val; 
            }
            if (currentLine.endsWith("GET /?ba")) {
                  my_rb.vf_d = -vf_d_val;
                  my_rb.dphi_d = 0;        
            }
            if (currentLine.endsWith("GET /?st")) {
                   my_rb.vf_d = 0;   
                   my_rb.dphi_d = 0;  
            }   

            
      } //if(client.available())
    } //while
      client.stop();
      Serial.println("Client Disconnected.");
   }// clientWebDisp(){




 
