
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define FWversion "1.00"  ///ファームウェアバージョン設定値
#define interval 1000     ///センサー計測周期(mm sec)
///#define USHRT_MAX 65535   ///照度計エラー値

#include <ArduinoJson.h>
#include <M5Stack.h>
#include "M5_ENV.h"
#include <M5GFX.h>
#include <M5_DLight.h>
#include <VL53L0X.h>

#define address                                     0x29  // I2C address

SHT3X sht30;                ///気温湿度計
QMP6988 qmp6988;            ///大気圧計
M5GFX display;
M5Canvas canvas(&display);
M5_DLight dlight;           ///照度計
VL53L0X tof;                ///距離計

uint16_t lux;
float tmp      = 0.0;   ///温度
float hum      = 0.0;   ///湿度
float pressure = 0.0;   ///大気圧
unsigned long prev, next ;      ///前回計測時
uint16_t dist = 0;  ///ToF 距離 (約1200mmまで計測する。2000mmまでの暗闇限定の長距離プロファイルが設定できるとデータシート上に記載されている)
float wind;  ///風速
String windtoprint;   ///文字列化風速 

DynamicJsonDocument dynamic_json_serialout(2048); ///シリアルから出力するJSON
StaticJsonDocument<512> flag_sensors;///センサー状態のフラグ 192が推奨メモリサイズ フラグ数が多くなる場合必要なメモリサイズも増えていく


///WindSensor From///
unsigned char in_buffer[256];
int ix;


int serial2read() {
  int crflag = 0;
  while ((Serial2.available() != 0) && (crflag == 0)) {
    unsigned char inByte = Serial2.read();
    if (inByte >= 0x10) {
      in_buffer[ix] = inByte;
      ix++;
    } else {
      if (inByte == 0x0d) {
        in_buffer[ix] = 0;
        crflag = 1;
        ix = 0;
      }
    }
  }
  return(crflag);
}
///WindSensor End///

void setup() {

  M5.begin(); //m5初期化
  prev = 0;         // 前回実行時刻を初期化


  M5.Lcd.setTextSize(2);    ///テキストサイズ
  M5.Lcd.fillScreen(BLACK); 
  Wire.begin();  // Wire init, adding the I2C bus.  
  M5.Power.begin();   
  Serial2.begin(38400);     ///風速計を繋ぐシリアルを速度を指定して起動
  qmp6988.init();           ///大気圧計起動
  JsonObject flag = flag_sensors.createNestedObject("ENV"); ///温湿度計が起動したかのフラグを入れる配列作成
  if(sht30.get() == 0){ ///温度湿度センサーチェック
    flag["setup"] = true; 
    M5.lcd.println("ENV sensor Ready!");  
  }
  else{
    flag["setup"] = false;    
    M5.lcd.println("ENV sensor isn't available");
  }
  M5.lcd.println();
  flag = flag_sensors.createNestedObject("LUX");    ///照度計が起動したかのフラグを入れる配列作成
  dlight.begin();                                   ///照度計起動
  dlight.setMode(CONTINUOUSLY_H_RESOLUTION_MODE);   ///照度計のモード指定
  if (dlight.getLUX()==USHRT_MAX){                  ///照度計が反応を返すかチェック
    flag["setup"] = false;    
    M5.lcd.println("LUX sensor isn't available");
  }else{
    flag["setup"] = true;    
    M5.lcd.println("LUX sensor ready! ");
  }
  M5.lcd.println();
  flag = flag_sensors.createNestedObject("TOF");    ///距離計が起動したかのフラグを入れる配列作成
  tof.setTimeout(200);                              ///距離計にタイムアウトする秒数を指定(200ms)
    if (!tof.init()){                               ///距離計が出来るかチェック
      flag["setup"] = false;    
      M5.lcd.println("TOF sensor isn't available");
    }else{
      flag["setup"] = true;    
      M5.lcd.println("TOF sensor ready!");
      tof.setMeasurementTimingBudget(200000);       ///距離計の計測時間200msに設定
    }
  M5.lcd.println();
  flag = flag_sensors.createNestedObject("WIND");   ///風速計が起動したかのフラグを入れる配列作成
  Serial2.printf("<RM>??\r\n");                     ///風速計に起動するようシリアルから命令を伝える
  int cnt = 0;
  while (cnt < 20) {
    if(Serial2.available()>0){                      ///風速計から値を取得できた、すなわち風速計が繋がってる時
      flag["setup"] = true;
      M5.lcd.println("WIND sensor ready!"); 
      delay(500);                                   ///M5Stackのこれまでの起動できたかのログを500msの間追加で表示を続ける
      break;
    }else{
      flag["setup"] = false;     
    }    
    cnt++;
    delay(10);
  }
  if(flag["setup"]==false){
    M5.lcd.println("WINDsensor isn't available");
    delay(2000);                                    ///風速計が起動できなかったらM5Stackのこれまでの起動できたかのログを2秒の間追加で表示を続ける
  }
  dynamic_json_serialout["FWV"]=FWversion;          ///ファームウェアバージョン
  uint32_t chipId = 0;
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  dynamic_json_serialout["ID"]=String(chipId);      ///macIDをJSONに記述  

}

void loop() {
  
  unsigned long curr = millis();    ///M5Stackが起動してからのmm secを取得する
  if ((curr - prev) >= interval) {  // 前回実行時刻から実行周期以上経過していたら
    // do periodic tasks            // センサー計測処理を実行
    prev += interval;               // 前回実行時刻に実行周期を加算
     
   


    if (sht30.get() == 0) {  // Obtain the data of shT30.  温湿度計
      flag_sensors["ENV"]["on_loop"] = true;  
      tmp = sht30.cTemp;   // Store the temperature obtained from shT30.温度計
      hum = sht30.humidity;  // Store the humidity obtained from the SHT30.湿度計
      qmp6988.init();        //
      pressure = qmp6988.calcPressure()/100; ///qmp6988から圧力を取得.単位をPaからhPaに変更
    } else {
      tmp = 0, hum = 0;
      flag_sensors["ENV"]["on_loop"] = false; 
    }
    
    /*if(flag_sensors["LUX"]["on_loop"] == false){
      dlight.begin(); 
      dlight.setMode(CONTINUOUSLY_H_RESOLUTION_MODE);
      if (dlight.getLUX()==USHRT_MAX){   ///照度計が反応を返すかチェック
        flag_sensors["LUX"]["on_loop"] = false;    
      }else{
        flag_sensors["LUX"]["on_loop"] = true;    
      }
    }*/
    lux = dlight.getLUX();
    if (lux==USHRT_MAX){   ///照度計が反応を返すかチェック
      flag_sensors["LUX"]["on_loop"] = false; 
    }else{
      flag_sensors["LUX"]["on_loop"] = true; 
    }
    
///ToF From
    if(flag_sensors["TOF"]["on_loop"] == false){
      if(tof.init()){
        flag_sensors["TOF"]["on_loop"] = true;  
        dist = tof.readRangeSingleMillimeters();        
      }
    }
    else if (tof.timeoutOccurred()) {
      flag_sensors["TOF"]["on_loop"] = false;
    }
    else {
      flag_sensors["TOF"]["on_loop"] = true;  
      dist = tof.readRangeSingleMillimeters();
    }
///ToF end      


///WindSensor From///
    flag_sensors["WIND"]["on_loop"] = false;
    Serial2.printf("<RM>??\r\n");
    while(Serial2.available()) {
      if (serial2read()) {
        String str1 = String((char *)in_buffer);
        int leng = str1.length();
        if (leng >= 18) {
          String str2 = str1.substring(0,4);
          if (str2.compareTo("<AM,") == 0) {
            String str3 = str1.substring(4,leng);
            int ix1 = str3.indexOf(',');
            if (ix1 > 0) {
              String str4 = str3.substring(0,ix1);
              windtoprint = str4;
              wind = str4.toFloat();
              flag_sensors["WIND"]["on_loop"] = true;
            }  
          }
        }  
        
      }
    }
///WindSensor End///


/// M5Stackにセンサー値を表示  ///
      
      M5.Lcd.clear(NAVY);
      // Fill the screen with black (to clear the
                             // screen).
      M5.lcd.setCursor(0, 20);
      String str_temp;
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf(" Temp:");   
      if(flag_sensors["ENV"]["on_loop"]==true){
        M5.Lcd.setTextColor(YELLOW); 
        str_temp = String(tmp)+"C\r\n";
        M5.Lcd.drawRightString(str_temp,300,20,1);
      }else{
        M5.Lcd.setTextColor(YELLOW); 
        M5.Lcd.drawRightString("N/A\r\n",300,20,1); 
      }
      
        M5.Lcd.setCursor(0,56 );
        M5.Lcd.setTextColor(WHITE); 
        M5.Lcd.printf(" Humi: ");          
      if(flag_sensors["ENV"]["on_loop"]==true){
        M5.Lcd.setTextColor(YELLOW);
        str_temp = String(hum)+"%\r\n";
        M5.Lcd.drawRightString(str_temp,300,56,1);   
      }else{
        M5.Lcd.setTextColor(YELLOW); 
        M5.Lcd.drawRightString("N/A\r\n",300,56,1); 
      }   
      
        M5.lcd.setCursor(0, 92);
        M5.Lcd.setTextColor(WHITE); 
        M5.Lcd.printf(" Pressure:");   
      if(flag_sensors["ENV"]["on_loop"]==true){
        M5.Lcd.setTextColor(YELLOW); 
        str_temp = String(pressure)+"hPa\r\n";
        M5.Lcd.drawRightString(str_temp,300,92,1);        
      }else{
        M5.Lcd.setTextColor(YELLOW);
        M5.Lcd.drawRightString("N/A\r\n",300,92,1); 
      }
      
      M5.lcd.setCursor(0, 128);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf(" Lumi: ");     
      if(flag_sensors["LUX"]["on_loop"]==true){
        M5.Lcd.setTextColor(YELLOW); 
        str_temp = String(lux)+" lx\r\n";
        M5.Lcd.drawRightString(str_temp,300,128,1);                      
      }else{
        M5.Lcd.setTextColor(YELLOW); 
        M5.Lcd.drawRightString("N/A\r\n",300,128,1); 
      }
      M5.lcd.setCursor(0, 164);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf(" Distance:"); 
      if(flag_sensors["TOF"]["on_loop"]==true){
        M5.Lcd.setTextColor(YELLOW);
        str_temp = String(dist)+"mm\r\n";
        M5.Lcd.drawRightString(str_temp,300,164,1);                
      }else{
        M5.Lcd.setTextColor(YELLOW); 
        M5.Lcd.drawRightString("N/A\r\n",300,164,1);
      }
      M5.lcd.setCursor(0, 200);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.printf(" Wind:");      
      if(flag_sensors["WIND"]["on_loop"]==true){
        M5.Lcd.setTextColor(YELLOW); 
        windtoprint+="m/S\r\n";
        M5.Lcd.drawRightString(windtoprint,300,200,1); 
      }else{
        M5.Lcd.setTextColor(YELLOW);
        M5.Lcd.drawRightString("N/A\r\n",300,200,1);        


/// M5Stackにセンサー値を表示  END///
      }            
   }

   
/// センサー値をJSONドキュメントに入れる///
   if(Serial.available()){
      char c = Serial.read();
      if (c == 'A'){ 
        JsonArray json_list = dynamic_json_serialout.createNestedArray("SENSORS");
        if(flag_sensors["ENV"]["setup"]==true&&flag_sensors["ENV"]["on_loop"]==true){   ///setup時とループ時に温度湿度気圧センサーが反応を返したかのフラグ
          JsonObject json_list_0 = json_list.createNestedObject();
          json_list_0["TYPE"] = "TEMP";
          json_list_0["VALUE"] = tmp;
          JsonObject json_list_1 = json_list.createNestedObject();
          json_list_1["TYPE"] = "HUMI";
          json_list_1["VALUE"] = hum;
          JsonObject json_list_2 = json_list.createNestedObject();
          json_list_2["TYPE"] = "BAROMETER";
          json_list_2["VALUE"] = pressure;      
        }
        if(flag_sensors["LUX"]["setup"]==true&&flag_sensors["LUX"]["on_loop"]==true){   ///setup時とループ時に照度センサーが反応を返したかのフラグ
          JsonObject json_list_3 = json_list.createNestedObject();
          json_list_3["TYPE"] = "LUMI";
          json_list_3["VALUE"] = lux;
        }
        if(flag_sensors["TOF"]["setup"]==true&&flag_sensors["TOF"]["on_loop"]==true){   ///setup時とループ時に距離センサーが反応を返したかのフラグ
          if(dist<8000){                                                                ///計測失敗時の外れ値を排除
            JsonObject json_list_4 = json_list.createNestedObject();
            json_list_4["TYPE"] = "DISTANCE";
            json_list_4["VALUE"] = dist;       
          }
        } 
        if(flag_sensors["WIND"]["setup"]==true&&flag_sensors["WIND"]["on_loop"]==true){   ///setup時とループ時に風速センサーが反応を返したかのフラグ
          JsonObject json_list_5 = json_list.createNestedObject();
          json_list_5["TYPE"] = "WIND";
          json_list_5["VALUE"] = wind;   
        }         
       
        dynamic_json_serialout.garbageCollect();    ///JSONドキュメント内の使わなくなったメモリを開放
  
        serializeJson(dynamic_json_serialout, Serial);    ///シリアルからJSONを送信
        Serial.println();
    
      }
   }
}
