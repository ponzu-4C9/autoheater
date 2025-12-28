#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "TimeProportionalPWM.h"

// --- Wi-Fi設定 ---
#define ssid "SRAS2G"
#define EAP_USERNAME "al25138"
#define EAP_PASSWORD "QFgGQfgGQfiH"

// --- ピン定義 ---
const int SSRPin = 2;
const int dataPin = 8;
const int clockPin = 10;
const int selectPin = 9;

// --- グローバル変数 ---
float currentTemp = 0.0;
double currentDT = 0.0; // 出力デューティ比 (0.0 - 1.0)
String IP = "";
wl_status_t preWstatus = WL_CONNECT_FAILED;

// --- オブジェクト初期化 ---
MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
WiFiServer server(80);
TimeProportionalPWM heater(SSRPin, 1000, true);

void setup() {
  Serial.begin(115200);
  
  // ヒーター初期化
  heater.begin();
  heater.setDuty(0.0); // 安全のため最初はOFF

  // 熱電対初期化
  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);

  // LCD初期化
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");

  // Wi-Fi接続 (WPA2 Enterprise)
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);
}

void loop() {
  // --- Wi-Fi接続処理 ---
  wl_status_t Wstatus = WiFi.status();
  if (preWstatus != WL_CONNECTED && Wstatus == WL_CONNECTED) {
    IP = WiFi.localIP().toString();
    Serial.println("\nIP: " + IP);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(IP);
    server.begin();
  }
  preWstatus = Wstatus;

  // --- Webサーバー処理 ---
  if (Wstatus == WL_CONNECTED) {
    WiFiClient client = server.available();
    if (client) {
      client.setTimeout(2000);
      String request = client.readStringUntil('\n');
      
      // リクエストボディを読み捨てる
      while (client.connected() && client.available()) {
        if (client.readStringUntil('\n') == "\r") break;
      }

      // 値の設定リクエスト: GET /set?dt=50 (0-100の整数)
      if (request.indexOf("GET /set?dt=") >= 0) {
        int pos = request.indexOf("dt=");
        int endPos = request.indexOf(" ", pos);
        String valStr = request.substring(pos + 3, endPos);
        int val = valStr.toInt();
        
        // 入力を0-100に制限し、0.0-1.0に変換
        val = constrain(val, 0, 100);
        currentDT = (double)val / 100.0;
        heater.setDuty(currentDT);

        // JSONで現在の状態を返す
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();
        client.print("{\"temp\":");
        client.print(currentTemp);
        client.print(",\"dt\":");
        client.print((int)(currentDT * 100));
        client.print("}");

      } 
      // データ取得のみ
      else if (request.indexOf("GET /data") >= 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();
        client.print("{\"temp\":");
        client.print(currentTemp);
        client.print(",\"dt\":");
        client.print((int)(currentDT * 100));
        client.print("}");
      }
      // メイン画面 (UI)
      else {
        sendWebPage(client);
      }

      delay(1);
      client.stop();
    }
  }

  // --- 温度計測と表示 (約1秒間隔) ---
  static unsigned long lastMeasure = 0;
  if (millis() - lastMeasure > 1000) {
    lastMeasure = millis();
    
    thermoCouple.read();
    currentTemp = thermoCouple.getCelsius();

    // LCD更新
    if (Wstatus == WL_CONNECTED) {
      char buf[21];
      lcd.setCursor(0, 0);
      lcd.print(IP);
      lcd.setCursor(0, 1);
      snprintf(buf, sizeof(buf), "T:%.1fOut:%3d%%", currentTemp, (int)(currentDT * 100));
      lcd.print(buf);
    }
  }

  // PWM制御更新 (これは頻繁に呼ぶ必要がある)
  heater.update();
}

// --- HTML出力関数 ---
void sendWebPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html><html><head>");
  client.println("<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  client.println("<title>Simple Heater</title>");
  client.println("<style>");
  client.println("body{font-family:sans-serif;text-align:center;padding:20px;background:#222;color:#fff;}");
  client.println("h1{color:#ff6b35;}");
  client.println(".val{font-size:3em;margin:10px;}");
  client.println("input[type=range]{width:80%;max-width:400px;height:30px;}");
  client.println("button{padding:10px 20px;font-size:1.2em;margin-top:20px;cursor:pointer;}");
  client.println("</style></head><body>");
  client.println("<h1>手動ヒーター制御</h1>");
  
  client.println("<div>現在温度</div>");
  client.println("<div id='temp' class='val'>-- °C</div>");
  
  client.println("<div>出力設定: <span id='dtVal'>0</span>%</div>");
  client.println("<input type='range' id='slider' min='0' max='100' value='0' oninput='updateLabel(this.value)' onchange='sendDT(this.value)'>");
  client.println("<br><button onclick='sendDT(0)'>緊急停止 (0%)</button>");

  client.println("<script>");
  client.println("function updateLabel(val){ document.getElementById('dtVal').innerText = val; }");
  
  client.println("async function sendDT(val){");
  client.println("  document.getElementById('slider').value = val;");
  client.println("  updateLabel(val);");
  client.println("  try { await fetch('/set?dt=' + val); } catch(e){}");
  client.println("}");

  client.println("async function getData(){");
  client.println("  try {");
  client.println("    const res = await fetch('/data');");
  client.println("    const d = await res.json();");
  client.println("    document.getElementById('temp').innerText = d.temp.toFixed(1) + ' °C';");
  // 他の操作でスライダーを動かしているときは更新しない
  client.println("    if(!document.getElementById('slider').matches(':active')){");
  client.println("       document.getElementById('slider').value = d.dt;");
  client.println("       updateLabel(d.dt);");
  client.println("    }");
  client.println("  } catch(e){}");
  client.println("}");
  
  client.println("setInterval(getData, 2000);"); // 2秒ごとに更新
  client.println("</script>");
  client.println("</body></html>");
}