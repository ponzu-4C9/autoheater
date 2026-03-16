#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>

// --- Wi-Fi 設定 ---
#define ssid "SRAS2G"
#define EAP_USERNAME "al25138"
#define EAP_PASSWORD "QFgGQfgGQfiH"

const int SSRPin = 2;
const int dataPin = 8;    // so
const int clockPin = 10;  // sck
const int selectPin = 9;  // cs

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);
WebServer server(80);

// --- グローバル変数 ---
// ESP32のdoubleは64bitなので、タスク間通信用にvolatileをつけています
volatile double DT = 0.0;   // デューティ比 (0.00 〜 1.00)
volatile double temp = 0.0; // 現在温度

// --- SSR制御関数 (1秒周期) ---
void DTcont(double duty) {
  if (isnan(duty)) duty = 0;
  if (duty <= 0) {
    digitalWrite(SSRPin, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  }
  if (duty >= 1.0) {
    digitalWrite(SSRPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  }
  
  // 1秒(1000ms)の中でONとOFFの時間を割り振る
  int onTime = (int)(1000 * duty);
  int offTime = 1000 - onTime;
  
  digitalWrite(SSRPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(onTime));
  digitalWrite(SSRPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(offTime));
}

// ===================================================
// 制御タスク (Core 0, 優先度 2)
// ===================================================
void ControlTask(void *pvParameters) {
  while (1) {
    // 温度読み取り
    thermoCouple.read();
    temp = thermoCouple.getCelsius();

    // LCD表示更新
    char buf[64];
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "Temp: %6.1f C  ", temp);
    lcd.print(buf);

    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "Out : %3.0f %%     ", DT * 100.0);
    lcd.print(buf);

    // SSRのPWM制御を実行 (ここで約1秒消費し、他のタスクに処理を譲る)
    DTcont(DT);
  }
}


// ===================================================
// Webサーバー関連の関数
// ===================================================
void handleRoot() {
  const char *html = R"raw(
HTTP/1.1 200 OK
Content-Type: text/html
Connection: close

<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>TBT 手動制御窯</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{font-family:'Segoe UI',Arial,sans-serif;background:#0a0a0a;color:#e0e0e0;padding:20px;text-align:center}
  .container{max-width:600px;margin:0 auto;background:linear-gradient(145deg,#1a1a1a,#0f0f0f);padding:30px;border-radius:15px;box-shadow:0 8px 32px rgba(0,0,0,0.4);border:1px solid #2a2a2a}
  h1{font-size:2em;margin-bottom:20px;color:#ff6b35;text-shadow:0 0 15px rgba(255,107,53,0.5)}
  .temp-display{font-size:4em;font-weight:bold;color:#4ecdc4;margin-bottom:30px}
  .unit{font-size:0.4em;color:#888}
  
  .control-group{margin-bottom:25px;background:#151515;padding:20px;border-radius:10px;border:1px solid #333}
  .control-group h3{color:#888;margin-bottom:15px;font-size:1em}
  
  /* スライダーのスタイル */
  input[type=range]{width:100%;height:10px;background:#333;border-radius:5px;outline:none;appearance:none;margin-bottom:20px}
  input[type=range]::-webkit-slider-thumb{appearance:none;width:25px;height:25px;border-radius:50%;background:#ff6b35;cursor:pointer;box-shadow:0 0 10px rgba(255,107,53,0.8)}
  
  /* 入力とボタンのスタイル */
  .input-row{display:flex;justify-content:center;align-items:center;gap:10px;margin-bottom:15px}
  input[type=number]{width:80px;padding:10px;font-size:1.5em;text-align:center;background:#222;color:#fff;border:1px solid #555;border-radius:5px}
  button{background:#2a2a2a;color:#e0e0e0;border:1px solid #444;padding:10px 15px;font-size:1.2em;border-radius:5px;cursor:pointer;transition:0.2s}
  button:hover{background:#ff6b35;color:#fff;border-color:#ff6b35}
  button:active{transform:scale(0.95)}
  
  .status{color:#666;font-size:0.9em;margin-top:20px}
</style>
</head>
<body>
<div class='container'>
  <h1>TBT 手動制御窯</h1>
  <div class='temp-display'><span id='temp'>--.-</span><span class='unit'>°C</span></div>
  
  <div class='control-group'>
    <h3>ヒーター出力設定 (%)</h3>
    <input type='range' id='dtSlider' min='0' max='100' step='1' oninput='syncUI(this.value)' onchange='sendDT()'>
    
    <div class='input-row'>
      <button onclick='adjustDT(-10)'>-10</button>
      <button onclick='adjustDT(-1)'>-1</button>
      <input type='number' id='dtInput' min='0' max='100' onchange='syncUI(this.value); sendDT()'>
      <button onclick='adjustDT(1)'>+1</button>
      <button onclick='adjustDT(10)'>+10</button>
    </div>
  </div>
  
  <div class='status'>最終更新: <span id='updateTime'>--:--:--</span></div>
</div>

<script>
  let isDragging = false;
  const slider = document.getElementById('dtSlider');
  const input = document.getElementById('dtInput');
  const tempEl = document.getElementById('temp');
  const timeEl = document.getElementById('updateTime');

  slider.addEventListener('mousedown', () => isDragging = true);
  slider.addEventListener('touchstart', () => isDragging = true);
  slider.addEventListener('mouseup', () => { isDragging = false; sendDT(); });
  slider.addEventListener('touchend', () => { isDragging = false; sendDT(); });

  // UI要素の同期
  function syncUI(val) {
    let num = parseInt(val);
    if(isNaN(num)) num = 0;
    if(num < 0) num = 0;
    if(num > 100) num = 100;
    slider.value = num;
    input.value = num;
  }

  // ボタンからの相対変更
  function adjustDT(delta) {
    let current = parseInt(input.value) || 0;
    syncUI(current + delta);
    sendDT();
  }

  // サーバーへ設定値を送信
  async function sendDT() {
    let val = parseInt(input.value) || 0;
    try {
      await fetch('/set?val=' + val);
    } catch(e) { console.error(e); }
  }

  // 定期的に現在状態を取得
  async function updateData() {
    try {
      const res = await fetch('/data');
      const d = await res.json();
      tempEl.textContent = d.temp.toFixed(1);
      
      // ユーザーがスライダー操作中でなければ、サーバーの値にUIを合わせる
      if(!isDragging && document.activeElement !== input) {
        syncUI(d.dt);
      }
      
      timeEl.textContent = new Date().toLocaleTimeString('ja-JP');
    } catch(e) { console.error(e); }
  }

  // 初回取得と定期更新 (1秒ごと)
  updateData();
  setInterval(updateData, 1000);
</script>
</body>
</html>
)raw";
  server.sendContent(html);
}

void handleData() {
  String json = "{\"temp\":" + String(temp, 1) + ", \"dt\":" + String(DT * 100.0, 0) + "}";
  server.send(200, "application/json", json);
}

void handleSetDT() {
  if (server.hasArg("val")) {
    int val = server.arg("val").toInt();
    if (val < 0) val = 0;
    if (val > 100) val = 100;
    
    DT = (double)val / 100.0; // 0.00 〜 1.00 に変換
    
    Serial.printf("Output Changed manually: %d %%\n", val);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// ===================================================
// Webタスク (Core 0, 優先度 1)
// ===================================================
void webTask(void *pvParameters) {
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set", handleSetDT); // 値変更用の新しいエンドポイント
  server.begin();

  while (1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10)); // WDTリセットとタスク切り替えのための必須ディレイ
  }
}

// ===================================================
// セットアップ
// ===================================================
void setup() {
  Serial.begin(115200);
  pinMode(SSRPin, OUTPUT);

  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(1000);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  // ESP32-C3はシングルコアなので、両方のタスクを Core 0 に割り当てます。
  // FreeRTOSのスケジューラが優先度(Priority)を見て適切に切り替えます。
  
  xTaskCreatePinnedToCore(
    ControlTask,
    "Control",
    10000,
    NULL,
    2,      // 優先度高：温度制御とSSRのタイミングを最優先
    NULL,
    0       // Core 0 (C3用)
  );

  xTaskCreatePinnedToCore(
    webTask,
    "Communication",
    10000,
    NULL,
    1,      // 優先度低：空いた時間でWeb通信をさばく
    NULL,
    0       // Core 0 (C3用)
  );
}

void loop() {
  // FreeRTOSを使用しているため、loop()は空にしておく
  vTaskDelete(NULL); 
}