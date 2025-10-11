#include "MAX6675.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <SPI.h>

#define ssid            "SRAS2G"
#define EAP_USERNAME    "al25138"
#define EAP_PASSWORD    "QFgGQfgGQfiH"

WiFiServer server(80);

void waitForConnection(uint32_t timeoutMs = 30000) {
  Serial.print("Connecting to ");
  Serial.print(ssid);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > timeoutMs) {
      Serial.println("\nTimeout.");
      break;
    }
  }
  Serial.println();
}

// SPI/センサーピン（必要に応じて変更可）
const int dataPin   = 8;  // SO (MISO)
const int clockPin  = 10; // SCK
const int selectPin = 9;  // CS

MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4);

// 温度キャッシュ
double lastTempC = NAN;
unsigned long lastReadMs = 0;
const unsigned long readIntervalMs = 300; // センサー読み取り間隔

// ルートHTML（/）
const char INDEX_HTML[] =
"<!doctype html>\n"
"<html lang=\"ja\">\n"
"<head>\n"
"  <meta charset=\"utf-8\" />\n"
"  <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\" />\n"
"  <title>MAX6675 Realtime Temperature</title>\n"
"  <style>\n"
"    html, body { font-family: system-ui, sans-serif; background:#121212; color:#fff; height:100%; margin:0; }\n"
"    .wrap { max-width: 640px; margin: 40px auto; padding: 16px; }\n"
"    .card { background:#1e1e1e; border-radius:12px; padding:24px; box-shadow:0 4px 16px rgba(0,0,0,0.4); }\n"
"    .value { font-size: 56px; font-weight: 700; letter-spacing: 1px; }\n"
"    .unit { font-size: 24px; opacity: .8; margin-left: 8px; }\n"
"    .row { display:flex; align-items:baseline; gap:8px; }\n"
"    .muted { color:#aaa; font-size: 14px; margin-top:8px; }\n"
"    button { margin-top:16px; padding:8px 14px; background:#2e7d32; color:#fff; border:none; border-radius:8px; cursor:pointer; }\n"
"    button:hover { background:#357a38; }\n"
"  </style>\n"
"</head>\n"
"<body>\n"
"  <div class=\"wrap\">\n"
"    <div class=\"card\">\n"
"      <div class=\"row\">\n"
"        <div id=\"temp\" class=\"value\">--.--</div>\n"
"        <div class=\"unit\">°C</div>\n"
"      </div>\n"
"      <div id=\"updated\" class=\"muted\">Updating...</div>\n"
"      <button id=\"refresh\">Refresh now</button>\n"
"    </div>\n"
"  </div>\n"
"  <script>\n"
"    async function fetchTemp() {\n"
"      try {\n"
"        const res = await fetch('/temp', { cache: 'no-store' });\n"
"        if (!res.ok) throw new Error('HTTP ' + res.status);\n"
"        const j = await res.json();\n"
"        const v = (typeof j.temp === 'number' && isFinite(j.temp)) ? j.temp.toFixed(2) : '--.--';\n"
"        document.getElementById('temp').textContent = v;\n"
"        document.getElementById('updated').textContent = 'Updated: ' + new Date().toLocaleTimeString();\n"
"      } catch (e) {\n"
"        console.error(e);\n"
"        document.getElementById('updated').textContent = 'Update failed: ' + e.message;\n"
"      }\n"
"    }\n"
"    document.getElementById('refresh').addEventListener('click', fetchTemp);\n"
"    setInterval(fetchTemp, 1000);\n"
"    fetchTemp();\n"
"  </script>\n"
"</body>\n"
"</html>\n";

// 簡易HTTP応答ユーティリティ
void sendHeaders(WiFiClient &client, const char* status, const char* contentType, size_t contentLength, bool nocache = true) {
  client.print("HTTP/1.1 ");
  client.print(status);
  client.print("\r\nContent-Type: ");
  client.print(contentType);
  client.print("\r\nConnection: close\r\n");
  if (nocache) {
    client.print("Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\nPragma: no-cache\r\n");
  }
  client.print("Content-Length: ");
  client.print(contentLength);
  client.print("\r\n\r\n");
}

void sendNotFound(WiFiClient &client) {
  const char body[] = "404 Not Found";
  sendHeaders(client, "404 Not Found", "text/plain; charset=utf-8", strlen(body));
  client.print(body);
}

void sendRoot(WiFiClient &client) {
  sendHeaders(client, "200 OK", "text/html; charset=utf-8", strlen(INDEX_HTML));
  client.print(INDEX_HTML);
}

void sendTemp(WiFiClient &client) {
  char json[64];
  if (isnan(lastTempC)) {
    snprintf(json, sizeof(json), "{\"temp\":null,\"unit\":\"C\"}");
  } else {
    snprintf(json, sizeof(json), "{\"temp\":%.2f,\"unit\":\"C\"}", lastTempC);
  }
  sendHeaders(client, "200 OK", "application/json; charset=utf-8", strlen(json));
  client.print(json);
}

void handleHttpClient(WiFiClient client) {
  if (!client) return;

  // 1行目(リクエストライン)を読む
  String requestLine = client.readStringUntil('\r');
  client.readStringUntil('\n'); // LFを捨てる

  // 残りヘッダを捨てる
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break; // 空行でヘッダ終端
  }

  // パスを判別
  // 例: "GET /temp HTTP/1.1"
  if (requestLine.startsWith("GET ")) {
    int sp1 = requestLine.indexOf(' ');
    int sp2 = requestLine.indexOf(' ', sp1 + 1);
    String path = requestLine.substring(sp1 + 1, sp2);

    if (path == "/" || path == "/index.html") {
      sendRoot(client);
    } else if (path == "/temp") {
      sendTemp(client);
    } else {
      sendNotFound(client);
    }
  } else {
    sendNotFound(client);
  }

  delay(1);
  client.stop();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== boot ===");

  // Wi-Fi接続（WPA2-Enterprise PEAP）
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);
  waitForConnection();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
  } else {
    Serial.println("WiFi not connected. Web server not started.");
  }

  // SPI/センサー初期化
  SPI.begin();
  thermoCouple.begin();
  thermoCouple.setSPIspeed(4000000);
  delay(500);

  // LCD初期化
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MAX6675 Web Server");
}

void loop() {
  // 定期的に温度を更新
  unsigned long now = millis();
  if (now - lastReadMs >= readIntervalMs) {
    lastReadMs = now;
    thermoCouple.read();
    lastTempC = thermoCouple.getCelsius();

    // LCD表示更新
    lcd.setCursor(0, 1);
    char buf[32];
    if (isnan(lastTempC)) {
      snprintf(buf, sizeof(buf), "--.-- C");
    } else {
      snprintf(buf, sizeof(buf), "%.2f C", lastTempC);
    }
    lcd.print("                "); // クリア
    lcd.setCursor(0, 1);
    lcd.print(buf);
  }

  // HTTPクライアント処理
  WiFiClient client = server.available();
  if (client) {
    handleHttpClient(client);
  }
}