/*
  MAX6675_web_server.ino

  オリジナルのコードに詳しい解説コメントを大量に追加した版です。
  - 各ブロック（インクルード、定数、初期化、各関数、setup、loop）ごとに
    何をしているか・なぜそうしているか・注意点を説明しています。
  - 実機で動かす際の注意点（セキュリティ、メモリ、Wi-Fi企業認証の扱いなど）も追記。

  変更は機能的にオリジナルと同等になるように留め、挙動は変えていません。
  実際の製品で使う場合は「平文でパスワードをコードに埋める」のは避けてください。
*/

#include "MAX6675.h"             // MAX6675 センサー用ライブラリ（SPIで温度を読む）
#include <LiquidCrystal_I2C.h>  // I2C接続のLCD用ライブラリ
#include <WiFi.h>               // ESP32向け WiFi（WPA2-Enterprise 含む）
#include <SPI.h>                // SPI バス制御

// ========================
// Wi-Fi (WPA2-Enterprise) 設定
// ========================
// 注意：ここに平文のSSID/ユーザー名/パスワードを書いているのは教育目的のみ。
// 実際の運用ではセキュアな方法で管理してください（シリアルには出力しない、
// 設定ファイルやシークレットストレージを使うなど）。
#define ssid            "SRAS2G"
#define EAP_USERNAME    "al25138"
#define EAP_PASSWORD    "QFgGQfgGQfiH"

// ローカルにHTTPサーバを作る（ポート80）
WiFiServer server(80);

/*
 waitForConnection
 - WiFi接続が確立するまで待機するユーティリティ。
 - timeoutMs（デフォルト30秒）を超えたらタイムアウトして抜ける。
 - 実機では無限ループで待たないようにタイムアウトを設けると安全。
 - 注意: WPA2-Enterprise では接続に時間がかかることがあるのでタイムアウトを調整すること。
*/
void waitForConnection(uint32_t timeoutMs = 30000) {
  Serial.print("Connecting to ");
  Serial.print(ssid);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);                  // ブロッキングだが簡潔。大規模アプリなら非ブロッキングが望ましい。
    Serial.print(".");
    if (millis() - start > timeoutMs) {
      Serial.println("\nTimeout.");
      break;
    }
  }
  Serial.println();
}

// ========================
// SPI / センサーのピン定義
// ========================
// 使用するボードで SPI の MISO/MOSI/SCK が固定されていることがあるので、
// ライブラリやボード仕様を確認して適切に設定してください。
// ここでは MAX6675 のために SO(MISO)=8, SCK=10, CS=9 を使う例。
const int dataPin   = 8;  // SO (MISO) - MAX6675 からデータを受ける
const int clockPin  = 10; // SCK (クロック)
const int selectPin = 9;  // CS / SS (チップセレクト)

// MAX6675 と LCD のインスタンスを作成
MAX6675 thermoCouple(selectPin, dataPin, clockPin);
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2Cアドレス0x27、20x4 のLCDを想定

// ========================
// 温度読み取りキャッシュ
// ========================
// センサー読み取りは時間がかかることがあるので、頻度を制限するためキャッシュを用いる。
// lastTempC: 最後に得られた温度（摂氏）、読み取り不能なら NAN
// lastReadMs: 最後に読み取った時刻（millis()）
// readIntervalMs: センサーを読む最小間隔
double lastTempC = NAN;
unsigned long lastReadMs = 0;
const unsigned long readIntervalMs = 300; // ms。センサー読み取り間隔（必要に応じて拡張）

// ========================
// ルート (index.html) のコンテンツ
// ========================
// 大きな HTML をそのまま const char[] に持っているためフラッシュメモリを消費します。
// ESP32 では const char[] はフラッシュ配置されることが多いですが、
// 他にも PROGMEM 指定などの工夫が可能です（メモリ節約目的）。
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

// ========================
// 簡易HTTP応答ユーティリティ
// - sendHeaders: HTTPヘッダをまとめて送る。Content-Length を正しく入れておくことで\n//   ブラウザがレスポンス終端を判定しやすくなる。
// - sendNotFound, sendRoot, sendTemp: 単純なエンドポイント処理。
// 注意点:
// - ここでは簡易実装のため Content-Length を事前に計算してから送っている。
// - 実運用では keep-alive を扱う、POST を扱う等の処理拡張が必要。
// ========================
void sendHeaders(WiFiClient &client, const char* status, const char* contentType, size_t contentLength, bool nocache = true) {
  client.print("HTTP/1.1 ");
  client.print(status);
  client.print("\r\nContent-Type: ");
  client.print(contentType);
  client.print("\r\nConnection: close\r\n"); // シンプルに close を宣言（keep-alive を扱わない）
  if (nocache) {
    // ブラウザのキャッシュを防ぐヘッダ
    client.print("Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\nPragma: no-cache\r\n");
  }
  client.print("Content-Length: ");
  client.print(contentLength);
  client.print("\r\n\r\n");
}

void sendNotFound(WiFiClient &client) {
  // 404 応答を返すユーティリティ
  const char body[] = "404 Not Found";
  sendHeaders(client, "404 Not Found", "text/plain; charset=utf-8", strlen(body));
  client.print(body);
}

void sendRoot(WiFiClient &client) {
  // ルートHTMLを返す
  sendHeaders(client, "200 OK", "text/html; charset=utf-8", strlen(INDEX_HTML));
  client.print(INDEX_HTML);
}

void sendTemp(WiFiClient &client) {
  // 簡単なJSONを返す。lastTempC が NAN の場合は null を返す。
  // 注意: json バッファサイズは有限（64 bytes）にしているため、出力フォーマットに注意。
  char json[64];
  if (isnan(lastTempC)) {
    // 温度読み取りに失敗している場合
    snprintf(json, sizeof(json), "{\"temp\":null,\"unit\":\"C\"}");
  } else {
    // 温度が有効な場合は数値を2桁で返す
    snprintf(json, sizeof(json), "{\"temp\":%.2f,\"unit\":\"C\"}", lastTempC);
  }
  // JSONの長さを Content-Length に指定して送信
  sendHeaders(client, "200 OK", "application/json; charset=utf-8", strlen(json));
  client.print(json);
}

// ========================
// handleHttpClient
// - 単純な HTTP GET のパス判定を行う。
// - 実際には HTTP のすべてのケースを扱っているわけではない（POSTや長いヘッダ、chunked等は非対応）。
// - requestLine のパースは簡易（"GET /path HTTP/1.1"の形式が前提）
// - クライアントとの通信はブロッキングで行う。
// ========================
void handleHttpClient(WiFiClient client) {
  if (!client) return;

  // 1行目(リクエストライン)を読む（CR まで）
  // 例: "GET /temp HTTP/1.1"
  String requestLine = client.readStringUntil('\r');
  client.readStringUntil('\n'); // LF を捨てる

  // 残りのヘッダ行を読み捨てる。空行("\r\n")が来たらヘッダ終端。
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    // ヘッダ終端は空行（CRだけ）か長さ0 の行で判定
    if (line == "\r" || line.length() == 0) break;
    // ここでヘッダを解析したければ line を調べる（例: Host, Connection 等）
  }

  // リクエストラインからメソッドとパスを取り出す（簡易パース）
  if (requestLine.startsWith("GET ")) {
    int sp1 = requestLine.indexOf(' ');
    int sp2 = requestLine.indexOf(' ', sp1 + 1);
    // substring は start から (sp2-1) までを抜き出す
    String path = requestLine.substring(sp1 + 1, sp2);

    // ここでパスごとに処理を分ける
    if (path == "/" || path == "/index.html") {
      sendRoot(client);
    } else if (path == "/temp") {
      sendTemp(client);
    } else {
      sendNotFound(client);
    }
  } else {
    // GET でない（POST 等）場合は未対応として 404 を返す簡易実装
    sendNotFound(client);
  }

  delay(1); // 少し待ってからソケットを閉じる（クライアントにデータ送信を余裕を持たせる）
  client.stop();
}

// ========================
// setup()
// - シリアル、Wi-Fi、SPI、センサー、LCD の初期化を行う
// - WPA2-Enterprise を使用する場合は platform や SDK の API に依存するため注意が必要。
//   ここでは WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);
//   としているが、プラットフォームによっては identity と username が
//   逆になる、もしくは追加の設定が要ることがあります。
// ========================
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== boot ===");

  // ===== Wi-Fi 接続（WPA2-Enterprise PEAP） =====
  // 引数の順序はライブラリのバージョンによって変わることがあります。
  // 正しい順序や API は使用している ESP-IDF / Arduino core のドキュメントを確認してください。
  // （identity, username, password の扱い）
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);

  // 接続が確立するまで待つ（タイムアウト付き）
  waitForConnection();
  
  if (WiFi.status() == WL_CONNECTED) {
    // 接続成功
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    // HTTP サーバ開始
    server.begin();
  } else {
    // 接続失敗時のログ。実稼働では再試行ロジックを入れること。
    Serial.println("WiFi not connected. Web server not started.");
  }

  // ===== SPI / センサー初期化 =====
  // SPI.begin() はボードのデフォルト SPI ピンを有効にします。
  // MAX6675 用のライブラリによっては begin() を内部で呼ぶ必要があるものもあります。
  SPI.begin();
  thermoCouple.begin();
  // SPIのクロックを指定（MAX6675 の許容範囲に合わせる）
  thermoCouple.setSPIspeed(4000000);
  delay(500); // 少し待ってデバイス安定化

  // ===== LCD 初期化 =====
  // I2C アドレスや LCD サイズが合っていることを確認してください。
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MAX6675 Web Server");
}
