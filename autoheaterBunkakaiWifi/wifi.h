#include <WiFi.h>
#define ssid            "SRAS2G"
#define EAP_USERNAME    "al25138"
#define EAP_PASSWORD    "QFgGQfgGQfiH"

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

void wifisetup(){
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_USERNAME, EAP_USERNAME, EAP_PASSWORD);
  waitForConnection();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
  }
}