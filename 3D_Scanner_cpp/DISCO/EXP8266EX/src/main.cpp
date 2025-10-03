#include <Arduino.h>
#include <ESP8266WiFi.h>

WiFiServer Server(23); // TCP server on port 23 (Telnet)
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  //Start WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP("3D-Scanner", "12345678");

  // Start TCP server
  Server.begin();
  Serial.println("3d Scanner AP started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  // put your main code here, to run repeatedly:
  WiFiClient client = Server.available();

  if(client) {
    Serial.println("Client connected");

    while("Client connected"){
      // Check for STM32 UART data
      if( Serial.available()){
        String stmData = Serial.readStringUntil('\n');
        client.println(stmData); // sends to TCP client
      }

      // Accept commands from client to STM32
      if(client.available()){
        String cmd = client.readStringUntil('\n');
        Serial.println("From client: " + cmd);
        Serial.println(cmd); // sends to STM32
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}
