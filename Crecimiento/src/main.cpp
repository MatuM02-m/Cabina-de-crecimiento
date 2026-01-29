#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en ["); Serial.print(topic); Serial.print("] ");
  
  // Si recibimos un '1' en el tópico de comando, encendemos el LED
  if ((char)payload[0] == '1') digitalWrite(2, HIGH);
  else digitalWrite(2, LOW);
}

void setup() {
  pinMode(2, OUTPUT); // LED interno
  Serial.begin(115200);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  client.setServer("192.168.1.121", 1883); // <--- IMPORTANTE: Pon la IP de tu notebook
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    while (!client.connect("ESP32_Matias")) { delay(500); }
    client.subscribe("cabina/comandos/led"); // Nos suscribimos a este canal
  }
  client.loop();

  static unsigned long lastTime = 0;
  if (millis() - lastTime > 5000) {
    lastTime = millis();
    client.publish("cabina/estado", "Conectado y listo");
  }
}