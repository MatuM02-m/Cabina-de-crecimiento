#include <Arduino.h>
#include <WiFi.h> // Librería nativa del ESP32 (no hace falta descargarla)
#include "secrets.h"

void setup() {
  Serial.begin(115200);
  
  // Pequeña pausa para que te dé tiempo a abrir el monitor
  delay(1000); 
  Serial.println("\n--- Iniciando ESP32 ---");

  // Iniciamos la conexión WiFi
  WiFi.mode(WIFI_STA); // Modo Estación (se conecta a tu router)
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a WiFi");

  // Bucle de espera hasta que conecte
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n¡Conectado!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Fuerza de señal (RSSI): ");
  Serial.println(WiFi.RSSI());
}

void loop() {
  // Aquí tu código del proyecto...
  // Por ahora no hace nada, solo mantiene la conexión
  delay(1000);
}