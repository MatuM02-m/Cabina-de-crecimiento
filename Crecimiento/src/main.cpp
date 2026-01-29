#include <Arduino.h>

// El LED interno en la mayoría de placas ESP32 (DevKit V1) está en el GPIO 2.
// Si tu placa es distinta, podrías necesitar cambiar este número.
#define LED_BUILTIN 2

void setup() {
  // Configuramos el pin como salida
  pinMode(LED_BUILTIN, OUTPUT);

  // Iniciamos el puerto serie para enviar mensajes a la PC
  Serial.begin(115200);
  Serial.println("¡Hola desde el ESP32!");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // Encender LED
  Serial.println("LED Encendido");
  delay(1000);                      // Esperar 1 segundo

  digitalWrite(LED_BUILTIN, LOW);   // Apagar LED
  Serial.println("LED Apagado");
  delay(1000);                      // Esperar 1 segundo
}