#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_adc_cal.h"

// ============================================================
//  PINES
//  Lado izquierdo del ESP32 NodeMCU:  LM35 + Relé
//  Lado derecho del ESP32 NodeMCU:    OLED (I2C)
// ============================================================
const int PIN_LM35 = 34;   // ADC1_CH6 — lado izquierdo
const int PIN_RELE = 26;   // GPIO26   — lado izquierdo

// ============================================================
//  CONFIGURACIÓN DEL LM35 (ADC)
// ============================================================
const int MUESTRAS = 64;
esp_adc_cal_characteristics_t adc_chars;

// ============================================================
//  CONFIGURACIÓN DEL RELÉ
//  Lógica activa en HIGH: HIGH = encendido, LOW = apagado
// ============================================================
#define RELE_ENCENDIDO HIGH
#define RELE_APAGADO   LOW

// ============================================================
//  PARÁMETROS DE CONTROL TÉRMICO (Histéresis)
//  Enciende al caer a (OBJETIVO - TOLERANCIA) = 29°C
//  Apaga al subir a  (OBJETIVO + TOLERANCIA) = 31°C
// ============================================================
const float TEMP_OBJETIVO = 30.0;
const float TOLERANCIA    = 1.0;

// ============================================================
//  CONFIGURACIÓN OLED — SSD1306 128x64 por I2C
//  SDA = GPIO21, SCL = GPIO22 (lado derecho)
// ============================================================
#define ANCHO_PANTALLA 128
#define ALTO_PANTALLA   64
#define OLED_RESET      -1
#define DIRECCION_I2C   0x3C

Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, OLED_RESET);

// ============================================================
//  ESTADO GLOBAL
// ============================================================
bool estufaEncendida = false;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println(" Cabina de Crecimiento - ESP32");
  Serial.println("========================================");

  // --- Relé: configurar y arrancar apagado ---
  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, RELE_APAGADO);
  Serial.println("[OK] Rele configurado en GPIO26 (arranca apagado)");

  // --- ADC: calibración para el LM35 ---
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12,
                           ADC_WIDTH_BIT_12, 1100, &adc_chars);
  analogSetPinAttenuation(PIN_LM35, ADC_11db);
  analogReadResolution(12);
  Serial.println("[OK] ADC calibrado (GPIO34, 12-bit, 11dB)");

  // --- OLED: inicializar ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, DIRECCION_I2C)) {
    Serial.println("[!!] Error: No se encontro la pantalla OLED");
    for (;;);  // Detener si no hay pantalla
  }
  Serial.println("[OK] Pantalla OLED inicializada");

  // Pantalla de inicio
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 25);
  display.println(F("Iniciando Sistema..."));
  display.display();
  delay(2000);

  Serial.printf("[OK] Setpoint: %.1f C  |  Tolerancia: +/-%.1f C\n",
                TEMP_OBJETIVO, TOLERANCIA);
  Serial.println("========================================\n");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  // --- 1. Leer temperatura (sobremuestreo + calibración ADC) ---
  uint32_t suma_raw = 0;
  for (int i = 0; i < MUESTRAS; i++) {
    suma_raw += analogRead(PIN_LM35);
    delay(2);
  }
  uint32_t promedio_raw = suma_raw / MUESTRAS;
  uint32_t voltaje_mV = esp_adc_cal_raw_to_voltage(promedio_raw, &adc_chars);
  float temperatura = voltaje_mV / 10.0;  // LM35: 10 mV/°C

  // --- 2. Control del relé con histéresis ---
  if (temperatura <= (TEMP_OBJETIVO - TOLERANCIA)) {
    // Cayó a 29°C o menos → encender estufa
    digitalWrite(PIN_RELE, RELE_ENCENDIDO);
    estufaEncendida = true;
  } else if (temperatura >= (TEMP_OBJETIVO + TOLERANCIA)) {
    // Subió a 31°C o más → apagar estufa
    digitalWrite(PIN_RELE, RELE_APAGADO);
    estufaEncendida = false;
  }
  // Si está entre 29 y 31, mantiene el estado actual (histéresis)

  // --- 3. Actualizar pantalla OLED ---
  display.clearDisplay();

  // Título y estado del relé
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Temp Local:"));

  display.setCursor(90, 0);
  if (estufaEncendida) {
    display.print(F("[ON]"));
  } else {
    display.print(F("[OFF]"));
  }

  // Temperatura en grande
  display.setTextSize(3);
  display.setCursor(10, 20);
  display.print(temperatura, 1);

  display.setTextSize(1);
  display.print((char)247);  // Símbolo de grado
  display.setTextSize(3);
  display.print("C");

  // Info técnica abajo
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(F("SP:"));
  display.print(TEMP_OBJETIVO, 0);
  display.print(F("C  Tol:"));
  display.print(TOLERANCIA, 0);
  display.print(F("C  "));
  display.print(voltaje_mV);
  display.print(F("mV"));

  display.display();

  // --- 4. Debug por Serial ---
  Serial.printf("T:%.1f C | %d mV | Estufa:%s | SP:%.0f+/-%.0f\n",
                temperatura, voltaje_mV,
                estufaEncendida ? "ON" : "OFF",
                TEMP_OBJETIVO, TOLERANCIA);

  delay(1000);
}