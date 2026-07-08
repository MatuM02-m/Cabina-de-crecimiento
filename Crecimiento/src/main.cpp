#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "esp_adc_cal.h"
#include "dashboard.h"

// ============================================================
//  CREDENCIALES (desde secrets.h — no se sube a git)
// ============================================================
#include "secrets.h"

// Los topics MQTT no son secretos
const char* MQTT_TOPIC_STATUS  = "cabina/estado";
const char* MQTT_TOPIC_HISTORY = "cabina/historial";

// ============================================================
//  PINES
//  Lado izquierdo del ESP32 NodeMCU:  LM35 + Relé
//  Lado derecho del ESP32 NodeMCU:    OLED (I2C)
// ============================================================
const int PIN_LM35 = 34;   // ADC1_CH6 — lado izquierdo
const int PIN_RELE = 26;   // GPIO26   — lado izquierdo
const int PIN_LED  = 2;    // LED built-in

// ============================================================
//  CONFIGURACIÓN DEL LM35 (ADC)
// ============================================================
const int MUESTRAS = 32;
unsigned long ultimaLecturaTimer = 0; // Temporizador para el sensor
const unsigned long INTERVALO_LECTURA = 500; // Refresco de 500 ms
esp_adc_cal_characteristics_t adc_chars;

// ============================================================
//  CALIBRACIÓN DEL ADC — recta empírica de 2 puntos
// ============================================================
const float CAL_A = 0.6512;
const float CAL_B = 89.59;

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
//  SERVIDOR WEB LOCAL + WEBSOCKET
// ============================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ============================================================
//  CLIENTE MQTT
// ============================================================
WiFiClientSecure espClientSecure;
PubSubClient mqttClient(espClientSecure);

unsigned long lastMqttReconnect = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;  // Reintentar cada 5s

// ============================================================
//  ESTADO GLOBAL
// ============================================================
float tempFiltrada = -1.0; // Arranca en negativo para forzar la inicialización
const float ALFA = 0.02;   // Factor de suavizado: 2% lectura nueva, 98% historia
float temperatura = 0.0;
bool estufaEncendida = false;
bool primeraLectura = false;
uint32_t ultimoVoltaje_mV = 0;
unsigned long ultimaActualizacion = 0;



// ============================================================
//  HISTORIAL DE TEMPERATURA
//  48 registros × 10 min = 8 horas de datos
// ============================================================
struct TempRecord {
  unsigned long timestamp;  // millis() al momento del registro
  float temp;
};
const int HIST_MAX = 48;
TempRecord hist[HIST_MAX];
int histCount = 0;
int histHead = 0;  // posición de escritura (buffer circular)
unsigned long lastRecordTime = 0;
const unsigned long RECORD_INTERVAL = 600000UL;  // 10 minutos en ms
bool firstRecordDone = false;

// ============================================================
//  LED INDICADOR (GPIO2, HIGH = encendido)
// ============================================================
unsigned long ledOffTime = 0;
const unsigned long LED_BLINK_MS = 50;

// ============================================================
//  FUNCIONES AUXILIARES — JSON
// ============================================================

String getStatusJson() {
  JsonDocument doc;
  doc["t"] = temperatura;
  doc["h"] = estufaEncendida;
  doc["sp"] = TEMP_OBJETIVO;
  doc["tol"] = TOLERANCIA;
  String json;
  serializeJson(doc, json);
  return json;
}

String getHistoryJson() {
  JsonDocument doc;
  doc["type"] = "hist";
  unsigned long now = millis();
  JsonArray arr = doc["r"].to<JsonArray>();

  int count = (histCount < HIST_MAX) ? histCount : HIST_MAX;
  int start = (histCount <= HIST_MAX) ? 0 : histHead;
  for (int i = 0; i < count; i++) {
    int idx = (start + i) % HIST_MAX;
    JsonArray entry = arr.add<JsonArray>();
    entry.add((long)((now - hist[idx].timestamp) / 1000));
    entry.add(round(hist[idx].temp * 10.0) / 10.0);
  }

  String json;
  serializeJson(doc, json);
  return json;
}

// ============================================================
//  MQTT — Conexión y publicación
// ============================================================

void mqttConnect() {
  if (mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttReconnect < MQTT_RECONNECT_INTERVAL) return;
  lastMqttReconnect = now;

  String clientId = "ESP32-Cabina-" + String(random(0xffff), HEX);
  Serial.printf("[MQTT] Conectando como '%s'... ", clientId.c_str());

  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println("OK!");
  } else {
    Serial.printf("Error (rc=%d)\n", mqttClient.state());
  }
}

void mqttPublishStatus() {
  if (!mqttClient.connected()) return;
  String json = getStatusJson();
  mqttClient.publish(MQTT_TOPIC_STATUS, json.c_str(), true);  // retained
}

void mqttPublishHistory() {
  if (!mqttClient.connected()) return;
  String json = getHistoryJson();
  mqttClient.publish(MQTT_TOPIC_HISTORY, json.c_str(), true);  // retained
}

// ============================================================
//  HISTORIAL — Registrar si corresponde
// ============================================================
void recordIfNeeded() {
  if (!primeraLectura) return;
  unsigned long now = millis();

  // Primer registro: apenas hay una lectura válida
  if (!firstRecordDone) {
    hist[histHead] = {now, temperatura};
    histHead = (histHead + 1) % HIST_MAX;
    histCount++;
    lastRecordTime = now;
    firstRecordDone = true;
    Serial.printf("[HIST] Primer registro: %.1f C\n", temperatura);
    ws.textAll(getHistoryJson());
    mqttPublishHistory();
    return;
  }

  // Registros siguientes: cada 10 minutos
  if (now - lastRecordTime >= RECORD_INTERVAL) {
    hist[histHead] = {now, temperatura};
    histHead = (histHead + 1) % HIST_MAX;
    histCount++;
    lastRecordTime = now;
    Serial.printf("[HIST] Registro #%d: %.1f C\n", histCount, temperatura);
    ws.textAll(getHistoryJson());
    mqttPublishHistory();
  }
}

// ============================================================
//  WEBSOCKET — Eventos de conexión/desconexión
// ============================================================
void onWsEvent(AsyncWebSocket *svr, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS cliente #%u conectado\n", client->id());
    if (primeraLectura) {
      client->text(getStatusJson());
    }
    if (histCount > 0) {
      client->text(getHistoryJson());
    }
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS cliente #%u desconectado\n", client->id());
  }
}

// ============================================================
//  LECTURA DEL SENSOR + CONTROL DEL RELÉ
//  Corre en cada iteración del loop (~130ms)
// ============================================================
void leerYControlar() {
  // 1. Lectura cruda inmediata (sin for ni delays)
  uint32_t valor_raw = analogRead(PIN_LM35);
  ultimoVoltaje_mV = esp_adc_cal_raw_to_voltage(valor_raw, &adc_chars);

  // Calibración: Compensación del offset del ADC (30 mV)
  ultimoVoltaje_mV = (uint32_t)(CAL_A * ultimoVoltaje_mV + CAL_B);

  float tempInst = ultimoVoltaje_mV / 10.0;
  
  // 2. Filtro digital IIR
  if (tempFiltrada < 0) {
    tempFiltrada = tempInst; // Inicializa de golpe en la primera lectura
    primeraLectura = true;
  } else {
    tempFiltrada = (ALFA * tempInst) + ((1.0 - ALFA) * tempFiltrada);
  }
  
  temperatura = tempFiltrada;

  // 3. Control del relé
  if (temperatura <= (TEMP_OBJETIVO - TOLERANCIA)) {
    digitalWrite(PIN_RELE, RELE_ENCENDIDO);
    estufaEncendida = true;
  } else if (temperatura >= (TEMP_OBJETIVO + TOLERANCIA)) {
    digitalWrite(PIN_RELE, RELE_APAGADO);
    estufaEncendida = false;
  }
}

// ============================================================
//  ACTUALIZAR PANTALLA + PUSH DE DATOS
//  Corre cada ~1 segundo
// ============================================================
void actualizarPantallaYPush() {
  // --- Pantalla OLED ---
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Temp Local:"));
  display.setCursor(90, 0);
  if (estufaEncendida) {
    display.print(F("[ON]"));
  } else {
    display.print(F("[OFF]"));
  }

  display.setTextSize(3);
  display.setCursor(10, 20);
  display.print(temperatura, 1);
  display.setTextSize(1);
  display.print((char)247);
  display.setTextSize(3);
  display.print("C");

  display.setTextSize(1);
  display.setCursor(0, 55);
  if (WiFi.status() == WL_CONNECTED) {
    display.print(WiFi.localIP());
    display.print(mqttClient.connected() ? " [MQTT]" : "");
  } else {
    display.print(F("WiFi: Desconectado"));
  }
  display.display();

  // --- Push WebSocket local ---
  if (WiFi.status() == WL_CONNECTED && ws.count() > 0) {
    ws.textAll(getStatusJson());
  }

  // --- Push MQTT ---
  mqttPublishStatus();

  // --- LED ---
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(PIN_LED, HIGH);
    ledOffTime = millis();
  }

  // --- Debug ---
  Serial.printf("T:%.1f C | %lu mV | Estufa:%s | WS:%u | MQTT:%s\n",
                temperatura, ultimoVoltaje_mV,
                estufaEncendida ? "ON" : "OFF",
                ws.count(),
                mqttClient.connected() ? "OK" : "NO");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println(" Cabina de Crecimiento - ESP32");
  Serial.println("========================================");

  // --- LED indicador ---
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // --- Relé: configurar y arrancar apagado ---
  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, RELE_APAGADO);
  Serial.println("[OK] Rele configurado en GPIO26 (arranca apagado)");

  // --- ADC: calibración para el LM35 ---
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0,
                           ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.println("[ADC] Calibracion: Two Point (fabrica) - buena");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.println("[ADC] Calibracion: eFuse Vref - aceptable");
  } else {
    Serial.println("[ADC] Calibracion: Default Vref - POCO CONFIABLE");
  }
  analogSetPinAttenuation(PIN_LM35, ADC_0db);
  analogReadResolution(12);
  Serial.println("[OK] ADC calibrado (GPIO34, 12-bit)");

  // --- OLED: inicializar ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, DIRECCION_I2C)) {
    Serial.println("[!!] Error: No se encontro la pantalla OLED");
    for (;;);
  }
  Serial.println("[OK] Pantalla OLED inicializada");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.println(F("Conectando WiFi..."));
  display.display();

  // --- WiFi ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[..] Conectando a '%s'", WIFI_SSID);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 40) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[OK] WiFi conectado!");
    Serial.print("[OK] Dashboard local: http://");
    Serial.println(WiFi.localIP());

    display.clearDisplay();
    display.setCursor(10, 10);
    display.println(F("WiFi conectado!"));
    display.setCursor(10, 30);
    display.print(F("IP: "));
    display.println(WiFi.localIP());
    display.display();
    delay(2000);
  } else {
    Serial.println("[!!] No se pudo conectar al WiFi");
    display.clearDisplay();
    display.setCursor(10, 25);
    display.println(F("Error WiFi!"));
    display.display();
    delay(5000);
    ESP.restart();
  }

  // --- MQTT: configurar cliente ---
  espClientSecure.setInsecure();  // Omitir verificación de certificado
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(2048);  // Buffer grande para el JSON de historial
  Serial.println("[OK] MQTT configurado (HiveMQ Cloud, TLS)");

  // --- WebSocket local ---
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // --- Rutas del servidor web local ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", getStatusJson());
  });

  server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", getHistoryJson());
  });

  server.begin();
  Serial.println("[OK] Servidor web local listo en puerto 80");
  Serial.printf("[OK] Historial cada %lu min (%d registros = 8h)\n",
                RECORD_INTERVAL / 60000,
                HIST_MAX);
  Serial.println("========================================\n");
}

// ============================================================
//  LOOP
//  El relé reacciona cada ~130ms (tiempo del sobremuestreo)
//  La pantalla y el push se actualizan cada ~1 segundo
// ============================================================
void loop() {
  mqttConnect();
  mqttClient.loop();

  // El filtro corre a máxima velocidad, amortiguando cualquier pico al instante
  leerYControlar();

  unsigned long ahora = millis();

  // Pantalla y MQTT cada 1 segundo
  if (ahora - ultimaActualizacion >= 1000) {
    ultimaActualizacion = ahora;
    actualizarPantallaYPush();
    recordIfNeeded();
  }

  if (ledOffTime > 0 && ahora - ledOffTime >= LED_BLINK_MS) {
    digitalWrite(PIN_LED, LOW);
    ledOffTime = 0;
  }

  ws.cleanupClients();
}