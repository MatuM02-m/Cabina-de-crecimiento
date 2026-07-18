#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
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
//  Lado izquierdo:  Relé, OneWire Bus 1
//  Lado derecho:    OLED (I2C), OneWire Bus 2 y 3
// ============================================================
const int PIN_OW_BUS1 = 4;    // OneWire Bus 1 — sensores izquierdos (↑Izq + ↓Izq)
const int PIN_OW_BUS2 = 16;   // OneWire Bus 2 — sensores derechos  (↑Der + ↓Der)
const int PIN_OW_BUS3 = 17;   // OneWire Bus 3 — sensor cultivo (seguridad)
const int PIN_RELE    = 26;   // GPIO26 — Relé estufa
const int PIN_FAN     = 27;   // GPIO27 — PWM ventilador (via BC337 + optoacoplador)
const int PIN_BUZZER  = 25;   // GPIO25 — Buzzer 5V (alerta de seguridad)
const int PIN_LED     = 2;    // LED built-in

// ============================================================
//  CONFIGURACIÓN DS18B20 — 3 buses OneWire
//  Cada bus necesita una resistencia pull-up de 4.7kΩ a 3.3V
// ============================================================
OneWire ow1(PIN_OW_BUS1);
OneWire ow2(PIN_OW_BUS2);
OneWire ow3(PIN_OW_BUS3);
DallasTemperature sensoresBus1(&ow1);
DallasTemperature sensoresBus2(&ow2);
DallasTemperature sensoresBus3(&ow3);

// Lectura asíncrona (no bloquea el loop durante la conversión)
bool conversionPendiente = false;
unsigned long tiempoConversion = 0;
const unsigned long CONVERSION_DELAY_MS = 750;  // 750ms para resolución de 12-bit

// Sensores descubiertos en cada bus
int sensoresBus1Count = 0;
int sensoresBus2Count = 0;
int sensoresBus3Count = 0;

// Índices para los 4 sensores de cabina
// NOTA: El índice dentro de cada bus depende del orden de dirección ROM.
//       Verificar por Serial al arrancar y reordenar físicamente si es necesario.
enum SensorCabina { ARRIBA_IZQ = 0, ABAJO_IZQ = 1, ARRIBA_DER = 2, ABAJO_DER = 3 };

// ============================================================
//  CONFIGURACIÓN DEL RELÉ
//  Lógica activa en HIGH: HIGH = encendido, LOW = apagado
// ============================================================
#define RELE_ENCENDIDO HIGH
#define RELE_APAGADO   LOW

// ============================================================
//  CONFIGURACIÓN DEL VENTILADOR (PWM)
//  Cooler de PC 12V controlado por BC337 + optoacoplador
//  PWM a 25kHz para evitar ruido audible, resolución 8 bits
// ============================================================
const int FAN_PWM_FREQ       = 25000;  // 25kHz (inaudible)
const int FAN_PWM_RESOLUTION = 8;      // 8 bits → 0-255
const int FAN_DUTY_OFF       = 0;      //   0%
const int FAN_DUTY_LOW       = 76;     // ~30%
const int FAN_DUTY_HIGH      = 191;    // ~75%
const int FAN_DUTY_MAX       = 255;    // 100%

// ============================================================
//  PARÁMETROS DE CONTROL TÉRMICO (Histéresis)
//  Enciende al caer a (OBJETIVO - TOLERANCIA) = 29°C
//  Apaga al subir a  (OBJETIVO + TOLERANCIA) = 31°C
// ============================================================
const float TEMP_OBJETIVO = 30.0;
const float TOLERANCIA    = 1.0;

// ============================================================
//  PARÁMETROS DE SEGURIDAD (Sensor cultivo)
//  Activa modo seguridad si cultivo > 35°C
//  Desactiva cuando cultivo < 33°C (histéresis de 2°C)
// ============================================================
const float TEMP_SEGURIDAD    = 35.0;  // Umbral de activación
const float TEMP_RECUPERACION = 33.0;  // Umbral de desactivación

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
float tempSensores[4] = {-127.0, -127.0, -127.0, -127.0};  // 4 sensores cabina
float tempCultivo     = -127.0;   // Sensor del cultivo (seguridad)
float temperatura     = 0.0;      // Promedio de cabina (para control)
bool estufaEncendida  = false;
int  fanPorcentaje    = 0;       // Porcentaje actual del ventilador (0-100)
bool modoSeguridad    = false;   // Lazo de seguridad activo
bool primeraLectura   = false;
unsigned long ultimaActualizacion = 0;

// ============================================================
//  HISTORIAL DE TEMPERATURA
//  48 registros × 10 min = 8 horas de datos
// ============================================================
struct TempRecord {
  unsigned long timestamp;  // millis() al momento del registro
  float temp;               // Promedio de cabina
  float tempCrop;           // Temperatura del cultivo
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
  doc["t"] = round(temperatura * 10.0) / 10.0;
  doc["h"] = estufaEncendida;
  doc["sp"] = TEMP_OBJETIVO;
  doc["tol"] = TOLERANCIA;
  doc["tCultivo"] = round(tempCultivo * 10.0) / 10.0;
  doc["fan"] = fanPorcentaje;
  doc["safety"] = modoSeguridad;

  JsonArray sensors = doc["sensors"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    if (tempSensores[i] > -50.0 && tempSensores[i] < 85.0) {
      sensors.add(round(tempSensores[i] * 10.0) / 10.0);
    } else {
      sensors.add(nullptr);  // null para sensores desconectados
    }
  }

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
    entry.add(round(hist[idx].tempCrop * 10.0) / 10.0);
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
    hist[histHead] = {now, temperatura, tempCultivo};
    histHead = (histHead + 1) % HIST_MAX;
    histCount++;
    lastRecordTime = now;
    firstRecordDone = true;
    Serial.printf("[HIST] Primer registro: Cabina=%.1f C, Cultivo=%.1f C\n",
                  temperatura, tempCultivo);
    ws.textAll(getHistoryJson());
    mqttPublishHistory();
    return;
  }

  // Registros siguientes: cada 10 minutos
  if (now - lastRecordTime >= RECORD_INTERVAL) {
    hist[histHead] = {now, temperatura, tempCultivo};
    histHead = (histHead + 1) % HIST_MAX;
    histCount++;
    lastRecordTime = now;
    Serial.printf("[HIST] Registro #%d: Cabina=%.1f C, Cultivo=%.1f C\n",
                  histCount, temperatura, tempCultivo);
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
//  LECTURA DE SENSORES DS18B20 (asíncrona, no bloquea el loop)
//  Ciclo: solicitar conversión → esperar 750ms → leer valores
// ============================================================
void leerSensores() {
  // 1. Si no hay conversión pendiente, solicitar una nueva
  if (!conversionPendiente) {
    sensoresBus1.requestTemperatures();
    sensoresBus2.requestTemperatures();
    sensoresBus3.requestTemperatures();
    conversionPendiente = true;
    tiempoConversion = millis();
    return;
  }

  // 2. Esperar a que termine la conversión (~750ms para 12-bit)
  if (millis() - tiempoConversion < CONVERSION_DELAY_MS) return;
  conversionPendiente = false;

  // 3. Leer temperaturas de cada bus
  // Bus 1 (GPIO4): sensores lado izquierdo
  if (sensoresBus1Count >= 1) tempSensores[ARRIBA_IZQ] = sensoresBus1.getTempCByIndex(0);
  if (sensoresBus1Count >= 2) tempSensores[ABAJO_IZQ]  = sensoresBus1.getTempCByIndex(1);

  // Bus 2 (GPIO16): sensores lado derecho
  if (sensoresBus2Count >= 1) tempSensores[ARRIBA_DER] = sensoresBus2.getTempCByIndex(0);
  if (sensoresBus2Count >= 2) tempSensores[ABAJO_DER]  = sensoresBus2.getTempCByIndex(1);

  // Bus 3 (GPIO17): sensor del cultivo
  if (sensoresBus3Count >= 1) tempCultivo = sensoresBus3.getTempCByIndex(0);

  // 4. Calcular promedio de cabina (solo sensores válidos)
  float suma = 0;
  int validos = 0;
  for (int i = 0; i < 4; i++) {
    // Rango válido del DS18B20: -55°C a +125°C
    // Usamos -50 a 85 como rango razonable para la cabina
    if (tempSensores[i] > -50.0 && tempSensores[i] < 85.0) {
      suma += tempSensores[i];
      validos++;
    }
  }

  if (validos > 0) {
    temperatura = suma / validos;
    if (!primeraLectura) primeraLectura = true;
  }
}

// ============================================================
//  CONTROL DEL RELÉ (Histéresis)
//  Usa el promedio de los 4 sensores de cabina
//  ANULADO por modo seguridad (estufa forzada OFF)
// ============================================================
void controlarEstufa() {
  if (!primeraLectura) return;

  // Modo seguridad: estufa SIEMPRE apagada
  if (modoSeguridad) {
    digitalWrite(PIN_RELE, RELE_APAGADO);
    estufaEncendida = false;
    return;
  }

  if (temperatura <= (TEMP_OBJETIVO - TOLERANCIA)) {
    digitalWrite(PIN_RELE, RELE_ENCENDIDO);
    estufaEncendida = true;
  } else if (temperatura >= (TEMP_OBJETIVO + TOLERANCIA)) {
    digitalWrite(PIN_RELE, RELE_APAGADO);
    estufaEncendida = false;
  }
  // Entre 29°C y 31°C: mantiene el estado actual (histéresis)
}

// ============================================================
//  CONTROL DEL VENTILADOR (PWM)
//  Distribuye el calor y mantiene circulación de aire
//  MODO SEGURIDAD: forzado a 100%
//  Normal: < 29°C → 75% | ≥ 29°C → 30%
// ============================================================
void controlarVentilador() {
  if (!primeraLectura) return;

  int duty;

  // Modo seguridad: ventilador al máximo
  if (modoSeguridad) {
    duty = FAN_DUTY_MAX;
    fanPorcentaje = 100;
  } else if (temperatura < (TEMP_OBJETIVO - TOLERANCIA)) {
    duty = FAN_DUTY_HIGH;   // 75% — distribuir calor
    fanPorcentaje = 75;
  } else {
    duty = FAN_DUTY_LOW;    // 30% — circulación suave
    fanPorcentaje = 30;
  }

  ledcWrite(PIN_FAN, duty);
}

// ============================================================
//  LAZO DE SEGURIDAD
//  Monitorea el sensor del cultivo (Bus 3)
//  Activa si cultivo > 35°C → estufa OFF, fan 100%, buzzer ON
//  Desactiva si cultivo < 33°C → vuelve a control normal
// ============================================================
void controlarSeguridad() {
  if (!primeraLectura) return;
  // Solo actuar si el sensor del cultivo está conectado
  if (tempCultivo < -50.0 || tempCultivo > 125.0) return;

  if (!modoSeguridad && tempCultivo >= TEMP_SEGURIDAD) {
    modoSeguridad = true;
    Serial.println("[!!] SEGURIDAD ACTIVADA: Temp cultivo > 35 C");
    Serial.printf("     Cultivo: %.1f C | Accion: Estufa OFF, Fan 100%%, Buzzer ON\n", tempCultivo);
  } else if (modoSeguridad && tempCultivo <= TEMP_RECUPERACION) {
    modoSeguridad = false;
    digitalWrite(PIN_BUZZER, LOW);  // Apagar buzzer al recuperar
    Serial.println("[OK] SEGURIDAD DESACTIVADA: Temp cultivo < 33 C");
    Serial.printf("     Cultivo: %.1f C | Volviendo a control normal\n", tempCultivo);
  }

  // Buzzer continuo mientras esté en modo seguridad
  digitalWrite(PIN_BUZZER, modoSeguridad ? HIGH : LOW);
}

// ============================================================
//  FUNCIÓN AUXILIAR — Imprimir dirección ROM de un DS18B20
// ============================================================
void printAddress(DeviceAddress addr) {
  for (int i = 0; i < 8; i++) {
    if (addr[i] < 0x10) Serial.print("0");
    Serial.print(addr[i], HEX);
  }
}

// ============================================================
//  ACTUALIZAR PANTALLA + PUSH DE DATOS
//  Corre cada ~1 segundo
// ============================================================
void actualizarPantallaYPush() {
  // --- Pantalla OLED ---
  display.clearDisplay();

  // Línea 1: Título + estado estufa (y=0)
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("Cabina:"));
  display.setCursor(64, 0);
  display.printf("E:%s F:%d%%",
    estufaEncendida ? "ON" : "OF", fanPorcentaje);

  // Línea 2: Temperatura promedio grande (y=10)
  display.setTextSize(3);
  display.setCursor(10, 10);
  display.print(temperatura, 1);
  display.setTextSize(1);
  display.print((char)247);
  display.setTextSize(3);
  display.print("C");

  // Línea 3: 4 sensores individuales (y=36)
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.printf("%.1f %.1f %.1f %.1f",
    tempSensores[ARRIBA_IZQ], tempSensores[ABAJO_IZQ],
    tempSensores[ARRIBA_DER], tempSensores[ABAJO_DER]);

  // Línea 4: Temperatura cultivo (y=46)
  display.setCursor(0, 46);
  if (modoSeguridad) {
    display.print(F("!! SEGURIDAD !! "));
    display.printf("%.1fC", tempCultivo);
  } else {
    display.printf("Cultivo: %.1f%cC", tempCultivo, (char)247);
  }

  // Línea 5: WiFi + MQTT (y=56)
  display.setCursor(0, 56);
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

  // --- Debug Serial ---
  Serial.printf("Prom:%.1f C | [%.1f %.1f %.1f %.1f] | Cult:%.1f | Est:%s | Fan:%d%% | Seg:%s | WS:%u | MQTT:%s\n",
                temperatura,
                tempSensores[0], tempSensores[1],
                tempSensores[2], tempSensores[3],
                tempCultivo,
                estufaEncendida ? "ON" : "OFF",
                fanPorcentaje,
                modoSeguridad ? "!!" : "OK",
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
  Serial.println(" 5x DS18B20 | Rele | Fan PWM | OLED | MQTT");
  Serial.println("========================================");

  // --- LED indicador ---
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // --- Relé: configurar y arrancar apagado ---
  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, RELE_APAGADO);
  Serial.println("[OK] Rele configurado en GPIO26 (arranca apagado)");

  // --- Ventilador: PWM a 25kHz ---
  ledcAttach(PIN_FAN, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcWrite(PIN_FAN, FAN_DUTY_OFF);  // Arranca apagado
  Serial.println("[OK] Ventilador PWM configurado en GPIO27 (25kHz, 8-bit)");

  // --- Buzzer: pin de salida, arranca apagado ---
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  Serial.printf("[OK] Buzzer configurado en GPIO%d (seguridad)\n", PIN_BUZZER);

  // --- DS18B20: inicializar los 3 buses OneWire ---
  sensoresBus1.begin();
  sensoresBus2.begin();
  sensoresBus3.begin();

  // Modo asíncrono: no bloquear durante la conversión
  sensoresBus1.setWaitForConversion(false);
  sensoresBus2.setWaitForConversion(false);
  sensoresBus3.setWaitForConversion(false);

  // Resolución: 12 bits (0.0625°C, 750ms de conversión)
  sensoresBus1.setResolution(12);
  sensoresBus2.setResolution(12);
  sensoresBus3.setResolution(12);

  // Descubrir sensores en cada bus
  sensoresBus1Count = sensoresBus1.getDeviceCount();
  sensoresBus2Count = sensoresBus2.getDeviceCount();
  sensoresBus3Count = sensoresBus3.getDeviceCount();

  Serial.printf("\n[DS18B20] Bus 1 (GPIO%d) - Lado izquierdo: %d sensor(es)\n",
                PIN_OW_BUS1, sensoresBus1Count);
  Serial.printf("[DS18B20] Bus 2 (GPIO%d) - Lado derecho:    %d sensor(es)\n",
                PIN_OW_BUS2, sensoresBus2Count);
  Serial.printf("[DS18B20] Bus 3 (GPIO%d) - Cultivo:         %d sensor(es)\n",
                PIN_OW_BUS3, sensoresBus3Count);

  // Imprimir direcciones ROM para identificación física
  DeviceAddress addr;
  Serial.println("\nDirecciones ROM:");
  for (int i = 0; i < sensoresBus1Count; i++) {
    if (sensoresBus1.getAddress(addr, i)) {
      Serial.printf("  Bus 1, Sensor %d (idx %d → %s): ",
                    i, i, i == 0 ? "Arriba Izq" : "Abajo Izq");
      printAddress(addr);
      Serial.println();
    }
  }
  for (int i = 0; i < sensoresBus2Count; i++) {
    if (sensoresBus2.getAddress(addr, i)) {
      Serial.printf("  Bus 2, Sensor %d (idx %d → %s): ",
                    i, i, i == 0 ? "Arriba Der" : "Abajo Der");
      printAddress(addr);
      Serial.println();
    }
  }
  for (int i = 0; i < sensoresBus3Count; i++) {
    if (sensoresBus3.getAddress(addr, i)) {
      Serial.printf("  Bus 3, Sensor %d (Cultivo): ", i);
      printAddress(addr);
      Serial.println();
    }
  }

  int totalSensores = sensoresBus1Count + sensoresBus2Count + sensoresBus3Count;
  if (totalSensores < 5) {
    Serial.printf("\n[!!] ATENCION: Se esperaban 5 sensores, se encontraron %d\n", totalSensores);
    Serial.println("     El sistema funcionara con los sensores disponibles.");
  } else {
    Serial.printf("\n[OK] %d sensores DS18B20 encontrados\n", totalSensores);
  }
  Serial.println("     Verificar que la asignacion fisica coincida con los indices.");

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
//  Los sensores se leen asíncronamente (~750ms por ciclo)
//  La pantalla y el push se actualizan cada ~1 segundo
// ============================================================
void loop() {
  mqttConnect();
  mqttClient.loop();

  // Lectura asíncrona de los DS18B20 (no bloquea)
  leerSensores();

  // Lazo de seguridad (prioridad máxima, antes de estufa y fan)
  controlarSeguridad();

  // Control del relé y ventilador basado en el promedio de cabina
  controlarEstufa();
  controlarVentilador();

  unsigned long ahora = millis();

  // Pantalla, WebSocket y MQTT cada 1 segundo
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