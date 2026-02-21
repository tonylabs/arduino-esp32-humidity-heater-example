#include "Wire.h"
#include "SHT31.h"
#include <FastLED.h>
#include <WiFi.h>
#include <WebServer.h>

const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

WebServer server(80);

// Latest readings for web page (updated in loop)
float webTemperature   = 0.0f;
float webHumidity      = 0.0f;
float webBatteryVoltage = 0.0f;
bool  webBatteryOk     = false;

// Relay: when true, humidity logic does not change relay (user controls via web)
bool relayManualOverride = false;

// Locate: set by web handler; locate task beeps 10 times
volatile bool locateRequested = false;

#define SHT31_ADDRESS   0x44
#define RELAY_PIN       33
#define BUZZER_PIN      14
#define LED_PIN         17
#define NUM_LEDS        4
#define CHARGER_CE_PIN  13
#define BQ27441_ADDR    0x55
#define BQ27441_REG_VOLTAGE 0x04
#define HUMIDITY_LIMIT  20.0
#define HUMIDITY_HYST   1.0
#define TEMP_SAFETY_C   40.0
#define BATTERY_FULL_V      4.20
#define BATTERY_RECHARGE_V  4.10
// BQ24075 CE is typically active-low: LOW=charge enabled, HIGH=disabled.
#define CHARGER_CE_ACTIVE_LEVEL LOW
#define CHARGER_CE_INACTIVE_LEVEL HIGH

uint32_t start;
uint32_t stop;

SHT31 sht;
bool relayIsOn = false;
CRGB leds[NUM_LEDS];
float lastTemperature = -1000.0;
int8_t tempTrend = 0;          // 1 = rising, -1 = falling, 0 = stable
uint8_t trendAnimPos = 0;
unsigned long trendAnimLastMs = 0;
bool chargingEnabled = true;

bool readBQ27441Word(uint8_t reg, uint16_t &value)
{
  Wire.beginTransmission(BQ27441_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (Wire.requestFrom((uint8_t)BQ27441_ADDR, (uint8_t)2) != 2)
  {
    return false;
  }

  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  value = ((uint16_t)msb << 8) | lsb;
  return true;
}

bool readBatteryVoltageV(float &voltageV)
{
  uint16_t voltageMv = 0;
  if (!readBQ27441Word(BQ27441_REG_VOLTAGE, voltageMv))
  {
    return false;
  }

  voltageV = voltageMv / 1000.0;
  return true;
}

void setChargingEnabled(bool enable)
{
  chargingEnabled = enable;
  digitalWrite(CHARGER_CE_PIN, enable ? CHARGER_CE_ACTIVE_LEVEL : CHARGER_CE_INACTIVE_LEVEL);
}

void alertBuzzer()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

// Runs in a separate task: beeps 10 times when locateRequested is set
void locateTask(void*)
{
  for (;;)
  {
    if (locateRequested)
    {
      locateRequested = false;
      for (int i = 0; i < 10; i++)
      {
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(150));
        digitalWrite(BUZZER_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void updateTemperatureTrend(float temperature)
{
  const float trendDeadband = 0.15;  // ignore tiny sensor fluctuations

  if (lastTemperature < -500.0)
  {
    lastTemperature = temperature;
    tempTrend = 0;
    return;
  }

  const float delta = temperature - lastTemperature;
  if (delta > trendDeadband)
  {
    tempTrend = 1;
  }
  else if (delta < -trendDeadband)
  {
    tempTrend = -1;
  }
  else
  {
    tempTrend = 0;
  }

  lastTemperature = temperature;
}

void showTemperatureLeds(float temperature)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }

  // 4-step "thermometer":
  // <18C: 1 blue, 18-25C: 2 green, 25-32C: 3 yellow, >=32C: 4 red
  if (temperature < 18.0)
  {
    leds[0] = CRGB::Blue;
  }
  else if (temperature < 25.0)
  {
    leds[0] = CRGB::Green;
    leds[1] = CRGB::Green;
  }
  else if (temperature < 32.0)
  {
    leds[0] = CRGB::Yellow;
    leds[1] = CRGB::Yellow;
    leds[2] = CRGB::Yellow;
  }
  else
  {
    leds[0] = CRGB::Red;
    leds[1] = CRGB::Red;
    leds[2] = CRGB::Red;
    leds[3] = CRGB::Red;
  }

  // Animate direction: rising moves left->right, falling right->left.
  if (temperature <= TEMP_SAFETY_C && tempTrend != 0)
  {
    const unsigned long now = millis();
    if (now - trendAnimLastMs >= 150)
    {
      trendAnimLastMs = now;
      if (tempTrend > 0)
      {
        trendAnimPos = (trendAnimPos + 1) % NUM_LEDS;
      }
      else
      {
        trendAnimPos = (trendAnimPos == 0) ? (NUM_LEDS - 1) : (trendAnimPos - 1);
      }
    }

    leds[trendAnimPos] = CRGB::White;
  }

  // Critical over-temperature warning blinks all LEDs red.
  if (temperature > TEMP_SAFETY_C)
  {
    static bool blinkState = false;
    blinkState = !blinkState;
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = blinkState ? CRGB::Red : CRGB::Black;
    }
  }

  FastLED.show();
}

void serveRoot()
{
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<title>SHT31 Monitor</title>";
  html += "<style>body{font-family:sans-serif;max-width:360px;margin:2em auto;padding:1em;background:#1a1a2e;color:#eee;}";
  html += "h1{color:#0f3460;} .card{background:#16213e;border-radius:8px;padding:1em;margin:0.5em 0;}";
  html += ".label{color:#aaa;} .value{font-size:1.5em;font-weight:bold;color:#e94560;}</style></head><body>";
  html += "<h1>SHT31 Monitor</h1>";
  html += "<div class='card'><span class='label'>Temperature</span><br><span class='value'>";
  html += String(webTemperature, 1);
  html += " &deg;C</span></div>";
  html += "<div class='card'><span class='label'>Humidity</span><br><span class='value'>";
  html += String(webHumidity, 1);
  html += " %</span></div>";
  html += "<div class='card'><span class='label'>Battery voltage</span><br><span class='value'>";
  if (webBatteryOk)
    html += String(webBatteryVoltage, 3) + " V";
  else
    html += "N/A";
  html += "</span></div>";
  html += "<div class='card'><span class='label'>Relay</span><br>";
  html += "<span class='value'>" + String(relayIsOn ? "ON" : "OFF");
  if (relayManualOverride) html += " (Manual)";
  html += "</span><br>";
  html += "<a href='/relay/toggle' style='display:inline-block;margin:0.25em 0.25em 0 0;padding:0.4em 0.8em;background:#0f3460;color:#eee;text-decoration:none;border-radius:4px;'>Toggle relay</a>";
  if (relayManualOverride)
    html += " <a href='/relay/auto' style='display:inline-block;margin:0.25em 0;padding:0.4em 0.8em;background:#2e7d32;color:#eee;text-decoration:none;border-radius:4px;'>Resume auto</a>";
  html += "</div>";
  html += "<div class='card'><a href='/locate' style='display:inline-block;padding:0.5em 1em;background:#e94560;color:#fff;text-decoration:none;border-radius:4px;'>Locate device (beep 10x)</a></div>";
  html += "<p style='color:#666;font-size:0.9em'>Page auto-refreshes every 5 s.</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleRelayToggle()
{
  relayManualOverride = true;
  relayIsOn = !relayIsOn;
  digitalWrite(RELAY_PIN, relayIsOn ? HIGH : LOW);
  Serial.println(relayIsOn ? "Relay ON (manual)" : "Relay OFF (manual)");
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleRelayAuto()
{
  relayManualOverride = false;
  Serial.println("Relay back to auto (humidity)");
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleLocate()
{
  locateRequested = true;
  Serial.println("Locate requested (10 beeps)");
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void setup()
{
  //  while(!Serial);  //  uncomment if needed
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("SHT31_LIB_VERSION: \t");
  Serial.println(SHT31_LIB_VERSION);
  Serial.println();

  Wire.begin();
  Wire.setClock(100000);
  sht.begin();
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CHARGER_CE_PIN, OUTPUT);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(64);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  setChargingEnabled(true);
  FastLED.clear();
  FastLED.show();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", serveRoot);
  server.on("/relay/toggle", handleRelayToggle);
  server.on("/relay/auto", handleRelayAuto);
  server.on("/locate", handleLocate);
  server.begin();

  xTaskCreatePinnedToCore(locateTask, "locate", 2048, NULL, 1, NULL, 0);
  Serial.println("Web server started. Open http://" + WiFi.localIP().toString() + " in a browser.");

  uint16_t stat = sht.readStatus();
  Serial.print(stat, HEX);
  Serial.println();
}


void loop()
{
  start = micros();
  sht.read();         //  default = true/fast       slow = false
  stop = micros();
  float humidity = sht.getHumidity();
  float temperature = sht.getTemperature();
  float batteryVoltageV = 0.0;
  bool batteryOk = readBatteryVoltageV(batteryVoltageV);

  webTemperature = temperature;
  webHumidity = humidity;
  webBatteryVoltage = batteryVoltageV;
  webBatteryOk = batteryOk;

  server.handleClient();

  if (batteryOk)
  {
    if (chargingEnabled && batteryVoltageV >= BATTERY_FULL_V)
    {
      setChargingEnabled(false);
      Serial.println("Charging DISABLED (battery full)");
    }
    else if (!chargingEnabled && batteryVoltageV <= BATTERY_RECHARGE_V)
    {
      setChargingEnabled(true);
      Serial.println("Charging ENABLED (battery recharged threshold)");
    }
  }

  updateTemperatureTrend(temperature);
  showTemperatureLeds(temperature);

  const float relayOnThreshold = HUMIDITY_LIMIT + HUMIDITY_HYST;
  const float relayOffThreshold = HUMIDITY_LIMIT - HUMIDITY_HYST;

  if (temperature > TEMP_SAFETY_C)
  {
    if (relayIsOn)
    {
      relayIsOn = false;
      digitalWrite(RELAY_PIN, LOW);
      alertBuzzer();
      Serial.println("Relay OFF (temperature safety)");
    }
  }
  else if (!relayManualOverride)
  {
    if (humidity >= relayOnThreshold && !relayIsOn)
    {
      relayIsOn = true;
      digitalWrite(RELAY_PIN, HIGH);
      alertBuzzer();
      Serial.println("Relay ON (humidity above upper threshold)");
    }
    else if (humidity <= relayOffThreshold && relayIsOn)
    {
      relayIsOn = false;
      digitalWrite(RELAY_PIN, LOW);
      alertBuzzer();
      Serial.println("Relay OFF (humidity below lower threshold)");
    }
  }

  Serial.print("\t");
  Serial.print(stop - start);
  Serial.print("\t");
  Serial.print(temperature, 1);
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t");
  if (batteryOk)
  {
    Serial.print(batteryVoltageV, 3);
  }
  else
  {
    Serial.print("N/A");
  }
  Serial.print("\t");
  Serial.println(chargingEnabled ? "CHG_ON" : "CHG_OFF");
  delay(100);
}


//  -- END OF FILE --

