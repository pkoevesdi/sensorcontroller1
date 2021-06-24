#include <ArduinoOTA.h>
#include <EspMQTTClient.h>
#include <EEPROM.h>
#include "RunningMedian.h"
#include "creds.h"

#define TOPICPREFIX "womo/sensorcontroller1"
#define READDELAYLPG 600000000 // Pause zwischen den Messungen in Mikrosekunden
#define READDELAYLPGSINGLE 100 // alle wieviel Millisekunden werden Einzelwerte für den Median ausgelesen

RunningMedian samples = RunningMedian(51); // Anzahl Einzelwerte, über die der Median gebildet wird

const float lpgArray[4] = {
    0, 0,
    1024, 1024/12.5};

unsigned long lastReadTimeLpg = 0;
unsigned long lastReadTimeLpgSingle = 0;
unsigned long timer = 0;
float lpg = 0;
unsigned int aVal = 0;
bool connected = false;
bool debug = false;

EspMQTTClient client(
    WIFI_SSID,
    WIFI_PASS,
    BROKER_IP,       // MQTT Broker server ip
    "sensorcontroller_1" // Client name that uniquely identify your device
);

float map_fl(unsigned int val)
{
  Serial.printf("mapping Value %d...\n", val);
  for (byte i = 0; i < 2; i = i + 2)
  {
    if (val >= lpgArray[i] && val <= lpgArray[i + 2])
    {
      return ((float)val - lpgArray[i]) * (lpgArray[i + 3] - lpgArray[i + 1]) / (lpgArray[i + 2] - lpgArray[i]) + lpgArray[i + 1];
    }
  }
  return -1;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting...");

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  client.enableDebuggingMessages();
  client.enableLastWillMessage(TOPICPREFIX "/status", "offline");
}

void onConnectionEstablished()
{
  Serial.println("Connection established.");
  client.subscribe(TOPICPREFIX "/debug", [](const String &payload)
                   { debug = payload.charAt(0) - 48; });
  client.publish(TOPICPREFIX "/status", "online", true);
  ArduinoOTA.begin();
  connected = true;
}

void loop()
{
  client.loop();

  if (!connected)
  {
    if (millis() - timer >= 500)
    {
      Serial.print(".");
      timer = millis();
    }
    return;
  }

  if (debug)
  {
    ArduinoOTA.handle();
    if ((millis() - lastReadTimeLpgSingle) > 1000)
    {
      aVal = analogRead(A0);
      client.publish(TOPICPREFIX "/lpg_raw", (String)aVal, true);
      client.publish(TOPICPREFIX "/lpg", (String)map_fl(aVal), true);
      lastReadTimeLpgSingle = millis();
    }
    return;
  }

  if ((millis() - lastReadTimeLpgSingle) > READDELAYLPGSINGLE)
  {
    aVal = analogRead(A0);
    samples.add(aVal);
    lastReadTimeLpgSingle = millis();
    Serial.printf("Sample %d von %d: %d\n", samples.getCount(), samples.getSize(), aVal);
  }

  if (samples.getCount() >= samples.getSize())
  {
    aVal = samples.getMedian();
    client.publish(TOPICPREFIX "/lpg", (String)map_fl(aVal), true);
    client.publish(TOPICPREFIX "/lpg_raw", (String)aVal, true);
    delay(500);
    ESP.deepSleep(READDELAYLPG);
  }
}