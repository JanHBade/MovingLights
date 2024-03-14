

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266Wifi.h>
#include <ESP8266Webserver.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Ticker.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WS2812FX.h>
#include <MQTT.h>
#include <ArduinoJson.h>

#define LEDPIN 13
#define NUMPIXELS 16
WS2812FX ws2812fx = WS2812FX(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);
int Hue = 240;

Ticker tAutoFarbe;
int tickerStart=2;

Adafruit_BME280 bme; // I2C

struct Values
{
	float Feuchtigkeit = 0;
	float Luftdruck = 0;
	float Temperatur = 0;
  bool Sensor;
  bool TimerActive;
	int32_t WlanSignal = 0;
	uint16_t Zaehler = 0;
};
Values values;

#define SENSOR_PIN 15
Ticker tStopLight;
bool Running = false;

WiFiClient espClient;
MQTTClient MqttClient;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

#define HOSTNAME "MovingLight"
#define JSON_BUFFER_SIZE 150

#define MQTT_BROKER "Isis.lan"
#define MQTT_PREFIX "SPS/"
#define MQTT_TOPIC_LED MQTT_PREFIX HOSTNAME "/Led"
#define MQTT_TOPIC_HW MQTT_PREFIX HOSTNAME "/HelloWorld"
#define MQTT_TOPIC_JSON MQTT_PREFIX HOSTNAME

char hw_buf[JSON_BUFFER_SIZE];
int volatile UpdateCnt = 0;

void reconnectMqtt()
{
  if (!MqttClient.connected())
	{
    Serial.print("MQTT Server: ");
    IPAddress ip;
    Serial.println(WiFi.hostByName(MQTT_BROKER, ip));
    Serial.println(ip);
    MqttClient.begin(ip, espClient);
    
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MqttClient.connect((HOSTNAME + WiFi.macAddress()).c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      MqttClient.publish(MQTT_TOPIC_HW, hw_buf, true, 0);
    }
    else
    {
      Serial.print("failed");
    }
  }
}

void reconnectWifi()
{
  if(!WiFi.isConnected())
  {
    //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;
    wm.setHostname(HOSTNAME);
    wm.setDarkMode(true);
    wm.setScanDispPerc(true); // display percentages instead of graphs for RSSI

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    // wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("MovingLight-12345678","12345678"); // password protected ap

    if(!res)
    {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else
    {
      Serial.println("WiFi connected");
      Serial.println("MAC: ");
      Serial.println(WiFi.macAddress());
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());

      StaticJsonDocument<JSON_BUFFER_SIZE> hw_doc;
      hw_doc["MAC"] = WiFi.macAddress();	
      hw_doc["IP"] = WiFi.localIP().toString();
      hw_doc["Gateway"] = WiFi.gatewayIP().toString();
      serializeJson(hw_doc, hw_buf, JSON_BUFFER_SIZE);
    }
  }
}

void checkFlash()
{
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();
  Serial.printf("Reale Flash ID:   %08X\n", ESP.getFlashChipId());
  Serial.printf("Reale Flash groesse: %u", realSize);
  Serial.print(" Byte\n\n");
  Serial.printf("In der IDE hinterlegte Flash groesse: %u", ideSize);
  Serial.print(" Byte\n");
  Serial.printf("In der IDE hinterlegte Flash geschwindigkeit: %u\n", ESP.getFlashChipSpeed());
  Serial.printf("In der IDE hinterlegter  Flash Modus:  %s\n\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  if(ideSize != realSize)
  {
    Serial.println("Flash konfiguration ist falsch!\n");
  } else
  {
    Serial.println("Flash konfigguration ist korrekt.\n");
  }
}

void scan()
{
    Serial.println("Scanning I2C Addresses Channel 1");
    uint8_t cnt = 0;
    for (uint8_t i = 0; i<128; i++) {
        Wire.beginTransmission(i);
        uint8_t ec = Wire.endTransmission(true);
        if (ec == 0) {
            if (i<16)Serial.print('0');
            Serial.print(i, HEX);
            cnt++;
        }
        else Serial.print("..");
        Serial.print(' ');
        if ((i & 0x0f) == 0x0f)Serial.println();
    }
    Serial.print("Scan Completed, ");
    Serial.print(cnt);
    Serial.println(" I2C Devices found.");
}

int x2i(String s)
{
  int x = 0;
  for (uint i = 0; i < s.length(); i++)
  {
    char c = s.charAt(i);
    if (c >= '0' && c <= '9')
    {
      x *= 16;
      x += c - '0';
    }
    else if (c >= 'A' && c <= 'F')
    {
      x *= 16;
      x += (c - 'A') + 10;
    }
    else if (c >= 'a' && c <= 'f')
    {
      x *= 16;
      x += (c - 'a') + 10;
    }
  }
  return x;
}

uint8_t d2i(String s)
{
  uint8_t x = 0;
  for (uint i = 0; i < s.length(); i++)
  {
    char c = s.charAt(i);
    if (c >= '0' && c <= '9')
    {
      x *= 10;
      x += c - '0';
    }
  }
  return x;
}

uint32_t HSVtoRGB(int hue, int sat, int val)
{
  // hue: 0-359, sat: 0-255, val (lightness): 0-255
  int r = 0, g = 0, b = 0, base;
  if (sat == 0)
  { // Achromatic color (gray).
    r = val;
    g = val;
    b = val;
  }
  else
  {
    base = ((255 - sat) * val) >> 8;
    switch (hue / 60)
    {
    case 0:
      r = val;
      g = (((val - base) * hue) / 60) + base;
      b = base;
      break;
    case 1:
      r = (((val - base) * (60 - (hue % 60))) / 60) + base;
      g = val;
      b = base;
      break;
    case 2:
      r = base;
      g = val;
      b = (((val - base) * (hue % 60)) / 60) + base;
      break;
    case 3:
      r = base;
      g = (((val - base) * (60 - (hue % 60))) / 60) + base;
      b = val;
      break;
    case 4:
      r = (((val - base) * (hue % 60)) / 60) + base;
      g = base;
      b = val;
      break;
    case 5:
      r = val;
      g = base;
      b = (((val - base) * (60 - (hue % 60))) / 60) + base;
      break;
    }
  }
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void fAutoFarbe()
{
  ws2812fx.setColor(HSVtoRGB(Hue, 255, 255));
  Hue++;
  if (360 == Hue)
      Hue = 0;    
}

void fStopLight()
{
  ws2812fx.stop();
  Running = false;
}

void IRAM_ATTR change()
{
    if (!ws2812fx.isRunning())
    {
      ws2812fx.start();      
    } 
    tStopLight.once(15,fStopLight);
    Running = true;
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Startup");
  
  ws2812fx.init();

  ws2812fx.setMode(FX_MODE_BREATH);
  ws2812fx.setColor(HSVtoRGB(Hue, 255, 255));
  ws2812fx.start();

  checkFlash();

  Wire.begin();
  scan();

  unsigned status;
  // default settings
  status = bme.begin(0x76);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");      
  }

  tAutoFarbe.attach(tickerStart, fAutoFarbe);

  pinMode(SENSOR_PIN,INPUT);
  attachInterrupt(SENSOR_PIN, &change, CHANGE);

  reconnectWifi();

  reconnectMqtt();

  httpUpdater.setup(&httpServer);  
	httpServer.begin();

  Serial.println("Startup fertig");
}

void loop()
{
  reconnectWifi();

  reconnectMqtt();

  values.Temperatur = 0.8 * values.Temperatur + 0.2 * bme.readTemperature();
  values.Feuchtigkeit = 0.8 * values.Feuchtigkeit + 0.2 * bme.readHumidity();
  values.Luftdruck = 0.8 * values.Luftdruck + 0.2 * bme.readPressure()/100.0;
  values.TimerActive = Running;
  values.WlanSignal = 0.8 * values.WlanSignal + 0.2 *  WiFi.RSSI();

  //steigende Flanke
  if(digitalRead(SENSOR_PIN) && !values.Sensor)
  {
    //Serial.println(millis());   
    values.Sensor = true;
    UpdateCnt = 0;
  }
  else if (!digitalRead(SENSOR_PIN) && values.Sensor) //fallend
  {
    values.Sensor = false;
    UpdateCnt = 0;
  }

  if (0 == UpdateCnt)
  {
    UpdateCnt = 100;
    StaticJsonDocument<JSON_BUFFER_SIZE> doc;
    doc["Temperatur"] = values.Temperatur;
    doc["Feuchte"] = values.Feuchtigkeit;
    doc["Druck"] = values.Luftdruck;
    doc["Sensor"] = values.Sensor;
    doc["Timer"] = values.TimerActive;
    doc["WlanSignal"] = values.WlanSignal;
    doc["Zaehler"] = values.Zaehler++;

    char buf[JSON_BUFFER_SIZE];
    serializeJson(doc, buf, JSON_BUFFER_SIZE);
/*#if DEBUG
    Serial.print("JSON: ");
    Serial.println(buf);
#endif // DEBUG*/

    MqttClient.publish(MQTT_TOPIC_JSON, buf);
  }
  else
    UpdateCnt--;

  for (int i = 0; i < 100; i++)
  {
    ws2812fx.service();    
    httpServer.handleClient();
		MqttClient.loop();
    delay(1);
  }
}
