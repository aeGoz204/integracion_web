/****************************************
 * Libraries
 ****************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>

#include "data.h"
#include "Settings.h"

#include <UbidotsEsp32Mqtt.h>

#include "DHT.h"

#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();

#define DATAPIN 27
#define DHTTYPE DHT11

DHT dht(DATAPIN, DHTTYPE);

/****************************************
 * Define Constants Button
 ****************************************/
#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s

/****************************************
 * Define Constants Ubidots
 ****************************************/
const char *UBIDOTS_TOKEN = "BBUS-Dcrq35CuJZHXkBgjCxZ84sUCGg8fjK"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "esp32";                                // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL2 = "Humedad";                           // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL1 = "Temp";
const char *SUBSCRIBE_SW1 = "SW1";
const char *SUBSCRIBE_SW2 = "SW2";

const uint8_t LED = 26;
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
unsigned long timer;

Ubidots ubidots(UBIDOTS_TOKEN);

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();

/****************************************
 * Auxiliar Functions
 ****************************************/
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String str = String(topic);
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if (str.substring(20) == "sw1/lv")
  {
    if ((char)payload[0] == '1')
    {
      tft.fillCircle(40, 125, 20, TFT_GREEN);
      digitalWrite(LED, HIGH);
    }
    else
    {
      tft.fillCircle(40, 125, 20, TFT_RED);
      digitalWrite(LED, LOW);
    }
  }
  if (str.substring(20) == "sw2/lv")
  {
    if ((char)payload[0] == '1')
    {
      tft.fillCircle(95, 125, 20, TFT_NAVY);
    }
    else
    {
      tft.fillCircle(95, 125, 20, TFT_RED);
    }
  }
}

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("fabio_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, SUBSCRIBE_SW1);
  ubidots.subscribeLastValue(DEVICE_LABEL, SUBSCRIBE_SW2);

  dht.begin();

  timer = millis();
}

/****************************************
 * Main Functions
 ****************************************/
void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo
  pinMode(LED, OUTPUT);
  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  tft.init();

  if (is_STA_mode())
  {
    int x = 22, y = 85;
    tft.fillScreen(TFT_BLACK);
    tft.drawRect(0, 0, 135, 240, TFT_WHITE);
    tft.fillRect(40, 125, 21, 21, TFT_WHITE);
    tft.fillRect(95, 125, 21, 21, TFT_WHITE);
    tft.drawString(F("Humedad:"), 5, 20, 4);
    tft.drawString(F("Temp.:"), 5, 155, 4);
    tft.drawString(F("%"), 100, 55, 4);
    tft.drawString(F("`C"), 100, 190, 4);

    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    tft.setTextColor(TFT_WHITE);
    tft.drawString(String(h, 1), 5, 55, 6);
    tft.drawString(String(t, 1), 5, 190, 6);

    if (!ubidots.connected())
    {
      ubidots.reconnect();
      ubidots.subscribeLastValue(DEVICE_LABEL, SUBSCRIBE_SW1);
      ubidots.subscribeLastValue(DEVICE_LABEL, SUBSCRIBE_SW2);
    }

    if ((millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      ubidots.add(VARIABLE_LABEL1, t); // Insert your variable Labels and the value to be sent
      ubidots.add(VARIABLE_LABEL2, h);
      ubidots.publish(DEVICE_LABEL);
      timer = millis();
    }
    ubidots.loop();

    delay(1000);
    tft.setTextColor(TFT_BLACK);
    tft.drawString(String(h, 1), 5, 55, 6);
    tft.drawString(String(t, 1), 5, 190, 6);
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

/****************************************
 * Auxiliar Functions
 ****************************************/

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}