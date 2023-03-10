/******************************************************
Sketch to gather all energy data of my house

written by Thorsten Heins
*******************************************************/

#include <Arduino.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include <WiFi.h>
#include "HardwareSerial.h"
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define SML_BUFFER_SIZE 1000
#define MSG_BUFFER_SIZE 50
#define LED 2                // GPIO 2 intere LED
#define ONE_WIRE_BUS 32      // GPIO 32 Verbindung zu Temperatursensoren
#define RX 13                // GPIO 13 RX von Hardware-Serial
#define TX 15                // GPIO 15 TX von Hardware-Serial
#define Water_Digital_In1 17 // GPIO 18 Input fuer Wasserzaehler 1
#define Water_Digital_In2 5  // GPIO 19 Input fuer Wasserzaehler 2
#define Gas_Digital_In1 19   // GPIO 32 Input fuer Gaszaehler 1
#define Gas_Digital_In2 18   // GPIO 33 Input fuer Gaszaehler 2
#define ControlLED 16        // Kontroll-LED für MQTT Verbindung

//-- WIFI section --------------------------------
const char *wifi_ssid = "SSID-WIFI";
const char *wifi_pass = "SSID-PW";
const char *mqtt_server = "192.168.178.216"; // IP-Adresse IO-Broker

// Strommessung
const char Code180[] = "77070100010800FF";  // Codefolge fuer 1.8.0 Meldung
const char Code1670[] = "77070100100700FF"; // Codefolge fuer 16.7.0 Meldung
char myBuffer[SML_BUFFER_SIZE];
char *smldata;
char *data180;          // pointer to the 1.8.0 data read
char *data1670;         // pointer to the 16.7.0 data read
char sml180string[20];  // String fuer 1.8.0 Code
char sml1670string[20]; // String fuer 16.7.0 Code
unsigned long stromsummenzaehler;
unsigned long kwh_value;
unsigned long stromistleistung;

// Wassermessung
boolean Wasser_Input1 = false;
boolean Wasser_Input2 = false;
int Wasser_Status = 0;
int old_Wasser_Status = 0;
long wassersummenzaehler;

// Gasmessung
boolean Gas_Input1 = false;
boolean Gas_Input2 = false;
int Gas_Status = 0;
int old_Gas_Status = 0;
long gassummenzaehler;
u_long millis_old = 0;

// Temperaturerfassung
int numberOfDevices;
float WVtempC;
float HVtempC;
float HRtempC;
float WWtempC;
float old_WVtempC;
float old_HVtempC;
float old_HRtempC;
float old_WWtempC;
DeviceAddress tempDeviceAddress;
DeviceAddress DallasWW = {40, 170, 230, 121, 81, 20, 1, 169}; // Device Adresse des Temperatursensors Warmwasserbehaelter
DeviceAddress DallasHR = {40, 170, 254, 120, 75, 20, 1, 68};  // Device Adresse des Temperatursensors Heizung Ruecklauf
DeviceAddress DallasWV = {40, 170, 7, 47, 81, 20, 1, 111};    // Device Adresse des Temperatursensors Wasser Vorlauf
DeviceAddress DallasHV = {40, 170, 63, 94, 81, 20, 1, 242};   // Device Adresse des Temperatursensors Heizung Vorlauf

// Sonstiges
char msg[MSG_BUFFER_SIZE];

// Klassen definieren
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial SerialX(2);
Preferences preferences;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
AsyncWebServer server(80);

// MQTT Anfrage bearbeiten
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived: ");
  Serial.println(topic);
  String topicStr = topic;
  char value[MSG_BUFFER_SIZE];
  char *p = value; // Zeiger p an den Anfang des Strings VALUE stellen

  for (int i = 0; i < length; i++)
  {
    *p++ = (char)payload[i]; // empfangene Char's zu String VALUE zusammenfuegen
  }
  *p = '\0'; // Ende des Strings VALUE kennzeichnen

  if (topicStr == "Energie/Abfrage")
  {
    // Energiedaten werden abgefragt
    if (*value == '1')
    {
      Serial.println("Energiewerte werden abgefragt");
      snprintf(msg, MSG_BUFFER_SIZE, "%ld", stromsummenzaehler); // Long in String umwandeln
      client.publish("Energie/Stromverbrauch", msg);             // Datum ueber MQTT an Broker schicken
      // Serial.println(msg);
      snprintf(msg, MSG_BUFFER_SIZE, "%ld", wassersummenzaehler); // Long in String umwandeln
      client.publish("Energie/Wasser", msg);                      // Datum ueber MQTT an Broker schicken
      // Serial.println(msg);
    }
  }

  if (topicStr == "Energie/Temperaturabfrage")
  {
    // Temperaturdaten werden abgefragt
    if (*value == '1')
    {
      // Serial.println("Temperaturen werden abgefragt");

      snprintf(msg, MSG_BUFFER_SIZE, "%.0f", WWtempC); // Float mit 1. Nachkommastelle in String umwandeln
      // Serial.print("; Publish Warmwasser: ");
      // Serial.println(msg);
      client.publish("Heizung/Warmwasser", msg); // Datum ueber MQTT an Broker schicken

      snprintf(msg, MSG_BUFFER_SIZE, "%.0f", HVtempC); // Float mit 1. Nachkommastelle in String umwandeln
      // Serial.print("; Publish HeizungVorlauf: ");
      // Serial.println(msg);
      client.publish("Heizung/HeizungVorlauf", msg); // Datum ueber MQTT an Broker schicken

      snprintf(msg, MSG_BUFFER_SIZE, "%.0f", HRtempC); // Float mit 1. Nachkommastelle in String umwandeln
      // Serial.print("; Publish HeizungRuecklauf: ");
      // Serial.println(msg);
      client.publish("Heizung/HeizungRuecklauf", msg); // Datum ueber MQTT an Broker schicken

      snprintf(msg, MSG_BUFFER_SIZE, "%.0f", WVtempC); // Float mit 1. Nachkommastelle in String umwandeln
      // Serial.print("; Publish WasserVorlauf: ");
      // Serial.println(msg);
      client.publish("Heizung/WasserVorlauf", msg); // Datum ueber MQTT an Broker schicken
    }
  }

  if (topicStr == "Energie/SollwertGaszaehler")
  {
    // Sollwert Gaszaehler setzen
    gassummenzaehler = strtol(value, NULL, 0);           // String to long umwandeln, Basis autodetected
    preferences.putLong("Gaszaehler", gassummenzaehler); // in EEPROM des ESP32 speichern
  }

  if (topicStr == "Energie/SollwertWasserzaehler")
  {
    // Sollwert Wasserzaehler setzen
    wassersummenzaehler = strtol(value, NULL, 0);              // String to long umwandeln, Bais autodetected
    preferences.putLong("Wasserzaehler", wassersummenzaehler); // in EEPROM des ESP32 speichern
  }
}

// MQTT Reconnect
void reconnect()
{
  // Reconnected wenn moeglich
  if (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), "mqtuser", "tho66hei+"))
    {
      Serial.println("connected");
      client.subscribe("Energie/Abfrage");
      client.subscribe("Energie/Temperaturabfrage");
      client.subscribe("Energie/SollwertGaszaehler");
      client.subscribe("Energie/SollwertWasserzaehler");
      digitalWrite(ControlLED, true);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      digitalWrite(ControlLED, false);
    }
  }
}

// MQTT Verbindung checken
void MQTT_Check(void *parameter)
{
  while (1)
  {
    if (!client.connected())
    { // Falls MQTT Client Verbindung nicht vorhanden -> neu verbinden
      reconnect();
    }
    client.loop(); // MQTT Loop durchlaufen

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// Impulse vom Wasserzaehler auslesen
void read_Water_ticks(void *parameter)
{
  while (1)
  {
    // Input1: __|---|____
    // Input2: ____|---|__
    // Status:  0 1 2 3 0

    Wasser_Input1 = digitalRead(Water_Digital_In1); // ersten Input vom Wasserzaehler einlesen
    Wasser_Input2 = digitalRead(Water_Digital_In2); // zweiten Input vom Wasserzaehler einlesen

    // Status eines Durchgangs ermitteln
    if (!Wasser_Input1 && !Wasser_Input2)
    {
      Wasser_Status = 0;
    }
    if (Wasser_Status == 0 && Wasser_Input1 && !Wasser_Input2)
    {
      Wasser_Status = 1;
    }
    if (Wasser_Status == 1 && Wasser_Input1 && Wasser_Input2)
    {
      Wasser_Status = 2;
    }
    if (Wasser_Status == 2 && !Wasser_Input1 && Wasser_Input2)
    {
      Wasser_Status = 3;
    }

    // Ein ordentlicher Durchlauf ist erfolgt
    if (old_Wasser_Status != 3 && Wasser_Status == 3)
    {
      wassersummenzaehler = wassersummenzaehler + 1;
      preferences.putLong("Wasserzaehler", wassersummenzaehler); // Variable wassersummenzaehler unter "Datei" Wasserzaehler in EEPROM speichern
      // Serial.print("Wasser neu: ");
      // Serial.println(wassersummenzaehler);
      snprintf(msg, MSG_BUFFER_SIZE, "%ld", wassersummenzaehler);
      client.publish("Energie/Wasser", msg); // Wasserwert über MQTT zu IoBroker übertragen
    }

    old_Wasser_Status = Wasser_Status;

    // erial.print("FlipFlop 1: ");
    // Serial.print(Wasser_Input1);
    // Serial.print(" ; FlipFlop2: ");
    // Serial.print(Wasser_Input2);
    // Serial.print(" ; Status: ");
    // Serial.print(Wasser_Status);
    // Serial.print(" ; Wasserzaehler: ");
    // Serial.println(wassersummenzaehler);

    vTaskDelay(100 / portTICK_PERIOD_MS); // Funktion alle 100ms durchlaufen
  }
}

// Impulse vom Gaszaehler auslesen
void read_Gas_ticks(void *parameter)
{
  while (1)
  {
    // Input1: __|---|____
    // Input2: ____|---|__
    // Status:  0 1 2 3 0

    Gas_Input1 = digitalRead(Gas_Digital_In1); // ersten Input vom Gaszaehler einlesen
    Gas_Input2 = digitalRead(Gas_Digital_In2); // zweiten Input vom Gaszaehler einlesen

    // Status eines Durchgangs ermitteln
    if (!Gas_Input1 && !Gas_Input2)
    {
      Gas_Status = 0;
    }
    if (Gas_Status == 0 && Gas_Input1 && !Gas_Input2)
    {
      Gas_Status = 1;
    }
    if (Gas_Status == 1 && Gas_Input1 && Gas_Input2)
    {
      Gas_Status = 2;
    }
    if (Gas_Status == 2 && !Gas_Input1 && Gas_Input2)
    {
      Gas_Status = 3;
    }

    // Ein ordentlicher Durchlauf ist erfolgt
    if (old_Gas_Status != 3 && Gas_Status == 3)
    {
      gassummenzaehler = gassummenzaehler + 10;
      preferences.putLong("Gaszaehler", gassummenzaehler); // Variable wassersummenzaehler unter "Datei" Wasserzaehler in EEPROM speichern
      // Serial.print("Gas neu: ");
      // Serial.print(gassummenzaehler);
      snprintf(msg, MSG_BUFFER_SIZE, "%ld", gassummenzaehler);
      client.publish("Energie/Gas", msg); // Gaswert über MQTT zu IoBroker übertragen
    }

    old_Gas_Status = Gas_Status;

    // Serial.print("FlipFlop 1: ");
    // Serial.print(Gas_Input1);
    // Serial.print(" ; FlipFlop2: ");
    // Serial.print(Gas_Input2);
    // Serial.print(" ; Status: ");
    // Serial.print(Gas_Status);
    // Serial.print(" ; Gaszaehler: ");
    // Serial.println(gassummenzaehler);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Rx-Daten auslesen fuer Stromzahler
void readIntoBuffer(void *parameter)
{
  while (1)
  {
    char *ptr = &myBuffer[0];   // Pointer an den Anfang von Array 'myBuffer' stellen
    while (SerialX.available()) // Sind Rx-Daten vorhandenen?
    {
      ptr += sprintf(ptr, "%02X", SerialX.read()); // Rx-Daten lesen, in HEX umwandeln und in Buffer schreiben
    }

    data180 = strstr(myBuffer, Code180); // Ist die Codierung fuer 1.8.0 im SML-Telegramm enthalten ?
    if (data180 != NULL)                 // Wenn ja, Telegrammteil uebergeben
    {
      char *p = sml180string;        // Zeiger p an den Anfang des Strings stelle
      for (int i = 40; i <= 53; i++) // Datenwert an Position 40 bis 53 aus dem Telegrammteil entnehmen
      {
        *p++ = *(data180 + i);
      }
      *p = '\0';                                   // Ende des Strings kennzeichnen
      kwh_value = strtoll(sml180string, NULL, 16); // String mit Datenwert in LongLong umrechnen
      // Serial.print("Zählerstand: ");
      stromsummenzaehler = long(kwh_value / 10);
      // Serial.print(stromsummenzaehler);
      // Serial.println(" kWh");
    }
    else
    {
      // Serial.println("180 nicht gefunden");
    }

    data1670 = strstr(myBuffer, Code1670); // Ist die Codierung fuer 1.8.0 im SML-Telegramm enthalten ?
    if (data1670 != NULL)                  // Wenn ja, Telegrammteil uebergeben
    {
      char *p = sml1670string;       // Zeiger p an den Anfang des Strings stelle
      for (int i = 30; i <= 37; i++) // Datenwert an Positon 30 bis 37 aus dem Telegrammteil entnehmen
      {
        *p++ = *(data1670 + i);
      }
      *p = '\0';                                           // Ende des Strings kennzeichnen
      stromistleistung = strtoll(sml1670string, NULL, 16); // String mit Datenwert in LongLong umrechnen
      // Serial.print("akt. Stromverbrauch: ");
      // Serial.print(stromistleistung);
      // Serial.println(" W");
      snprintf(msg, MSG_BUFFER_SIZE, "%ld", stromistleistung); // LongLong in String umwandeln
      client.publish("Energie/Stromgesamtleistung", msg);      // Wert per MQTT an Broker uebertragen
    }
    else
    {
      // Serial.println("1670 nicht gefunden");
    }

    vTaskDelay(10000 / portTICK_PERIOD_MS); // Funktion alle 10s wiederholen
  }
}

// Temperaturen auslesen von DALLAS Sensoren
void DALLASTemperatures(void *parameter)
{
  while (1)
  {
    sensors.requestTemperatures(); // Temperaturanfrage anfordern

    old_WWtempC = WWtempC; // vorige Werte zwischenspeichern
    old_HVtempC = HVtempC;
    old_HRtempC = HRtempC;
    old_WVtempC = WVtempC;

    WWtempC = sensors.getTempC(DallasWW); // neuen Werte auslesen
    HVtempC = sensors.getTempC(DallasHV);
    HRtempC = sensors.getTempC(DallasHR);
    WVtempC = sensors.getTempC(DallasWV);

    if ((WWtempC < -10) | (WWtempC > 120)) // Fehlmessung, vorigen Wert beibehalten
      WWtempC = old_WWtempC;
    if ((HVtempC < -10) | (HVtempC > 120))
      HVtempC = old_HVtempC;
    if ((HRtempC < -10) | (HRtempC > 120))
      HRtempC = old_HRtempC;
    if ((WVtempC < -10) | (WVtempC > 120))
      WVtempC = old_WVtempC;

    vTaskDelay(30000 / portTICK_PERIOD_MS); // Funktion alle 30s wiederholen
  }
}

// ESP32-Board LED blinken lassen und MQTT Verbindung checken
void Blinken(void *parameter)
{
  while (1)
  {
    // Blinken
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
  }
}

void setup()
{
  /**** serial part ****/
  Serial.begin(115200);
  SerialX.begin(9600, SERIAL_8N1, RX, TX);

  pinMode(LED, OUTPUT);
  pinMode(Water_Digital_In1, INPUT_PULLUP);
  pinMode(Water_Digital_In2, INPUT_PULLUP);
  pinMode(Gas_Digital_In1, INPUT_PULLUP);
  pinMode(Gas_Digital_In2, INPUT_PULLUP);
  pinMode(ControlLED, OUTPUT);

  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  Serial.print("Sensor Number:");
  Serial.println(numberOfDevices);

  /**** WIFI ****/
  Serial.println("Connecting to wifi ... ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1886); // MQTT Verbindung herstellen
  client.setCallback(callback);

  if (!client.connected())
  { // MQTT Client Verbindung herstellen
    reconnect();
    Serial.println("Client wurde beim Start verbunden");
  }
  else
  {
    Serial.println("Client war beim Start verbunden");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Energiezaehler ist da!"); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();

  preferences.begin("tho", false);                               // EEPROM Datei "tho" oeffnen
  gassummenzaehler = preferences.getLong("Gaszaehler", 0);       // EEPROM Wert aus Feld "Gaszaehler" entnehmen, oder bei leer 0
  wassersummenzaehler = preferences.getLong("Wasserzaehler", 0); // EEPROM Wert aus Feld "Wasserzaehler" entnehmen, oder bei leer 0

  // FreeRTOS Funktionen definieren
  xTaskCreate(readIntoBuffer, "Daten_lesen", 4096, NULL, 1, NULL);
  xTaskCreate(Blinken, "Blinken", 4096, NULL, 1, NULL);
  xTaskCreate(MQTT_Check, "MQTT_senden", 4096, NULL, 1, NULL);
  xTaskCreate(read_Water_ticks, "WasserzaehlerEinlesen", 4096, NULL, 2, NULL);
  xTaskCreate(read_Gas_ticks, "GaszaehlerEinlesen", 4096, NULL, 2, NULL);
  xTaskCreate(DALLASTemperatures, "Temperaturen Einlesen", 4096, NULL, 1, NULL);
}

void loop()
{
}