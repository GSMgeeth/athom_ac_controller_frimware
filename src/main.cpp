#include <cstdlib>
#include <stdbool.h>
#include <string.h>
#include <sstream>
#include <cstring>
#include <cctype>
#include <time.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

// Libraries for MQTT client, WiFi connection and SAS-token generation.
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <base64.h>
#include <libb64/cdecode.h>

// Azure IoT SDK for C includes
#include <az_core.h>
#include <az_iot.h>
#include <azure_ca.h>

// Additional sample headers
#include "iot_configs.h"

#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266httpUpdate.h>
#include <ESP8266HTTPClient.h>
#include <LittleFS.h>
#include "ir.h"

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <assert.h>
#include <IRac.h>
#include <IRtext.h>

const uint16_t ir_led = 4;
IRsend irsend(ir_led);
const int kRecvPin = 5;
const int buttonPin = 0; // Push button connected to digital pin 2
const uint16_t kCaptureBufferSize = 400;
#if DECODE_AC
// Some A/C units have gaps in their protocols of ~40ms. e.g. Kelvinator
// A value this large may swallow repeats of some protocols
const uint8_t kTimeout = 50;
#else  // DECODE_AC
// Suits most messages, while not swallowing many repeats.
const uint8_t kTimeout = 15;
#endif // DECODE_AC
const uint16_t kMinUnknownSize = 12;
const uint8_t kTolerancePercentage = kTolerance; // kTolerance is normally 25%
#define LEGACY_TIMING_INFO false

String hostnameBase = "ATH-IR-CUS";
String hostname = "";
String hipen = "-";
String ssid = "";
String wifiSsid = "";
String wifiPw = "";
String host = "";
String device_id = "";
String device_key = "";
bool apStarted = false;
bool isWifiConnected = false;
String updateStarted = "false";
String firmwareURL = "";
ESP8266WebServer server(80);
// ESP8266WebServer serverOTA(80);
unsigned long lastSerialReadTime = 0;
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);

decode_results results; // Somewhere to store the results
bool is_protocol_set = false;
// Variables to hold button state
int buttonState = 0;
int lastButtonState = 0;

// variables to check whether credentials have been recieved
bool flag = false;           // The flag you want to check
unsigned long startTime = 0; // Stores the time when flag is set to true

// Variables for timing the long press
unsigned long lastDebounceTime = 0;
unsigned long longPressInterval = 2000; // Adjust this value for long press duration

// volatile bool buttonPressed = false; // Flag for button press
// unsigned long pressStartTime = 0;
// unsigned long longPressInterval = 2000; // 2 seconds for a long press

int prot_address = 0;
int temp_address = 20;
int fanspeed_address = 22;
int PROTOCOL = -1;
int PROTOCOL_RECV = -1;
int POWER = 2;
int FAN_SPEED = 3;
int MODE = 1;
int TEMPERATURE = 22;

// When developing for your own Arduino-based platform,
// please follow the format '(ard;<platform>)'.
#define AZURE_SDK_CLIENT_USER_AGENT "c%2F" AZ_SDK_VERSION_STRING "(ard;esp8266)"

// Utility macros and defines
#define LED_PIN 13
#define sizeofarray(a) (sizeof(a) / sizeof(a[0]))
#define ONE_HOUR_IN_SECS 3600
#define NTP_SERVERS "pool.ntp.org", "time.nist.gov"
#define MQTT_PACKET_SIZE 1024

#define INCOMING_DATA_BUFFER_SIZE 256
static char incoming_data[INCOMING_DATA_BUFFER_SIZE];
static const int port = 8883;

// Memory allocated for the sample's variables and structures.
static WiFiClientSecure wifi_client;
static X509List cert((const char *)ca_pem);
static PubSubClient mqtt_client(wifi_client);
static az_iot_hub_client client;
static char sas_token[200];
static uint8_t signature[512];
static unsigned char encrypted_signature[32];
static char base64_decoded_device_key[32];

// function definitions
static void initializeClients();
void readStringFromEEPROM(int startAddress, char *buffer);
void writeStringToEEPROM(int startAddress, const char *string);
void saveDataToEEPROM();
void saveProtocol();
void loadDataFromEEPROM();
void handleRoot();
void handleSubmit();
void handleCm();
void handleStatus();
void handleTestIR();
void handleIP();
void handleConfigs();
void loadCredentials();
void loadMqqtParams();
bool connectToWiFi();
void startAPServer();
void startServer();
void startServerOTA();
void saveCredentials(String ssid = "", String password = "");
void saveMqttParams(String MqttHost = "", String MqttClient = "", String MqttPassword = "");
void saveURL(String URL, String updateStarted);
String NetworkUniqueId(void);
uint32_t ESP_getChipId(void);
void handleGetFirmwareURL();
// void updateFirmware(String url);
void setOTA();
void recieveProtocol();
void recieveProtocolWithTimer();
void handleLongPress();
// void IRAM_ATTR handleButtonPress();
void clearEEPROM();
// Auxiliary functions

void clearEEPROM()
{
  EEPROM.begin(512); // Adjust size if needed

  // Clear EEPROM
  for (int i = 0; i < 512; i++)
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();

  Serial.println("EEPROM cleared. Restarting...");
}

// void IRAM_ATTR handleButtonPress()
// {
//   if (digitalRead(buttonPin) == LOW)
//   {
//     buttonPressed = true;
//     pressStartTime = millis();
//   }
// }

void handleLongPress()
{
  Serial.println("Long press detected. Erasing EEPROM and resetting ESP8266...");
  // Erase EEPROM - Replace with your EEPROM library's function
  // Reset ESP8266
  saveCredentials();
  saveMqttParams();
  clearEEPROM();
  ESP.restart();
}
void recieveProtocolWithTimer()
{
  // variables to create timer for recieving protocol
  const unsigned long interval = 60000; // 1 minute in milliseconds
  unsigned long previousMillis = millis();
  Serial.println("Waiting for IR code...");
  while (millis() - previousMillis <= interval)
  {
    if (irrecv.decode(&results))
    {
      // Display the tolerance percentage if it has been change from the default.
      if (kTolerancePercentage != kTolerance)
        Serial.printf(D_STR_TOLERANCE " : %d%%\n", kTolerancePercentage);
      // Display the basic output of what we found.
      Serial.print(resultToHumanReadableBasic(&results));
      Serial.println();
      Serial.print("Decoded PROTOCOL in int: ");
      PROTOCOL = static_cast<int>(results.decode_type);
      Serial.println(PROTOCOL);

      yield(); // Feed the WDT as the text output can take a while to print.
#if LEGACY_TIMING_INFO
      // Output legacy RAW timing info of the result.
      Serial.println(resultToTimingInfo(&results));
      yield(); // Feed the WDT (again)
#endif         // LEGACY_TIMING_INFO
      // Output the results as source code
      yield(); // Feed the WDT (again)
      irrecv.resume();
      return;
    }
    yield(); // Feed the WDT (again)
  }
  irrecv.resume();
  Serial.println("Timeout: No IR command received within 1 minute.");
}

void recieveProtocol()
{
  // Serial.println("Waiting for IR code...");
  if (irrecv.decode(&results))
  {
    // Display the tolerance percentage if it has been change from the default.
    if (kTolerancePercentage != kTolerance)
      Serial.printf(D_STR_TOLERANCE " : %d%%\n", kTolerancePercentage);
    // Display the basic output of what we found.
    Serial.print(resultToHumanReadableBasic(&results));
    Serial.println();
    Serial.print("Decoded PROTOCOL in int: ");
    Serial.println(PROTOCOL_RECV);
    int recieved_prot = static_cast<int>(results.decode_type);
    if (recieved_prot != -1)
    {
      PROTOCOL_RECV = recieved_prot;
    }

    yield(); // Feed the WDT as the text output can take a while to print.
#if LEGACY_TIMING_INFO
    // Output legacy RAW timing info of the result.
    Serial.println(resultToTimingInfo(&results));
    yield(); // Feed the WDT (again)
#endif       // LEGACY_TIMING_INFO
    // Output the results as source code
    yield(); // Feed the WDT (again)
    irrecv.resume();
  }
}

// void updateFirmware(String url)
// {
//   HTTPClient http;

//   Serial.print("[HTTP] begin...\n");
//   if (http.begin(url))
//   {
//     Serial.print("[HTTP] GET...\n");
//     int httpCode = http.GET();
//     if (httpCode == HTTP_CODE_OK)
//     {
//       Serial.println("[HTTP] Firmware update available.");
//       WiFiUDP::stopAll(); // Ensure no other services are using port 8266

//       Serial.print("Updating firmware from: ");
//       Serial.println(url);

//       // t_httpUpdate_return ret = ESPhttpUpdate.update(url);
//       // switch (ret)
//       // {
//       // case HTTP_UPDATE_FAILED:
//       //   Serial.printf("[OTA] Update failed (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
//       //   break;
//       // case HTTP_UPDATE_NO_UPDATES:
//       //   Serial.println("[OTA] No updates available.");
//       //   break;
//       // case HTTP_UPDATE_OK:
//       //   Serial.println("[OTA] Update successful.");
//       //   delay(2000);
//       //   ESP.restart();
//       //   break;
//       // }
//       http.end();
//     }
//     else
//     {
//       Serial.printf("[HTTP] Unable to connect\n");
//     }
//   }
// }

// void handleGetFirmwareURL()
// {
//   // String firmwareUrl = "http://your-firmware-server.com/firmware.bin";
//   String URL = serverOTA.arg("OtaUrl");
//   int semicolonPos = URL.indexOf(";");

//   // Extract the URL substring
//   String firmwareURL = URL.substring(8, semicolonPos);
//   // const char*  = firmwareURL.c_str();
//   //   char charMinimalURL[firmwareURL.length() + 1]; // +1 for null terminator
//   //   firmwareURL.toCharArray(charMinimalURL, firmwareURL.length() + 1);

//   //   Serial.print("Recieved firmware URL: ");
//   //   Serial.println(firmwareURL);
//   updateStarted = "true";
//   //  // char charMinimalURL[] = "http://ota.tasmota.com/tasmota/tasmota.bin.gz";
//   //   const char searchString[] = ".bin.gz";
//   //   const char replaceString[] = "-minimal.bin.gz";

//   //   // Find the position of the substring to replace
//   //   char* pos = strstr(charMinimalURL, searchString);
//   //   if (pos != NULL) {
//   //       // Calculate the length of the replacement string
//   //       size_t replaceLen = strlen(replaceString);
//   //       size_t searchLen = strlen(searchString);
//   //       size_t originalLen = strlen(charMinimalURL);

//   //       // Calculate the length difference
//   //       int lengthDiff = replaceLen - searchLen;

//   //       // Shift the remaining characters to accommodate the new string
//   //       memmove(pos + replaceLen, pos + searchLen, originalLen - (pos - charMinimalURL) - searchLen + 1);

//   //       // Copy the replacement string into the position
//   //       memcpy(pos, replaceString, replaceLen);

//   //         // memmove(pos + replaceLen, pos + searchLen, originalLen - (pos - charMinimalURL) - searchLen + 1);

//   //         // // Copy the replacement string into the position
//   //         // memcpy(pos, replaceString, replaceLen);

//   // Your original string
//   // String firmwareURL = "example_file.bin.gz";
//   // Substring to be replaced
//   String searchString = ".bin.gz";
//   // Replacement substring
//   String replaceString = "-minimal.bin.gz";

//   // Find the position of the substring to be replaced
//   int pos = firmwareURL.indexOf(searchString);

//   // If the substring is found
//   if (pos != -1)
//   {
//     // Maximum length of the modified string
//     const int bufferSize = firmwareURL.length() + (replaceString.length() - searchString.length()) + 1;
//     // Buffer to hold the modified string
//     char charMinimalURL[bufferSize + 10];

//     // Copy characters before the substring
//     firmwareURL.substring(0, pos).toCharArray(charMinimalURL, bufferSize);
//     // Concatenate the replacement substring
//     strcat(charMinimalURL, replaceString.c_str());
//     // Concatenate characters after the substring
//     strcat(charMinimalURL, firmwareURL.substring(pos + searchString.length()).c_str());

//     // String minimalURL = String(charMinimalURL);
//     Serial.println(charMinimalURL);
//     String minimalURL(charMinimalURL);
//     Serial.println("Minimal firmware url = " + minimalURL);
//     saveURL(firmwareURL, updateStarted);
//     serverOTA.send(200, "text/plain", "OTA firmware recieved");
//     updateFirmware(minimalURL);
//   }
//   Serial.println("invalid URL");
// }

String NetworkUniqueId(void)
{
  String unique_id = WiFi.macAddress();
  unique_id.replace(":", ""); // Full 12 chars MAC address as ID
  String firstSix = unique_id.substring(0, 6);
  return firstSix;
}

uint32_t ESP_getChipId(void)
{
  return ESP.getChipId();
}

void generateWifiHost()
{
  String networkId = NetworkUniqueId();
  char chipId[30];
  // std::string chipId = std::to_string(ESP_getChipId() & 0x1FFF);
  sprintf(chipId, "%u", ESP_getChipId() & 0x1FFF);
  hostname = hostnameBase + hipen + networkId + hipen + chipId;
  ssid = hostname;
  device_id = hostname;
}

const char chunk1[] PROGMEM = R"(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP8266 Configuration</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 0;
      display: flex;
      justify-content: center;
      align-items: flex-start;
      min-height: 100vh;
      background-color: #f4f4f9;
    }
    .container {
      width: 90%;
      max-width: 400px;
      margin: 20px auto;
      padding: 20px;
      border-radius: 8px;
      background-color: #ffffff;
      box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
    }
    h2 {
      text-align: center;
      color: #333;
    }
    label {
      font-size: 0.9rem;
      color: #555;
    }
    input[type="text"], input[type="password"], select {
      width: 100%;
      padding: 8px;
      margin: 8px 0;
      border: 1px solid #ddd;
      border-radius: 4px;
      box-sizing: border-box;
    }
    button {
      width: 100%;
      padding: 10px;
      border: none;
      border-radius: 4px;
      color: white;
      background-color: #4CAF50;
      cursor: pointer;
    }
    button:hover {
      background-color: #45a049;
    }
    .section {
      margin-top: 20px;
    }
    .status {
      text-align: center;
      margin-top: 20px;
      font-size: 0.9rem;
      color: #666;
    }
  </style>
</head>)";

const char chunk2[] PROGMEM = R"rawliteral(<body>
<div class="container">
  <h2>ESP8266 Configuration</h2>
  <form id="config-form" onsubmit="return saveConfig()">
    <label for="mqtt-host">MQTT Host</label>
    <input type="text" id="mqtt-host" name="mqttHost" required>

    <label for="mqtt-user">MQTT User</label>
    <input type="text" id="mqtt-user" name="mqttUser" required>

    <label for="mqtt-password">MQTT Password</label>
    <input type="password" id="mqtt-password" name="mqttPassword" required>

    <label for="mqtt-topic">MQTT Topic</label>
    <input type="text" id="mqtt-topic" name="mqttTopic" required>

    <label for="mqtt-fulltopic">MQTT Full Topic</label>
    <input type="text" id="mqtt-fulltopic" name="mqttFullTopic" required>

    <label for="ssid">SSID</label>
    <input type="text" id="ssid" name="ssid" required>

    <label for="wifi-password">WiFi Password</label>
    <input type="password" id="wifi-password" name="wifiPassword" required>

    <label for="protocol">Protocol</label>
    <select id="protocol" name="protocol" required>
      
    </select>

    <button type="submit">Save Configuration</button>
  </form>


  <div class="section">
    <h3>IR Control</h3>
    <div class="status" id="ir-detected">Detected Protocol: <span id="detected-protocol">None</span></div>
 <label for="test-protocol">Select Protocol:</label>
 <select id="test-protocol" name="protocol" required>
 </select>
    <button onclick="sendTestCommand()">Send Test IR Command</button>
  </div>

  <div class="status" id="status-message"></div>
</div>)rawliteral";

const char chunk3[] PROGMEM = R"rawliteral(
<script>

  const protocols = "UNKNOWN,UNUSED,RC5,RC6,NEC,SONY,PANASONIC,JVC,SAMSUNG,WHYNTER,AIWA_RC_T501,LG,SANYO,MITSUBISHI,DISH,SHARP,COOLIX,DAIKIN,DENON,KELVINATOR,SHERWOOD,MITSUBISHI_AC,RCMM,SANYO_LC7461,RC5X,GREE,PRONTO,NEC_LIKE,ARGO,TROTEC,NIKAI,RAW,GLOBALCACHE,TOSHIBA_AC,FUJITSU_AC,MIDEA,MAGIQUEST,LASERTAG,CARRIER_AC,HAIER_AC,MITSUBISHI2,HITACHI_AC,HITACHI_AC1,HITACHI_AC2,GICABLE,HAIER_AC_YRW02,WHIRLPOOL_AC,SAMSUNG_AC,LUTRON,ELECTRA_AC,PANASONIC_AC,PIONEER,LG2,MWM,DAIKIN2,VESTEL_AC,TECO,SAMSUNG36,TCL112AC,LEGOPF,MITSUBISHI_HEAVY_88,MITSUBISHI_HEAVY_152,DAIKIN216,SHARP_AC,GOODWEATHER,INAX,DAIKIN160,NEOCLIMA,DAIKIN176,DAIKIN128,AMCOR,DAIKIN152,MITSUBISHI136,MITSUBISHI112,HITACHI_AC424,SONY_38K,EPSON,SYMPHONY,HITACHI_AC3,DAIKIN64,AIRWELL,DELONGHI_AC,DOSHISHA,MULTIBRACKETS,CARRIER_AC40,CARRIER_AC64,HITACHI_AC344,CORONA_AC,MIDEA24,ZEPEAL,SANYO_AC,VOLTAS,METZ,TRANSCOLD,TECHNIBEL_AC,MIRAGE,ELITESCREENS,PANASONIC_AC32,MILESTAG2,ECOCLIM,XMP,TRUMA,HAIER_AC176,TEKNOPOINT,KELON,TROTEC_3550,SANYO_AC88,BOSE,ARRIS,RHOSS,AIRTON,COOLIX48,HITACHI_AC264,KELON168,HITACHI_AC296,DAIKIN200,HAIER_AC160,CARRIER_AC128,TOTO,CLIMABUTLER,TCL96AC,BOSCH144,SANYO_AC152,DAIKIN312,GORENJE,WOWWEE,CARRIER_AC84,YORK";

  const protocolArray = protocols.split(",");

 
  const selectElement = document.getElementById("protocol");
  const selectElement1 = document.getElementById("test-protocol");


  protocolArray.forEach((protocol, index) => {
    const option = document.createElement("option");
    option.value = index - 1; 
    option.textContent = protocol;
    const option1 = document.createElement("option");
    option1.value = index - 1; 
    option1.textContent = protocol;
    selectElement.appendChild(option);
    selectElement1.appendChild(option1)
  });
  function saveConfig() {
    const mqttHost = document.getElementById('mqtt-host').value;
    const mqttUser = document.getElementById('mqtt-user').value;
    const mqttPassword = document.getElementById('mqtt-password').value;
    const mqttTopic = document.getElementById('mqtt-topic').value;
    const mqttFullTopic = document.getElementById('mqtt-fulltopic').value;
    const ssid = document.getElementById('ssid').value;
    const wifiPassword = document.getElementById('wifi-password').value;
    const protocol = document.getElementById('protocol').value;
    const url = `http://192.168.4.1/cm?user=admin&cmnd=${encodeURIComponent(`Backlog MqttHost ${mqttHost}; MqttUser ${mqttUser}; MqttClient ${mqttUser}; MqttPassword ${mqttPassword}; Topic ${mqttTopic}; FullTopic ${mqttFullTopic}; SSID1 ${ssid}; Password1 ${wifiPassword}; Protocol ${protocol};`)}`
    fetch(url)
      .then(response => {
        if (response.ok) {
          document.getElementById('status-message').innerText = 'Configuration Saved!';
        } else {
          document.getElementById('status-message').innerText = 'Failed to Save Configuration.';
        }
      })
      .catch(error => {
        document.getElementById('status-message').innerText = 'Error saving configuration.';
        console.error('Error:', error);
      });
    return false;
  }
)rawliteral";
const char chunk4[] PROGMEM = R"rawliteral(
  function sendTestCommand() {
    const protocol = document.getElementById('test-protocol').value;
    const url = `http://192.168.4.1/testIR?command=protocol%20${protocol}%3B%20power%201%3B%20temp%2021%3B%20fan_speed%202%3B`;
    fetch(url)
      .then(response => {
        if (response.ok) {
          document.getElementById('status-message').innerText = 'Test IR Command Sent!';
        } else {
          document.getElementById('status-message').innerText = 'Failed to Send Test IR Command.';
        }
      })
      .catch(error => {
        document.getElementById('status-message').innerText = 'Error sending test IR command.';
        console.error('Error:', error);
      });
  }

  function fetchStatus() {
    fetch('http://192.168.4.1/status')
      .then(response => response.json())
      .then(data => {

        document.getElementById('detected-protocol').innerText = protocolArray[parseInt(data.detectedProtocol)+1] || 'None';
      })
      .catch(error => {
        console.error('Error fetching status:', error);
      });
  }
    function fetchConfig() {
    fetch('http://192.168.4.1/configs')
      .then(response => response.json())
      .then(data => {

        document.getElementById('mqtt-user').value = data.MqttClient;
        document.getElementById('mqtt-topic').value = data.MqttClient;
        document.getElementById('mqtt-fulltopic').value = data.MqttClient;
        document.getElementById('mqtt-password').value = data.MqttPassword;
        document.getElementById('mqtt-host').value = data.MqttHost;
        document.getElementById('protocol').value = parseInt(data.Protocol);
       
      })
      .catch(error => {
        console.error('Error fetching status:', error);
      });
  }
  fetchConfig()

  setInterval(fetchStatus, 5000); 
</script>
</body>
</html>
)rawliteral";
void handleRoot()
{
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); // Set to unknown to enable chunked transfer
  server.send(200, "text/html", "");               // Start response
                                                   // Sending each part of the HTML page
  server.sendContent_P(chunk1);
  server.sendContent_P(chunk2);
  server.sendContent_P(chunk3);
  server.sendContent_P(chunk4);
  server.sendContent(""); // End of chunked response
}
void startAPServer()
{
  apStarted = true;
  // Set up AP (Access Point)
  WiFi.softAP(ssid);
  Serial.println("AP Started. Connect to network: " + String(ssid));
  if (!LittleFS.begin())
  {
    Serial.println("An error has occurred while mounting LittleFS");
    return;
  }
  // Handle root URL ("/")
  server.on("/", HTTP_GET, handleRoot);

  // Handle form submission
  server.on("/submit", HTTP_POST, handleSubmit);
  // Handle configuration data (wifi credentials, mqtt credentials and protocol)
  server.on("/cm", HTTP_GET, handleCm);
  // Endpoint to send device state
  server.on("/status", HTTP_GET, handleStatus);
  // Endpoint to handle testing ir commands
  server.on("/testIR", HTTP_GET, handleTestIR);
  // Endpoit to send ip address to be connected on router endpoint
  server.on("/ip", HTTP_GET, handleIP);
  // Endpoint to send configured device settings
  server.on("/configs", HTTP_GET, handleConfigs);

  server.begin();
  // delay(2000);
  // WiFi.softAPdisconnect(true);
  // Serial.println("Access Point ended.");
}

void startServer()
{

  if (!LittleFS.begin())
  {
    Serial.println("An error has occurred while mounting LittleFS");
    return;
  }
  // Handle root URL ("/")
  server.on("/", HTTP_GET, handleRoot);

  // Handle form submission
  server.on("/submit", HTTP_POST, handleSubmit);
  // Handle configuration data (wifi credentials, mqtt credentials and protocol)
  server.on("/cm", HTTP_GET, handleCm);
  // Endpoint to send device state
  server.on("/status", HTTP_GET, handleStatus);
  // Endpoint to handle testing ir commands
  server.on("/testIR", HTTP_GET, handleTestIR);
  // Endpoit to send ip address to be connected on router endpoint
  server.on("/ip", HTTP_GET, handleIP);
  // Endpoint to send configured device settings
  server.on("/configs", HTTP_GET, handleConfigs);

  server.begin();
  // delay(2000);
  // WiFi.softAPdisconnect(true);
  // Serial.println("Access Point ended.");
}
// void startServerOTA()
// {
//   serverOTA.on("/firmware_url", HTTP_GET, handleGetFirmwareURL);
//   serverOTA.begin();
// }

void handleConfigs()
{
  Serial.println("Received request for /configs");
  StaticJsonDocument<300> jsonDoc;
  jsonDoc["MqttHost"] = host;
  jsonDoc["MqttClient"] = hostname;
  jsonDoc["MqttPassword"] = device_key;
  jsonDoc["SSID1"] = wifiSsid;
  jsonDoc["Password1"] = wifiPw;
  jsonDoc["Protocol"] = PROTOCOL;

  String jsonResponse;
  serializeJson(jsonDoc, jsonResponse);

  server.send(200, "application/json", jsonResponse);
  Serial.println("Response sent for /configs");
}

void handleCm()
{
  String cmString = server.arg("cmnd");
  // Find the position of the first semicolon to ignore the "Backlog " part
  int startPos = cmString.indexOf(" ") + 1; // Skip space

  // Split the substring starting from startPos by commas
  String parts[9]; // Assuming there are 8 parts separated by commas
  int count = 0;
  for (int i = startPos; i < cmString.length(); i++)
  {
    if (cmString.charAt(i) == ';')
    {
      parts[count++] = cmString.substring(startPos, i);
      startPos = i + 2; // Skip the semicolon and space and move to next part
    }
  }
  // Last part (no semicolon at the end)
  parts[count++] = cmString.substring(startPos);

  // Assign each part to separate variables
  String MqttHost = parts[0].substring(parts[0].indexOf(" ") + 1);
  String MqttUser = parts[1].substring(parts[1].indexOf(" ") + 1);
  String MqttClient = parts[2].substring(parts[2].indexOf(" ") + 1);
  String MqttPassword = parts[3].substring(parts[3].indexOf(" ") + 1);
  String Topic = parts[4].substring(parts[4].indexOf(" ") + 1);
  String FullTopic = parts[5].substring(parts[5].indexOf(" ") + 1);
  String SSID1 = parts[6].substring(parts[6].indexOf(" ") + 1);
  String Password1 = parts[7].substring(parts[7].indexOf(" ") + 1);
  String Protocol = parts[8].substring(parts[8].indexOf(" ") + 1);

  // Print the values to Serial
  Serial.println("MqttHost: " + MqttHost);
  Serial.println("MqttUser: " + MqttUser);
  Serial.println("MqttClient: " + MqttClient);
  Serial.println("MqttPassword: " + MqttPassword);
  Serial.println("Topic: " + Topic);
  Serial.println("FullTopic: " + FullTopic);
  Serial.println("SSID1: " + SSID1);
  Serial.println("Password1: " + Password1);
  Serial.println("Protocol: " + Protocol);

  saveCredentials(SSID1, Password1);
  saveMqttParams(MqttHost, MqttClient, MqttPassword);
  PROTOCOL = Protocol.toInt();
  saveProtocol();
  server.send(200, "text/html", "Configuration saved. Restarting...");
  delay(500);
  // loadCredentials();
  WiFi.mode(WIFI_AP_STA);
  flag = true;
  // connectToWiFi();
  // delay(10000);
  // WiFi.softAPdisconnect(true);
  // Serial.println("Access Point ended.");
  // delay(100);
  // ESP.restart();
}

void handleStatus()
{
  Serial.println("Received request for /status");
  // recieveProtocol();
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["detectedProtocol"] = PROTOCOL_RECV;
  jsonDoc["deviceId"] = hostname;
  jsonDoc["uptime"] = millis() / 1000;
  jsonDoc["signalStrength"] = WiFi.RSSI();

  String jsonResponse;
  serializeJson(jsonDoc, jsonResponse);

  server.send(200, "application/json", jsonResponse);
  Serial.println("Response sent for /status");
}

void handleTestIR()
{
  Serial.println("Recieved request for /test");
  // Get the command parameter from the URL
  String command = server.arg("command");

  // Initialize variables to hold the extracted values
  int protocol = -1;
  int power = -1;
  int temp = -1;
  int fan_speed = -1;

  // Split the command by ';' to separate key-value pairs
  int startPos = 0;
  String parts[4]; // Assuming there are 4 parts separated by commas
  int count = 0;
  for (int i = startPos; i < command.length(); i++)
  {
    if (command.charAt(i) == ';')
    {
      parts[count++] = command.substring(startPos, i);
      startPos = i + 2; // Skip the semicolon and space and move to next part
    }
  }
  // Last part (no semicolon at the end)
  parts[count++] = command.substring(startPos);

  // Assign each part to separate variables
  protocol = parts[0].substring(parts[0].indexOf(" ") + 1).toInt();
  power = parts[1].substring(parts[1].indexOf(" ") + 1).toInt();
  temp = parts[2].substring(parts[2].indexOf(" ") + 1).toInt();
  fan_speed = parts[3].substring(parts[3].indexOf(" ") + 1).toInt();
  // Print results to Serial Monitor
  Serial.print("Protocol: ");
  Serial.println(protocol);
  Serial.print("Power: ");
  Serial.println(power);
  Serial.print("Temperature: ");
  Serial.println(temp);
  Serial.print("Fan Speed: ");
  Serial.println(fan_speed);

  ir_msg msg;
  msg.protocol = protocol;
  msg.power = power;
  msg.fan_speed = fan_speed;
  msg.mode = MODE;
  msg.temp = temp;
  // sending ir command
  send_ir(msg, ir_led);
  // Serial.println("ir command sent");

  // Send response back to client
  String response = "IR command sent";
  server.send(200, "text/plain", response);
}

void handleIP()
{
  String ip = WiFi.localIP().toString();
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["IP"] = ip;
  String jsonResponse;
  serializeJson(jsonDoc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void handleSubmit()
{
  String ssidRes = server.arg("ssid");
  String passwordRes = server.arg("password");
  Serial.println("Received SSID: " + ssidRes);
  Serial.println("Received Password: " + passwordRes);
  // You can save the credentials or use them as needed

  // Send a response back to the client
  server.send(200, "text/html", "Configuration saved. Restarting...");
  saveCredentials(ssidRes, passwordRes);
  delay(2000);

  // WiFi.softAPdisconnect(true);
  // Serial.println("Access Point ended.");
  // Restart the ESP8266 to apply the new configuration
  ESP.restart();
}

void saveCredentials(String ssid, String password)
{
  Serial.println("Saving credentials");
  // Open the credentials file for writing
  File file = LittleFS.open("/credentials.txt", "w");
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Write SSID and password to the file
  file.println(ssid);
  file.println(password);

  // Close the file
  file.close();
}

void saveMqttParams(String MqttHost, String MqttClient, String MqttPassword)
{
  Serial.println("Saving mqtt parameters");
  // Open the mqttParams file for writing
  File file = LittleFS.open("/mqttParams.txt", "w");
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.println(MqttHost);
  file.println(MqttClient);
  file.println(MqttPassword);
  file.close();
}

void saveURL(String URL, String updateStarted)
{
  Serial.println("Saving URL and update flag");
  File file = LittleFS.open("/URL.txt", "w");
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.println(URL);
  file.println(updateStarted);
  file.close();
}

void loadCredentials()
{
  // Open the credentials file for reading
  if (!LittleFS.begin())
  {
    Serial.println("Failed to mount file system");
    return;
  }

  File file = LittleFS.open("/credentials.txt", "r");
  if (!file)
  {
    Serial.println("No saved credentials found");
    return;
  }

  // Read SSID and password from the file
  String ssidRes = file.readStringUntil('\n');
  String passwordRes = file.readStringUntil('\n');
  ssidRes.trim();

  passwordRes.trim();
  wifiSsid = ssidRes;
  wifiPw = passwordRes;
  // Close the file
  file.close();

  Serial.println(ssidRes);
  Serial.println(passwordRes);
}

void loadMqqtParams()
{
  if (!LittleFS.begin())
  {
    Serial.println("Failed to mount file system");
    return;
  }

  File file = LittleFS.open("/mqttParams.txt", "r");
  if (!file)
  {
    Serial.println("No saved mqtt parameters found");
    return;
  }
  String MqttHost = file.readStringUntil('\n');
  String MqttClient = file.readStringUntil('\n');
  String MqttPassword = file.readStringUntil('\n');
  MqttHost.trim();
  MqttClient.trim();
  MqttPassword.trim();
  //////////////////////Setting mqtt parameters
  host = MqttHost;
  device_id = MqttClient;
  if (MqttClient != "")
  {
    hostname = MqttClient;
  }

  device_key = MqttPassword;
  Serial.print("loading mqtt host; ");
  Serial.println(host);
  Serial.print("loading device id; ");
  Serial.println(device_id);
  Serial.print("loading device key; ");
  Serial.println(device_key);
}

bool connectToWiFi()
{
  Serial.println("Connecting to WiFi");

  Serial.println(wifiSsid);
  Serial.println(wifiPw);

  // Connect to Wi-Fi network using loaded credentials
  WiFi.begin(wifiSsid.c_str(), wifiPw.c_str());

  // Wait until connected or timeout (10 seconds)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    int reading = digitalRead(buttonPin);
    if (reading == LOW)
    {
      return false;
    }
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  // Check connection result
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
    return true;
  }
  else
  {
    // Serial.println("\nFailed to connect to WiFi. Check your credentials or try again.");
    Serial.println("\nFailed to connect to WiFi. AP Started. Connect to network: " + String(ssid) + "and reconfigure wifi.");
    startAPServer();
    return false;
  }
}

// Function to read a string from EEPROM
void readStringFromEEPROM(int startAddress, char *buffer)
{
  EEPROM.begin(512);
  int i = 0;

  // Read each character until null-terminator is encountered
  while (true)
  {
    char character = EEPROM.read(startAddress + i);
    buffer[i] = character;

    if (character == '\0')
    {
      break;
    }

    i++;
  }
  EEPROM.end();
}

void writeStringToEEPROM(int startAddress, char *string)
{
  EEPROM.begin(512);
  int length = strlen(string);

  for (int i = 0; i < length; i++)
  {
    EEPROM.write(startAddress + i, string[i]);
  }

  // Write the null-terminator at the end of the string
  EEPROM.write(startAddress + length, '\0');

  // Commit the changes to EEPROM
  EEPROM.commit();
  EEPROM.end();
}

void saveDataToEEPROM()
{
  EEPROM.begin(512);
  EEPROM.write(temp_address, TEMPERATURE);
  EEPROM.write(fanspeed_address, FAN_SPEED);
  EEPROM.end();
}

void saveProtocol()
{
  EEPROM.begin(512);
  EEPROM.write(prot_address, PROTOCOL);
  EEPROM.end();
  Serial.print("Saving protocol: ");
  Serial.println(PROTOCOL);
}

void loadDataFromEEPROM()
{
  EEPROM.begin(512);
  if (EEPROM.read(temp_address) != 255)
  {
    TEMPERATURE = EEPROM.read(temp_address);
  }
  if (EEPROM.read(fanspeed_address) != 255)
  {
    FAN_SPEED = EEPROM.read(fanspeed_address);
  }
  if (EEPROM.read(prot_address) != 255)
  {
    PROTOCOL = EEPROM.read(prot_address);
  }
  EEPROM.end();
}

void setOTA()
{
  // Hostname for OTA identification
  ArduinoOTA.setHostname("myESP8266");

  // Password for OTA authentication (optional)
  // ArduinoOTA.setPassword("admin");

  ArduinoOTA.begin();
}

static void initializeTime()
{
  Serial.print("Setting time using SNTP");

  configTime(-5 * 3600, 0, NTP_SERVERS);
  time_t now = time(NULL);
  while (now < 1510592825)
  {
    delay(500);
    Serial.print(".");
    now = time(NULL);
  }
  Serial.println("done!");
}

static char *getCurrentLocalTimeString()
{
  time_t now = time(NULL);
  return ctime(&now);
}

static void printCurrentTime()
{
  Serial.print("Current time: ");
  Serial.print(getCurrentLocalTimeString());
}

void publishToMqtt(const char *topic, const char *payload, int len)
{
  int result = mqtt_client.publish(
      topic,
      payload);
  if (result == 0)
  {
    Serial.println("Publish failure with 0");
  }
  else if (result == -1)
  {
    Serial.println("Publish failure with -1");
  }
  else
  {
    Serial.println("Message published successfully");
  }
}

void onDeviceTwinGet(const char *payload, size_t length)
{
  Serial.print("twin payload recieved : ");
  Serial.println(payload);
}

void receivedCallback(char *topic, byte *payload, unsigned int length)
{
  ////////////////////////////////////////
  String topic_data;
  int topic_index;
  int req_id_index;
  int twin_index;
  String method;
  String req_id;
  // std::string data;
  String data;
  /////////////////////////////////////////

  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    data += (char)payload[i];
  }
  Serial.println("");

  StaticJsonDocument<256> doc; // Adjust the size based on your JSON
  DeserializationError error = deserializeJson(doc, data);

  if (error)
  {
    Serial.print(F("Failed to deserialize JSON: "));
    Serial.println(error.c_str());
  }

  topic_data = topic;

  topic_index = topic_data.indexOf("POST/");
  req_id_index = topic_data.indexOf("?$rid=");
  method = topic_data.substring(topic_index + 5, req_id_index - 1);
  Serial.print(method);
  Serial.println(" method invoked");
  req_id = topic_data.substring(req_id_index + 6);
  if (method == "OtaUrl")
  {
    const char *url = doc["state"];
    firmwareURL = String(url);
  }
  else if (method == "Upgrade")
  {
    const char *state = doc["state"];
    if (strcmp(state, "1") == 0)
    {
      if (firmwareURL)
      {
        updateStarted = "true";
        // Substring to be replaced
        String searchString = ".bin.gz";
        // Replacement substring
        String replaceString = "-minimal.bin.gz";

        // Find the position of the substring to be replaced
        int pos = firmwareURL.indexOf(searchString);

        // If the substring is found
        if (pos != -1)
        {
          // Maximum length of the modified string
          const int bufferSize = firmwareURL.length() + (replaceString.length() - searchString.length()) + 1;
          // Buffer to hold the modified string
          char charMinimalURL[bufferSize + 10];

          // Copy characters before the substring
          firmwareURL.substring(0, pos).toCharArray(charMinimalURL, bufferSize);
          // Concatenate the replacement substring
          strcat(charMinimalURL, replaceString.c_str());
          // Concatenate characters after the substring
          strcat(charMinimalURL, firmwareURL.substring(pos + searchString.length()).c_str());
          Serial.println(charMinimalURL);
          String minimalURL(charMinimalURL);
          Serial.println("Minimal firmware url = " + minimalURL);
          updateStarted = "true";
          saveURL(firmwareURL, updateStarted);
          // updateFirmware(minimalURL);
        }
      }
    }
  }
  else
  {
    // Handling kelon protocol for hisene ac; This protocol was not properly implemented in IRremoteESP8266.h//
    if (PROTOCOL == 103 || PROTOCOL == 112)
    {
      decode_results results;
      const uint16_t kKelonHdrMark = 9000;
      const uint16_t kKelonHdrSpace = 4600;
      const uint16_t kKelonBitMark = 560;
      const uint16_t kKelonOneSpace = 1680;
      const uint16_t kKelonZeroSpace = 600;
      const uint32_t kKelonGap = 2 * kDefaultMessageGap;
      const uint16_t kKelonFreq = 38000;
      uint8_t duty = kDutyDefault;
      irsend.begin();

      int nbits = 48;
      uint64_t data;
      uint64_t dataOnOff = 0x02840683; // ON/OFF // 10010010000001000000011010000011
      uint64_t dataFan0 = 0x92000683;
      uint64_t dataFan1 = 0x92010683;
      uint64_t dataFan2 = 0x92010683;
      uint64_t dataFan3 = 0x92010683;
      uint64_t data32 = 0xE2000683;
      uint64_t data31 = 0xD2000683;
      uint64_t data30 = 0xC2000683;
      uint64_t data29 = 0xB2000683;
      uint64_t data28 = 0xA2000683;
      uint64_t data27 = 0x92000683; // 25         // 10010010000000000000011010000011
      uint64_t data26 = 0x82000683; // 23
      uint64_t data25 = 0x72000683; // 23
      uint64_t data24 = 0x62000683; // 22
      uint64_t data23 = 0x52000683; // 21
      uint64_t data22 = 0x42000683; // 20
      uint64_t data21 = 0x32000683; // 19
      uint64_t data20 = 0x22000683; // 18
      uint64_t data19 = 0x12000683; // 17
      uint64_t data18 = 0x2000683;  // 18
      uint64_t data1 = 0x10000000;

      if (method == "power")
      {
        const char *power = doc["state"];
        if (strcmp(power, "ON") == 0)
        {
          loadDataFromEEPROM();
          Serial.print("Last saved temperature: ");
          Serial.println(TEMPERATURE);
          for (int i = 0; i < (TEMPERATURE - 18); i++)
          {
            dataOnOff += data1;
          }
          data = dataOnOff;
          irsend.sendGeneric(kKelonHdrMark, kKelonHdrSpace,
                             kKelonBitMark, kKelonOneSpace,
                             kKelonBitMark, kKelonZeroSpace,
                             kKelonBitMark, kKelonGap,
                             data, nbits, kKelonFreq, false, // LSB First.
                             0, kDutyDefault);
        }
        else if (strcmp(power, "OFF") == 0)
        {
          saveDataToEEPROM();
          Serial.print("Saving temperature: ");
          Serial.println(TEMPERATURE);
          Serial.print("Saving fan_speed: ");
          Serial.println(FAN_SPEED);

          data = dataOnOff;
          irsend.sendGeneric(kKelonHdrMark, kKelonHdrSpace,
                             kKelonBitMark, kKelonOneSpace,
                             kKelonBitMark, kKelonZeroSpace,
                             kKelonBitMark, kKelonGap,
                             data, nbits, kKelonFreq, false, // LSB First.
                             0, kDutyDefault);
        }
      }
      else if (method == "temp")
      {
        TEMPERATURE = doc["temp"];
        // if (TEMPERATURE == 16) data = data16;
        // else if (TEMPERATURE == 17) data = data17;
        if (TEMPERATURE == 18)
          data = data18;
        else if (TEMPERATURE == 19)
          data = data19;
        else if (TEMPERATURE == 20)
          data = data20;
        else if (TEMPERATURE == 21)
          data = data21;
        else if (TEMPERATURE == 22)
          data = data22;
        else if (TEMPERATURE == 23)
          data = data23;
        else if (TEMPERATURE == 24)
          data = data24;
        else if (TEMPERATURE == 25)
          data = data25;
        else if (TEMPERATURE == 26)
          data = data26;
        else if (TEMPERATURE == 27)
          data = data27;
        else if (TEMPERATURE == 28)
          data = data28;
        else if (TEMPERATURE == 29)
          data = data29;
        else if (TEMPERATURE == 30)
          data = data30;
        else if (TEMPERATURE == 31)
          data = data31;
        else if (TEMPERATURE == 32)
          data = data32;
        else if (TEMPERATURE > 32)
          data = data32;
        else if (TEMPERATURE < 18)
          data = data18;

        irsend.sendGeneric(kKelonHdrMark, kKelonHdrSpace,
                           kKelonBitMark, kKelonOneSpace,
                           kKelonBitMark, kKelonZeroSpace,
                           kKelonBitMark, kKelonGap,
                           data, nbits, kKelonFreq, false, // LSB First.
                           3, kDutyDefault);
        delay(50);
      }
    }
    else
    {
      if (method == "power")
      {
        ir_msg msg;
        const char *power = doc["state"];
        if (strcmp(power, "ON") == 0)
        {
          POWER = 1;
        }
        else if (strcmp(power, "OFF") == 0)
        {
          POWER = 0;
        }

        if (POWER == 1)
        {
          loadDataFromEEPROM();
          Serial.print("Saved protocol: ");
          Serial.println(PROTOCOL);
          Serial.print("Last saved temperature: ");
          Serial.println(TEMPERATURE);
          Serial.print("Last saved fan_speed: ");
          Serial.println(FAN_SPEED);
        }
        else if (POWER == 0)
        {
          saveDataToEEPROM();
          Serial.print("Saving temperature: ");
          Serial.println(TEMPERATURE);
          Serial.print("Saving fan_speed: ");
          Serial.println(FAN_SPEED);
        }
        msg.protocol = PROTOCOL;
        msg.power = POWER;
        msg.fan_speed = FAN_SPEED;
        msg.mode = MODE;
        msg.temp = TEMPERATURE;
        // sending ir command
        send_ir(msg, ir_led);
        // Serial.println("ir command sent");
      }

      else if (method == "temp")
      {
        ir_msg msg;
        TEMPERATURE = doc["temp"];
        msg.protocol = PROTOCOL;
        msg.power = 1;
        msg.fan_speed = FAN_SPEED;
        msg.mode = MODE;
        msg.temp = TEMPERATURE;
        // sending ir command
        send_ir(msg, ir_led);
        // Serial.println("ir command sent");
      }

      else if (method == "fan")
      {
        ir_msg msg;
        FAN_SPEED = doc["fanSpeed"];
        msg.protocol = PROTOCOL;

        msg.power = 1;
        msg.fan_speed = FAN_SPEED;
        msg.mode = MODE;
        msg.temp = TEMPERATURE;
        // sending ir command
        send_ir(msg, ir_led);
        // Serial.println("ir command sent");
      }

      else if (method == "mode")
      {
        ir_msg msg;
        MODE = doc["mode"];
        msg.protocol = PROTOCOL;
        msg.power = 1;
        msg.mode = MODE;
        msg.fan_speed = FAN_SPEED;
        msg.temp = TEMPERATURE;
        // sending ir command
        send_ir(msg, ir_led);
        // Serial.println("ir command sent");
      }
    }
  }
  if (method == "protocol")
  {
    PROTOCOL = doc["protocol"];
    // writeStringToEEPROM(prot_address, PROTOCOL);
    saveProtocol();
  }

  String response_topic = "$iothub/methods/res/200/?$rid=" + req_id;
  String response_payload = "success";
  Serial.print("response topic");
  Serial.println(response_topic);

  StaticJsonDocument<200> doc_res; // Choose a capacity that suits your data size
  doc_res["Success"] = true;

  // Optional: Print the JsonDocument to Serial for debugging
  String json;
  ArduinoJson::V6215PB2::serializeJson(doc_res, json);

  ///////////////////////
  publishToMqtt(response_topic.c_str(), json.c_str(), response_payload.length());
}

static void initializeClients()
{
  az_iot_hub_client_options options = az_iot_hub_client_options_default();
  options.user_agent = AZ_SPAN_FROM_STR(AZURE_SDK_CLIENT_USER_AGENT);

  wifi_client.setTrustAnchors(&cert);
  if (az_result_failed(az_iot_hub_client_init(
          &client,
          az_span_create((uint8_t *)(host.c_str()), host.length()),
          az_span_create((uint8_t *)(device_id.c_str()), device_id.length()),
          &options)))
  {
    Serial.println("Failed initializing Azure IoT Hub client");
    return;
  }

  mqtt_client.setServer(host.c_str(), port);
  mqtt_client.setCallback(receivedCallback);
}

/*
 * @brief           Gets the number of seconds since UNIX epoch until now.
 * @return uint32_t Number of seconds.
 */
static uint32_t getSecondsSinceEpoch() { return (uint32_t)time(NULL); }

static int generateSasToken(char *sas_token, size_t size)
{
  az_span signature_span = az_span_create((uint8_t *)signature, sizeofarray(signature));
  az_span out_signature_span;
  az_span encrypted_signature_span = az_span_create((uint8_t *)encrypted_signature, sizeofarray(encrypted_signature));

  uint32_t expiration = getSecondsSinceEpoch() + ONE_HOUR_IN_SECS;

  // Get signature
  if (az_result_failed(az_iot_hub_client_sas_get_signature(
          &client, expiration, signature_span, &out_signature_span)))
  {
    Serial.println("Failed getting SAS signature");
    return 1;
  }

  // Base64-decode device key
  int base64_decoded_device_key_length = base64_decode_chars(device_key.c_str(), device_key.length(), base64_decoded_device_key);

  if (base64_decoded_device_key_length == 0)
  {
    Serial.println("Failed base64 decoding device key");
    return 1;
  }

  // SHA-256 encrypt
  br_hmac_key_context kc;
  br_hmac_key_init(
      &kc, &br_sha256_vtable, base64_decoded_device_key, base64_decoded_device_key_length);

  br_hmac_context hmac_ctx;
  br_hmac_init(&hmac_ctx, &kc, 32);
  br_hmac_update(&hmac_ctx, az_span_ptr(out_signature_span), az_span_size(out_signature_span));
  br_hmac_out(&hmac_ctx, encrypted_signature);

  // Base64 encode encrypted signature
  String b64enc_hmacsha256_signature = base64::encode(encrypted_signature, br_hmac_size(&hmac_ctx));

  az_span b64enc_hmacsha256_signature_span = az_span_create(
      (uint8_t *)b64enc_hmacsha256_signature.c_str(), b64enc_hmacsha256_signature.length());

  // URl-encode base64 encoded encrypted signature
  if (az_result_failed(az_iot_hub_client_sas_get_password(
          &client,
          expiration,
          b64enc_hmacsha256_signature_span,
          AZ_SPAN_EMPTY,
          sas_token,
          size,
          NULL)))
  {
    Serial.println("Failed getting SAS token");
    return 1;
  }

  return 0;
}

void sendDeviceTwin()
{
  String twin_topic = "$iothub/twin/PATCH/properties/reported/?$rid=10";
  StaticJsonDocument<512> twin_doc;
  char twin_payload[300];
  twin_doc["hostname"] = hostname;
  twin_doc["ip"] = WiFi.localIP().toString();
  twin_doc["rssi"] = String(WiFi.RSSI());
  twin_doc["ssid"] = wifiSsid;
  twin_doc["state"] = "ON";
  twin_doc["version"] = "v2.4";
  twin_doc["protocol"] = PROTOCOL;

  serializeJson(twin_doc, twin_payload);
  publishToMqtt(twin_topic.c_str(), String(twin_payload).c_str(), String(twin_payload).length());
  Serial.println("Sent device twin");
}

static int connectToAzureIoTHub()
{
  size_t client_id_length;
  char mqtt_client_id[128];
  if (az_result_failed(az_iot_hub_client_get_client_id(
          &client, mqtt_client_id, sizeof(mqtt_client_id) - 1, &client_id_length)))
  {
    Serial.println("Failed getting client id");
    return 1;
  }

  mqtt_client_id[client_id_length] = '\0';

  char mqtt_username[128];
  // Get the MQTT user name used to connect to IoT Hub
  if (az_result_failed(az_iot_hub_client_get_user_name(
          &client, mqtt_username, sizeofarray(mqtt_username), NULL)))
  {
    Serial.println("Failed to get MQTT username");
    strcpy(mqtt_username, host.c_str());
    strcat(mqtt_username, "/");
    strcat(mqtt_username, device_id.c_str());
    Serial.println("Username generated");
  }

  Serial.print("Client ID: ");
  Serial.println(mqtt_client_id);

  Serial.print("Username: ");
  Serial.println(mqtt_username);

  mqtt_client.setBufferSize(MQTT_PACKET_SIZE);

  while (!mqtt_client.connected())
  {
    time_t now = time(NULL);

    Serial.print("MQTT connecting ... ");

    if (mqtt_client.connect(mqtt_client_id, mqtt_username, sas_token))
    {
      Serial.println("connected.");
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      Serial.print("failed, status code =");
      Serial.print(mqtt_client.state());
      Serial.println(". Trying again in 5 seconds.");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

  int r;
  r = mqtt_client.subscribe(AZ_IOT_HUB_CLIENT_METHODS_SUBSCRIBE_TOPIC);
  if (r == -1)
  {
    Serial.println("Could not subscribe for cloud-to-device messages.");
  }
  else
  {
    Serial.println("Subscribed for cloud-to-device messages; message id:" + String(r));
  }

  r = mqtt_client.subscribe(AZ_IOT_HUB_CLIENT_TWIN_RESPONSE_SUBSCRIBE_TOPIC);
  if (r == -1)
  {
    Serial.println("Could not subscribe for twin.");
  }
  else
  {
    Serial.println("Subscribed for twin messages; message id:" + String(r));
    sendDeviceTwin();
  }
  return 0;
}

static void establishConnection()
{
  Serial.println("Launcing ac controller...");
  loadCredentials();
  loadMqqtParams();
  loadDataFromEEPROM();
  Serial.print("Saved protocol: ");
  Serial.println(PROTOCOL);
  if (wifiSsid == "") /////////////////////////change for testing purposes////////////////// original =  if (wifiSsid == "")//////////////////////////
  {
    startAPServer();
  }
  else
  {
    if (flag)
    {
      WiFi.mode(WIFI_AP_STA);
    }
    else
    {
      WiFi.mode(WIFI_STA);
    }
    // startServer();

    if (!WiFi.isConnected())
    {
      isWifiConnected = connectToWiFi();
    }

    if (WiFi.isConnected())
    {
      setOTA();
      // startServerOTA();
      initializeTime();
      printCurrentTime();
      initializeClients();
      // The SAS token is valid for 1 hour by default in this sample.
      // After one hour the sample must be restarted, or the client won't be able
      // to connect/stay connected to the Azure IoT Hub.
      if (generateSasToken(sas_token, sizeofarray(sas_token)) != 0)
      {
        Serial.println("Failed generating MQTT password");
      }
      else
      {
        connectToAzureIoTHub();
      }
    }

    // digitalWrite(LED_PIN, LOW);
    // }
  }
}

void setup()
{
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  pinMode(LED_PIN, OUTPUT);
  // pinMode(buttonPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);

  digitalWrite(LED_PIN, LOW);
#if DECODE_HASH
  // Ignore messages with less than minimum on or off pulses.
  irrecv.setUnknownThreshold(kMinUnknownSize);
#endif                                       // DECODE_HASH
  irrecv.setTolerance(kTolerancePercentage); // Override the default tolerance.
  irrecv.enableIRIn();                       // Start the receiver
  // initializeWifi();
  generateWifiHost();
  establishConnection();
}

void loop()
{
  if (apStarted)
  {
    recieveProtocol();
  }
  else
  {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // if ((wifiSsid != "") && (WiFi.status() != WL_CONNECTED))
    if ((wifiSsid != "") && ((WiFi.status() != WL_CONNECTED) || !mqtt_client.connected()))
    {
      Serial.println("Wifi not connected");
      digitalWrite(LED_PIN, LOW);
      establishConnection();
      delay(500);
    }

    else
    {
      ArduinoOTA.handle();
      mqtt_client.loop();
      // serverOTA.handleClient();     web server for ota updates is disabled
      delay(500);
    }
  }
  server.handleClient();
  //////////////////////Handle reset with long press////////////////////////////////////////////////////////////////////////////////////
  int reading = digitalRead(buttonPin);

  // Check if the button state has changed
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis(); // Reset the debounce timer
  }

  // Check for long press
  if ((millis() - lastDebounceTime) > longPressInterval)
  {
    if (reading == LOW)
    {
      handleLongPress(); // Call function to handle long press
    }
  }
  // Save the current button state for the next loop iteration
  lastButtonState = reading;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ////////handle establish connection after recieving credentials//////////////////////////////////////////////////////////////////
  if (flag)
  {
    // If startTime is 0, set it to the current millis() value
    if (startTime == 0)
    {
      connectToWiFi();
      startTime = millis();
    }

    // Check if 30 seconds have passed
    if (millis() - startTime >= 5000)
    {
      WiFi.softAPdisconnect(true);
      apStarted = false;
      Serial.println("Access Point ended.");
      // WiFi.mode(WIFI_STA);
      flag = false;  // Reset the flag
      startTime = 0; // Reset startTime for the next time flag is true
      ESP.restart();
    }
  }
}
