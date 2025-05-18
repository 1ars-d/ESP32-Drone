#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "WifiTuning.h"

const char *ssid = "";
const char *password = "";

AsyncWebServer server(80);

const char *PARAM_P_RATE_ROLL = "pRateRoll";
const char *PARAM_I_RATE_ROLL = "iRateRoll";
const char *PARAM_D_RATE_ROLL = "dRateRoll";

const char *PARAM_P_RATE_PITCH = "pRatePitch";
const char *PARAM_I_RATE_PITCH = "iRatePitch";
const char *PARAM_D_RATE_PITCH = "dRatePitch";

const char *PARAM_P_RATE_YAW = "pRateYaw";
const char *PARAM_I_RATE_YAW = "iRateYaw";
const char *PARAM_D_RATE_YAW = "dRateYaw";

float WifiPRateRoll = 0.5;
float WifiIRateRoll = 0.30;
float WifiDRateRoll = 0.005;

float WifiPRatePitch = 0.55;
float WifiIRatePitch = 0.30;
float WifiDRatePitch = 0.005;

float WifiPRateYaw = 1.0;
float WifiIRateYaw = 0.1;
float WifiDRateYaw = 0.005;

float WifiPAngle;
float WifiIAngle;
float WifiDAngle;

// HTML web page to handle 6 input fields of PID gains
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);
    }
  </script></head>

  <body>

     <form action="/get" target="hidden-form"><br>
    ESP32 Webserver for PID Gain value tuning of Quadcopter 
  </form><br><br>

  <form action="/get" target="hidden-form">
    P Rate Roll (current value %pRateRoll%): <input type="number" step="any" name="pRateRoll">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    I Rate Roll (current value %iRateRoll%)<input type="number" step="any" name="iRateRoll">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    D Rate Roll (current value %dRateRoll%): <input type="number" step="any" name="dRateRoll">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>

  <form action="/get" target="hidden-form">
    P Rate Pitch (current value %pRatePitch%): <input type="number" step="any" name="pRatePitch">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    I Rate Pitch (current value %iRatePitch%)<input type="number" step="any" name="iRatePitch">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    D Rate Pitch (current value %dRatePitch%): <input type="number" step="any" name="dRatePitch">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>

  <form action="/get" target="hidden-form">
    P Rate Yaw (current value %pRateYaw%): <input type="number" step="any" name="pRateYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    I Rate Yaw (current value %iRateYaw%)<input type="number" step="any" name="iRateYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    D Rate Yaw (current value %dRateYaw%): <input type="number" step="any" name="dRateYaw">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>

  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

String processor(const String &var)
{
  if (var == "pRateRoll")
  {
    return String(WifiPRateRoll);
  }

  if (var == "iRateRoll")
  {
    return String(WifiIRateRoll);
  }

  if (var == "dRateRoll")
  {
    return String(WifiDRateRoll);
  }

  if (var == "pRatePitch")
  {
    return String(WifiPRatePitch);
  }

  if (var == "iRatePitch")
  {
    return String(WifiIRatePitch);
  }

  if (var == "dRatePitch")
  {
    return String(WifiDRatePitch);
  }

  if (var == "pRateYaw")
  {
    return String(WifiPRateYaw);
  }

  if (var == "iRateYaw")
  {
    return String(WifiIRateYaw);
  }

  if (var == "dRateYaw")
  {
    return String(WifiDRateYaw);
  }

  return "";
}

void setup__wifi_tuning()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 20)
  {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\n✅ WiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\n❌ WiFi failed after 10 seconds.");
  }
  Serial.println();
  Serial.print("IP Address: ");

  Serial.println(WiFi.localIP());
  delay(2000);
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html, processor); });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String inputMessage;
    // GET P Gain value on <ESP_IP>/get?pGain=<inputMessage>
    if (request->hasParam(PARAM_P_RATE_ROLL)) {
        inputMessage = request->getParam(PARAM_P_RATE_ROLL)->value();
        WifiPRateRoll = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_I_RATE_ROLL)) {
        inputMessage = request->getParam(PARAM_I_RATE_ROLL)->value();
        WifiIRateRoll = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_D_RATE_ROLL)) {
        inputMessage = request->getParam(PARAM_D_RATE_ROLL)->value();
        WifiDRateRoll = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_P_RATE_PITCH)) {
        inputMessage = request->getParam(PARAM_P_RATE_PITCH)->value();
        WifiPRatePitch = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_I_RATE_PITCH)) {
        inputMessage = request->getParam(PARAM_I_RATE_PITCH)->value();
        WifiIRatePitch = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_D_RATE_PITCH)) {
        inputMessage = request->getParam(PARAM_D_RATE_PITCH)->value();
        WifiDRatePitch = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_P_RATE_YAW)) {
        inputMessage = request->getParam(PARAM_P_RATE_YAW)->value();
        WifiPRateYaw = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_I_RATE_YAW)) {
        inputMessage = request->getParam(PARAM_I_RATE_YAW)->value();
        WifiIRateYaw = inputMessage.toFloat();
    }
    else if (request->hasParam(PARAM_D_RATE_PITCH)) {
        inputMessage = request->getParam(PARAM_D_RATE_YAW)->value();
        WifiDRateYaw = inputMessage.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/text", inputMessage); });

  server.onNotFound(notFound);
  server.begin();
}