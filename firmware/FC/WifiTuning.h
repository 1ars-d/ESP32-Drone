#ifndef WIFI_TUNING_H
#define WIFI_TUNING_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

// WiFi credentials
extern const char *ssid;
extern const char *password;

// Web server instance
extern AsyncWebServer server;

// Parameter names for PID gains
extern const char *PARAM_P_RATE_ROLL; // Pitch & Roll RATE
extern const char *PARAM_I_RATE_ROLL; // Pitch & Roll RATE
extern const char *PARAM_D_RATE_ROLL; // Pitch & Roll RATE

extern const char *PARAM_P_RATE_PITCH;
extern const char *PARAM_I_RATE_PITCH;
extern const char *PARAM_D_RATE_PITCH;

extern const char *PARAM_P_RATE_YAW;
extern const char *PARAM_I_RATE_YAW;
extern const char *PARAM_D_RATE_YAW;

extern float WifiPRateRoll;
extern float WifiIRateRoll;
extern float WifiDRateRoll;

extern float WifiPRatePitch;
extern float WifiIRatePitch;
extern float WifiDRatePitch;

extern float WifiPRateYaw;
extern float WifiIRateYaw;
extern float WifiDRateYaw;

extern float WifiPAngle;
extern float WifiIAngle;
extern float WifiDAngle;

// HTML content
extern const char index_html[] PROGMEM;

// Function prototypes
void notFound(AsyncWebServerRequest *request);
String processor(const String &var);
void setup__wifi_tuning();

#endif // WIFI_TUNING_H