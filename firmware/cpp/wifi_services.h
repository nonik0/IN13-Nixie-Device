#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <vector>
#include <WebServer.h>
#include <WiFi.h>

#include "secrets.h"

using namespace std;
using SetDisplayCallback = std::function<void(bool)>;
using SetMessageCallback = std::function<const char *(const char *)>;

class WifiServices
{
private:
  const int ConnectionTimeoutMs = 10 * 1000;
  const int StatusCheckIntervalMs = 60 * 1000;
  const int TaskPriority = 1; // lowest priority

  const char *_hostname;
  const char *_hostnameLower;
  int _disconnectCount = 0;
  unsigned long _lastStatusCheckMs = 0;
  bool _displayState = false;
  ArduinoOTAClass _ota;
  WebServer _restServer;
  vector<SetDisplayCallback> _setDisplayCallbacks;

public:
  void setup(const char *hostname);
  void createTask();
  void registerSetDisplayCallback(SetDisplayCallback callback);
  void registerSetMessageCallback(const char *endpoint, SetMessageCallback callback);

private:
  void task();
  void checkWifiStatus();

  bool wifiSetup();
  void otaSetup();
  void mDnsSetup();
  void restSetup();

  void restIndex();
  void restDisplay();
  void restMessage(SetMessageCallback setMessage);
};

void WifiServices::setup(const char *hostname)
{
  log_i("Setting up Wifi services for %s", hostname);

  _hostname = hostname;
  _hostnameLower = strdup(hostname);
  char *p = (char *)(_hostnameLower);
  while (*p)
  {
    *p = tolower(*p);
    p++;
  }

  if (!wifiSetup())
  {
    return;
  }

  mDnsSetup();
  otaSetup();
  restSetup();

  log_i("WiFi services setup complete");
}

void WifiServices::createTask()
{
  log_i("Starting WifiServicesTask");

  xTaskCreate(
      [](void *p)
      { ((WifiServices *)p)->task(); },
      "WifiServicesTask",
      8192,
      this,
      TaskPriority,
      NULL);
}

void WifiServices::registerSetDisplayCallback(SetDisplayCallback callback)
{
  _setDisplayCallbacks.push_back(callback);
}

void WifiServices::registerSetMessageCallback(const char *endpoint, SetMessageCallback callback)
{
  _restServer.on(endpoint, [this, callback]()
                 { restMessage(callback); });
}

void WifiServices::task()
{
  while (1)
  {
    checkWifiStatus();
    _ota.handle();
    _restServer.handleClient();

    delay(10);
  }
}

void WifiServices::checkWifiStatus()
{
  if (millis() - _lastStatusCheckMs > StatusCheckIntervalMs)
  {
    _lastStatusCheckMs = millis();

    try
    {
      if (WiFi.status() != WL_CONNECTED)
      {
        _disconnectCount++;

        log_w("Wifi disconnecting, attempting to reconnect");
        WiFi.disconnect();
        WiFi.reconnect();
        log_w("Reconnected to WiFi");
      }
    }
    catch (const std::exception &e)
    {
      log_e("Wifi error: %s", String(e.what()));
    }
  }
}

bool WifiServices::wifiSetup()
{
  log_i("Wifi setting up...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startMs = millis();
  while (WiFi.waitForConnectResult() != WL_CONNECTED && millis() - startMs < ConnectionTimeoutMs)
    ;

  if (WiFi.status() != WL_CONNECTED)
  {
    log_e("Wifi failed to connect after %d ms", ConnectionTimeoutMs);
    return false;
  }

  log_i("Wifi setup complete, IP address: %s", WiFi.localIP().toString().c_str());
  return true;
}

void WifiServices::otaSetup()
{
  log_i("OTA setting up...");

  _ota = ArduinoOTA; // use the global instance

  _ota.setHostname(_hostname);
  //_ota.setPasswordHash(OTA_PASS_HASH); TODO: add password hash based off IP?

  _ota
      .onStart([this]()
               { log_i("Start updating %s", _ota.getCommand() == U_FLASH ? "sketch" : "filesystem"); })
      .onEnd([]()
             { log_i("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { log_i("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
        switch(error) {
          case OTA_AUTH_ERROR:
            log_e("Error[%u]: Auth Failed", error);
            break;
          case OTA_BEGIN_ERROR:
            log_e("Error[%u]: Begin Failed", error);
            break;
          case OTA_CONNECT_ERROR:
            log_e("Error[%u]: Connect Failed", error);
            break;
          case OTA_RECEIVE_ERROR:
            log_e("Error[%u]: Receive Failed", error);
            break;
          case OTA_END_ERROR:
            log_e("Error[%u]: End Failed", error);
            break;
        } });

  _ota.begin();

  log_i("OTA setup complete");
}

void WifiServices::mDnsSetup()
{
  if (!MDNS.begin(_hostnameLower))
  {
    log_e("Error setting up mDNS!");
    return;
  }

  log_i("mDNS setup complete");
}

void WifiServices::restIndex()
{
  log_i("Serving index.html");
  _restServer.send(200, "text/plain", _hostname);
  log_i("Served index.html");
}

void WifiServices::restDisplay()
{
  if (_restServer.hasArg("plain"))
  {
    String body = _restServer.arg("plain");
    body.toLowerCase();

    bool isValid = false;
    if (body == "off" || body == "false")
    {
      _displayState = false;
      isValid = true;
    }
    else if (body == "on" || body == "true")
    {
      _displayState = true;
      isValid = true;
    }

    if (!isValid)
    {
      _restServer.send(400, "text/plain", "Invalid value sent: " + body);
      return;
    }

    for (auto setDisplayCallback : _setDisplayCallbacks)
    {
      setDisplayCallback(_displayState);
    }
  }

  _restServer.send(200, "text/plain", _displayState ? "on" : "off");
}

void WifiServices::restMessage(SetMessageCallback setMessage)
{
  String newMessage = "";

  if (_restServer.hasArg("plain"))
  {
    newMessage = _restServer.arg("plain");
  }
  else if (_restServer.hasArg("message"))
  {
    newMessage = _restServer.arg("message");
  }

  const char *curMessage = setMessage(newMessage.c_str());

  _restServer.send(200, "text/plain", curMessage);
}

void WifiServices::restSetup()
{
  _restServer.on("/", [this]()
                 { restIndex(); });
  _restServer.on("/display", [this]()
                 { restDisplay(); });
  _restServer.begin();

  log_i("REST server setup complete");
}