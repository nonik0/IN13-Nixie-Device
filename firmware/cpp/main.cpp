#include "cathode_control_task.h"
#include "temperature_sensor_task.h"
#include "wifi_services.h"

CathodeControlTaskHandler cathodeControl;
TemperatureSensorTaskHandler temperatureSensor;
WifiServices wifiServices;

void setup()
{
  delay(5000);

  Serial.begin(115200);
  log_d("Starting setup...");

  wifiServices.setup(DEVICE_NAME);

  cathodeControl.createTask();
  temperatureSensor.createTask();
  wifiServices.createTask();

  wifiServices.registerGetDataCallback<float>("/temperature", []() -> float
                                       { return temperatureSensor.isSensorFound() ? temperatureSensor.getTemperature() : -1; });
  wifiServices.registerSetDisplayCallback([&](bool state)
                                          { cathodeControl.setDisplay(state); });
  wifiServices.registerSetMessageCallback("/cathodeControl", [](const char *message)
                                          { if (strlen(message) > 0)  cathodeControl.setMessage(message);
                                            return cathodeControl.getMessage(); });

  log_d("Setup complete");
}

void loop()
{
}
