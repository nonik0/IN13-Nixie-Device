#pragma once

#include <OneWire.h>

#include "display_task_handler.h"

class TemperatureSensorTaskHandler : public DisplayTaskHandler
{
private:
    const uint8_t ONEWIRE_PIN = 9;
    static const uint8_t TASK_PRIORITY = 5;
    const int DELAY_MS = 5000;

    OneWire _onewire;
    bool _sensorFound;
    float _currentTemperature;

public:
    TemperatureSensorTaskHandler() {}
    bool createTask() override;
    // void setMessage(const char *message) override;
    bool isSensorFound() { return _sensorFound; }
    float getTemperature() { return _currentTemperature; }
    void setValue(uint8_t value);

private:
    void task(void *parameters) override;
    void initializeMessage();
};

bool TemperatureSensorTaskHandler::createTask()
{
    if (_taskHandle != NULL)
    {
        log_w("Task already started");
        return false;
    }

    _sensorFound = false;
    _currentTemperature = 0.0;
    _onewire.begin(ONEWIRE_PIN);

    xTaskCreate(taskWrapper, "TemperatureSensorTask", 4096, this, TASK_PRIORITY, &_taskHandle);
    log_d("Temperature sensor initialized and task started");

    return true;
}

void TemperatureSensorTaskHandler::task(void *parameters)
{
    delay(1000); // wait for other tasks to start
    log_d("Starting TemperatureSensorTask");

    byte addr[8];
    byte data[12];
    float celsius, fahrenheit;

    while (1)
    {
        if (!_sensorFound)
        {
            if (!_onewire.search(addr))
            {
                log_w("No sensor found, retrying in 5 seconds...");
                _onewire.reset_search();
                delay(DELAY_MS);
                continue;
            }

            if (OneWire::crc8(addr, 7) != addr[7])
            {
                log_w("CRC is not valid!");
                _onewire.reset_search();
                delay(DELAY_MS);
                continue;
            }

            if (addr[0] != 0x28)
            {
                log_w("Device is not a DS18B20 family device.");
                _onewire.reset_search();
                delay(DELAY_MS);
                continue;
            }

            log_i("DS18B20 sensor found");
            _sensorFound = true;
        }

        try
        {
            if (!_onewire.reset())
            {
                log_w("Reset failed");
                _sensorFound = false;
                continue;
            }
            _onewire.select(addr);
            _onewire.write(0x44, 1); // start conversion, with parasite power on at the end

            delay(1000); // maybe 750ms is enough, maybe not
            // we might do a _onewire.depower() here, but the reset will take care of it.

            if (!_onewire.reset())
            {
                log_w("Reset failed");
                _sensorFound = false;
                continue;
            }
            _onewire.select(addr);
            _onewire.write(0xBE); // Read Scratchpad

            for (int i = 0; i < 9; i++)
            {
                data[i] = _onewire.read();
            }

            int16_t raw = (data[1] << 8) | data[0];
            if (data[7] == 0x10)
            {
                raw = (raw & 0xFFF0) + 12 - data[6];
            }
            celsius = (float)raw / 16.0;
            fahrenheit = celsius * 1.8 + 32.0;

            if (fahrenheit != _currentTemperature)
            {
                log_d("Current temperature: %.1f°C (%.1f°F)", celsius, fahrenheit);
                _sensorFound = true;
                _currentTemperature = fahrenheit;
            }

            delay(DELAY_MS);
        }
        catch (const std::exception &e)
        {
            log_w("Error reading temperature: %s", String(e.what()));
            _sensorFound = false;
            continue;
        }
    }
}