#pragma once

#include "display_task_handler.h"
#include "temperature_sensor_task.h"

extern TemperatureSensorTaskHandler temperatureSensor;

class CathodeControlTaskHandler : public DisplayTaskHandler
{
private:
    static const uint8_t TASK_PRIORITY = 5;
    static const int DELAY_MS = 50;
    static const int DEFAULT_TEMP_MIN = 60;
    static const int DEFAULT_TEMP_MAX = 90;
    static const uint32_t DEFAULT_PWM_DELAY_MS = 1;
    static const uint8_t DEFAULT_PWM_MAX_PERCENT = 42;
    static const uint8_t PWM_RESOLUTION = 9;
    static const uint8_t HV_EN_PIN = 8;
    static const uint8_t PWM_PIN = 44;

    // configurable tuning
    int _tempMin;
    int _tempMax;
    int _pwmDelayMs;
    int _pwmMax;
    // current state
    bool _isAuto;
    bool _hvEnabled;
    int _curPwm;
    int _targetPwm;

public:
    CathodeControlTaskHandler() {}
    bool createTask() override;
    void setMessage(const char *message) override;
    void setTargetPwm(uint8_t value);

private:
    void task(void *parameters) override;
};

bool CathodeControlTaskHandler::createTask()
{
    if (_taskHandle != NULL)
    {
        log_w("Task already started");
        return false;
    }

    _display = false;

    _tempMin = DEFAULT_TEMP_MIN;
    _tempMax = DEFAULT_TEMP_MAX;
    _pwmDelayMs = DEFAULT_PWM_DELAY_MS;
    _pwmMax = (DEFAULT_PWM_MAX_PERCENT / 100.0) * ((1 << PWM_RESOLUTION) - 1);

    _isAuto = true;
    _hvEnabled = false;
    _curPwm = 0;
    _targetPwm = -1;

    pinMode(HV_EN_PIN, OUTPUT);
    digitalWrite(HV_EN_PIN, _hvEnabled);

    pinMode(PWM_PIN, OUTPUT);
    analogWriteResolution(PWM_RESOLUTION);
    analogWriteFrequency(24000);
    analogWrite(PWM_PIN, _curPwm);

    xTaskCreate(taskWrapper, "CathodeControlTask", 4096, this, TASK_PRIORITY, &_taskHandle);
    log_d("Cathode control initialized and task started");

    return true;
}

void CathodeControlTaskHandler::setMessage(const char *message)
{
    DisplayTaskHandler::setMessage(message);

    if (strcmp(message, "auto") == 0)
    {
        _isAuto = true;
        log_i("Auto mode");
    }
    else if (strncmp(message, "manual|", 7) == 0)
    {
        _isAuto = false;
        int value = atoi(message + 7);
        setTargetPwm(constrain(value, 0x00, 0xFF));
        log_i("Manual mode, PWM: %d", _targetPwm);
    }
    else if (strncmp(message, "automaxf|", 9) == 0)
    {
        _tempMax = atoi(message + 9);
        log_i("Auto max temperature set to: %d", _tempMax);
    }
    else if (strncmp(message, "autominf|", 9) == 0)
    {
        _tempMin = atoi(message + 9);
        log_i("Auto min temperature set to: %d", _tempMin);
    }
    else if (strncmp(message, "maxpwmpct|", 10) == 0)
    {
        float pct = atof(message + 10) / 100.0;
        _pwmMax = static_cast<int>(pct * ((1 << PWM_RESOLUTION) - 1));
        log_i("Max PWM percentage set to: %.2f%%", pct * 100);
    }
    else if (strncmp(message, "pwmdelayms|", 11) == 0)
    {
        _pwmDelayMs = atoi(message + 11);
        log_i("PWM delay set to: %d ms", _pwmDelayMs);
    }
    else
    {
        log_w("Invalid message: %s", message);
    }
}

void CathodeControlTaskHandler::setTargetPwm(uint8_t value)
{
    // map value from 0-255 to 0-_pwmMax
    _targetPwm = map(value, 0x00, 0xFF, 0, _pwmMax);
}

void CathodeControlTaskHandler::task(void *parameters)
{
    delay(2000); // wait for other tasks to start
    log_d("Starting CathodeControlTask");

    log_i("Performing ramp up test for IN-13");
    digitalWrite(HV_EN_PIN, true);
    delay(DELAY_MS);
    for (int i = 0; i <= _pwmMax; i++)
    {
        analogWrite(PWM_PIN, i);
        delay(_pwmDelayMs);
    }
    for (int i = _pwmMax; i >= 0; i--)
    {
        analogWrite(PWM_PIN, i);
        delay(_pwmDelayMs);
    }
    digitalWrite(HV_EN_PIN, false);
    delay(DELAY_MS);
    log_i("Ramp up test complete");

    while (1)
    {
        // determine target PWM based on mode and temperature sensor
        if (!_display)
        {
            if (_targetPwm != 0)
            {
                log_w("Display is off, Target PWM: 0");
                _targetPwm = 0;
            }
        }
        else if (_isAuto)
        {
            if (!temperatureSensor.isSensorFound())
            {
                if (_targetPwm != 0)
                {
                    log_w("Temperature sensor not found, Target PWM: 0");
                    _targetPwm = 0;
                }
            }
            else
            {
                float temperature = temperatureSensor.getTemperature();
                uint8_t newTargetPwm = map(temperature, _tempMin, _tempMax, 0, _pwmMax);
                if (newTargetPwm != _targetPwm)
                {
                    _targetPwm = newTargetPwm;
                    log_i("Temperature: %.1f°F, Target PWM: %d", temperature, _targetPwm);
                }
            }
        }

        // manage HV enable/disable
        if (_hvEnabled && _curPwm == 0 && _targetPwm == 0)
        {
            log_i("Disabling HV");
            _hvEnabled = false;
            digitalWrite(HV_EN_PIN, _hvEnabled);
            delay(DELAY_MS);
            continue;
        }
        else if (!_hvEnabled && _curPwm == 0 && _targetPwm > 0)
        {
            log_i("Enabling HV");
            _hvEnabled = true;
            digitalWrite(HV_EN_PIN, _hvEnabled);
            delay(DELAY_MS);
            continue;
        }

        // ramp up/down PWM incrementally each cycle
        _curPwm = _curPwm < _targetPwm ? _curPwm + 1 : _curPwm > _targetPwm ? _curPwm - 1 : _curPwm;

        if (_curPwm != _targetPwm)
        {
            log_d("Current PWM: %d, Target PWM: %d", _curPwm, _targetPwm);
        }

        analogWrite(PWM_PIN, _curPwm);
        delay(_pwmDelayMs);
    }
}