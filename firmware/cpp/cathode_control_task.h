#pragma once

#include "display_task_handler.h"
#include "temperature_sensor_task.h"

extern TemperatureSensorTaskHandler temperatureSensor;

// PWM_MAX is raw value from 2^PWM_RESOLUTION-1 and defines entire range of raw PWM values [0, PWM_MAX]
// MAX_DUTY is percentage of PWM_MAX and limits total PWM range/duty cycle [0, (MAX_DUTY*PWM_MAX)/100]
// Level is 0-255 and is mapped to PWM_DUTY's range

class CathodeControlTaskHandler : public DisplayTaskHandler
{
private:
    static const uint8_t TASK_PRIORITY = 5;
    static const uint8_t PWM_RESOLUTION = 8;
    static const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;
    static const uint8_t HV_EN_PIN = 8;
    static const uint8_t PWM_PIN = 44;
    static const uint32_t DELAY_MS = 50;
    // default values for configurable tuning parameters
    static const uint32_t DEFAULT_PWM_DELAY_MS = 6;
    static constexpr uint8_t DEFAULT_PWM_MAX_DUTY = 42;
    static constexpr float DEFAULT_TEMPF_MIN = 60.0;
    static constexpr float DEFAULT_TEMPF_MAX = 80.0;

    // configurable tuning parameters
    volatile int _pwmDelayMs;
    volatile int _pwmMax;
    volatile float _tempFMin;
    volatile float _tempFMax;
    // current state
    bool _hvEnabled;
    volatile bool _isAuto; // temperature based PWM control
    // PWM values are [0,2^PWM_RESOLUTION-1]
    volatile int _manualTargetPwm;

public:
    CathodeControlTaskHandler() {}
    bool createTask() override;
    bool isAutoMode() { return _isAuto; }
    bool isHvEnabled() { return _hvEnabled; }
    void setMessage(const char *message) override;

private:
    void pwmRamp(int start, int end);
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

    _pwmDelayMs = DEFAULT_PWM_DELAY_MS;
    _pwmMax = (DEFAULT_PWM_MAX_DUTY * PWM_MAX) / 100;
    _tempFMin = DEFAULT_TEMPF_MIN;
    _tempFMax = DEFAULT_TEMPF_MAX;

    _hvEnabled = false;
    _isAuto = true;
    _manualTargetPwm = 0;

    pinMode(HV_EN_PIN, OUTPUT);
    digitalWrite(HV_EN_PIN, _hvEnabled);

    pinMode(PWM_PIN, OUTPUT);
    analogWriteResolution(PWM_RESOLUTION);
    analogWriteFrequency(24000);
    analogWrite(PWM_PIN, 0);

    xTaskCreate(taskWrapper, "CathodeControlTask", 4096, this, TASK_PRIORITY, &_taskHandle);
    log_d("Cathode control initialized and task started");

    return true;
}

void CathodeControlTaskHandler::setMessage(const char *message)
{
    if (strncmp(message, "manual|", 7) == 0)
    {
        _isAuto = false;
        // map value from 0-255 to 0-_pwmMax
        int value = constrain(atoi(message + 7), 0, 255);
        _manualTargetPwm = map(value, 0x00, 0xFF, 0, _pwmMax);
        log_i("Manual mode, PWM: %d", _manualTargetPwm);
        DisplayTaskHandler::setMessage("Manual mode, PWM: %d/255 (%d/%d adj)", value, _manualTargetPwm, _pwmMax);
    }
    else if (strcmp(message, "auto") == 0)
    {
        _isAuto = true;
        log_i("Auto mode");
        DisplayTaskHandler::setMessage("Auto mode");
    }
    else if (strncmp(message, "automaxf|", 9) == 0)
    {
        _isAuto = true;
        _tempFMax = constrain(atoi(message + 9), _tempFMin, 185);
        log_i("Auto mode, Max F: %d", _tempFMax);
        DisplayTaskHandler::setMessage("Auto mode, Max F: %.1f", _tempFMax);
    }
    else if (strncmp(message, "autominf|", 9) == 0)
    {
        _isAuto = true;
        _tempFMin = constrain(atoi(message + 9), -50, _tempFMax);
        log_i("Auto mode, Min F: %d", _tempFMin);
        DisplayTaskHandler::setMessage("Auto mode, Min F: %.1f", _tempFMin);
    }
    else if (strncmp(message, "pwmdelayms|", 11) == 0)
    {
        _pwmDelayMs = constrain(atoi(message + 11), 1, 50);
        log_i("PWM delay: %d ms", _pwmDelayMs);
        DisplayTaskHandler::setMessage("PWM delay: %d ms", _pwmDelayMs);
    }
    else if (strncmp(message, "pwmmaxduty|", 11) == 0)
    {
        int newPwmMaxDuty = constrain(atoi(message + 11), 0, 100);
        int newPwmMax = (newPwmMaxDuty * PWM_MAX) / 100;
        if (newPwmMax != _pwmMax)
        {
            bool savedDisplay = _display;

            if (isHvEnabled())
            {
                log_i("Disabling HV to adjust PWM max duty");
                _display = false; // task will ramp down and turn off HV

                unsigned long startTime = millis();
                while (isHvEnabled())
                {
                    if (millis() - startTime > 5000)
                    {
                        log_e("PWM max duty: HV too slow to disable");
                        DisplayTaskHandler::setMessage("PWM max duty: HV too slow to disable");
                        return;
                    }
                    delay(10);
                }
            }

            // adjust _manualTargetPwm to maintain same relative level
            _manualTargetPwm = map(_manualTargetPwm, 0, _pwmMax, 0, newPwmMax);
            _pwmMax = newPwmMax;
            _display = savedDisplay; // HV and PWM will be re-enabled by task

            log_i("PWM max duty: %d% (%d/%d)", newPwmMaxDuty, _pwmMax, PWM_MAX);
            DisplayTaskHandler::setMessage("PWM max duty: %d%% (%d/%d)", newPwmMaxDuty, _pwmMax, PWM_MAX);
        }
        else
        {
            log_i("PWM max duty: %d% (%d/%d), no update", newPwmMaxDuty, _pwmMax, PWM_MAX);
            DisplayTaskHandler::setMessage("PWM max duty: %d%% (%d/%d), no update", newPwmMaxDuty, _pwmMax, PWM_MAX);
        }
    }
    else
    {
        log_w("Invalid message: %s", message);
        DisplayTaskHandler::setMessage("Invalid message: %s", message);
    }
}

void CathodeControlTaskHandler::pwmRamp(int start, int end)
{
    int step = (start < end) ? 1 : -1;
    for (int i = start; i != end + step; i += step)
    {
        analogWrite(PWM_PIN, i);
        delay(_pwmDelayMs);
    }
}

void CathodeControlTaskHandler::task(void *parameters)
{
    delay(2000); // wait for other tasks to start
    log_d("Starting CathodeControlTask");

    log_i("Performing ramp up test for IN-13");
    digitalWrite(HV_EN_PIN, true);
    delay(DELAY_MS);
    pwmRamp(0, _pwmMax);
    pwmRamp(_pwmMax, 0);
    digitalWrite(HV_EN_PIN, false);
    delay(DELAY_MS);
    log_i("Ramp up test complete");

    int currentPwm = 0;
    int targetPwm = -1;
    while (1)
    {
        // determine target PWM based on mode and temperature sensor
        if (!_display)
        {
            if (targetPwm != 0)
            {
                log_w("Display is off, Target PWM: 0");
                targetPwm = 0;
            }
        }
        else if (_isAuto)
        {
            if (!temperatureSensor.isSensorFound())
            {
                if (targetPwm != 0)
                {
                    log_w("Temperature sensor not found, Target PWM: 0");
                    targetPwm = 0;
                }
            }
            else
            {
                float temperature = temperatureSensor.getTemperature();
                uint8_t newTargetPwm = map(temperature, _tempFMin, _tempFMax, 0, _pwmMax);
                if (newTargetPwm != targetPwm)
                {
                    targetPwm = newTargetPwm;
                    log_i("Temperature: %.1f°F, Target PWM: %d", temperature, targetPwm);
                }
            }
        }
        else if (targetPwm != _manualTargetPwm)
        {
            targetPwm = _manualTargetPwm;
            log_i("Manual mode, Target PWM: %d", targetPwm);
        }

        // manage HV enable/disable
        if (_hvEnabled && currentPwm == 0 && targetPwm == 0)
        {
            log_i("Disabling HV");
            _hvEnabled = false;
            digitalWrite(HV_EN_PIN, _hvEnabled);
            delay(DELAY_MS);
            continue;
        }
        else if (!_hvEnabled && currentPwm == 0 && targetPwm > 0)
        {
            log_i("Enabling HV");
            _hvEnabled = true;
            digitalWrite(HV_EN_PIN, _hvEnabled);
            delay(DELAY_MS);
            continue;
        }

        // ramp up/down PWM incrementally each cycle
        currentPwm = currentPwm < targetPwm ? currentPwm + 1 : currentPwm > targetPwm ? currentPwm - 1
                                                                                      : currentPwm;

        if (currentPwm != targetPwm)
        {
            log_d("Current PWM: %d, Target PWM: %d", currentPwm, targetPwm);
        }

        analogWrite(PWM_PIN, currentPwm);
        delay(_pwmDelayMs);
    }
}
