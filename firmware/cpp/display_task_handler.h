#pragma once
#include <Arduino.h>

class DisplayTaskHandler
{
protected:
    static const int MaxMessageSize = 100;

    bool _display = true;
    char _message[MaxMessageSize];
    TaskHandle_t _taskHandle = NULL;

public:
    virtual bool createTask() = 0;

    char *getMessage() { return _message; }

    virtual void setDisplay(bool displayState) { _display = displayState; }

    virtual void setMessage(const char *message) { strncpy(_message, message, MaxMessageSize); }

    bool suspendTask()
    {
        if (_taskHandle != NULL)
        {
            vTaskSuspend(_taskHandle);
            return true;
        }
        return false;
    }

protected:
    virtual void task(void *parameters) = 0;

    static void taskWrapper(void *parameters)
    {
        DisplayTaskHandler *handler = static_cast<DisplayTaskHandler *>(parameters);
        handler->task(parameters);
    }
};