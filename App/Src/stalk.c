#include "stalk.h"

#define PIN_STATES 6

static float pinVoltageRange [PIN_STATES][2] = {
    {0, 0.175}, // GND
    {0.176, 0.515}, // LOW
    {0.516, 0.875}, // MID
    {0.876, 1.335}, // MID_HIGH
    {1.336, 2.45}, //HIGH
    {2.46, 3.3}, // VCC
};

typedef enum {
    STALK_PIN_GND = 0,
    STALK_PIN_LOW = 1,
    STALK_PIN_MID = 2,
    STALK_PIN_MID_HIGH = 3,
    STALK_PIN_HIGH = 4,
    STALK_PIN_VCC = 5
} STALK_pinState_t;

static STALK_pinState_t getPinState(float voltage) {
    for (int i = 0; i < PIN_STATES; i++) {
        if (voltage >= pinVoltageRange[i][0] && voltage <= pinVoltageRange[i][1]) {
            return (STALK_pinState_t) i;
        }
    }
    return STALK_PIN_GND; // Default to GND if out of range
}

STALK_lState_t getStalkState(float pin0Voltage, float pin1Voltage, float pin2voltage)
{
    STALK_pinState_t pin0State = getPinState(pin0Voltage);
    STALK_pinState_t pin1State = getPinState(pin1Voltage);
    STALK_pinState_t pin2State = getPinState(pin2voltage);

    uint8_t state = 0;
    // ONCE/CONSTANT blinking
    state |= (pin0State == STALK_PIN_VCC) ? 0b1000 : 0;

    // R/L BLINK
    if(pin1State != STALK_PIN_MID)
    {
        if(pin1State == STALK_PIN_MID_HIGH)
        {
            // L_BLINK
            state |= 0b0001;
        }
        else if(pin1State == STALK_PIN_LOW)
        {
            // R_BLINK
            state |= 0b0010;
        }
    }

    // HB
    state |= (pin2State != STALK_PIN_HIGH) ? 0b0100 : 0;

    // NO MATCH
    if (state == 0)
    {
        state = NORMAL;
    }

    return (STALK_lState_t) state;
}




