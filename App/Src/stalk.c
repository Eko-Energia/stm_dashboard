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

static float selectorVoltageRange [4][2] = {
    {0.09, 0.545}, // D day
    {0.545, 1.71}, // N positon
    {1.71, 2.765}, // R auto
    {2.765, 3.3} // P off
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
        if (voltage > pinVoltageRange[i][0] && voltage <= pinVoltageRange[i][1]) {
            return (STALK_pinState_t) i;
        }
    }
    return STALK_PIN_GND; // Default to GND if out of range
}

STALK_lState_t getLeftStalkState(float pin0Voltage, float pin1Voltage, float pin2voltage)
{
    STALK_pinState_t pin0State = getPinState(pin0Voltage);
    STALK_pinState_t pin1State = getPinState(pin1Voltage);
    STALK_pinState_t pin2State = getPinState(pin2voltage);

    uint8_t state = 0;

    switch (pin0State) 
    {
        case STALK_PIN_VCC:
            state |= 0b1000; // CONSTANT
            break;
        case STALK_PIN_GND:
            state |= 0b0000; // ONCE
            break;
        default:
            // pin state not defined, don't change the state
            break;
    }

    switch (pin1State) 
    {
        case STALK_PIN_MID_HIGH:
            state |= 0b0001; // L_BLINK
            break;
        case STALK_PIN_LOW:
            state |= 0b0010; // R_BLINK
            break;
        default:
            // pin state not defined, don't change the state
            break;
    }

    switch (pin2State)
    {
        case STALK_PIN_HIGH:
            state |= 0b0000; // NOT_HB
            break;
        case STALK_PIN_MID:
        case STALK_PIN_MID_HIGH:
            state |= 0b0100; // HB
            break;
        default:
            // pin state not defined, don't change the state
            break;
    }

    if (state == 0) 
    {
        // This state only happens in the begining of the program when adc waits for first samples
        state = 0b1000; // Default to NORMAL if no valid state is detected
    }

    return (STALK_lState_t) state;
}

STALK_rState_t getRightStalkState(float pin0Voltage, float pin1Voltage)
{
    STALK_pinState_t pin0State = getPinState(pin0Voltage);
    STALK_pinState_t pin1State = getPinState(pin1Voltage);
    
    STALK_rState_t state = R_NORMAL;
    
    switch (pin0State)
    {
        case STALK_PIN_LOW:
            state = WIPE_ONCE;
        break;
        case STALK_PIN_MID_HIGH:
            state = WIPE_INTERVAL;
        break;
        case STALK_PIN_HIGH:
            state = WIPE_LOW;
        break;
        default:
            // pin state not defined, don't change the state
            break;
    }

    switch (pin1State)
    {
        case STALK_PIN_MID:
            state = FLUID;
        break;
        case STALK_PIN_MID_HIGH:
            state = BACK_WIPE;
        break;
        case STALK_PIN_LOW:
            state = BACK_FLUID;
        break;
        default:
            // pin state not defined, don't change the state
        break;
    }

    return state;
}

GearSelector_State_t getGearSelectorState(float voltage) {
    // iterate through 4 gear selector states
    for (int i = 0; i < 4; i++) {
        if (voltage >= selectorVoltageRange[i][0] && voltage <= selectorVoltageRange[i][1]) {
            return (GearSelector_State_t) i;
        }
    }
    return GearSelector_P; // Default to park if out of range
}

LightSelector_State_t getLightSelectorState(float voltage) {
    return (LightSelector_State_t) getGearSelectorState(voltage);
}



