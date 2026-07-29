#ifndef __STALK_H__
#define __STALK_H__

#include "stdint.h"

typedef enum {
    ADC_STALK_L1 = 0, ADC_STALK_L2, ADC_STALK_L3,
    ADC_STALK_R1, ADC_STALK_R2,
    ADC_GEAR, ADC_LIGHT
} ADC_Channel_t;

/*
* @brief Stalk states
* @note bit 0 (1): L_BLINK
*       bit 1 (1): R_BLINK
*       bit 2 (1): HB
        bit 3 (0/1): ONCE/CONSTANT
*/
typedef enum {
    L_NORMAL = 0b1000, // 8
    L_BLINK_ONCE = 0b0001, // 1
    L_BLINK = 0b1001, // 9
    R_BLINK_ONCE = 0b0010, // 2
    R_BLINK = 0b1010, // 10
    HB_ONCE = 0b0100, // 4
    HB = 0b1100 // 12
} STALK_lState_t;

typedef enum {
    R_NORMAL = 0,
    WIPE_ONCE = 1,
    WIPE_INTERVAL = 2,
    WIPE_LOW = 3,
    WIPE_HIGH = 4,
    BACK_WIPE = 5,
    BACK_FLUID = 6,
    FLUID = 7
} STALK_rState_t;

typedef enum {
    GearSelector_P = 3,
    GearSelector_R = 2,
    GearSelector_N = 1,
    GearSelector_D = 0
} GearSelector_State_t;

typedef enum {
    LightSelector_Default = 3,
    LightSelector_Mode1 = 2,
    LightSelector_Mode2 = 1,
    LightSelector_Mode3 = 0
} LightSelector_State_t;

STALK_lState_t getLeftStalkState(float pin0Voltage, float pin1Voltage, float pin2voltage);

STALK_rState_t getRightStalkState(float pin0Voltage, float pin1Voltage);

GearSelector_State_t getGearSelectorState(float voltage);

LightSelector_State_t getLightSelectorState(float voltage);

#endif // __STALK_H__