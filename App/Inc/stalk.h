#ifndef __STALK_H__
#define __STALK_H__

typedef enum {
    L_BLINK_ONCE = 0b000,
    L_BLINK = 0b001,
    R_BLINK_ONCE = 0b010,
    R_BLINK = 0b011,
    HB_ONCE = 0b100,
    HB = 0b101
} STALK_lState_t;

STALK_lState_t getStalkState(float pin0Voltage, float pin1Voltage, float pin2voltage);

#endif // __STALK_H__