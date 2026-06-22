#ifndef __STALK_H__
#define __STALK_H__

#include "stdint.h"

/*
* @brief Stalk states
* @note bit 0 (1): L_BLINK
*       bit 1 (1): R_BLINK
*       bit 2 (1): HB
        bit 3 (0/1): ONCE/CONSTANT
*/
typedef enum {
    NORMAL = 0b1000, // 8
    L_BLINK_ONCE = 0b0001, // 1
    L_BLINK = 0b1001, // 9
    R_BLINK_ONCE = 0b0010, // 2
    R_BLINK = 0b1010, // 10
    HB_ONCE = 0b0100, // 4
    HB = 0b1100 // 12
} STALK_lState_t;

STALK_lState_t getStalkState(float pin0Voltage, float pin1Voltage, float pin2voltage);

#endif // __STALK_H__