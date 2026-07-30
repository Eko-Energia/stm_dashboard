#ifndef APP_CAN_H
#define APP_CAN_H

#include "can.h"
#include "stalk.h"
#include "CAN_DB.h"

typedef enum
{
    CAN_OK = 0,
    CAN_TX_ERROR = 1,
    CAN_RX_ERROR = 2,
    CAN_INIT_ERROR = 3
} CAN_State_t;

extern volatile CAN_State_t CAN_state;

void CAN_SendLightsFrame(CAN_HandleTypeDef *hcan, struct Dashboard_Lights_t *lightsData, STALK_lState_t stalkLeftState, LightSelector_State_t lightSelectorState, GPIO_PinState emergencyButtonState);

void CAN_SendControlFrame(CAN_HandleTypeDef *hcan, struct Dashboard_Control_t *controlData, GearSelector_State_t gearSelectorState, GPIO_PinState modeButtonState);

void CAN_SendWipersFrame(CAN_HandleTypeDef *hcan, struct Dashboard_Wipers_t *wipersData, STALK_rState_t stalkRightState);

#endif /* APP_CAN_H */
