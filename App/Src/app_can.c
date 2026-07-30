#include "app_can.h"
#include "main.h"
#include "can_driver.h"

extern struct CAN_scheduledMsgList canScheduler;

void CAN_SendControlFrame(CAN_HandleTypeDef *hcan, struct Dashboard_Control_t *controlData, GearSelector_State_t gearSelectorState)
{
    switch(gearSelectorState)
    {
        case GearSelector_P:
            controlData->PRND = DASHBOARD_CONTROL_PRND_P_CHOICE;
        break;
        case GearSelector_R:
            controlData->PRND = DASHBOARD_CONTROL_PRND_R_CHOICE;
        break;
        case GearSelector_N:
            controlData->PRND = DASHBOARD_CONTROL_PRND_N_CHOICE;
        break;
        case GearSelector_D:
            controlData->PRND = DASHBOARD_CONTROL_PRND_D_CHOICE;
        break;
        default:
            // gear selector state not defined, don't change the state
        break;
    }

    uint8_t data[DASHBOARD_CONTROL_LENGTH];
    Dashboard_Control_pack(data, controlData, DASHBOARD_CONTROL_LENGTH);

    CAN_TxHeaderTypeDef header = {
        .StdId = DASHBOARD_CONTROL_FRAME_ID,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = DASHBOARD_CONTROL_LENGTH
    };

    if(HAL_CAN_AddTxMessage(hcan, &header, data, &canScheduler.txMailbox) != HAL_OK)
    {
        CAN_state = CAN_TX_ERROR;
    }
}

void CAN_SendLightsFrame(CAN_HandleTypeDef *hcan, struct Dashboard_Lights_t *lightsData, STALK_lState_t stalkLeftState, LightSelector_State_t lightSelectorState)
{
    switch(lightSelectorState)
    {
        case LightSelector_Default:
            lightsData->Headlights = DASHBOARD_LIGHTS_HEADLIGHTS_OFF_CHOICE;
        break;
        case LightSelector_Mode1:
            lightsData->Headlights = DASHBOARD_LIGHTS_HEADLIGHTS_AUTO_CHOICE;
        break;
        case LightSelector_Mode2:
            lightsData->Headlights = DASHBOARD_LIGHTS_HEADLIGHTS_DAY_CHOICE;
        break;
        case LightSelector_Mode3:
            lightsData->Headlights = DASHBOARD_LIGHTS_HEADLIGHTS_NIGHT_CHOICE;
        break;
        default:
            // light selector state not defined, don't change the state
        break;
    }
    switch(stalkLeftState)
    {
        case L_NORMAL:
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_OFF_CHOICE;
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_OFF_CHOICE;
        break;
        case L_BLINK_ONCE:
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_ONCE_CHOICE;
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_OFF_CHOICE;
            break;
        case L_BLINK:
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_ON_CHOICE;
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_OFF_CHOICE;
            break;
        case R_BLINK_ONCE:
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_ONCE_CHOICE;
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_OFF_CHOICE;
            break;
        case R_BLINK:
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_ON_CHOICE;
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_OFF_CHOICE;
            break;
        case HB_ONCE:
        case HB:
            lightsData->Headlights = DASHBOARD_LIGHTS_HEADLIGHTS_HIGHBEAMS_CHOICE;
            break;
    }

    uint8_t data[DASHBOARD_LIGHTS_LENGTH];
    Dashboard_Lights_pack(data, lightsData, DASHBOARD_LIGHTS_LENGTH);

    CAN_TxHeaderTypeDef header = {
        .StdId = DASHBOARD_LIGHTS_FRAME_ID,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = DASHBOARD_LIGHTS_LENGTH
    };

    if(HAL_CAN_AddTxMessage(hcan, &header, data, &canScheduler.txMailbox) != HAL_OK)
    {
        CAN_state = CAN_TX_ERROR;
    }
}

void CAN_SendWipersFrame(CAN_HandleTypeDef *hcan, struct Dashboard_Wipers_t *wipersData, STALK_rState_t stalkRightState)
{
    Dashboard_Wipers_init(wipersData);
    wipersData->Status = stalkRightState;
    
    uint8_t data[DASHBOARD_WIPERS_LENGTH];
    Dashboard_Wipers_pack(data, wipersData, DASHBOARD_WIPERS_LENGTH);

    CAN_TxHeaderTypeDef header = {
        .StdId = DASHBOARD_WIPERS_FRAME_ID,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = DASHBOARD_WIPERS_LENGTH
    };

    if(HAL_CAN_AddTxMessage(hcan, &header, data, &canScheduler.txMailbox) != HAL_OK)
    {
        CAN_state = CAN_TX_ERROR;
    }

}
