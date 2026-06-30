#include "main.h"
#include "can_driver.h"
#include "led_driver.h"
#include "stalk.h"
#include "CAN_DB.h"

/*
* External variables
*/
extern CAN_HandleTypeDef hcan1;
extern ADC_HandleTypeDef hadc1;

/*
* CAN
*/
struct CAN_scheduledMsgList canScheduler =
{
    .size = 0,
    .txMailbox = 0
};
struct CAN_IncomingMsgList canRxBuffer =
{
    .head = 0,
    .tail = 0,
    .count = 0,
    .receiveFlag = 0
};

struct Dashboard_Lights_t CAN_lightsData;
struct Dashboard_Control_t CAN_controlData;

/*
* ADC
*/
#define ADC_CHANNELS 7
#define ADC_SAMPLES 10

static uint16_t ADC_buffer[ADC_CHANNELS] = {0};
static float ADC_Voltage[ADC_CHANNELS] = {0};

static volatile uint8_t ADC_ConvCplt = 0;

/*
* LEDs
*/
struct LED LED_RED = {LED_OFF, LED_RED_GPIO_Port, LED_RED_Pin, 0};
struct LED LED_GREEN = {LED_OFF, LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0};

/*
* Private functions prototypes
*/
void ProcessADC1Data(void);
void CAN_SendLightsFrame(struct Dashboard_Lights_t *lightsData, STALK_lState_t stalkLeftState);
void CAN_SendControlFrame(struct Dashboard_Control_t *controlData, GearSelector_State_t gearSelectorState);

/*
* Callbacks
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[CAN_MAX_DLC];
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            Error_Handler();
        }

        if(CAN_AddIncomingMsg(&canRxBuffer, &header, data) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {
        ADC_ConvCplt++;
    }
}

void app_main(void)
{
    CAN_Init(&hcan1);

    STALK_lState_t stalkLeftState = getLeftStalkState(ADC_Voltage[0], ADC_Voltage[1], ADC_Voltage[2]);
    STALK_lState_t newStalkLeftState = getLeftStalkState(ADC_Voltage[0], ADC_Voltage[1], ADC_Voltage[2]);
    STALK_rState_t stalkRightState = getRightStalkState(ADC_Voltage[3], ADC_Voltage[4]);
    STALK_rState_t newStalkRightState = getRightStalkState(ADC_Voltage[3], ADC_Voltage[4]);
    GearSelector_State_t gearSelectorState = getGearSelectorState(ADC_Voltage[6]);
    GearSelector_State_t newGearSelectorState = getGearSelectorState(ADC_Voltage[6]);

    Dashboard_Lights_init(&CAN_lightsData);
    Dashboard_Control_init(&CAN_controlData);
    
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_buffer, ADC_CHANNELS);


    LED_ChangeState(&LED_GREEN, LED_BLINK);
    while (1)
    {
        
        if(ADC_ConvCplt)
        {
            ADC_ConvCplt = 0;
            ProcessADC1Data();
            newStalkLeftState = getLeftStalkState(ADC_Voltage[0], ADC_Voltage[1], ADC_Voltage[2]);
            newStalkRightState = getRightStalkState(ADC_Voltage[3], ADC_Voltage[4]);
            newGearSelectorState = getGearSelectorState(ADC_Voltage[6]);
        }

        if(newStalkLeftState != stalkLeftState)
        {
        	stalkLeftState = newStalkLeftState;
            // SEND DASHBOARD_LIGHTS_FRAME_ID
        	CAN_SendLightsFrame(&CAN_lightsData, stalkLeftState);
        }

        if(newStalkRightState != stalkRightState)
        {
            stalkRightState = newStalkRightState;
            // SEND DASHBOARD_WIPERS_FRAME_ID
            // NOT IMPLEMENTED BCS NOT USED YET
            // CAN_SendWipersFrame(&CAN_wipersData, stalkRightState);
        }

        if(newGearSelectorState != gearSelectorState)
        {
            gearSelectorState = newGearSelectorState;
            CAN_SendControlFrame(&CAN_controlData, gearSelectorState);
        }

        CAN_HandleScheduled(&hcan1, &canScheduler);
        LED_Handle(&LED_GREEN);
        LED_Handle(&LED_RED);
    }
}

void ProcessADC1Data(void)
{
    const float ADC_vRef = 3.3f; // Reference voltage
    const float ADC_resolution = 4096.0f; // 12-bit ADC resolution
    static uint8_t sampleIndex = 0;
    static uint8_t samplesCollected = 0;
    static uint16_t ADC_Samples[ADC_CHANNELS][ADC_SAMPLES] = {0};
    uint16_t ADC_snapshot[ADC_CHANNELS] = {0};
    
    // create a snapshot of the ADC values to avoid race conditions
    for(uint8_t channel = 0; channel < ADC_CHANNELS; channel++)
    {
        ADC_snapshot[channel] = ADC_buffer[channel] & 0x0FFFu;
    }

    for (uint8_t channel = 0; channel < ADC_CHANNELS; channel++)
    {
        ADC_Samples[channel][sampleIndex] = ADC_snapshot[channel];

        if(samplesCollected < ADC_SAMPLES)
        {
            continue;
        }

        // Calculate the average of the samples for each channel (remove max and min for better accuracy)
        uint32_t sum = 0;
        uint16_t min = 0xFFFFu;
        uint16_t max = 0;
        
        for (uint8_t i = 0; i < ADC_SAMPLES; i++)
        {
        	uint16_t sample = ADC_Samples[channel][i];
            sum += sample;
            if (sample < min) min = sample;
            if (sample > max) max = sample;
        }

        float average = 0.0f;
		sum -= (min + max);
        average = (float) sum / (ADC_SAMPLES - 2); 

        ADC_Voltage[channel] = average * (ADC_vRef/ ADC_resolution);
    }

    if(samplesCollected < ADC_SAMPLES)
    {
        samplesCollected++;
    }
    sampleIndex++;
    if (sampleIndex >= ADC_SAMPLES)
    {
        sampleIndex = 0;
    }
}

void CAN_SendControlFrame(struct Dashboard_Control_t *controlData, GearSelector_State_t gearSelectorState)
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
    }

    uint8_t data[DASHBOARD_CONTROL_LENGTH];
    Dashboard_Control_pack(data, controlData, DASHBOARD_CONTROL_LENGTH);

    CAN_TxHeaderTypeDef header = {
        .StdId = DASHBOARD_CONTROL_FRAME_ID,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = DASHBOARD_CONTROL_LENGTH
    };

    if(HAL_CAN_AddTxMessage(&hcan1, &header, data, &canScheduler.txMailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_SendLightsFrame(struct Dashboard_Lights_t *lightsData, STALK_lState_t stalkLeftState)
{
    switch(stalkLeftState)
    {
        case L_NORMAL:
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_OFF_CHOICE;
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_OFF_CHOICE;
            lightsData->Headlights = DASHBOARD_LIGHTS_HEADLIGHTS_OFF_CHOICE;
        break;
        case L_BLINK_ONCE:
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_ONCE_CHOICE;
            break;
        case L_BLINK:
            lightsData->TurnSignal_Left = DASHBOARD_LIGHTS_TURNSIGNAL_LEFT_ON_CHOICE;
            break;
        case R_BLINK_ONCE:
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_ONCE_CHOICE;
            break;
        case R_BLINK:
            lightsData->TurnSignal_Right = DASHBOARD_LIGHTS_TURNSIGNAL_RIGHT_ON_CHOICE;
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

    if(HAL_CAN_AddTxMessage(&hcan1, &header, data, &canScheduler.txMailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_SendWipersFrame(struct Dashboard_Wipers_t *wipersData, STALK_rState_t stalkRightState)
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

    if(HAL_CAN_AddTxMessage(&hcan1, &header, data, &canScheduler.txMailbox) != HAL_OK)
    {
        Error_Handler();
    }

}
