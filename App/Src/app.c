#include "main.h"
#include "can_driver.h"
#include "led_driver.h"
#include "stalk.h"
#include "CAN_DB.h"
#include "app_can.h"

/*
* External variables
*/
extern CAN_HandleTypeDef hcan1;
extern ADC_HandleTypeDef hadc1;

/*
* CAN
*/
volatile CAN_State_t CAN_state = CAN_OK;

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
struct Dashboard_Wipers_t CAN_wipersData;
struct Dashboard_NODE_t CAN_nodeData;

/*
* ADC
*/
#define ADC_CHANNELS 7
#define ADC_SAMPLES 10

volatile static uint16_t ADC_buffer[ADC_CHANNELS] = {0};
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
static uint8_t ProcessADC1Data(void);

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
            return;
        }

        if(CAN_AddIncomingMsg(&canRxBuffer, &header, data) != HAL_OK)
        {
            return;
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

    // Initialize states
    STALK_lState_t stalkLeftState = L_NORMAL;
    STALK_lState_t newStalkLeftState = L_NORMAL;
    STALK_rState_t stalkRightState = R_NORMAL;
    STALK_rState_t newStalkRightState = R_NORMAL;
    GearSelector_State_t gearSelectorState = GearSelector_P;
    GearSelector_State_t newGearSelectorState = GearSelector_P;
    LightSelector_State_t lightSelectorState = LightSelector_Default;
    LightSelector_State_t newLightSelectorState = LightSelector_Default;
    GPIO_PinState emergencyButtonState = GPIO_PIN_RESET;
    GPIO_PinState modeButtonState = GPIO_PIN_RESET;

    // initialize CAN rx buffers
    Dashboard_Lights_init(&CAN_lightsData);
    Dashboard_Control_init(&CAN_controlData);
    Dashboard_Wipers_init(&CAN_wipersData);
 //   CAN_ScheduleNodeFrame(&canScheduler, &CAN_nodeData);
    
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_buffer, ADC_CHANNELS);

    LED_ChangeState(&LED_GREEN, LED_BLINK);
    while (1)
    {   
        // ADC handling
        if(ADC_ConvCplt)
        {
            ADC_ConvCplt = 0;
            if (ProcessADC1Data())
            {
                newStalkLeftState = getLeftStalkState(ADC_Voltage[ADC_STALK_L1], ADC_Voltage[ADC_STALK_L2], ADC_Voltage[ADC_STALK_L3]);
                newStalkRightState = getRightStalkState(ADC_Voltage[ADC_STALK_R1], ADC_Voltage[ADC_STALK_R2]);
                newGearSelectorState = getGearSelectorState(ADC_Voltage[ADC_GEAR]);
                newLightSelectorState = getLightSelectorState(ADC_Voltage[ADC_LIGHT]);
            }
        }

        // stalk state handling
        if(newStalkLeftState != stalkLeftState || newLightSelectorState != lightSelectorState)
        {
        	stalkLeftState = newStalkLeftState;
            lightSelectorState = newLightSelectorState;
            // SEND DASHBOARD_LIGHTS_FRAME_ID
        	CAN_SendLightsFrame(&hcan1, &CAN_lightsData, stalkLeftState, lightSelectorState, emergencyButtonState);
        }

        if(newStalkRightState != stalkRightState)
        {
            stalkRightState = newStalkRightState;
            // READY, DELETE COMMENTS
            // NOT USED YET
             CAN_SendWipersFrame(&hcan1, &CAN_wipersData, stalkRightState);
        }

        if(newGearSelectorState != gearSelectorState)
        {
            gearSelectorState = newGearSelectorState;
            CAN_SendControlFrame(&hcan1, &CAN_controlData, gearSelectorState, modeButtonState);
        }

        if(emergencyButtonState != HAL_GPIO_ReadPin(EMERGENCY_PIN_GPIO_Port, EMERGENCY_PIN_Pin))
        {
            emergencyButtonState = HAL_GPIO_ReadPin(EMERGENCY_PIN_GPIO_Port, EMERGENCY_PIN_Pin);
            CAN_SendLightsFrame(&hcan1, &CAN_lightsData, stalkLeftState, lightSelectorState, emergencyButtonState);
        }

        if(modeButtonState != HAL_GPIO_ReadPin(MODE_PIN_GPIO_Port, MODE_PIN_Pin))
        {
            modeButtonState = HAL_GPIO_ReadPin(MODE_PIN_GPIO_Port, MODE_PIN_Pin);
            CAN_SendControlFrame(&hcan1, &CAN_controlData, gearSelectorState, modeButtonState);
        }

        // CAN / DEBUG LED handling
        //handleCanRx(&canRxBuffer);
        if (CAN_state != CAN_OK) {
            if (LED_RED.state != LED_BLINK) {
                LED_ChangeState(&LED_RED, LED_BLINK);  // once, on transition
            }
        } else if (LED_RED.state != LED_OFF) {
            LED_ChangeState(&LED_RED, LED_OFF);        // once, on clear
        }
        CAN_HandleScheduled(&hcan1, &canScheduler);
        LED_Handle(&LED_GREEN);
        LED_Handle(&LED_RED);
    }
}

static uint8_t ProcessADC1Data(void)
{
    const float ADC_vRef = 3.3f; // Reference voltage
    const float ADC_resolution = 4096.0f; // 12-bit ADC resolution
    static uint8_t sampleIndex = 0;
    static uint8_t samplesCollected = 0;
    static uint16_t ADC_Samples[ADC_CHANNELS][ADC_SAMPLES] = {0};
    uint16_t ADC_snapshot[ADC_CHANNELS] = {0};

    __disable_irq(); // Disable interrupts to prevent race conditions while
    // create a snapshot of the ADC values (mask to keep only 12 bits)
    for(uint8_t channel = 0; channel < ADC_CHANNELS; channel++)
    {
        ADC_snapshot[channel] = ADC_buffer[channel] & 0x0FFFu;
    }
    __enable_irq(); // Re-enable interrupts after snapshot is taken

    for (uint8_t channel = 0; channel < ADC_CHANNELS; channel++)
    {
        ADC_Samples[channel][sampleIndex] = ADC_snapshot[channel];

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

		sum -= (min + max);
        float average = (float) sum / (ADC_SAMPLES - 2); 

        ADC_Voltage[channel] = average * (ADC_vRef/ ADC_resolution);
    }

    sampleIndex++;
    if (sampleIndex >= ADC_SAMPLES)
    {
        sampleIndex = 0;
    }

    // usage not allowed without ADC_SAMPLES samples
    if(samplesCollected < ADC_SAMPLES)
    {
        samplesCollected++;
        return 0; // voltages cannot be updated yet, try again later
    }
    
    return 1; // voltages can be updated now
}
