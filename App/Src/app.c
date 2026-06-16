#include "main.h"
#include "can_driver.h"

/*
* External variables
*/
extern CAN_HandleTypeDef hcan1;

/*
* CAN
*/
struct CAN_scheduledMsgList canScheduler;


void app_main(void)
{

    CAN_Init(&hcan1);
    while (1)
    {
        CAN_Handle(&hcan1);
    }
}