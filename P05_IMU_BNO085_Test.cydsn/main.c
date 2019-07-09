/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "include/IMU_BNO085.h"
#include <stdio.h> 


#define BUZZER_TIMER_CLK 8000000
uint16 buzzer_freq = 20;
uint16 buzzer_clock_div = 2;
uint16 buzzer_timer_peorid = 1;
uint16 buzzer_timer_cmp1 = 0;

uint16 buzzer_chirp_dir = 1;

// IMU
#define USBFS_DEVICE    (0u)

/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define USBUART_BUFFER_SIZE (64u)
#define LINE_STR_LENGTH     (20u)

uint8_t IMU_READY = 0;

/*
CY_ISR(IMU_ISR_Handler){
    
    
    IMU_READY = 1;
    //USBUART_PutChar('l');
    IMU_INT_ClearInterrupt();
}
*/

uint16_t cnt = 4;
uint8_t buff[64];
    
sh2_Hal_t *pSh2Hal;    

static void start_reports(){
    static sh2_SensorConfig_t config;
    int status;
    int sensorID;
    
    static const int enabledSensors[] = {
        SH2_GAME_ROTATION_VECTOR
    };
    
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    // Select a report interval.
    config.reportInterval_us = 10000;  // microseconds (100Hz)
    // config.reportInterval_us = 2500;   // microseconds (400Hz)
    // config.reportInterval_us = 1000;   // microseconds (1000Hz)

    for (unsigned int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorID = enabledSensors[n];
        status = sh2_setSensorConfig(sensorID, &config);
        if (status != 0) {
            return;
        }
    }
}

static sh2_SensorEvent_t sensor_event;
//static float i = 0.0;
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent){
    //for(int i = 0; i < 5; i++){}
    sensor_event = *pEvent;
    
    
    //int len = sprintf(output, "%4.2f\n", 0.324f);
    
    //USBUART_PutString(output);

    return;
}

static sh2_AsyncEvent_t async_event;
static void eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent){
    for(int i = 0; i < 5; i++){}
    async_event = *pEvent;
    return;
}

#define USB_STARTUP         0
#define IMU_STARTUP         1
#define DATA_TRANSM         2

uint8_t state = USB_STARTUP;
static char output_buff [64];

//Main loop

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    PWM_LED_Start();
    PWM_EN_Start();
    PWM_BUZZER_Start();
    

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	
	USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    
    for(;;)
    {
        switch(state)
        {
            case USB_STARTUP:
                if (0u != USBUART_IsConfigurationChanged()){
                    /* Initialize IN endpoints when device is configured. */
                    if (0u != USBUART_GetConfiguration()){
                        /* Enumeration is done, enable OUT endpoint to receive data 
                         * from host. */
                        USBUART_CDC_Init();
                    }
                }

                /* If we are ready, move to the next stage */
                if (0u != USBUART_GetConfiguration()){                    
                    state = IMU_STARTUP;
                }
                
                
            break;
            
            case IMU_STARTUP:
                pSh2Hal = sh2_hal_init();
                int status = sh2_open(pSh2Hal, eventHandler, NULL);
                sh2_setSensorCallback(sensorHandler, NULL);
                start_reports();
                
                state = DATA_TRANSM;
            break;
            
            case DATA_TRANSM:
                sh2_service();
                
                //sprintf(output_buff, "%4.2f\n", i);
                    /*while (0u == USBUART_CDCIsReady())
                            {
                            }*/
                    //USBUART_PutString("hi");
                //USBUART_PutString(output_buff);
                
                sh2_SensorValue_t value;
                // Convert event to value
                sh2_decodeSensorEvent(&value, &sensor_event);
                
                //char output[30] = {0};
                //i = value.un.gameRotationVector.i;
    
                
                float i = value.un.gameRotationVector.i;
                float j = value.un.gameRotationVector.j;
                float k = value.un.gameRotationVector.k;
                float r = value.un.gameRotationVector.real;
                
                //sprintf(output_buff, "Val: %.3f \n", value.un.gameRotationVector.i);
                
                sprintf(output_buff, "i: %.2f \tj: %.2f \ti: %.2f \treal: %.2f \n", i, j, k, r);
                USBUART_PutString(output_buff);
                //USBUART_PutString("Doing Stuff\n");
            break;
        }
        
   
    }

    for(;;)
    {
        /* Place your application code here. */
        //LED_R_Write(~KEY1_Read());
        //LED_G_Write(~KEY2_Read());
        
        // Start Buzzer
        buzzer_freq = 4000;
        buzzer_clock_div = BUZZER_TIMER_CLK / buzzer_freq;
        buzzer_timer_peorid = buzzer_clock_div - 1;
        buzzer_timer_cmp1 = buzzer_clock_div/2-1;
        PWM_BUZZER_WritePeriod(buzzer_timer_peorid);
        PWM_BUZZER_WriteCompare(buzzer_timer_cmp1);

    }
}


/* [] END OF FILE */
