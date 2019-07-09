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
#include "sh2_hal.h"
#include "sh2_err.h"

#define ADDR_SH2_0          0x4A
#define RESET_DELAY_MS      10
#define STARTUP_DELAY_MS    2000
//Number of bytes to read to determine length
#define READ_LEN            2

enum BusState_e {
    BUS_INIT,
    BUS_IDLE,
    BUS_READING_LEN,
    BUS_GOT_LEN,
    BUS_READING_TRANSFER,
    BUS_WRITING,
    BUS_READING_DFU,
    BUS_WRITING_DFU,
};
static uint32_t current_time = 0;
volatile uint32_t rx_timestamp_us;            // timestamp of INTN event


enum BusState_e I2C_state;

// Receive Buffer
static uint8_t rx_buf[SH2_HAL_MAX_TRANSFER_IN];      // data
static uint32_t rx_buf_len;   // valid bytes stored in rxBuf (0 when buf empty)
static uint16_t payload_len;

// Transmit buffer
static uint8_t tx_buf[SH2_HAL_MAX_TRANSFER_OUT];

static uint32_t discards = 0;

static uint8_t IS_OPEN = 0;
static uint8_t IMU_READY = 0;

// True after INTN observed, until read starts
static uint8_t RX_DATA_READY = 0;


/* Time Keeping Code */
CY_ISR(Time_Handler){
    //Every time the timer completes, add to the time counter
    current_time += TIMER_US_ReadPeriod();
}

static uint32_t get_timestamp(){
    return current_time + (uint32_t)TIMER_US_ReadCounter;
}

static uint32_t imu_read(uint32_t addr, uint8_t *buff, uint16_t len){
    uint32_t ct = 0;
    uint32_t result = 0; 
    do{
        result = IMU_I2C_MasterReadBuf(addr, buff, len, IMU_I2C_MODE_COMPLETE_XFER);

        while (0u == (IMU_I2C_MasterStatus() & IMU_I2C_MSTAT_RD_CMPLT))
        {
            CyDelay(1);
            ct++;
            if(ct > 100) return 0;
        }
    }while(result != IMU_I2C_MSTR_NO_ERROR);
    return IMU_I2C_MasterGetReadBufSize();
}


/* SHTP HAL */
static uint16_t imu_get_message(){
    if(rx_buf_len > 0){
        discards++;
        rx_buf_len = 0;
    }
    
    I2C_state = BUS_READING_LEN;
    //Read the length of the packet
    imu_read(ADDR_SH2_0, rx_buf, READ_LEN);
    uint16_t len = (rx_buf[0] + (rx_buf[1] << 8)) & ~0x8000;
    
    I2C_state = BUS_GOT_LEN;
    
    if(len > SH2_HAL_MAX_TRANSFER_IN){
        payload_len = SH2_HAL_MAX_TRANSFER_IN;
    } else {
        payload_len = len;
    }
    
    I2C_state = BUS_READING_TRANSFER;
    imu_read(ADDR_SH2_0, rx_buf, payload_len);
    
    I2C_state = BUS_IDLE;
    rx_buf_len = payload_len;
    
    return rx_buf_len;
}

CY_ISR(IMU_ISR_Handler){
    /* If not initialized yet, exit */
    if(I2C_state == BUS_INIT){
        //IMU_INT_ClearInterrupt();
        return;
    }
    
    rx_timestamp_us = get_timestamp(); //FIX
    /*if(IMU_READY){
        if(I2C_state == BUS_IDLE){
            imu_get_message();
            RX_DATA_READY = 0;
        } else {
            RX_DATA_READY = 1;
        }
    } else {
        RX_DATA_READY = 1;
        IMU_READY = 1;
    }*/
    IMU_READY = 1;
    RX_DATA_READY = 1;
       
    //IMU_INT_ClearInterrupt();

    
}


//static void imu_set_reset(uint8_t val){
//    IMU_RESET_Write(val);
//}

static void imu_i2c_startup(void){
    IMU_I2C_Start();
    IMU_ISR_StartEx(IMU_ISR_Handler);
    //STAMP_ISR_StartEx(Time_Handler);
    TIMER_US_Start();
}

static void imu_i2c_stop(void){
    IMU_ISR_Stop();
    IMU_I2C_Stop();
}

//static void imu_set_boot(uint8_t val){
//    IMU_BOOT_Write(val);
//}

static void imu_disable_ints(void){
    IMU_ISR_Disable();
}

static void imu_enable_ints(void){
    IMU_ISR_Enable();
}

static int sh2_i2c_hal_open(sh2_Hal_t *self){
    //If the port is open already, return an error
    if(IS_OPEN){
        return SH2_ERR;
    }
    
    /* Prepare flags */
    IS_OPEN = 1;
    I2C_state = BUS_INIT;
    
    /* Initialize the I2C module */
    imu_i2c_startup();
    I2C_state = BUS_IDLE;
    
    /* Send the IMU a reset signal */
    //imu_set_reset(0);
    IMU_READY = 0; //Flag that the IMU is in reset
//    imu_set_boot(1); //Tell the IMU to not enter bootloader mode
    CyDelay(RESET_DELAY_MS); //Delay to make sure the reset takes
    //imu_set_reset(1); //Deassert the reset pin
    
    
    
    //Wait for the IMU to be ready for at most STARTUP_DELAY_MS
    uint16_t count = 0;
    while(!IMU_READY && count < STARTUP_DELAY_MS){
        count++;
        CyDelay(1);
    }
    
    //Prepare for the first read
    //RX_DATA_READY = 1;
    
    //If we haven't timed out, return SH2_OK
    if(count < STARTUP_DELAY_MS)
        return SH2_OK;
    else
        return SH2_ERR;
}

static void sh2_i2c_hal_close(sh2_Hal_t *self){
    //Reset the IMU
    //imu_set_reset(0);
    //imu_set_boot(1);
    
    I2C_state = BUS_INIT;
    
    //Stop the API
    imu_i2c_stop();
    
    //Mark that the port is closed
    IS_OPEN = 0;
}

static int sh2_i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t){
    int retval = 0;
    
    imu_disable_ints();
    
    if(rx_buf_len > 0){
        if(rx_buf_len > len){
            retval = 0;
            return retval;
        }
        
        memcpy(pBuffer, rx_buf, payload_len);
        retval = rx_buf_len;
        rx_buf_len = 0;
    } else if (RX_DATA_READY){
        retval = imu_get_message();
        
        
        if(rx_buf_len > len){
            retval = 0;
            return retval;
        }
        
        memcpy(pBuffer, rx_buf, rx_buf_len);
        rx_buf_len = 0;
        RX_DATA_READY = 0;
    } else {
        retval = 0;
    }
    
    imu_enable_ints();
    *t = get_timestamp();
    
    return retval;
    
}

static int sh2_i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len){
    int retval = 0;
    
    if ((pBuffer == 0) || (len == 0) || (len > SH2_HAL_MAX_TRANSFER_OUT)){
        return SH2_ERR_BAD_PARAM;
    }
    
    //Disable interrupts temporarily to prevent the bus state from changing
    imu_disable_ints();
    
    if(I2C_state == BUS_IDLE){
        I2C_state = BUS_WRITING;
        
        memcpy(tx_buf, pBuffer, len);
        IMU_I2C_MasterWriteBuf(ADDR_SH2_0, tx_buf, len, IMU_I2C_MODE_COMPLETE_XFER);
        retval = len;
    }
    
    imu_enable_ints();
    return retval;
}

static sh2_Hal_t sh2Hal;

sh2_Hal_t *sh2_hal_init(void){
    sh2Hal.open = sh2_i2c_hal_open;
    sh2Hal.close = sh2_i2c_hal_close;
    sh2Hal.read = sh2_i2c_hal_read;
    sh2Hal.write = sh2_i2c_hal_write;
    sh2Hal.getTimeUs = get_timestamp;
    
    return &sh2Hal;
}


/* [] END OF FILE */
