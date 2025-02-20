/*
 * Application program for CANopen IO device on Max32 board with PIC32
 *
 * @file        appl_max32_demo.c
 * @author      --
 * @copyright   2021 --
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


/** 
 * Modified version, by EF.
 * The LEDs are re-mapped:
 * RUN_LED is set as LED_TESTDEV
 * ERROR_LED is set as LED_ERROR although it may collide with current. TODO: another pin? or just a dummy variable?
 */

#include "CO_application.h"
#include "OD.h"

#include "device.h" // LEDs!

/* CANopen LED diodes, already initialized in DEV_Init(), renamed here for easier integration. */
#define CAN_INIT_LEDS() _nop()
//#define CAN_RUN_LED     LED_TESTDEV_WR
#define CAN_RUN_LED     LED_ERROR_WR
#define CAN_ERROR_LED   LED_ERROR_WR


/******************************************************************************/
CO_ReturnError_t app_programStart(uint16_t *bitRate,
                                  uint8_t *nodeId,
                                  uint32_t *errInfo)
{
    /* CANopen led diodes */
//    CAN_INIT_LEDS(); // Already done in device.h
    CAN_RUN_LED     = 0;
    CAN_ERROR_LED   = 1;

    /* Place for peripheral or any other startup configuration. See main_PIC32.c
     * for defaults. */

    /* Set initial CAN bitRate and CANopen nodeId. May be configured by LSS. */
    if (*bitRate == 0) *bitRate = 250;
    if (*nodeId == 0) *nodeId = 0x30;

    return CO_ERROR_NO;
}


/******************************************************************************/
void app_communicationReset(CO_t *co) {
    
    if (!co->nodeIdUnconfigured) {

    }
}


/******************************************************************************/
void app_programEnd() {
    CAN_RUN_LED = 0; CAN_ERROR_LED = 0;
}


/******************************************************************************/
void app_programAsync(CO_t *co, uint32_t timer1usDiff) {
    /* Here can be slower code, all must be non-blocking. Mind race conditions
     * between this functions and following three functions, which all run from
     * realtime timer interrupt */
    
   
    // testing sending one message, e.g. NMT START to COB-ID 32
    static bool IsNmtOp = false;
    static bool IsDenOperationModeSet = false;
    if( false == IsNmtOp )
    {
        static unsigned int count = 0U;
        
        if( 0U == count )
        {
            co->NMT->HB_TXbuff->CMSGSID = 0x00; // NMT COB-ID is 0x00 always!
            co->NMT->HB_TXbuff->CMSGEID = 0x02; // payload length is placed here
            co->NMT->HB_TXbuff->data[0] = (uint8_t)0x01; // Start remote node command
            co->NMT->HB_TXbuff->data[1] = (uint8_t)0x03; // W4 Node ID
            (void)CO_CANsend(co->NMT->HB_CANdevTx, co->NMT->HB_TXbuff);            
        }
        
        else if( 1000U == count )
        {
            co->NMT->HB_TXbuff->CMSGSID = 0x00; // NMT COB-ID is 0x00 always!
            co->NMT->HB_TXbuff->CMSGEID = 0x02; // payload length is placed here
            co->NMT->HB_TXbuff->data[0] = (uint8_t)0x01; // Start remote node command
            co->NMT->HB_TXbuff->data[1] = (uint8_t)0x04; // W4 Node ID
            (void)CO_CANsend(co->NMT->HB_CANdevTx, co->NMT->HB_TXbuff);            
        }
        
        else if( 2000U == count )
        {
            co->NMT->HB_TXbuff->CMSGSID = 0x00; // NMT COB-ID is 0x00 always!
            co->NMT->HB_TXbuff->CMSGEID = 0x02; // payload length is placed here
            co->NMT->HB_TXbuff->data[0] = (uint8_t)0x01; // Start remote node command
            co->NMT->HB_TXbuff->data[1] = (uint8_t)0x05; // W5 Node ID
            (void)CO_CANsend(co->NMT->HB_CANdevTx, co->NMT->HB_TXbuff);            
        }
        
        else if( 3000U == count )
        {
            co->NMT->HB_TXbuff->CMSGSID = 0x00; // NMT COB-ID is 0x00 always!
            co->NMT->HB_TXbuff->CMSGEID = 0x02; // payload length is placed here
            co->NMT->HB_TXbuff->data[0] = (uint8_t)0x01; // Start remote node command
            co->NMT->HB_TXbuff->data[1] = (uint8_t)0x06; // W6 Node ID
            (void)CO_CANsend(co->NMT->HB_CANdevTx, co->NMT->HB_TXbuff);    
            
            IsNmtOp = true; // end
        }

        else { } // good practice
        
        ++count;
    }
    
    else if( false == IsDenOperationModeSet ) // send operation mode
    {
        static unsigned int it = 0U;
        
        if( 0U == it )
        {
            co->SDOclient->CANtxBuff->CMSGSID = 0x600 + 0x03;               // SDO to W3
            co->SDOclient->CANtxBuff->CMSGEID = 0x06;                       // payload length is placed here
            co->SDOclient->CANtxBuff->data[0] = (uint8_t)(0b1 << 5) | (0b10 << 2) | (0b1 <<1) | 0b1;  // Download | 2bytes not used | expedited | bytes not used selector flag
            co->SDOclient->CANtxBuff->data[1] = (uint8_t)0x14;              // index LSB - Operation mode 
            co->SDOclient->CANtxBuff->data[2] = (uint8_t)0x20;              // index MSB - Operation mode 
            co->SDOclient->CANtxBuff->data[3] = (uint8_t)0x00;              // subindex 
#ifdef CANOPEN_INTERMEDIATE_MAPPING_CO3_RIGHT_SIDE_HARDCODED_AS_REPO
//            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4 | (1 << 4);      // Position profiler
            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4U;                 // Position - no latch required
#else // hard-coded as TELEOP
            co->SDOclient->CANtxBuff->data[4] = (uint8_t)2U;                 // Current - no latch required - so it helps in motion
#endif
            (void)CO_CANsend(co->SDOclient->CANdevTx, co->SDOclient->CANtxBuff);            
        }
        
        else if( 100U == it )
        {
            co->SDOclient->CANtxBuff->CMSGSID = 0x600 + 0x04;       // SDO to W4
            co->SDOclient->CANtxBuff->CMSGEID = 0x06;               // payload length is placed here
            co->SDOclient->CANtxBuff->data[0] = (uint8_t)(0b1 << 5) | (0b10 << 2) | (0b1 <<1) | 0b1;  // Download | 2bytes not used | expedited | bytes not used selector flag
            co->SDOclient->CANtxBuff->data[1] = (uint8_t)0x14;            // index LSB - Operation mode 
            co->SDOclient->CANtxBuff->data[2] = (uint8_t)0x20;            // index MSB - Operation mode 
            co->SDOclient->CANtxBuff->data[3] = (uint8_t)0x00;            // subindex 
#ifdef CANOPEN_INTERMEDIATE_MAPPING_CO3_RIGHT_SIDE_HARDCODED_AS_REPO
//            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4 | (1 << 4);      // Position profiler
            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4U;                 // Position - no latch required
#else // hard-coded as TELEOP
            co->SDOclient->CANtxBuff->data[4] = (uint8_t)2U;                 // Current - no latch required - weight compensation
#endif    
            (void)CO_CANsend(co->SDOclient->CANdevTx, co->SDOclient->CANtxBuff);              
        }
        
        else if( 200U == it )
        {
            co->SDOclient->CANtxBuff->CMSGSID = 0x600 + 0x05;       // SDO to W5
            co->SDOclient->CANtxBuff->CMSGEID = 0x06;               // payload length is placed here
            co->SDOclient->CANtxBuff->data[0] = (uint8_t)(0b1 << 5) | (0b10 << 2) | (0b1 <<1) | 0b1;  // Download | 2bytes not used | expedited | bytes not used selector flag
            co->SDOclient->CANtxBuff->data[1] = (uint8_t)0x14;            // index LSB - Operation mode 
            co->SDOclient->CANtxBuff->data[2] = (uint8_t)0x20;            // index MSB - Operation mode 
            co->SDOclient->CANtxBuff->data[3] = (uint8_t)0x00;            // subindex 
//            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4 | (1 << 4);      // Position profiler
            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4U;                 // Position - no latch required - both in TELEOP and REPO modes! special demonstrator case
            (void)CO_CANsend(co->SDOclient->CANdevTx, co->SDOclient->CANtxBuff);
        }
        
        else if( 300U == it )
        {
            co->SDOclient->CANtxBuff->CMSGSID = 0x600 + 0x06;       // SDO to W6
            co->SDOclient->CANtxBuff->CMSGEID = 0x06;               // payload length is placed here
            co->SDOclient->CANtxBuff->data[0] = (uint8_t)(0b1 << 5) | (0b10 << 2) | (0b1 <<1) | 0b1;  // Download | 2bytes not used | expedited | bytes not used selector flag
            co->SDOclient->CANtxBuff->data[1] = (uint8_t)0x14;            // index LSB - Operation mode 
            co->SDOclient->CANtxBuff->data[2] = (uint8_t)0x20;            // index MSB - Operation mode 
            co->SDOclient->CANtxBuff->data[3] = (uint8_t)0x00;            // subindex 
#ifdef CANOPEN_INTERMEDIATE_MAPPING_CO3_RIGHT_SIDE_HARDCODED_AS_REPO
//            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4 | (1 << 4);      // Position profiler
            co->SDOclient->CANtxBuff->data[4] = (uint8_t)4U;                 // Position - no latch required
#else // hard-coded as TELEOP
            co->SDOclient->CANtxBuff->data[4] = (uint8_t)2U;                 // Current - no latch required - very low values just to help in dynamics
#endif      
            (void)CO_CANsend(co->SDOclient->CANdevTx, co->SDOclient->CANtxBuff);      
            
            IsDenOperationModeSet = true; // end
        }

        else { } // good practice
        
        ++it;
    }
    
    else { } // good practice
}


/******************************************************************************/
void app_programRt(CO_t *co, uint32_t timer1usDiff) {

    // DO NOT DO ANYTHING HERE (IF POSSIBLE), JUST FOR SIMPLE TESTING
    // 0.Functional demonstrator!!
//    OD_entry_t* odEntry = OD_find(OD, *OD_ENTRY_H2000_W3DenaliTPDO.index ); // OD is a global pointer, testing with its own RPDO1
//    
//    char val[4]; // testing 4-bytes
//    OD_size_t len = 2U;
//    ODR_t odrValue = OD_get_value(odEntry, 0x01, (void*)val, len, false); // or TRUE, test it later if it fails...
    // end (0)

//    void* ptr = &OD_PERSIST_COMM.x2001_W3DenaliRPDO;
//    
//    // 1. Updates TPDO simple version!
//    OD_PERSIST_COMM.x2001_W3DenaliRPDO.digitalOutputs = 1U;
//    
    // Real code
    // 1. Basic coding...
    // 2. Copy all data to be accessed later on via the CANOpenManager?
 //   x2000_W3DenaliTPDO testOd;
//    OD_PERSIST_COMM_t
//    &OD->list[49];
    
//    OD_ENTRY_H2000_W3DenaliTPDO.index;
}


/******************************************************************************/
void app_peripheralRead(CO_t *co, uint32_t timer1usDiff) {

    /* All analog inputs must be read or interrupt source for RT thread won't be
     * cleared. See analog input configuration in main_PIC32.c */
//    volatile uint32_t dummyRead;
//    dummyRead = ADC1BUF0;
//    dummyRead = ADC1BUF1;
//    dummyRead = ADC1BUF2;
//    dummyRead = ADC1BUF3;
//    dummyRead = ADC1BUF4;
//    dummyRead = ADC1BUF5;
//    dummyRead = ADC1BUF6;
//    dummyRead = ADC1BUF7;
//    dummyRead = ADC1BUF8;
//    dummyRead = ADC1BUF9;
//    dummyRead = ADC1BUFA;
//    dummyRead = ADC1BUFB;
//    dummyRead = ADC1BUFC;
//    dummyRead = ADC1BUFD;
//    dummyRead = ADC1BUFE;
//    dummyRead = ADC1BUFF;
    //OD_RAM.x6401_readAnalogInput_16_bit[0xF] = ADC1BUFF;

    /* Read digital inputs */
    //uint8_t digIn = 0;
    //if(PORTDbits.RD6 != 0) digIn |= 0x08;
    //if(PORTDbits.RD7 != 0) digIn |= 0x04;
    //if(PORTDbits.RD13 != 0) digIn |= 0x01;
    //OD_RAM.x6000_readDigitalInput_8_bit[0] = digIn;
}


/******************************************************************************/
void app_peripheralWrite(CO_t *co, uint32_t timer1usDiff) {
    CAN_RUN_LED     = CO_LED_GREEN(co->LEDs, CO_LED_CANopen);
    CAN_ERROR_LED   = CO_LED_RED(co->LEDs, CO_LED_CANopen);

    /* Write to digital outputs */
    //uint8_t digOut = OD_RAM.x6200_writeDigitalOutput_8_bit[0];
    //LATAbits.LATA0 = (digOut & 0x01) ? 1 : 0;
    //LATAbits.LATA1 = (digOut & 0x02) ? 1 : 0;
    //LATAbits.LATA4 = (digOut & 0x10) ? 1 : 0;
    //LATAbits.LATA5 = (digOut & 0x20) ? 1 : 0;
    //LATAbits.LATA6 = (digOut & 0x40) ? 1 : 0;
    //LATAbits.LATA7 = (digOut & 0x80) ? 1 : 0;
}
