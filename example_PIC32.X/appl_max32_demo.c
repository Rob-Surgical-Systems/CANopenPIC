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
#define CAN_RUN_LED     LED_TEST_WR
#define CAN_ERROR_LED   LED_ERROR_WR

enum SdoUploadFrame_t                       /// The SDO upload frames sent periodically to each Denali Wx driver.
{
    SDO_UPLOAD_BUS_VOLTAGE = 0,             ///< 0x2060.
    SDO_UPLOAD_POWER_STAGE_TEMPERATURE,     ///< 0x2061.
    SDO_UPLOAD_SYSTEM_LAST_ERROR,           ///< 0x5e49.
    SDO_UPLOAD_STO_STATUS,                  ///< 0x251a.
    SDO_UPLOAD_ERROR_TOTAL_NUMBER,          ///< 0x264d.
    SDO_UPLOAD_TOTAL,                       ///< The total number of elements.
};

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
   
    // 1. NMT START to each CAN ID
    static bool IsNmtOp = false;
    
    if( false == IsNmtOp )
    {
        static unsigned int count = 0;
        
        if( 20000U == count )
        {
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 0x02);    
}

//        else if( 20100U == count )
//        {
//            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 0x04);
//        }
//
//        else if( 20200U == count )
//        {
//            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 0x05);  
//        }
//        
//        else if( 20300U == count )
//        {
//            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 0x06);
//            
//            IsNmtOp = true; // end
//        }

        else { } // good practice
        
        ++count;
        
        return;
    }
    
    else
    {
        // 1.1 NMT auto-recovery, this is the NMT master
//        if ( ( CO_NMT_OPERATIONAL != co->NMT->operatingState ) && ( CO_NMT_OPERATIONAL == co->NMT->operatingStatePrev ) ) // it may fail if many consecutive "fails" happen
        if ( CO_NMT_OPERATIONAL != co->NMT->operatingState ) // simplest, hotfix
        {
            CO_NMT_sendInternalCommand( co->NMT, CO_NMT_OPERATIONAL);
        }
        
        static bool isReset = false;
        // 1.2. Denali & CO3 recovery from EMCY - so simple, just a test!        
        
        if ( true == isReset )
        {
            isReset = false;
            CO_NMT_sendInternalCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL);           // 1.2.1. second, recover to OP, just itself or...
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 2);                 // to all nodes? including itself
        }
        
        if ( 0 != co->CANmodule->CANerrorStatus ) // checks flag bits - any        
        {
            co->CANmodule->CANerrorStatus = 0; // needed? wrong?

            CO_NMT_sendInternalCommand( co->NMT, CO_NMT_RESET_COMMUNICATION);           // 1.2.2. first, reset comms      
            isReset = true;
        }
                    
        // 2. Cyclic SDO - Simple FSM for SDO upload frames iteration, which are directly related to the EtherCAT PDI
        static enum SdoUploadFrame_t sdoFrame       = SDO_UPLOAD_BUS_VOLTAGE;   // current fsm frame
        static enum SdoUploadFrame_t sdoFrameNext   = SDO_UPLOAD_BUS_VOLTAGE;   // next fsm frame

        const unsigned char DECIMATOR_MAX   = 60U;                       // 60 iterations @ 5 [ms] = 300 [ms] between messages
        static unsigned char decimator      = 60U;                       // the first one should not be too early... after NMT at least...
        // COB-ID for each Wx, all SDO uploads but not with the same payload lenght, index or subindex.
        const unsigned short WX_CAN_ID_MAX  = 6U; // W6
        const unsigned short WX_CAN_ID_MIN  = 3U; // W3 
        static unsigned short wxId          = 3U;

        uint16_t idx        = 0x2060;           // default for SDO_UPLOAD_BUS_VOLTAGE SDO frame
        uint8_t subidx      = 0x00;             // always zero...

        CO_SDO_abortCode_t  abortCode;
        size_t              sizeTransferred;
        CO_SDO_return_t     SDO_ret;
        uint32_t            timeDiff_us = 1000U;
        uint32_t            timeNext_us = 2000U;

        bool updateWxId = false;                // set to TRUE either via Rx message or timeout
        static bool isSdoClientReady = false;   // true when setup/configured and initialized

        sdoFrame = sdoFrameNext; // updates FSM

        if ( ( false == isSdoClientReady ) && ( --decimator < 1U ) ) // 2.1., note decimator only evaluated&decremented if SDO client is not ready
        {
            decimator = DECIMATOR_MAX; // restart!

            switch ( sdoFrame )
            {
                case SDO_UPLOAD_BUS_VOLTAGE:
                    idx         = 0x2060;                
                    break;

                case SDO_UPLOAD_POWER_STAGE_TEMPERATURE:                
                    idx         = 0x2061;                
                    break;

                case SDO_UPLOAD_SYSTEM_LAST_ERROR:                
                    idx         = 0x5E49;                
                    break;

                case SDO_UPLOAD_STO_STATUS:                
                    idx         = 0x251A;                
                    break;

                case SDO_UPLOAD_ERROR_TOTAL_NUMBER:                
                    idx         = 0x264D;                
                    break;

                case SDO_UPLOAD_TOTAL:
                default:
                    sdoFrameNext = (enum SdoUploadFrame_t)(0); // safety!
                    return; // error!
            }
            
            /* setup client */
            SDO_ret = CO_SDOclient_setup(co->SDOclient, (uint32_t)CO_CAN_ID_SDO_CLI + wxId, (uint32_t)CO_CAN_ID_SDO_SRV + wxId, wxId); // all the same value, it works ok
            if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
                return; // error!
            }

            /* initiate upload */
            uint16_t timeout_ms = 100U;
            SDO_ret = CO_SDOclientUploadInitiate(co->SDOclient, idx, subidx, timeout_ms, true);
            if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
                return; // error!
            }

            else{
                isSdoClientReady = true;
            }            
        }

        // 2.2. Loops over Rx but only if SDO client is ready
        if( true == isSdoClientReady )
        {
            SDO_ret = CO_SDOclientUpload(co->SDOclient, timeDiff_us, false, &abortCode, NULL, &sizeTransferred, &timeNext_us);                 // non-blocking

            if (SDO_ret < CO_SDO_RT_ok_communicationEnd) {
                updateWxId = true; // error
            }
            /* Response data must be read, partially or whole */
            else if ((SDO_ret == CO_SDO_RT_uploadDataBufferFull) || (SDO_ret == CO_SDO_RT_ok_communicationEnd)) {

                // Parsing! hence, ready via EtherCAT too!
                uint16_t rxIdx      = co->SDOclient->CANrxData[1];          // LSB
                rxIdx               |= (co->SDOclient->CANrxData[2] << 8);  // MSB
                uint8_t rxSubIdx    = co->SDOclient->CANrxData[3];

                //app_sdoCustomParsing( wxId, rxIdx, rxSubIdx, &co->SDOclient->CANrxData[4] );
                updateWxId = true;

                // Function must be called after finish of each SDO client communication cycle
                CO_SDOclientClose(co->SDOclient);
            }

            else // still waiting?
            {
                static unsigned char timeout = 0xFF;
                if ( 0 == --timeout ){ // overflows automatically!
                    updateWxId = true; // timeout error!
                }
            }

            // 3. Updates Wx ID and messages ID (FSM)
            if( true == updateWxId )        // note this is a local variable initialized to false always
            {
                isSdoClientReady = false;   // 3.1. Resets SDO client ready flag

                if(++wxId > WX_CAN_ID_MAX)  // 3.2. Increments and checks Wx ID overflow!
                {
                    wxId = WX_CAN_ID_MIN;
                    if( ++sdoFrameNext >= SDO_UPLOAD_TOTAL)         // 3.3. Increments and checks SDO frame ID overflow
                        sdoFrameNext = (enum SdoUploadFrame_t)(0);  // safest reset!
                }
            }
        }
    }
}


/******************************************************************************/
void app_programRt(CO_t *co, uint32_t timer1usDiff) {

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
