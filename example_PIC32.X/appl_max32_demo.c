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
#include "SharedParameters.h"

/* CANopen LED diodes, already initialized in DEV_Init(), renamed here for easier integration. */
#define CAN_INIT_LEDS() _nop()
#define CAN_RUN_LED     LED_ERROR_WR
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






/**
 * 
 * @param SDO_C
 * @param nodeId
 * @param index
 * @param subIndex
 * @param buf
 * @param bufSize
 * @param readSize
 * @return isInProgress
 */
//bool prepare_read_SDO( CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex,
//        uint8_t* buf, size_t bufSize, size_t* readSize) 
//{
//    CO_SDO_return_t SDO_ret;
//
//    // setup client (this can be skipped, if remote device don't change)
//    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
//    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
//        return false;
//    }
//
//    // initiate upload
//    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
//    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
//        return false;
//    }
//
//    return true;
//}
    





//    
//bool dummy(){    
//    //ce qui est là dessous doit être appelé dans la boucle principale (après avoir fait ce qui est au dessus) jsq retourner 0
//    // upload data
//    do {
//        uint32_t timeDifference_us = 10000;
//        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
//
//        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);
//        if (SDO_ret < 0) {
//            return abortCode;
//        }
//
//        sleep_us(timeDifference_us);
//    } while (SDO_ret > 0);
//
//    // copy data to the user buffer (for long data function must be called several times inside the loop)
//    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);
//
//    return CO_SDO_AB_NONE;
//}



bool prepare_write_SDO ( CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t* data, size_t dataSize ) 
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) 
    {
        return false;
    }

    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) 
    {
        return false;
    }

    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
    if (nWritten < dataSize) {
        bufferPartial = true;
        // If SDO Fifo buffer is too small, data can be refilled in the loop.
    }

    return true;
}
    




//bool dummy2(){
//    // download data
//    do {
//        uint32_t timeDifference_us = 10000;
//        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
//
//        CO_SDO_return_t SDO_ret = CO_SDOclientDownload(, timeDifference_us, false, false, &abortCode, NULL, NULL);
//        if (SDO_ret < 0) {
//            return abortCode;
//        }
//
//        sleep_us(timeDifference_us);
//    } while (SDO_ret > 0);
//
//    return CO_SDO_AB_NONE;
//}


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

        else if( 20100U == count )
        {
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 0x03);
        }
        
        else if( 20200U == count )
        {
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 0x04);
            IsNmtOp = true;
        }

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
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 2);//beta                 
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 3); //w4
            CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 4); //pp2
            // pl?
            //CO_NMT_sendCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL, 0);                 // to all nodes? including itself

        }
        
        if ( 0 != co->CANmodule->CANerrorStatus ) // checks flag bits - any        
        {
            co->CANmodule->CANerrorStatus = 0; // needed? wrong?

            CO_NMT_sendInternalCommand( co->NMT, CO_NMT_RESET_COMMUNICATION);           // 1.2.2. first, reset comms      
            isReset = true;
        }
        
        

        
        static bool inProgress = false;
        static int flagInProgress = -1;
        
        if ( false == inProgress )
        {

            //checking sdo flags - received data from ethercat COEs?

            //1 capitan constants

            int totalFlags = FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KD - FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KP;

            for ( int i = 0 ; i < totalFlags ; i++ )
            {

                int flag = (int)FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KP + i;

                if ( true == DEV_PeriphParams_GetUpdateFlag( (FLAGS_PERIPH_PARAMS)flag ) )
                {

        //up to here ok
                    uint8_t nodeId = 2;

                    //initiate sdo upload
                    if(flag >= FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KP)
                    {
                        nodeId = 3;
                    }


                    uint8_t* data = (uint8_t*)&OD_RAM.x250A_betaVelocityLoopKp;
                    uint16_t reg = 0x250A;
                    
                    //CAREFUL work because data is in the right order on both sides. Do not move things around carelessly
                    data += sizeof(float) * (flag - FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KP);
                    
                    

                    switch ( flag )
                    {
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KP :
                        {
                            //initialised value
                            break;
                        }
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KI :
                        {
                            reg = 0x250B;
                            break;
                        }
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KD :
                        {
                            reg = 0x250C;
                            break;
                        }
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_POSITION_KP :
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KP :
                        {
                            reg = 0x2511;
                            break;
                        }
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_POSITION_KI :
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KI :
                        {
                            reg = 0x2512;
                            break;
                        }
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_POSITION_KD :
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KD :
                        {
                            reg = 0x2513;
                            break;
                        }
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_TORQUE_KP :
                        {
                            reg = 0x2523;
                            break;
                        }
                        case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_TORQUE_KI :
                        {
                            reg = 0x2524;
                            break;
                        }



                    }


                    inProgress = prepare_write_SDO ( co->SDOclient, nodeId, reg, 0, data, sizeof(float) );
                    if( true == inProgress )
                    {
                        
                        flagInProgress = flag;
                    }
                    break;
                }

            }
        }
        else //finish exchange
        {
                
                CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

                CO_SDO_return_t SDO_ret = CO_SDOclientDownload(co->SDOclient, timer1usDiff, false, false, &abortCode, NULL, NULL);
                if (SDO_ret <= 0) 
                {
                    if (SDO_ret == 0);//if exchange finished
                    {
                        LED_TEST_ON
                        DEV_PeriphParams_ClearUpdateFlag( (FLAGS_PERIPH_PARAMS) flagInProgress );
                    }
                    inProgress = false;
                    flagInProgress = -1;
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
