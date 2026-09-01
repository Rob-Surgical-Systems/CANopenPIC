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
bool prepare_read_SDO( CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, size_t* readSize) 
{
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return false;
    }

    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return false;
    }

    return true;
}
    




/**
 * 
 * @param SDO_C
 * @param nodeId
 * @param index
 * @param subIndex
 * @param data
 * @param dataSize
 * @return 
 */
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
        static int eventTimerInProgress = -1;
        static bool eventTimerSet[2][4] = {0};
        static bool allEventTimersSet = false;
        static uint8_t* readBufInProgress = NULL;
        static size_t   readBufSizeInProgress = 0;
        

        if ( false == inProgress )
        {
            
            if ( false == allEventTimersSet )
            {
                

                for(int j = 0 ; j <= 1 ; j++ )  
                {
                    for ( int i = 0 ; i < 4 ; i++ )
                    {
                        if ( false == eventTimerSet[j][i] )
                        {

                            //up to here ok
                            uint8_t nodeId = j+2;

                            static uint16_t eventTimerDummy = 5;
                            uint8_t* data = (uint8_t*)&eventTimerDummy;
                            uint16_t reg = 0x1800 + i;

                            inProgress = prepare_write_SDO ( co->SDOclient, nodeId, reg, 5, data, 2 );
                            if( true == inProgress )
                            {
                                eventTimerInProgress = j*4 + i;
                            }

                            break;//out tpdo loop
                        }
                        
                        //we only reach this portion if the break just above is not reached, in which case all event timers are set.
                        allEventTimersSet = true;

                    }//for TPDOs

                    if( true == inProgress )
                        break;// out node loop

                }//for nodes  


            }
            else //if all event timers already set
            {
                //checking sdo flags - received data from ethercat COEs?

                //1 capitan constants

                int totalFlags = FLAGS_PERIPH_PARAMS_READ_CAPITAN_W4_POSITION_KD - FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_QUAD_CURRENT_KP;

                for ( int i = 0 ; i < totalFlags ; i++ )
                {

                    int flag = (int)FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_QUAD_CURRENT_KP + i;

                    if ( true == DEV_PeriphParams_GetUpdateFlag( (FLAGS_PERIPH_PARAMS)flag ) )
                    {
            //up to here ok
                        uint8_t nodeId = 2;

                        //initiate sdo download (client pov) or upload (client pov) (clients are pp2 and capitans)

                        if( (flag >= FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KP && flag <= FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KD) 
                            || (flag >= FLAGS_PERIPH_PARAMS_READ_CAPITAN_W4_POSITION_KP && flag <= FLAGS_PERIPH_PARAMS_READ_CAPITAN_W4_POSITION_KD)    
                                )
                            
                        {
                            nodeId = 3;
                        }
                        
                        

                        //selecting right register
                        uint16_t reg = 0x2500;
                        
                        switch ( flag )
                        {
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_QUAD_CURRENT_KP :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_QUAD_CURRENT_KP :
                            {
                                //INIT VALUE
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_QUAD_CURRENT_KI :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_QUAD_CURRENT_KI :
                            {
                                reg = 0x2501;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_QUAD_CURRENT_KD :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_QUAD_CURRENT_KD :
                            {
                                reg = 0x2502;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_CURRENT_KP :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_CURRENT_KP :
                            {
                                reg = 0x2505;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_CURRENT_KI :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_CURRENT_KI :
                            {
                                reg = 0x2506;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_CURRENT_KD :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_CURRENT_KD :
                            {
                                reg = 0x2507;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KP :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_VELOCITY_KP :
                            {
                                reg = 0x250A;//initialised value
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KI :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_VELOCITY_KI :
                            {
                                reg = 0x250B;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_VELOCITY_KD :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_VELOCITY_KD :
                            {
                                reg = 0x250C;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_POSITION_KP :
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KP :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_POSITION_KP :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_W4_POSITION_KP :
                            {
                                reg = 0x2511;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_POSITION_KI :
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KI :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_POSITION_KI :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_W4_POSITION_KI :
                            {
                                reg = 0x2512;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_POSITION_KD :
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_W4_POSITION_KD :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_POSITION_KD :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_W4_POSITION_KD :
                            {
                                reg = 0x2513;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_TORQUE_KP :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_TORQUE_KP :
                            {
                                reg = 0x2523;
                                break;
                            }
                            case FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_TORQUE_KI :
                            case FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_TORQUE_KI :
                            {
                                reg = 0x2524;
                                break;
                            }
                        }                        
                        
                        
                        //Selecting memory zone to access and preparing action
                        //CAREFUL work because data is in the right order on both sides. Do not move things around carelessly
                        
                        uint8_t* data = (uint8_t*)&OD_RAM.x2500_betaCurrentQuadratureLoopKp;//first possible zone
                        
                        if( flag >= FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_QUAD_CURRENT_KP)//read request
                        {
//                            LED_TEST_ON
                            data += sizeof(float) * (flag - FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_QUAD_CURRENT_KP );//where received data will be stored
                            size_t dummyReadSize = 0;
                            inProgress = prepare_read_SDO ( co->SDOclient, nodeId, reg, 0, &dummyReadSize );
                            
                            if ( true == inProgress )
                            {
                                readBufInProgress     = data;
                                readBufSizeInProgress = sizeof(float);
                            }

                        }
                        else //write request
                        {
                            data += sizeof(float) * (flag - FLAGS_PERIPH_PARAMS_WRITE_CAPITAN_BETA_QUAD_CURRENT_KP );//where data to send is stored
                            inProgress = prepare_write_SDO ( co->SDOclient, nodeId, reg, 0, data, sizeof(float) );
                        }
                        
                        
                        if( true == inProgress )
                        {
                            flagInProgress = flag;
                            
                        }
                        break;
                    }

                }//loop flags coming from ethercat COEs
            }//if-else conditions to send sdo 
            
        }//end if not in progress

        else //if in progress, finish exchange
        {
            CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;    
            
            
            if( flagInProgress >= FLAGS_PERIPH_PARAMS_READ_CAPITAN_BETA_QUAD_CURRENT_KP )
            {// read sdo in progress


                CO_SDO_return_t SDO_ret = CO_SDOclientUpload(co->SDOclient, timer1usDiff, false, &abortCode, NULL, NULL, NULL);
                if (SDO_ret <= 0) 
                {
                    if (SDO_ret == 0)//if exchange finished successfully
                    {
                        LED_TEST_ON
                        
                        
                        size_t readSize = CO_SDOclientUploadBufRead(co->SDOclient, readBufInProgress, readBufSizeInProgress);

//                        //should check readSize?                        
//                        if(readSize < readBufSizeInProgress)
//                            //todo error? retry? how many times?

                        DEV_PeriphParams_ClearUpdateFlag( (FLAGS_PERIPH_PARAMS) flagInProgress );

                    }
                    
                    //resetting in progress flags
                    eventTimerInProgress = -1;
                    flagInProgress = -1;
                    inProgress = false;
                    readBufInProgress       = NULL;
                    readBufSizeInProgress   = 0;
                 }
                
            }
            else //write sdo in progress (from coe or initial write of event timers)
            {

                CO_SDO_return_t SDO_ret = CO_SDOclientDownload(co->SDOclient, timer1usDiff, false, false, &abortCode, NULL, NULL);
                if (SDO_ret <= 0) //if exchange finished
                {
                    if (SDO_ret == 0);//if exchange finished successfully
                    {
                        
                        // is an eventtimer sdo in progress?
                        if( 0 <= eventTimerInProgress )
                        {
                            int rest = eventTimerInProgress%4;
                            int div = eventTimerInProgress/4;
                            eventTimerSet[div][rest] = true;
                            
                        }
                        else //other sdo triggered by ethercat COE
                        {
                            DEV_PeriphParams_ClearUpdateFlag( (FLAGS_PERIPH_PARAMS) flagInProgress );
                        }
                        
                        
                    }
                    
                    //resetting in progress flags
                    eventTimerInProgress = -1;
                    flagInProgress = -1;
                    inProgress = false;
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
