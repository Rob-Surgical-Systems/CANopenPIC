/*
 * CANopen main program file for PIC32 microcontroller.
 *
 * @file        main_PIC32.c
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
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
 * Modified version, by EF, so the compilation succeeds in a F03 firmware.
 * Each modification is numbered, i.e. 1)
 */

//#include <xc.h>
//#include <sys/attribs.h>
//#include <plib.h>

#include "CANopen.h"
//#include "storage/CO_storageEeprom.h"
#include "OD.h"
#include "CO_application.h"

// 1) Configuration bits were removed from here, as they are declared in its own file

// 2) ADC configuration is removed, ADC is processed differently
//  2.1) CO_PERIPHERAL_CONFIG does not exist anymore
//  2.2.) ...

// Just to remember
// CO_PBCLK and CO_FSYS are declared in CO_driver_target.h, at some point, they should be redefined or reviewed to avoid duplicates, to meet vocabulary constraints, ..

// 3) Real time thread configuration, coupled to TMR2 and as triiger to ADC thread, and what they call the Real Time Thread, is deleted too
// 3.1.) Real time thread may be set to another timer? reuse TMR5 from EtherCAT? called by the Update method?

// 4) TMR2_ISR and ADC_ISR methods have been deleted too!
// Note that TMR2 was set to 1 [ms]! And it enabled the ADC sampling. Once ADC samples are ready, the ADC ISR will do

// 5.) CANRx configuration is removed, it set the ISR IP to 5. Now it would be IP=6
/* CAN receive interrupt definitions */
/* Configure CAN rx interrupt, priority is 5, higher than timer */

// 6.) Default CAN_1_VECTOR ISR is commented --> it was empty, probably redefined somewhere, so here it compiles but links to another one
/* Default interrupt handler, use same priority as in CO_CANRX_CONFIG() */
//#ifndef CO_CANRX_ISR
//#define CO_CANRX_ISR_DEFAULT
//#define CO_CANRX_ISR() void __ISR(_CAN_1_VECTOR, IPL5SOFT) _canRxIsr(void)
//#endif

// 6.1.) next ones are keep but probably removed in a near future
/* Enable interrupt, 0 or 1 */
#ifndef CO_CANRX_ENABLE
#define CO_CANRX_ENABLE(ENABLE) IEC1bits.CAN1IE = ENABLE
#endif
/* Interrupt flag bit, used inside CO_CANRX_ISR */
#ifndef CO_CANRX_ISR_FLAG
#define CO_CANRX_ISR_FLAG IFS1bits.CAN1IF
#endif

// 7. Clear watchdog is modified, to an empty method, as it is already implemented somewhere else with other logics.

/* Watchdog timer */
#ifndef CO_clearWDT
#define CO_clearWDT() (WDTCONSET = _WDTCON_WDTCLR_MASK)
#endif


/* default values for CO_CANopenInit() */
#ifndef NMT_CONTROL
#define NMT_CONTROL \
            CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION
#endif
#ifndef FIRST_HB_TIME
#define FIRST_HB_TIME 500
#endif
#ifndef SDO_SRV_TIMEOUT_TIME
#define SDO_SRV_TIMEOUT_TIME 1000
#endif
#ifndef SDO_CLI_TIMEOUT_TIME
#define SDO_CLI_TIMEOUT_TIME 500
#endif
#ifndef SDO_CLI_BLOCK
#define SDO_CLI_BLOCK false
#endif
#ifndef OD_STATUS_BITS
#define OD_STATUS_BITS NULL
#endif

/* Definitions for application specific data storage objects */
#ifndef CO_STORAGE_APPLICATION
#define CO_STORAGE_APPLICATION
#endif
/* Interval for automatic data storage in microseconds */
#ifndef CO_STORAGE_AUTO_INTERVAL
#define CO_STORAGE_AUTO_INTERVAL 60000000
#endif


/* CANopen object */
CO_t *CO = NULL;

/* Active node-id, copied from pendingNodeId in the communication reset */
static uint8_t CO_activeNodeId = CO_LSS_NODE_ID_ASSIGNMENT;

/* Timer for time measurement */
volatile uint32_t CO_timer_us = 0;

/* Data block for mainline data, which can be stored to non-volatile memory */
typedef struct {
    /* Pending CAN bit rate, can be set by switch or LSS slave. */
    uint16_t pendingBitRate;
    /* Pending CANopen NodeId, can be set by switch or LSS slave. */
    uint8_t pendingNodeId;
} mainlineStorage_t;

static mainlineStorage_t mlStorage = {0};

/* callback for storing node id and bitrate */
static bool_t LSScfgStoreCallback(void *object, uint8_t id, uint16_t bitRate) {
    mainlineStorage_t *mainlineStorage = object;
    mainlineStorage->pendingBitRate = bitRate;
    mainlineStorage->pendingNodeId = id;
    return true;
}

// 8.) Main is renamed to CO_main
// 8.1) It has only the Initialization stage logics
int CO_Init () {
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    bool_t firstRun = true;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;
    CO_storage_entry_t storageEntries[] = {
        {
            .addr = &OD_PERSIST_COMM,
            .len = sizeof(OD_PERSIST_COMM),
            .subIndexOD = 2,
            .attr = CO_storage_cmd | CO_storage_restore
        },
        {
            .addr = &mlStorage,
            .len = sizeof(mlStorage),
            .subIndexOD = 4,
            .attr = CO_storage_cmd | CO_storage_auto | CO_storage_restore
        },
        CO_STORAGE_APPLICATION
    };
    uint8_t storageEntriesCount = sizeof(storageEntries)
                                / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    
// 8.2.) Removes PIC configuration, as it is done in the main.cpp file!

    /* Allocate memory for CANopen objects */
    uint32_t heapMemoryUsed = 0;
    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL) {
        while (1);
    }


#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageEeprom_init(&storage,
                                CO->CANmodule,
                                NULL,
                                OD_ENTRY_H1010_storeParameters,
                                OD_ENTRY_H1011_restoreDefaultParameters,
                                storageEntries,
                                storageEntriesCount,
                                &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        while (1);
    }
#endif

    /* Configure peripherals */
    CO_PERIPHERAL_CONFIG();

    /* Execute external application code */
    uint32_t errInfo_app_programStart = 0;
    err = app_programStart(&mlStorage.pendingBitRate,
                           &mlStorage.pendingNodeId,
                           &errInfo_app_programStart);
    if (err != CO_ERROR_NO) {
        while (1);
    }

    /* verify stored values */
    if (!CO_LSSchkBitrateCallback(NULL, mlStorage.pendingBitRate)) {
        mlStorage.pendingBitRate = 125;
    }
    if (mlStorage.pendingNodeId < 1 || mlStorage.pendingNodeId > 127) {
        mlStorage.pendingNodeId = CO_LSS_NODE_ID_ASSIGNMENT;
    }

    LATFbits.LATF3      = 0; // reset LED, or it may be already turned on!
    TRISFbits.TRISF3    = 1; // FAULT LED - Rx

    while (reset != CO_RESET_APP) {
/* CANopen communication reset - initialize CANopen objects *******************/
        uint32_t errInfo;
        static uint32_t CO_timer_us_previous = 0;

        /* disable CAN receive interrupts */
        CO_CANRX_ENABLE(0);

        /* initialize CANopen */
        err = CO_CANinit(CO, (void *)_CAN1_BASE_ADDRESS,
                         mlStorage.pendingBitRate);
        if (err != CO_ERROR_NO) {
            while (1) CO_clearWDT();
        }

        CO_LSS_address_t lssAddress = {.identity = {
            .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
            .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
            .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
            .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
        }};
        err = CO_LSSinit(CO, &lssAddress,
                         &mlStorage.pendingNodeId, &mlStorage.pendingBitRate);
        if (err != CO_ERROR_NO) {
            while (1) CO_clearWDT();
        }

        CO_activeNodeId = mlStorage.pendingNodeId;
        errInfo = 0;

        err = CO_CANopenInit(CO,                /* CANopen object */
                             NULL,              /* alternate NMT */
                             NULL,              /* alternate em */
                             OD,                /* Object dictionary */
                             OD_STATUS_BITS,    /* Optional OD_statusBits */
                             NMT_CONTROL,       /* CO_NMT_control_t */
                             FIRST_HB_TIME,     /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                             CO_activeNodeId,
                             &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            while (1) CO_clearWDT();
        }

        /* Emergency messages in case of errors */
        if (!CO->nodeIdUnconfigured) {
            if (errInfo == 0) errInfo = errInfo_app_programStart;
            if (errInfo != 0) {
                CO_errorReport(CO->em, CO_EM_INCONSISTENT_OBJECT_DICT,
                               CO_EMC_DATA_SET, errInfo);
            }
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if (storageInitError != 0) {
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY,
                               CO_EMC_HARDWARE, storageInitError);
            }
#endif
        }

        /* initialize callbacks */
        CO_LSSslave_initCkBitRateCall(CO->LSSslave, NULL,
                                      CO_LSSchkBitrateCallback);
        CO_LSSslave_initCfgStoreCall(CO->LSSslave, &mlStorage,
                                     LSScfgStoreCallback);


        /* First time only initialization. */
        if (firstRun) {
            firstRun = false;

            /* Configure real time thread and CAN receive interrupt */
            CO_RT_THREAD_CONFIG();
            CO_CANRX_CONFIG();

            CO_timer_us_previous = CO_timer_us;
        } /* if(firstRun) */

        /* Execute external application code */
        app_communicationReset(CO);

        errInfo = 0;
        err = CO_CANopenInitPDO(CO,             /* CANopen object */
                                CO->em,         /* emergency object */
                                OD,             /* Object dictionary */
                                CO_activeNodeId,
                                &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            while (1) CO_clearWDT();
        }


        /* start CAN and enable interrupts */
        CO_CANsetNormalMode(CO->CANmodule);
        CO_RT_THREAD_ENABLE(1);
        CO_CANRX_ENABLE(1);
        reset = CO_RESET_NOT;
    } /* while(reset != CO_RESET_APP */
    
    return EXIT_SUCCESS;
}
    

int CO_Config ()
{
    return EXIT_SUCCESS;
}

// 8.3.) Update stage
#warning "New Update method, but SSL service may require to reuse some code, or all, of the Initialization stage"
int CO_Update ()
{
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t CO_timer_us_previous = 0;
    
    while (reset == CO_RESET_NOT)
    {
/* loop for normal program execution ******************************************/

        /* calculate time difference since last cycle */
        uint32_t timer_us_copy = CO_timer_us;
        uint32_t timeDifference_us = timer_us_copy - CO_timer_us_previous;
        CO_timer_us_previous = timer_us_copy;

        CO_clearWDT();

        /* process CANopen objects */
        reset = CO_process(CO, false, timeDifference_us, NULL);

        CO_clearWDT();

        /* Execute external application code */
        app_programAsync(CO, timeDifference_us);

        CO_clearWDT();

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        CO_storageEeprom_auto_process(&storage, false);
#endif
    }

/* program exit ***************************************************************/
    CO_RT_THREAD_ENABLE(0);
    CO_CANRX_ENABLE(0);

    /* Execute external application code */
    app_programEnd();

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storageEeprom_auto_process(&storage, true);
#endif

    /* delete objects from memory */
    CO_CANsetConfigurationMode(CO->CANmodule->CANptr);
    CO_delete(CO);

    /* reset microcontroller */
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    RSWRSTSET = 1;
    (void) RSWRST;
    while (1);
}


/* timer interrupt function executes every millisecond ************************/
#ifdef CO_RT_THREAD_ISR_DEFAULT
CO_RT_THREAD_ISR() {
    CO_timer_us += CO_RT_THREAD_INTERVAL_US;

    /* Execute external application code */
    app_peripheralRead(CO, CO_RT_THREAD_INTERVAL_US);

    CO_RT_THREAD_ISR_FLAG = 0;

    /* No need to CO_LOCK_OD(co->CANmodule); this is interrupt */
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal)
    {
        bool_t syncWas = false;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, CO_RT_THREAD_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(CO, syncWas, CO_RT_THREAD_INTERVAL_US, NULL);
#endif

        /* Execute external application code */
        app_programRt(CO, CO_RT_THREAD_INTERVAL_US);

#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(CO, syncWas, CO_RT_THREAD_INTERVAL_US, NULL);
#endif

        /* verify timer overflow */
        if (CO_RT_THREAD_ISR_FLAG == 1) {
            CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW,
                           CO_EMC_SOFTWARE_INTERNAL, 0);
            CO_RT_THREAD_ISR_FLAG = 0;
        }

        (void) syncWas;
    }

    /* Execute external application code */
    app_peripheralWrite(CO, CO_RT_THREAD_INTERVAL_US);
}
#endif /* CO_RT_THREAD_ISR_DEFAULT */


// 9.) Deletes the CAN interrupt function *****************************************************/
#ifdef CO_CANRX_ISR_DEFAULT
CO_CANRX_ISR() {
    
    CO_CANinterrupt(CO->CANmodule);
    /* Clear combined Interrupt flag */
    CO_CANRX_ISR_FLAG = 0;
}
#endif
