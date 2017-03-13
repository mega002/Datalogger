/******************************************************************************

 @file  data_logger.c

 @brief This file contains the Simple BLE Peripheral sample application
        definitions and prototypes.

 Target Device: CC2650

 ******************************************************************************
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */



/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti/mw/display/Display.h>
#include <ti/mw/extflash/ExtFlash.h>

#include "board.h"

#include "simple_peripheral.h"
#include "data_service.h"



/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     80

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         2


// Task configurations
#define SBP_TASK_PRIORITY       1
#define SBP_TASK_TEMP_PRIORITY  2
#define SBP_TASK_STACK_SIZE     800
#define DL_TASK_PRIORITY        2
#define DL_TASK_STACK_SIZE      800

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT    0x0001
#define SBP_CFG_CHANGE_EVT      0x0002
#define SBP_SEND_PASSCODE_EVT   0x0003
#define SBP_CONN_EVT_END_EVT    0x0008

// PDU configuration
#define MAX_PDU_SIZE            251
//#define MAX_NUM_PDU             6

// The combined overhead for L2CAP and ATT notification headers
#define TOTAL_PACKET_OVERHEAD   7

// Datalogger application constants
#define MEASURMENT_LEN          64
#define MEASURMENT_LEN_BYTES    8           // MEASURMENT_LEN / sizeof(Char)
#define BLE_CHUNK_LEN           192
#define BLE_CHUNK_LEN_BYTES     24
#define MIN_FLASH_OFFSET        0x1000      // address of the first sector
#define MAX_FLASH_OFFSET        0xFEFF0     // last divisible address by measurement size
                                            // in the last sector, after reducing MIN_FLASH_OFFSET
#define CURRENT_OFFSET          0xFF000
#define LAST_OFFSET             0xFF100
#define CYCLE_OFFSET            0xFF200

#define SECTOR_SIZE             4096
#define LAST_SECTOR             255
#define FIRST_SECTOR            1
#define SECTOR_SIZE             0x1000
#define BAUD_RATE               115200

#define READ_RET_LEN(x,y)   {                            \
                                if ((x) == 0)            \
                                {                        \
                                    return 0;            \
                                }                        \
                                temp_last_offset += (y); \
                                return (y);              \
                            }

#define DEFAULT_PASSCODE 641416



/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t 		        hdr;  // event header
  gapPasskeyNeededEvent_t   *pData;
} sbpEvt_t;

// Struct for measurement queue element
typedef struct Measurement
{
      Queue_Elem    _elem;
      uint8_t       data[MEASURMENT_LEN_BYTES];
} measurement_t;



/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;



/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID   selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore  sem;

// Clock instances for internal periodic events.
static Clock_Struct     periodicClock;

// Queue object used for app messages
static Queue_Struct     appMsg;
static Queue_Handle     appMsgQueue;

// SBP task configuration
Task_Struct             sbpTask;
Char                    sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'D', 'a', 't', 'a', 'l', 'o', 'g', 'g', 'e', 'r',

  // connection interval range
  0x05, // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02, // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0     // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t          attDeviceName[GAP_DEVICE_NAME_LEN] = "Datalogger";

// Globals used for ATT Response retransmission
static gattMsgEvent_t   *pAttRsp    = NULL;
static uint8_t          rspTxRetry  = 0;

// DL task configuration
Task_Struct             dlTask;
Char                    dlTaskStack[SBP_TASK_STACK_SIZE];

// UART communication handlers and objects
Semaphore_Struct        semStruct;
Semaphore_Handle        semHandle;
char                    rxMsg[MEASURMENT_LEN_BYTES];
Queue_Handle            measurementQueueHandle;
UART_Handle             uart;


// Pin driver handles
static PIN_Handle       ledPinHandle;
static PIN_State        ledPinState;

PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


// External flash offsets
size_t      current_offset  = MIN_FLASH_OFFSET;
size_t      last_offset     = MIN_FLASH_OFFSET;
bool        flag_cycle      = 0;

size_t      temp_last_offset     = MIN_FLASH_OFFSET;
bool        temp_flag_cycle      = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void     SimpleBLEPeripheral_init( void );
static void     SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t  SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t  SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void     SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void     SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);

static void     SimpleBLEPeripheral_sendAttRsp(void);
static void     SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void     SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
static void     SimpleBLEPeripheral_cfgChangeCB(uint16_t connHandle, uint16_t svcUuid,
                                                uint8_t paramID, uint8_t *pValue, uint16_t len);
static void     user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison);
static void     user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status);
static void     SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

static void     updateLastOffset();
static void     sendChunk();
uint32_t        readChunkFlash(uint8_t *readbuf);
bool            storeMeasurementFlash(uint8_t *buf);
static void     readOffsetsFlash();
static void     updateOffsetsFlash();

static void     Datalogger_init(void);
void            Datalogger_taskFxn(UArg a0, UArg a1);
static void     uartReadCallback(UART_Handle handle, void *buf, size_t count);



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
     SimpleBLEPeripheral_stateChangeCB  // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
    user_gapBondMgr_passcodeCB, // Passcode callback
    user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

// Simple GATT Profile Callbacks
/*
 * Callbacks in the user application for events originating from BLE services.
 */

// Data Service callback handler.
static DataServiceCBs_t user_Data_ServiceCBs =
{
    .pfnCfgChangeCb = SimpleBLEPeripheral_cfgChangeCB, // Noti/ind configuration callback handler
};



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{
	// ******************************************************************
	// NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
	// ******************************************************************
	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages via ICall to Stack.
    ICall_registerApp(&selfEntity, &sem);

	// Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
	dispHandle = Display_open(Display_Type_LCD, NULL);

	// Initialize queue for application messages.
	// Note: Used to transfer control to application thread from e.g. interrupts.
	Queue_construct(&appMsg, NULL);
	appMsgQueue = Queue_handle(&appMsg);


	// ******************************************************************
	// BLE Stack initialization
	// ******************************************************************

	// Setup the GAP Peripheral Role Profile
	uint8_t initialAdvertEnable   = TRUE;     // Advertise on power-up

	// By setting this to zero, the device will go into the waiting state after
	// being discoverable. Otherwise wait this long [ms] before advertising again.
	uint16_t advertOffTime        = 0;        // miliseconds

	// Set advertisement enabled.
	GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
	                       &initialAdvertEnable);

	// Configure the wait-time before restarting advertisement automatically
	GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
	                       &advertOffTime);

	// Initialize Scan Response data
	GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

	// Initialize Advertisement data
	GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

	// Set advertising interval
	uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

	GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
	GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
	GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
	GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

	// Set duration of advertisement before stopping in Limited adv mode.
	GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30);          // Seconds

	// ******************************************************************
	// BLE Bond Manager initialization
	// ******************************************************************
	uint32_t  passkey     = DEFAULT_PASSCODE;
	uint8_t   pairMode    = GAPBOND_PAIRING_MODE_INITIATE;
	uint8_t   mitm        = TRUE;
	uint8_t   ioCap       = GAPBOND_IO_CAP_DISPLAY_ONLY;
	uint8_t   bonding     = TRUE;

	GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),&passkey);
	GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
	GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
	GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
	GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

	// ******************************************************************
	// BLE Service initialization
	// ******************************************************************

	// Add services to GATT server
	GGS_AddService(GATT_ALL_SERVICES);           // GAP
	GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
	DevInfo_AddService();                        // Device Information Service

	// Set the device name characteristic in the GAP Profile
	GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

	// Add services to GATT server and give ID of this task for Indication acks.
	DataService_AddService(selfEntity);

	// Register callbacks with the generated services that
	// can generate events (writes received) to the application
	DataService_RegisterAppCBs(&user_Data_ServiceCBs);

	// Initalization of characteristics in Data_Service that can provide data.
	uint8_t initVal[DS_STREAM_LEN] = {0};
	DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);

	// Start the stack in Peripheral mode.
	VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

	// Start Bond Manager
	VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

	// Register with GAP for HCI/Host messages
	GAP_RegisterForMsgs(selfEntity);

	// Register for GATT local events and ATT Responses pending for transmission
	GATT_RegisterForMsgs(selfEntity);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(0);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID        dest;
      ICall_ServiceEnum     src;
      ICall_HciExtEvt       *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

  }
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            asm(" NOP ");
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }

  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }

  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, 4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // an acknowledgment of the last sent indication
  else if (pMsg->method == ATT_HANDLE_VALUE_CFM)
  {
      // update task priority to have a sole access to the flash
      // and avoid reading and writing at the same time
      Task_Handle hBLETask = Task_self();
      UInt origPri = Task_setPri(hBLETask, DL_TASK_PRIORITY);

      // clear the acknowledged data from flash
      // by updating the last offset pointer
      updateLastOffset();

      // send another chuck in case there's still more data stored
      if ((flag_cycle == 1) || (last_offset < current_offset))
      {
          sendChunk();
      }

      // restore the original task priority
      Task_setPri(hBLETask, origPri);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp     = NULL;
    rspTxRetry  = 0;
  }
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  Task_Handle hBLETask = Task_self();

  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->hdr.state);
      break;

    case SBP_CFG_CHANGE_EVT:
        // update task priority to have a sole access to the flash
        // and avoid writing during the dumping process
        Task_setPri(hBLETask, SBP_TASK_TEMP_PRIORITY);

        if ((flag_cycle == 1) || (last_offset < current_offset))
            sendChunk();

        // restore the original task priority
        Task_setPri(hBLETask, SBP_TASK_PRIORITY);
    	break;

    case SBP_SEND_PASSCODE_EVT:

        // Send passcode response.
        GAPBondMgr_PasscodeRsp(pMsg->pData->connectionHandle, SUCCESS, DEFAULT_PASSCODE);

        if (pMsg->pData != NULL)
        {
            ICall_free(pMsg->pData);
            pMsg->pData = NULL;
        }
        break;

    default:
      break;
  }
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, 2, 0, "Advertising");
      break;


    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        Display_print0(dispHandle, 2, 0, "Connected");
        Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));

      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, NULL, NULL);

      Display_print0(dispHandle, 2, 0, "Disconnected");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, NULL, NULL);

      Display_print0(dispHandle, 2, 0, "Timed Out");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_cfgChangeCB
 *
 * @brief   Callback from Simple Profile indicating a configuration
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_cfgChangeCB(uint16_t connHandle, uint16_t svcUuid,
											uint8_t paramID, uint8_t *pValue,
											uint16_t len)
{
	uint16_t configValue = *pValue;

	if (configValue == GATT_CLIENT_CFG_INDICATE)
	{
		SimpleBLEPeripheral_enqueueMsg(SBP_CFG_CHANGE_EVT, paramID);
	}
}


/*********************************************************************
 * @fn      user_gapBondMgr_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode
 * @param   uiOutputs  - display passcode
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs,
                                       uint32 numComparison)
{
    sbpEvt_t *pMsg;

    // Create dynamic pointer to message.
    if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
    {
      pMsg->hdr.event = SBP_SEND_PASSCODE_EVT;

      if ((pMsg->pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
      {
          memcpy(pMsg->pData->deviceAddr, deviceAddr, B_ADDR_LEN);
          pMsg->pData->connectionHandle = connHandle;
          pMsg->pData->uiInputs = uiInputs;
          pMsg->pData->uiOutputs = uiOutputs;

          // Enqueue the message.
          Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
      }
      else
      {
          ICall_free(pMsg);
      }
    }
}


/*********************************************************************
 * @fn      user_gapBondMgr_pairStateCB
 *
 * @brief   Pairing state callback. Do nothing.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
    return;
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}


/*********************************************************************
 * @fn      sendChunk
 *
 * @brief   Send a chunk of data read from the external flash
 *          over BLE
 *
 * @param   None
 *
 * @return  None
 */
static void sendChunk()
{
    uint32_t    res_read    = BLE_CHUNK_LEN_BYTES;
    uint8_t     readbuf[BLE_CHUNK_LEN_BYTES];

    res_read = readChunkFlash(readbuf);

    // send the data over BLE if reading succeeded
    if (res_read > 0)
    {
        DataService_SetParameter(DS_STREAM_ID, res_read, readbuf);
    }
}


/*********************************************************************
 * @fn      readChunkFlash
 *
 * @brief   Read a chunk of data from the external flash to send
 *          over BLE
 *
 * @param   readbuf - a buffer to store the read data
 *
 * @return  The number of read bytes
 */
uint32_t readChunkFlash(uint8_t *readbuf)
{
    uint32_t    len         = BLE_CHUNK_LEN_BYTES;
    bool        res_read    = 0;

    temp_last_offset = last_offset;
    temp_flag_cycle = flag_cycle;

    if (flag_cycle == 0)
    {
        // we've passed current_offset
        if (last_offset + BLE_CHUNK_LEN_BYTES > current_offset)
        {
            len = current_offset - last_offset;
            res_read = ExtFlash_read(last_offset, len, readbuf);
            READ_RET_LEN(res_read,len);
        }

        // common case
        else
        {
            res_read = ExtFlash_read(last_offset, len, readbuf);
            READ_RET_LEN(res_read,len);
        }
    }

    else // flag_cycle == 1
    {
        // "de-cycle" event
        if (last_offset + BLE_CHUNK_LEN_BYTES > MAX_FLASH_OFFSET)
        {
            // reading until the maximum offset
            len = MAX_FLASH_OFFSET - last_offset;
            if (len > 0)
            {
                res_read = ExtFlash_read(last_offset, len, readbuf);
                if (res_read == 0)
                {
                    return 0;
                }
            }
            temp_last_offset = MIN_FLASH_OFFSET;
            temp_flag_cycle = 0;

            /*
             * reading the remaining bytes from the minimum offset
             */
            // read all remaining bytes
            if (current_offset - MIN_FLASH_OFFSET >= BLE_CHUNK_LEN_BYTES - len)
            {
                res_read = ExtFlash_read(MIN_FLASH_OFFSET, BLE_CHUNK_LEN_BYTES - len, readbuf + len);
                temp_last_offset += BLE_CHUNK_LEN_BYTES - len;
                len = BLE_CHUNK_LEN_BYTES; // len += BLE_CHUNK_LEN_BYTES - len;
            }
            // read remaining bytes until current offset
            else
            {
                res_read = ExtFlash_read(MIN_FLASH_OFFSET, current_offset - MIN_FLASH_OFFSET, readbuf + len);
                temp_last_offset = current_offset;
                len += current_offset - MIN_FLASH_OFFSET;
            }

            // read successfully until MAX_FLASH_OFFSET only
            if (res_read == 0)
            {
                temp_last_offset = MIN_FLASH_OFFSET;
                len = MAX_FLASH_OFFSET - last_offset;
            }

            return len;

        }

        // common case
        res_read = ExtFlash_read(last_offset, len, readbuf);
        READ_RET_LEN(res_read,len);
    }
}


/*********************************************************************
 * @fn      readOffsetsFlash
 *
 * @brief   Read the global pointers from the external flash. This
 *          function runs when the board starts.
 *
 * @param   None.
 *
 * @return  None.
 */
static void readOffsetsFlash()
{
    bool res = 0;
    uint8_t buf[4];

    res = ExtFlash_read(CURRENT_OFFSET, 3, buf);
    if (res == 0)
    {
        current_offset = MIN_FLASH_OFFSET;
    }
    else
    {
        memcpy(&current_offset, buf, 3);
    }

    res = ExtFlash_read(LAST_OFFSET, 3, buf);
    if (res == 0)
    {
        last_offset = MIN_FLASH_OFFSET;
    }
    else
    {
        memcpy(&last_offset, buf, 3);
    }

    res = ExtFlash_read(CYCLE_OFFSET, 1, buf);
    if (res == 0)
    {
        if (current_offset < last_offset)
        {
            flag_cycle = 1;
        }
        else
        {
            flag_cycle = 0;
        }
    }
    else
    {
        memcpy(&flag_cycle, buf, 1);
    }

}


/*********************************************************************
 * @fn      updateOffsetsFlash
 *
 * @brief   Write the global pointers to the external flash. This
 *          function runs after storing a new measurement, and after
 *          sent data has been acknowledged by the peer device.
 *
 * @param   None.
 *
 * @return  None.
 */
static void updateOffsetsFlash()
{
    uint8_t buf[4];

    ExtFlash_erase(LAST_SECTOR * SECTOR_SIZE, SECTOR_SIZE);

    memcpy(buf, &current_offset, 3);
    ExtFlash_write(CURRENT_OFFSET, 3, buf);

    memcpy(buf, &last_offset, 3);
    ExtFlash_write(LAST_OFFSET, 3, buf);

    memcpy(buf, &flag_cycle, 1);
    ExtFlash_write(CYCLE_OFFSET, 1, buf);

}


/*********************************************************************
 * @fn      updateLastOffset
 *
 * @brief   Update the last offset global pointer. This function runs
 *          after sent data has been acknowledged by the peer device.
 *
 * @param   None.
 *
 * @return  None.
 */
static void updateLastOffset()
{
    last_offset = temp_last_offset;
    flag_cycle = temp_flag_cycle;

    updateOffsetsFlash();
}


/*********************************************************************
 * @fn      storeMeasurementFlash
 *
 * @brief   Store a new measurement on the external flash, and update
 *          the global pointers accordingly. It also ensures that the
 *          area to which it writes is always erased before writing.
 *
 * @param   buf - pointer to a buffer of fixed length
 *            MEASURMENT_LEN_BYTES, with a new measurement to store.
 *
 * @return  1 if writing succeeded, 0 otherwise
 */
bool storeMeasurementFlash(uint8_t *buf)
{
	bool res = 0;

	uint16_t current_sector     = current_offset / EXT_FLASH_PAGE_SIZE;
	uint16_t next_sector        = (current_offset + MEASURMENT_LEN_BYTES) / EXT_FLASH_PAGE_SIZE;
	uint16_t last_offset_sector = last_offset / EXT_FLASH_PAGE_SIZE;

	// entering a new sector - erase it before
	if (next_sector > current_sector)
	{
		// cycle to the first sector
		if (next_sector > LAST_SECTOR-1)
			next_sector = FIRST_SECTOR;

		// erase next sector
		ExtFlash_erase(next_sector * SECTOR_SIZE, SECTOR_SIZE);

		// fix last_offset to the following sector
		if (last_offset_sector == next_sector)
		{
			if (next_sector == LAST_SECTOR-1)
			{
			    last_offset = MIN_FLASH_OFFSET;
			    flag_cycle  = 0;
			}
			else
			{
			    last_offset = (next_sector + 1) * SECTOR_SIZE;
			}
		}
	}

	// writing data to flash
	res = ExtFlash_write(current_offset, MEASURMENT_LEN_BYTES, buf);

	// update offset if writing succeeded
	if (res == 1)
	{
		current_offset += MEASURMENT_LEN_BYTES;
		if (current_offset >= MAX_FLASH_OFFSET)
		{
		    current_offset = MIN_FLASH_OFFSET;

			// special case of a second cycle (as if "flag_cycle" = 2)
		    // this is the only situation where (flag_cycle == 1) && (current_offset < last_offset)
			if (flag_cycle == 1)
			{
			    last_offset = (next_sector + 1) * SECTOR_SIZE;
			}

			flag_cycle = 1;
		}

		if ((flag_cycle == 1) && (current_offset > last_offset))
		{
		    last_offset = (next_sector + 1) * SECTOR_SIZE;
		}
	}

	// handle the case we're storing between readings
	temp_last_offset = last_offset;
	temp_flag_cycle = flag_cycle;

	updateOffsetsFlash();
	return res;
}


/*********************************************************************
 * @fn      Datalogger_createTask
 *
 * @brief   Task creation function for the Datalogger
 *
 * @param   None.
 *
 * @return  None.
 */
void Datalogger_createTask(void)
{
  Task_Params taskParams;

  Task_Params_init(&taskParams);
  taskParams.stack      = dlTaskStack;
  taskParams.stackSize  = DL_TASK_STACK_SIZE;
  taskParams.priority   = DL_TASK_PRIORITY;

  Task_construct(&dlTask, Datalogger_taskFxn, &taskParams, NULL);
}


/*********************************************************************
 * @fn      Datalogger_init
 *
 * @brief   Task initialization function for the Datalogger. Opens
 *          the UART, the LED, and the external flash. Initialize
 *          global pointers.
 *
 * @param   None.
 *
 * @return  None.
 */
static void Datalogger_init(void)
{
	Semaphore_Params semParams;

	/* Construct a Semaphore object to be use as a resource lock, inital count 0 */
	Semaphore_Params_init(&semParams);
	Semaphore_construct(&semStruct, 0, &semParams);

	/* Obtain instance handle */
	semHandle = Semaphore_handle(&semStruct);

	/* Construct a Queue object for receiving massages */
	measurementQueueHandle = Queue_create(NULL, NULL);

	/* Open LED pins */
	ledPinHandle = PIN_open(&ledPinState, ledPinTable);

    /* Create a UART with data processing off. */
	UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode    = UART_DATA_BINARY;
    uartParams.readDataMode     = UART_DATA_BINARY;
    uartParams.readReturnMode   = UART_RETURN_FULL;
    uartParams.readMode         = UART_MODE_CALLBACK;
    uartParams.readCallback     = uartReadCallback;
    uartParams.readEcho         = UART_ECHO_OFF;
    uartParams.baudRate         = BAUD_RATE;

    uart = UART_open(Board_UART0, &uartParams);

    // Open external flash and erase the first sector
    ExtFlash_open();

    /*
     * This code should be run a single time only,
     * before starting using the Datalogger.
     * Its purpose is to write the pointer's initial
     * values to the external flash.
     *

    uint8_t buf[4];
    ExtFlash_erase(LAST_SECTOR * SECTOR_SIZE, SECTOR_SIZE);
    current_offset = MIN_FLASH_OFFSET;
    last_offset = MIN_FLASH_OFFSET;
    //current_offset = 0xFEFC0;
    //last_offset = 0xFEFC0;
    flag_cycle = 0;
    memcpy(buf, &current_offset, 4);
    ExtFlash_write(CURRENT_OFFSET, 4, buf);
    memcpy(buf, &last_offset, 4);
    ExtFlash_write(LAST_OFFSET, 4, buf);
    memcpy(buf, &flag_cycle, 1);
    ExtFlash_write(CYCLE_OFFSET, 1, buf);
    ExtFlash_erase(MIN_FLASH_OFFSET, SECTOR_SIZE);
    //ExtFlash_erase(0xFE000, SECTOR_SIZE);
    */


    readOffsetsFlash();
}


/*********************************************************************
 * @fn      Datalogger_taskFxn
 *
 * @brief   Application task entry point for the Datalogger
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
void Datalogger_taskFxn(UArg a0, UArg a1)
{
    bool res = 0;

	Datalogger_init();

	UART_read(uart, &rxMsg, MEASURMENT_LEN_BYTES);
	while (1)
    {
		Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
		while (!Queue_empty(measurementQueueHandle))
		{
		    measurement_t *meaPtr = Queue_dequeue(measurementQueueHandle);

			res = storeMeasurementFlash(meaPtr->data);
			free(meaPtr);
            if (res == 0)
            {
                break;
            }
		}
    }
}


/*********************************************************************
 * @fn      uartReadCallback
 *
 * @brief   A callback function for UART read. Enqueue the new
 *          received measurement, and post the semaphore to release
 *          the Datalogger task.
 *
 * @param   handle  - handle to the UART
 *          buf     - a buffer with the new received measurement
 *          count   - length of the data stored in buf
 *
 * @return  None.
 */
static void uartReadCallback(UART_Handle handle, void *buf, size_t count)
{
    // Enqueue new measurement
	measurement_t *measurmentPtr = (measurement_t *) malloc(sizeof(measurement_t));
    memcpy(measurmentPtr->data, buf, count);
	Queue_enqueue(measurementQueueHandle, &measurmentPtr->_elem);

	// Post Semaphore to release DL task
	Semaphore_post(semHandle);

	// Toggle the green LED and read another measurement
	PIN_setOutputValue(ledPinHandle, Board_LED1, !PIN_getOutputValue(Board_LED1));
	UART_read(uart, &rxMsg, MEASURMENT_LEN_BYTES);
}

/*********************************************************************
*********************************************************************/
