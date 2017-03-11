/******************************************************************************

 @file  data_service.c

 @brief This file contains the implementation of the service.

 Target Device: CC2650

 ******************************************************************************
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <xdc/runtime/Diags.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "data_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// Data_Service Service UUID
CONST uint8_t DataServiceUUID[ATT_UUID_SIZE] =
{
  DATA_SERVICE_SERV_UUID_BASE128(DATA_SERVICE_SERV_UUID)
};

// Stream UUID
CONST uint8_t ds_StreamUUID[ATT_UUID_SIZE] =
{
  DS_STREAM_UUID_BASE128(DS_STREAM_UUID)
};


/*********************************************************************
 * LOCAL VARIABLES
 */

static DataServiceCBs_t *pAppCBs                = NULL;
static uint8_t          ds_icall_rsp_task_id    = INVALID_TASK_ID;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t DataServiceDecl             = { ATT_UUID_SIZE, DataServiceUUID };

// Characteristic "Stream" Properties (for declaration)
static uint8_t              ds_StreamProps              = GATT_PROP_INDICATE;

// Characteristic "Stream" Value variable
static uint8_t              ds_StreamVal[DS_STREAM_LEN] = {0};

// Length of data in characteristic "Stream" Value variable, initialized to minimal size.
static uint16_t             ds_StreamValLen             = DS_STREAM_LEN_MIN;

// Characteristic "Stream" Client Characteristic Configuration Descriptor
static gattCharCfg_t        *ds_StreamConfig;



/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t Data_ServiceAttrTbl[] =
{
 // Data_Service Service Declaration
 {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&DataServiceDecl
  },
    // Stream Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ds_StreamProps
    },
      // Stream Characteristic Value
      {
        { ATT_UUID_SIZE, ds_StreamUUID },
        GATT_PERMIT_WRITE,
        0,
        ds_StreamVal
      },
      // Stream CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&ds_StreamConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Data_Service_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method);
static bStatus_t Data_Service_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple Profile Service Callbacks
CONST gattServiceCBs_t Data_ServiceCBs =
{
  Data_Service_ReadAttrCB,      // Read callback function pointer
  Data_Service_WriteAttrCB,     // Write callback function pointer
  NULL                          // Authorization callback function pointer
};


/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * DataService_AddService- Initializes the DataService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t DataService_AddService(uint8_t rspTaskId)
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  ds_StreamConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) * linkDBNumConns);
  if (ds_StreamConfig == NULL)
  {
    return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, ds_StreamConfig);

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(Data_ServiceAttrTbl,
                                       GATT_NUM_ATTRS(Data_ServiceAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &Data_ServiceCBs);

  ds_icall_rsp_task_id = rspTaskId;

  return (status);
}

/*
 * DataService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t DataService_RegisterAppCBs( DataServiceCBs_t *appCallbacks )
{
  if (appCallbacks)
  {
    pAppCBs = appCallbacks;
    return (SUCCESS);
  }
  else
  {
    return (FAILURE);
  }
}

/*
 * DataService_SetParameter - Set a DataService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t DataService_SetParameter(uint8_t param, uint16_t len, void *value)
{
  bStatus_t     ret         = SUCCESS;
  uint8_t       *pAttrVal;
  uint16_t      *pValLen;
  uint16_t      valMinLen;
  uint16_t      valMaxLen;
  uint8_t       sendNotiInd = FALSE;
  gattCharCfg_t *attrConfig;
  uint8_t       needAuth;

  switch (param)
  {
    case DS_STREAM_ID:
      pAttrVal      =  ds_StreamVal;
      pValLen       = &ds_StreamValLen;
      valMinLen     =  DS_STREAM_LEN_MIN;
      valMaxLen     =  DS_STREAM_LEN;
      sendNotiInd   = TRUE;
      attrConfig    = ds_StreamConfig;
      needAuth      = TRUE;            // Change if authenticated link is required for sending
      break;

    default:
      return INVALIDPARAMETER;
  }

  // Check bounds, update value and send notification or indication if possible
  if ((len <= valMaxLen) && (len >= valMinLen))
  {
    memcpy(pAttrVal, value, len);
    *pValLen = len; // Update length for read and get

    if (sendNotiInd)
    {
      // Try to send indication
      GATTServApp_ProcessCharCfg(attrConfig, pAttrVal, needAuth,
                                 Data_ServiceAttrTbl, GATT_NUM_ATTRS(Data_ServiceAttrTbl),
                                 ds_icall_rsp_task_id,  Data_Service_ReadAttrCB);
    }
  }
  else
  {
    ret = bleInvalidRange;
  }

  return ret;
}


/*
 * DataService_GetParameter - Get a DataService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t DataService_GetParameter(uint8_t param, uint16_t *len, void *value)
{
  bStatus_t ret = SUCCESS;
  switch (param)
  {
    case DS_STREAM_ID:
      *len = MIN(*len, ds_StreamValLen);
      memcpy(value, ds_StreamVal, *len);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}

/*********************************************************************
 * @internal
 * @fn          Data_Service_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref data_service.h) or 0xFF if not found.
 */
static uint8_t Data_Service_findCharParamId(gattAttribute_t *pAttr)
{
  // Is this a Client Characteristic Configuration Descriptor?
  if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
    return Data_Service_findCharParamId(pAttr - 1); // Assume the value attribute precedes CCCD and recurse

  // Is this attribute in "Stream"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, ds_StreamUUID, pAttr->type.len))
    return DS_STREAM_ID;

  // Not found. Return invalid.
  else
    return 0xFF;
}

/*********************************************************************
 * @fn          Data_Service_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t Data_Service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status      = SUCCESS;
  uint16_t  valueLen;
  uint8_t   paramID     = 0xFF;

  // Find settings for the characteristic to be read
  paramID = Data_Service_findCharParamId(pAttr);
  switch (paramID)
  {
    case DS_STREAM_ID:
      valueLen = ds_StreamValLen;
      break;

    default:
      return ATT_ERR_ATTR_NOT_FOUND;
  }

  // Check bounds and return the value
  if (offset > valueLen)  // Prevent malicious ATT ReadBlob offsets
  {
    status = ATT_ERR_INVALID_OFFSET;
  }
  else
  {
    *pLen = MIN(maxLen, valueLen - offset);  // Transmit as much as possible
    memcpy(pValue, pAttr->pValue + offset, *pLen);
  }

  return status;
}

/*********************************************************************
 * @fn      Data_Service_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t Data_Service_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method)
{
  bStatus_t status          = SUCCESS;
  uint8_t   paramID         = 0xFF;
  uint16_t  writeLenMin;
  uint16_t  writeLenMax;
  uint16_t  *pValueLenVar;

  // See if request is regarding a Client Characterisic Configuration
  if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
  {
    // Allow notification and indication, but do not check if really allowed per CCCD
    status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                            offset, GATT_CLIENT_CFG_NOTIFY |
                                                     GATT_CLIENT_CFG_INDICATE);
    if (SUCCESS == status && pAppCBs && pAppCBs->pfnCfgChangeCb)
       pAppCBs->pfnCfgChangeCb(connHandle, DATA_SERVICE_SERV_UUID,
                                Data_Service_findCharParamId(pAttr), pValue, len);

     return status;
  }

  // Find settings for the characteristic to be written
  paramID = Data_Service_findCharParamId(pAttr);
  switch (paramID)
  {
    case DS_STREAM_ID:
      writeLenMin  = DS_STREAM_LEN_MIN;
      writeLenMax  = DS_STREAM_LEN;
      pValueLenVar = &ds_StreamValLen;
      break;

    default:
      return ATT_ERR_ATTR_NOT_FOUND;
  }

  // Check whether the length is within bounds
  if (offset >= writeLenMax)
  {
    status = ATT_ERR_INVALID_OFFSET;
  }

  else if (offset + len > writeLenMax)
  {
    status = ATT_ERR_INVALID_VALUE_SIZE;
  }

  else if ((offset + len < writeLenMin) &&
          ((method == ATT_EXECUTE_WRITE_REQ) || (method == ATT_WRITE_REQ)))
  {
    // Refuse writes that are lower than minimum.
    // Note: Cannot determine if a Reliable Write (to several chars) is finished, so those will
    //       only be refused if this attribute is the last in the queue (method is execute).
    //       Otherwise, reliable writes are accepted and parsed piecemeal.
    status = ATT_ERR_INVALID_VALUE_SIZE;
  }

  else
  {
    // Copy pValue into the variable we point to from the attribute table.
    memcpy(pAttr->pValue + offset, pValue, len);

    // Only notify application and update length if enough data is written.
    //
    // Note: If reliable writes are used (meaning several attributes are written to using ATT PrepareWrite),
    //       the application will get a callback for every write with an offset + len larger than _LEN_MIN.
    // Note: For Long Writes (ATT Prepare + Execute towards only one attribute) only one callback will be issued,
    //       because the write fragments are concatenated before being sent here.
    if (offset + len >= writeLenMin)
    {
      *pValueLenVar = offset + len; // Update data length
    }
  }

  return status;
}


