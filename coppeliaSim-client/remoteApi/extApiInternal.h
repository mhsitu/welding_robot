#ifndef __EXTAPIINTERNAL_
#define __EXTAPIINTERNAL_

#include "extApiPlatform.h"

#define SIMX_INIT_BUFF_SIZE 500
#define SIMX_MIN_BUFF_INCR 500

#define _REPLY_WAIT_TIMEOUT_IN_MS 5000
#define _MIN_SPLIT_AMOUNT_IN_BYTES 100

/* Out buffer for messages */
simxUChar* _messageToSend[MAX_EXT_API_CONNECTIONS];
simxInt _messageToSend_bufferSize[MAX_EXT_API_CONNECTIONS];
simxInt _messageToSend_dataSize[MAX_EXT_API_CONNECTIONS];

/* Temp buffer for split commands to send */
simxUChar* _splitCommandsToSend[MAX_EXT_API_CONNECTIONS];
simxInt _splitCommandsToSend_bufferSize[MAX_EXT_API_CONNECTIONS];
simxInt _splitCommandsToSend_dataSize[MAX_EXT_API_CONNECTIONS];

/* In buffer for messages */
simxUChar* _messageReceived[MAX_EXT_API_CONNECTIONS];
simxInt _messageReceived_bufferSize[MAX_EXT_API_CONNECTIONS];
simxInt _messageReceived_dataSize[MAX_EXT_API_CONNECTIONS];

/* Temp buffer for split commands received */
simxUChar* _splitCommandsReceived[MAX_EXT_API_CONNECTIONS];
simxInt _splitCommandsReceived_bufferSize[MAX_EXT_API_CONNECTIONS];
simxInt _splitCommandsReceived_dataSize[MAX_EXT_API_CONNECTIONS];

/* Temp buffer for last fetched command */
simxUChar* _commandReceived[MAX_EXT_API_CONNECTIONS];
simxInt _commandReceived_bufferSize[MAX_EXT_API_CONNECTIONS];
simxInt _commandReceived_simulationTime[MAX_EXT_API_CONNECTIONS];

/* Other variables */
simxInt _nextConnectionID[MAX_EXT_API_CONNECTIONS];
simxInt _replyWaitTimeoutInMs[MAX_EXT_API_CONNECTIONS];

simxInt _connectionPort[MAX_EXT_API_CONNECTIONS];
simxChar* _connectionIP[MAX_EXT_API_CONNECTIONS];

simxInt _minCommunicationDelay[MAX_EXT_API_CONNECTIONS];
simxUChar _communicationThreadRunning[MAX_EXT_API_CONNECTIONS];
simxInt _nextMessageIDToSend[MAX_EXT_API_CONNECTIONS];
simxInt _waitBeforeSendingAgainWhenMessageIDArrived[MAX_EXT_API_CONNECTIONS];
simxInt _lastReceivedMessageID[MAX_EXT_API_CONNECTIONS];
simxInt _connectionID[MAX_EXT_API_CONNECTIONS];
const simxChar* _tempConnectionAddress[MAX_EXT_API_CONNECTIONS];
simxInt _tempConnectionPort[MAX_EXT_API_CONNECTIONS];
simxUChar _tempDoNotReconnectOnceDisconnected[MAX_EXT_API_CONNECTIONS];

simxUChar* _tmpBuffer[MAX_EXT_API_CONNECTIONS];


#endif /* __EXTAPIINTERNAL_ */
