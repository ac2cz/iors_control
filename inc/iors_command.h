/*
 * iors_command.h
 *
 *  Created on: Nov 27, 2023
 *      Author: g0kla
 */

#ifndef IORS_COMMAND_H_
#define IORS_COMMAND_H_

#include <stdint.h>

#define AUTH_VECTOR_SIZE 32
#define SW_COMMAND_SIZE 18
#define OUR_ADDRESS 0x1A // Initial test address - redundant as we have a callsign??
#define COMMAND_TIME_TOLLERANCE 30 // An identical command received within this many seconds is ACK but ignored
#define MAX_COMMAND_TIME 2082690000 //Dec 31 2035
#define MIN_COMMAND_TIME 1672462800 //Dec 31 2022

/*
 * Following is the data structure representing software uplink commands
 */
typedef struct  __attribute__((__packed__)) {
	uint16_t command;
	uint16_t arguments[4];
} CommandAndArgs;


typedef struct  __attribute__((__packed__)){
	uint32_t dateTime;
	uint8_t reserved;
	uint8_t address;
	uint8_t special;
	uint8_t namespaceNumber;
	CommandAndArgs comArg;
    uint8_t AuthenticationVector[32];
} SWCmdUplink;

/*
 * Here are the definitions of the name spaces and commands
 */

typedef enum {
	 SWCmdNSReserved=0
	,SWCmdNSOps
	,SWCmdNSTelemetry
	,SWCmdNSPacsat
}SWCommandNameSpace;

typedef enum {
     SWCmdIntReserved=0
    ,SWCmdIntAutosafeMode
}SWInternalCommands;

typedef enum {
	 SWCmdOpsReserved=0
	,SWCmdOpsPM1 = 1
	,SWCmdOpsXBandRepeaterMode = 2
	,SWCmdOpsSSTVSend = 3
	,SWCmdOpsSSTVLoop = 4
	,SWCmdOpsSSTVStop = 5
	,SWCmdOpsAPRSMode = 6
	,SWCmdOpsPacsatMode = 7
	,SWCmdOpsExecuteScript = 8
	,SWCmdOpsTime = 9
    ,SWCmdOpsNumberOfCommands
}SWOpsCommands;

//const static uint8_t SWCmdOpsArgSize[SWCmdOpsNumberOfCommands]={0,0,0,1,0,0,0};

typedef enum {
    SWCmdTlmReserved=0
   ,SWCmdTlmNumberOfCommands
}SWTlmCommands;

//const static uint8_t SWCmdTlmArgSize[SWCmdTlmNumberOfCommands]={0,1,1};

typedef enum {
	 SwCmdPacsatReserved=0
	,SWCmdPacsatEnablePB
	,SWCmdPacsatEnableUplink
	,SWCmdPacsatDeleteFile
	,SWCmdPacsatInstallFile
	,SwCmdPacsatNumberOfCommands
}SWPacsatCommands;

int load_last_commmand_time();
//int DecodeSoftwareCommand(SWCmdUplink *softwareCommand);
int AuthenticateSoftwareCommand(SWCmdUplink *uplink);
//int DispatchSoftwareCommand(SWCmdUplink *uplink,bool local);

#endif /* IORS_COMMAND_H_ */
