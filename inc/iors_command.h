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

/*
 * Following is the data structure representing software uplink commands
 */
typedef struct  __attribute__((__packed__)) {
	uint16_t command;
	uint16_t arguments[4];
} CommandAndArgs;


typedef struct  __attribute__((__packed__)){
	uint16_t resetNumber;
	unsigned int secondsSinceReset:24; // Actually this will be pairs of seconds, i.e. seconds/2
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
	,SWCmdOpsSafeMode
	,SWCmdOpsXBandRepeaterMode = 8 // enable pb
	,SWCmdOpsSSTVMode = 12 // enable uplink
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
	,SwCmdPacsatNumberOfCommands
}SWPacsatCommands;

int DecodeSoftwareCommand(SWCmdUplink *softwareCommand);
int AuthenticateSoftwareCommand(SWCmdUplink *uplink);
int DispatchSoftwareCommand(SWCmdUplink *uplink,bool local);

#endif /* IORS_COMMAND_H_ */
