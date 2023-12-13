/*
 * iors_command.c
 *
 *  Created on: Nov 27, 2023
 *      Author: g0kla
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "config.h"
#include "iors_command.h"
#include "hmac_sha256.h"
#include "keyfile.h"
#include "radio.h"
#include "iors_controller.h"

/* Forwards */
int store_last_command_time();
int CommandTimeOK(SWCmdUplink *uplink);
//int OpsSWCommands(CommandAndArgs *comarg);
//int TlmSWCommands(CommandAndArgs *comarg);

uint32_t last_command_time = 0x0; /* Keep track of the time that the last command was received */

int load_last_commmand_time() {
	FILE * fd = fopen(g_iors_last_command_time_path, "r");
	if (fd == NULL) {
		// no command time file, make a new one
		fd = fopen(g_iors_last_command_time_path, "w");
		if (fd == NULL) {
			error_print("Could not create the time command file\n");
			// This is not fatal, but we dont remember the command time after restart
		}
		fprintf(fd, "%d\n", last_command_time);
	} else {
		char line [ MAX_CONFIG_LINE_LENGTH ]; /* or other suitable maximum line size */
		fgets ( line, sizeof line, fd ); /* read a line */
		line[strcspn(line,"\n")] = 0; // Move the nul termination to get rid of the new line
		last_command_time = atol(line);
		//int pid = strtol(line, NULL, 0);
		debug_print("Last Command Time was: %d\n",last_command_time);
	}
	fclose(fd);
	return EXIT_SUCCESS;
}

int store_last_command_time() {
	FILE * fd = fopen(g_iors_last_command_time_path, "w");
	if (fd != NULL) {
		fprintf(fd, "%d\n", last_command_time);
	} else {
		// Not fatal but we will forget the time when we restart
		error_print("Could not write to the time command file\n");
	}
	fclose(fd);
	return EXIT_SUCCESS;
}

#ifdef OLD
/**
 * DecodeSoftwareCommand()
 *
 */
int DecodeSoftwareCommand(SWCmdUplink *softwareCommand) {

    if(AuthenticateSoftwareCommand(softwareCommand)){
        //debug_print("Command: Authenticated!\n");
        /*
         * Here we have a command that was received on the uplink and ready to act on.
         */
        int rc = DispatchSoftwareCommand(softwareCommand,true);
        return rc;
    } else {
        debug_print("Command: does not authenticate\n");
    }
    return false;
}
#endif

int AuthenticateSoftwareCommand(SWCmdUplink *uplink) {
    uint8_t localSecureHash[32];
    bool shaOK;

    hmac_sha256(hmac_sha_key, AUTH_KEY_SIZE,
                (uint8_t *) uplink, SW_COMMAND_SIZE,
                localSecureHash, sizeof(localSecureHash));
    shaOK = (memcmp(localSecureHash, uplink->AuthenticationVector, 32) == 0);
    if (0) {
        debug_print("Local: ");
        int i;
        for (i=0; i<sizeof(localSecureHash);i++)
        	debug_print("%x ", localSecureHash[i]);
        debug_print("\nUplink: ");
        for (i=0; i<sizeof(uplink->AuthenticationVector);i++)
        	debug_print("%x ", uplink->AuthenticationVector[i]);
        debug_print("\n");
    }
    if(shaOK){
        uplink->comArg.command = (uplink->comArg.command); // We might have to look to determine if authenticated
        return CommandTimeOK(uplink);
    } else {
       // localErrorCollection.DCTCmdFailAuthenticateCnt++;
        return false;
    }

}

/**
 * CommandTimeOK()
 * Here we attempt to prevent a replay attack.  Firstly we keep track of the time that the last
 * command was received.  Our clock on the space station may be wrong but we assume the time
 * in the receieved packet is correct.  A command can not have a time that is the same or
 * before a previous command.
 *
 * If a command was not received but was intercepted by another station then they could replay the
 * command.  This is a small risk because we probablly wanted to send the command anyway.  This risk
 * is also ended once a new command is received as the time of the later packet will have been saved.
 *
 * If our clock is working on the station then we could check that the time on a received packet is
 * close to the actual time.  This mitigates replay attacks for packets that are not received but
 * we may not be able to send commands if the time on the station is wrong or corrupted.
 *
 * One other nuance.  It is common for the ground station to send a command, which is accepted, but
 * to miss the ACK.  It then sends the command again.  These commands have the same dateTime.  A
 * duplicate command received within a tolerance period based should receive an ACK but should
 * not make further changes.  This means that commands should check if they have just executed and
 * do nothing if called again.  e.g. don't change channels if already on that channel.
 *
 */
int CommandTimeOK(SWCmdUplink *uplink) {

	if (last_command_time > MAX_COMMAND_TIME || last_command_time < MIN_COMMAND_TIME) {
		// Then it is likely corrupt, set to zero for this check and take the time from the command
		last_command_time = 0;
	}
    if ((uplink->dateTime + COMMAND_TIME_TOLLERANCE) <= last_command_time) {
    	debug_print("Command: Bad time on command!\n");
    	debug_print("Command: %d Last Command %d\n",uplink->dateTime, last_command_time);
    	return false;
    } else {
    	last_command_time = uplink->dateTime;
    	store_last_command_time();
    }
    return true;
}

#ifdef OLD
int DispatchSoftwareCommand(SWCmdUplink *uplink,bool local) {
    uint8_t nameSpace;

    /*
     * Ok, we have a command that has been CRC checked, authenticated, timestamped,
     * etc.
     *
     * Next we check other stuff that has to be right.  Specifically, we check the satellite address
     * and then authenticate, and also check the timestamp for something reasonable.
     */
    nameSpace = uplink->namespaceNumber;

    // LEAVE THE ADDRESS AS THIS COULD BE USED TO APPLY TO SUB MODULES - but dont validate it for now
    if(uplink->address != OUR_ADDRESS){
        //debug_print("Wrong address %x\n\r",uplink->address);
        //return FALSE;
    }
//    if(local && (nameSpace != SWCmdNSInternal)){
//        //Enter in the telemetry ring buffer only if it originated on this
//        //CPU (and don't report internal commands)
//        INSERT_IN_PAIR_RING_BUFFER(nameSpace,uplink->comArg.command);
//    }

//    /*
//     * If Command Time Checking was enabled and we got a command, we know it is working
//     * so we can turn off the timeout function
//     */
//    if(CommandTimeEnabled) {  // - is this missing { }??  Otherwise it applies to the command_print?? - { } added by G0KLA
//        //SetInternalSchedules(NoTimeCommandTimeout,TIMEOUT_NONE);
//    }

//    debug_print("Namespace=%d,command=%d,arg=%d\n",nameSpace,uplink->comArg.command,
//                  uplink->comArg.arguments[0]);

    switch(nameSpace){
    case SWCmdNSOps: {
        return OpsSWCommands(&uplink->comArg);
    }
    case SWCmdNSTelemetry:{
        return TlmSWCommands(&uplink->comArg);
    }
    default:
        printf("Command: Unknown namespace %d\n\r",nameSpace);
      //  localErrorCollection.DCTCmdFailNamespaceCnt++;
        return FALSE;

    }
}

/*
 * Just for initial pacsat code most of the operations that commands do are commented out.  We just
 * print the command so the uplink can be tested.
 */

int OpsSWCommands(CommandAndArgs *comarg) {

    switch((int)comarg->command){

    case SWCmdOpsXBandRepeaterMode: {
       // bool turnOn;
       // turnOn = (comarg->arguments[0] != 0);
       // if(turnOn){
        	if (g_iors_control_state != STATE_CROSS_BAND_REPEATER) {
        		debug_print("Command: Cross band Repeater mode\n");
        		if (radio_set_cross_band_mode() == EXIT_SUCCESS)
        			g_iors_control_state = STATE_CROSS_BAND_REPEATER;
        	} else {
        		debug_print("Command: Already in Cross band Repeater mode\n");
        	}
//        } else {
//        	if (g_iors_control_state != STATE_SSTV_MODE) {
//        		debug_print("Enable SSTV\n\r");
//        		if (radio_set_sstv_mode() == EXIT_SUCCESS)
//        			g_iors_control_state = STATE_SSTV_MODE;
//        	}
//        }
        break;
    }
    case SWCmdOpsSSTVMode: {
    	if (g_iors_control_state != STATE_SSTV) {
        	debug_print("Command: Enable SSTV\n");
        	if (radio_set_sstv_mode() == EXIT_SUCCESS)
        		g_iors_control_state = STATE_SSTV;
    	} else {
        	debug_print("Command: Already in SSTV mode\n");
    	}
    	break;
    }

     default:
//        localErrorCollection.DCTCmdFailCommandCnt++;
        debug_print("Command: Unknown Ops Command %d\n\r",comarg->command);
        return FALSE;
    }
    return TRUE;
}
//void EnableCommandPrint(bool enable){
//    PrintCommandInfo = enable;
//}
//void EnableCommandTimeCheck(bool enable){
//    CommandTimeEnabled = enable;
//    if(enable){
//        //SetInternalSchedules(NoTimeCommandTimeout,SW_COMMAND_TIME_TIMEOUT);
//    } else {
//        //SetInternalSchedules(NoTimeCommandTimeout,TIMEOUT_NONE);
//    }
//    WriteMRAMBoolState(StateCommandTimeCheck,CommandTimeEnabled);
//}

int TlmSWCommands(CommandAndArgs *comarg) {
   switch(comarg->command){
//    case SWCmdTlmWODSaveSize:
//        command_print("Change WOD size to %d\n\r",comarg->arguments[0]);
//        //ChangeWODSaved(comarg->arguments[0]);
//        break;
//
//    case SWCmdTlmLegacyGain:
//        command_print("Tlm legacy gain to %d\n\r",comarg->arguments[0]);
//        break;
//
//    case SWCmdTlmDCTDrivePwr:{
//        int myCpuIndex = 0;
//        uint16_t lowPower = comarg->arguments[myCpuIndex]; // Arg 0 and 1 are for low
//        uint16_t highPower = comarg->arguments[myCpuIndex+2]; //Arg 2 and 3 are for high
//        command_print("Drive power reg for this proc are Low: %d, high: %d\n",lowPower,highPower);
//        //SetDCTDriveValue(lowPower,highPower);
//        break;
//    }
//
    default:
     //   localErrorCollection.DCTCmdFailCommandCnt++;
        debug_print("Command: Unknown Tlm Command\n\r");
        return FALSE;
    }
    return TRUE;
}

#endif
