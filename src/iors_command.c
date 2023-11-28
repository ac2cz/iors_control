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

/* Forwards */
int CommandTimeOK(SWCmdUplink *uplink);
int OpsSWCommands(CommandAndArgs *comarg);
int TlmSWCommands(CommandAndArgs *comarg);


/**
 * DecodeSoftwareCommand()
 *
 */
int DecodeSoftwareCommand(SWCmdUplink *softwareCommand) {

    if(AuthenticateSoftwareCommand(softwareCommand)){
        debug_print("\n\rCommand Authenticated!\n");

        softwareCommand->comArg.arguments[0] = (softwareCommand->comArg.arguments[0]);
        softwareCommand->comArg.arguments[1] = (softwareCommand->comArg.arguments[1]);
        softwareCommand->comArg.arguments[2] = (softwareCommand->comArg.arguments[2]);
        softwareCommand->comArg.arguments[3] = (softwareCommand->comArg.arguments[3]);

        /*
         * Here we have a command that was received on the uplink and ready to act on.
         */

        int rc = DispatchSoftwareCommand(softwareCommand,true);

        return rc;
    } else {
        debug_print("\n\rCommand does not authenticate\n");
    }
    return false;

}

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

int CommandTimeOK(SWCmdUplink *uplink) {
//    logicalTime_t timeNow;
//    static int lastCommandTime = 0;
//    int secondsOff,uplinkSeconds;
//    uint16_t uplinkEpoch;
    int goodTime = true;
//
//    /*
//     * Here we are checking the time included in the command to make sure it is within the appropriate
//     * range of the satellite time.  This is to avoid replay attacks.
//     *
//     * The algorithm is as follows:
//     *
//     * 1) The reset epoch in the command must match the satellite exactly
//     * 2) The seconds in the command MUST be greater than the seconds in the last command
//     * 3) The seconds in the command has to be within a tolerance (specified in FoxConfig) of the satellite time
//     */
//    uplinkEpoch = ttohs(uplink->resetNumber);
//    uplinkSeconds = ttoh24(uplink->secondsSinceReset);
//
//    if(CommandTimeEnabled){
//        /*
//         * If command time is not enabled, always return true, i.e. it's ok.
//         * Also as a safety valve, allow us to disable the time check if we can't seem to get it right by
//         * putting in a specific random number as the 4th argument for the disable command
//         */
//        if((uplink->namespaceNumber == SWCmdNSSpaceCraftOps) &&
//                (uplink->comArg.command == SWCmdOpsEnableCommandTimeCheck)  &&
//                (uplink->comArg.arguments[3] == SPECIAL_RANDOM_NUMBER)
//        ) return true;
//        getTimestamp(&timeNow);
//        secondsOff = uplinkSeconds - timeNow.METcount;
//        command_print("UplinkTime,delta %d/%d,%d\n\r",uplink->resetNumber,uplink->secondsSinceReset,secondsOff);
//        if(
//                /*
//                 * The reset epoch in the command HAS to match the satellite, and the satellite time must not
//                 * be greater than the "expiration time" in the command.
//                 *
//                 * In addition, to avoid a reply attack, the command HAS to be newer than the last command received.
//                 * And to avoid totally circumventing this whole check, the transmitted time can't be too far behind
//                 * or too far ahead of the current satellite time.
//                 */
//
//                (uplinkEpoch != timeNow.IHUresetCnt)   || // The uplinked reset count is wrong
//                (uplinkSeconds <= lastCommandTime)  || // The uplinked second time is less or same as that of last command
//                (uplinkSeconds < timeNow.METcount) || // The uplinked command has expired
//                (secondsOff < SECONDS_AUTHENTICATION_MAX_BEHIND) || // Uplinked time too far behind sat time
//                (secondsOff > SECONDS_AUTHENTICATION_MAX_AHEAD)  // They uplink had an expiration time too far ahead
//        ){
//            goodTime = false;
//            localErrorCollection.DCTCmdFailTimeCnt++;
//        }
//        if(goodTime)lastCommandTime = uplinkSeconds; // Do not allow the next command to have the same or earlier
//    }
    return goodTime;
}

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

    debug_print("Namespace=%d,command=%d,arg=%d\n",nameSpace,uplink->comArg.command,
                  uplink->comArg.arguments[0]);

    switch(nameSpace){
    case SWCmdNSOps: {
        return OpsSWCommands(&uplink->comArg);
    }
    case SWCmdNSTelemetry:{
        return TlmSWCommands(&uplink->comArg);
    }
    default:
        printf("Unknown namespace %d\n\r",nameSpace);
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
        bool turnOn;
        turnOn = (comarg->arguments[0] != 0);
        if(turnOn){
        	if (g_iors_control_state != STATE_X_BAND_REPEATER_MODE) {
        		debug_print("Enable Repeater\n\r");
        		if (radio_set_cross_band_mode() == EXIT_SUCCESS)
        			g_iors_control_state = STATE_X_BAND_REPEATER_MODE;
        	}
        } else {
        	if (g_iors_control_state != STATE_SSTV_MODE) {
        		debug_print("Disable Repeater\n\r");
        		if (radio_set_sstv_mode() == EXIT_SUCCESS)
        			g_iors_control_state = STATE_SSTV_MODE;
        	}
        }
        break;
    }

     default:
//        localErrorCollection.DCTCmdFailCommandCnt++;
        debug_print("Unknown Ops Command %d\n\r",comarg->command);
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
        debug_print("Unknown Tlm Command\n\r");
        return FALSE;
    }
    return TRUE;
}
