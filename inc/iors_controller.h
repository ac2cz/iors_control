/*
 * iors_controller.h
 *
 *  Created on: Dec 1, 2023
 *      Author: g0kla
 */

#ifndef IORS_CONTROLLER_H_
#define IORS_CONTROLLER_H_

#include "iors_command.h"

#define TIMER_T1_DEFAULT_LIMIT 3
#define TIMER_T3_DEFAULT_LIMIT 60 * 3; // 3 min timeout
#define TIMER_T4_DEFAULT_LIMIT 60 * 60 * 8; // 8 hours
#define PERIOD_TO_CHECK_RADIO_CONNECTED 10 // 60*1 // Every minute
#define PERIOD_TO_CHECK_TNC_CONNECTED 60*1 // Every minute
#define PERIOD_TO_CHECK_PROGRAMS 2 // Every 2 seconds

/* IORS Control State machine */
enum IORS_Controller_State {
	STATE_INIT
	,STATE_RADIO_CONNECTED
	,STATE_TNC_CONNECTED
	,STATE_CROSS_BAND_REPEATER
	,STATE_APRS
	,STATE_SSTV
	,STATE_EMCOMM
	,STATE_PACSAT_CONNECTED
};
//#define STATE_INIT 0
//#define STATE_RADIO_CONNECTED 1
//#define STATE_TNC_CONNECTED 2
//#define STATE_PACKET_MODE 3
//#define STATE_SSTV_MODE 4
//#define STATE_X_BAND_REPEATER_MODE 5
//#define STATE_X_SAFE_MODE 9

/* SSTV State Machine */
#define STATE_SSTV_INIT 0
#define STATE_SSTV_SEND 1
#define STATE_SSTV_MONITOR 2

enum IORS_Event {
	 EVENT_RADIO_CONNECTED
	,EVENT_RADIO_DISCONNECTED
	,EVENT_TNC_CONNECTED
	,EVENT_TNC_DISCONNECTED
	,EVENT_SSTV_EXITED
	,EVENT_TNC_EXITED
	,EVENT_PACSAT_EXITED
	,COMMAND_RX
	,TIMER_T1_EXPIRED
	,TIMER_T3_EXPIRED
	,TIMER_T4_EXPIRED
};

struct t_iors_event {
	enum IORS_Event event;
	uint8_t nameSpace;
	CommandAndArgs comarg;
};


void iors_control_loop();

#endif /* IORS_CONTROLLER_H_ */
