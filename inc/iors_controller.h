/*
 * iors_controller.h
 *
 *  Created on: Dec 1, 2023
 *      Author: g0kla
 */

#ifndef IORS_CONTROLLER_H_
#define IORS_CONTROLLER_H_

/* IORS Control State machine */
#define STATE_INIT 0
#define STATE_RADIO_CONNECTED 1
#define STATE_TNC_CONNECTED 2
#define STATE_PACKET_MODE 3
#define STATE_SSTV_MODE 4
#define STATE_X_BAND_REPEATER_MODE 5
#define STATE_X_SAFE_MODE 9

/* SSTV State Machine */
#define STATE_SSTV_INIT 0
#define STATE_SSTV_SEND 1
#define STATE_SSTV_MONITOR 2

void iors_control_loop();

#endif /* IORS_CONTROLLER_H_ */
