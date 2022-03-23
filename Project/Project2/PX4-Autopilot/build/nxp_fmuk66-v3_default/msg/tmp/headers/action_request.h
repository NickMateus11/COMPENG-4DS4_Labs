/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/ds/Documents/g16/PX4-Autopilot/msg/action_request.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define ACTION_REQUEST_ACTION_DISARM 0
#define ACTION_REQUEST_ACTION_ARM 1
#define ACTION_REQUEST_ACTION_TOGGLE_ARMING 2
#define ACTION_REQUEST_ACTION_UNKILL 3
#define ACTION_REQUEST_ACTION_KILL 4
#define ACTION_REQUEST_ACTION_SWITCH_MODE 5
#define ACTION_REQUEST_ACTION_VTOL_TRANSITION_TO_MULTICOPTER 6
#define ACTION_REQUEST_ACTION_VTOL_TRANSITION_TO_FIXEDWING 7
#define ACTION_REQUEST_SOURCE_RC_STICK_GESTURE 0
#define ACTION_REQUEST_SOURCE_RC_SWITCH 1
#define ACTION_REQUEST_SOURCE_RC_BUTTON 2
#define ACTION_REQUEST_SOURCE_RC_MODE_SLOT 3

#endif


#ifdef __cplusplus
struct __EXPORT action_request_s {
#else
struct action_request_s {
#endif
	uint64_t timestamp;
	uint8_t action;
	uint8_t source;
	uint8_t mode;
	uint8_t _padding0[5]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t ACTION_DISARM = 0;
	static constexpr uint8_t ACTION_ARM = 1;
	static constexpr uint8_t ACTION_TOGGLE_ARMING = 2;
	static constexpr uint8_t ACTION_UNKILL = 3;
	static constexpr uint8_t ACTION_KILL = 4;
	static constexpr uint8_t ACTION_SWITCH_MODE = 5;
	static constexpr uint8_t ACTION_VTOL_TRANSITION_TO_MULTICOPTER = 6;
	static constexpr uint8_t ACTION_VTOL_TRANSITION_TO_FIXEDWING = 7;
	static constexpr uint8_t SOURCE_RC_STICK_GESTURE = 0;
	static constexpr uint8_t SOURCE_RC_SWITCH = 1;
	static constexpr uint8_t SOURCE_RC_BUTTON = 2;
	static constexpr uint8_t SOURCE_RC_MODE_SLOT = 3;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(action_request);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const action_request_s& message);
#endif
