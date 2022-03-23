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

/* Auto-generated by genmsg_cpp from file /home/ds/Documents/g16/PX4-Autopilot/msg/tecs_status.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define TECS_STATUS_TECS_MODE_NORMAL 0
#define TECS_STATUS_TECS_MODE_UNDERSPEED 1
#define TECS_STATUS_TECS_MODE_TAKEOFF 2
#define TECS_STATUS_TECS_MODE_LAND 3
#define TECS_STATUS_TECS_MODE_LAND_THROTTLELIM 4
#define TECS_STATUS_TECS_MODE_BAD_DESCENT 5
#define TECS_STATUS_TECS_MODE_CLIMBOUT 6

#endif


#ifdef __cplusplus
struct __EXPORT tecs_status_s {
#else
struct tecs_status_s {
#endif
	uint64_t timestamp;
	float altitude_sp;
	float altitude_filtered;
	float height_rate_setpoint;
	float height_rate;
	float equivalent_airspeed_sp;
	float true_airspeed_sp;
	float true_airspeed_filtered;
	float true_airspeed_derivative_sp;
	float true_airspeed_derivative;
	float true_airspeed_derivative_raw;
	float true_airspeed_innovation;
	float total_energy_error;
	float energy_distribution_error;
	float total_energy_rate_error;
	float energy_distribution_rate_error;
	float total_energy;
	float total_energy_rate;
	float total_energy_balance;
	float total_energy_balance_rate;
	float total_energy_sp;
	float total_energy_rate_sp;
	float total_energy_balance_sp;
	float total_energy_balance_rate_sp;
	float throttle_integ;
	float pitch_integ;
	float throttle_sp;
	float pitch_sp_rad;
	uint8_t mode;
	uint8_t _padding0[3]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t TECS_MODE_NORMAL = 0;
	static constexpr uint8_t TECS_MODE_UNDERSPEED = 1;
	static constexpr uint8_t TECS_MODE_TAKEOFF = 2;
	static constexpr uint8_t TECS_MODE_LAND = 3;
	static constexpr uint8_t TECS_MODE_LAND_THROTTLELIM = 4;
	static constexpr uint8_t TECS_MODE_BAD_DESCENT = 5;
	static constexpr uint8_t TECS_MODE_CLIMBOUT = 6;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(tecs_status);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const tecs_status_s& message);
#endif
