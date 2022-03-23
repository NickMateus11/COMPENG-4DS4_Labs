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

/* Auto-generated by genmsg_cpp from file /home/ds/Documents/g16/PX4-Autopilot/msg/estimator_selector_status.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT estimator_selector_status_s {
#else
struct estimator_selector_status_s {
#endif
	uint64_t timestamp;
	uint64_t last_instance_change;
	uint32_t instance_changed_count;
	uint32_t accel_device_id;
	uint32_t baro_device_id;
	uint32_t gyro_device_id;
	uint32_t mag_device_id;
	float combined_test_ratio[9];
	float relative_test_ratio[9];
	float accumulated_gyro_error[4];
	float accumulated_accel_error[4];
	uint8_t primary_instance;
	uint8_t instances_available;
	bool healthy[9];
	bool gyro_fault_detected;
	bool accel_fault_detected;
	uint8_t _padding0[7]; // required for logger


#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(estimator_selector_status);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const estimator_selector_status_s& message);
#endif
