/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{206};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	action_request = 0,
	actuator_armed = 1,
	actuator_controls = 2,
	actuator_controls_0 = 3,
	actuator_controls_1 = 4,
	actuator_controls_2 = 5,
	actuator_controls_3 = 6,
	actuator_controls_status = 7,
	actuator_controls_status_0 = 8,
	actuator_controls_status_1 = 9,
	actuator_controls_virtual_fw = 10,
	actuator_controls_virtual_mc = 11,
	actuator_motors = 12,
	actuator_outputs = 13,
	actuator_outputs_sim = 14,
	actuator_servos = 15,
	actuator_servos_trim = 16,
	actuator_test = 17,
	adc_report = 18,
	airspeed = 19,
	airspeed_validated = 20,
	airspeed_wind = 21,
	autotune_attitude_control_status = 22,
	battery_status = 23,
	camera_capture = 24,
	camera_status = 25,
	camera_trigger = 26,
	cellular_status = 27,
	collision_constraints = 28,
	collision_report = 29,
	commander_state = 30,
	control_allocator_status = 31,
	cpuload = 32,
	debug_array = 33,
	debug_key_value = 34,
	debug_value = 35,
	debug_vect = 36,
	differential_pressure = 37,
	distance_sensor = 38,
	ekf2_timestamps = 39,
	esc_report = 40,
	esc_status = 41,
	estimator_attitude = 42,
	estimator_baro_bias = 43,
	estimator_event_flags = 44,
	estimator_global_position = 45,
	estimator_gps_status = 46,
	estimator_innovation_test_ratios = 47,
	estimator_innovation_variances = 48,
	estimator_innovations = 49,
	estimator_local_position = 50,
	estimator_odometry = 51,
	estimator_optical_flow_vel = 52,
	estimator_selector_status = 53,
	estimator_sensor_bias = 54,
	estimator_states = 55,
	estimator_status = 56,
	estimator_status_flags = 57,
	estimator_visual_odometry_aligned = 58,
	estimator_wind = 59,
	event = 60,
	failure_detector_status = 61,
	follow_target = 62,
	fw_virtual_attitude_setpoint = 63,
	generator_status = 64,
	geofence_result = 65,
	gimbal_device_attitude_status = 66,
	gimbal_device_information = 67,
	gimbal_device_set_attitude = 68,
	gimbal_manager_information = 69,
	gimbal_manager_set_attitude = 70,
	gimbal_manager_set_manual_control = 71,
	gimbal_manager_status = 72,
	gimbal_v1_command = 73,
	gps_dump = 74,
	gps_inject_data = 75,
	heater_status = 76,
	home_position = 77,
	hover_thrust_estimate = 78,
	input_rc = 79,
	internal_combustion_engine_status = 80,
	iridiumsbd_status = 81,
	irlock_report = 82,
	landing_gear = 83,
	landing_target_innovations = 84,
	landing_target_pose = 85,
	led_control = 86,
	log_message = 87,
	logger_status = 88,
	mag_worker_data = 89,
	magnetometer_bias_estimate = 90,
	manual_control_input = 91,
	manual_control_setpoint = 92,
	manual_control_switches = 93,
	mavlink_log = 94,
	mavlink_tunnel = 95,
	mc_virtual_attitude_setpoint = 96,
	mission = 97,
	mission_result = 98,
	mount_orientation = 99,
	navigator_mission_item = 100,
	npfg_status = 101,
	obstacle_distance = 102,
	obstacle_distance_fused = 103,
	offboard_control_mode = 104,
	onboard_computer_status = 105,
	optical_flow = 106,
	orb_multitest = 107,
	orb_test = 108,
	orb_test_large = 109,
	orb_test_medium = 110,
	orb_test_medium_multi = 111,
	orb_test_medium_queue = 112,
	orb_test_medium_queue_poll = 113,
	orb_test_medium_wrap_around = 114,
	orbit_status = 115,
	parameter_update = 116,
	ping = 117,
	position_controller_landing_status = 118,
	position_controller_status = 119,
	position_setpoint = 120,
	position_setpoint_triplet = 121,
	power_button_state = 122,
	power_monitor = 123,
	pps_capture = 124,
	pwm_input = 125,
	px4io_status = 126,
	radio_status = 127,
	rate_ctrl_status = 128,
	rc_channels = 129,
	rc_parameter_map = 130,
	rpm = 131,
	rtl_time_estimate = 132,
	safety = 133,
	satellite_info = 134,
	sensor_accel = 135,
	sensor_accel_fifo = 136,
	sensor_baro = 137,
	sensor_combined = 138,
	sensor_correction = 139,
	sensor_gps = 140,
	sensor_gyro = 141,
	sensor_gyro_fft = 142,
	sensor_gyro_fifo = 143,
	sensor_hygrometer = 144,
	sensor_mag = 145,
	sensor_preflight_mag = 146,
	sensor_selection = 147,
	sensors_status_imu = 148,
	system_power = 149,
	takeoff_status = 150,
	task_stack_info = 151,
	tecs_status = 152,
	telemetry_status = 153,
	test_motor = 154,
	timesync = 155,
	timesync_status = 156,
	trajectory_bezier = 157,
	trajectory_setpoint = 158,
	trajectory_waypoint = 159,
	transponder_report = 160,
	tune_control = 161,
	uavcan_parameter_request = 162,
	uavcan_parameter_value = 163,
	ulog_stream = 164,
	ulog_stream_ack = 165,
	vehicle_acceleration = 166,
	vehicle_air_data = 167,
	vehicle_angular_acceleration = 168,
	vehicle_angular_acceleration_setpoint = 169,
	vehicle_angular_velocity = 170,
	vehicle_angular_velocity_groundtruth = 171,
	vehicle_attitude = 172,
	vehicle_attitude_groundtruth = 173,
	vehicle_attitude_setpoint = 174,
	vehicle_command = 175,
	vehicle_command_ack = 176,
	vehicle_constraints = 177,
	vehicle_control_mode = 178,
	vehicle_global_position = 179,
	vehicle_global_position_groundtruth = 180,
	vehicle_gps_position = 181,
	vehicle_imu = 182,
	vehicle_imu_status = 183,
	vehicle_land_detected = 184,
	vehicle_local_position = 185,
	vehicle_local_position_groundtruth = 186,
	vehicle_local_position_setpoint = 187,
	vehicle_magnetometer = 188,
	vehicle_mocap_odometry = 189,
	vehicle_odometry = 190,
	vehicle_rates_setpoint = 191,
	vehicle_roi = 192,
	vehicle_status = 193,
	vehicle_status_flags = 194,
	vehicle_thrust_setpoint = 195,
	vehicle_torque_setpoint = 196,
	vehicle_trajectory_bezier = 197,
	vehicle_trajectory_waypoint = 198,
	vehicle_trajectory_waypoint_desired = 199,
	vehicle_vision_attitude = 200,
	vehicle_visual_odometry = 201,
	vtol_vehicle_status = 202,
	wheel_encoders = 203,
	wind = 204,
	yaw_estimator_status = 205,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
