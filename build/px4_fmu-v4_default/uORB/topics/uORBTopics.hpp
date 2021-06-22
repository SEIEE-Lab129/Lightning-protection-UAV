/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

static constexpr size_t ORB_TOPICS_COUNT{183};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	actuator_armed = 0,
	actuator_controls = 1,
	actuator_controls_0 = 2,
	actuator_controls_1 = 3,
	actuator_controls_2 = 4,
	actuator_controls_3 = 5,
	actuator_controls_4 = 6,
	actuator_controls_5 = 7,
	actuator_controls_virtual_fw = 8,
	actuator_controls_virtual_mc = 9,
	actuator_outputs = 10,
	adc_report = 11,
	airspeed = 12,
	airspeed_validated = 13,
	airspeed_wind = 14,
	battery_status = 15,
	camera_capture = 16,
	camera_trigger = 17,
	camera_trigger_secondary = 18,
	cellular_status = 19,
	collision_constraints = 20,
	collision_report = 21,
	commander_state = 22,
	control_allocator_status = 23,
	cpuload = 24,
	debug_array = 25,
	debug_key_value = 26,
	debug_value = 27,
	debug_vect = 28,
	differential_pressure = 29,
	distance_sensor = 30,
	ekf2_timestamps = 31,
	ekf_gps_drift = 32,
	esc_report = 33,
	esc_status = 34,
	estimator_attitude = 35,
	estimator_event_flags = 36,
	estimator_global_position = 37,
	estimator_innovation_test_ratios = 38,
	estimator_innovation_variances = 39,
	estimator_innovations = 40,
	estimator_local_position = 41,
	estimator_odometry = 42,
	estimator_optical_flow_vel = 43,
	estimator_selector_status = 44,
	estimator_sensor_bias = 45,
	estimator_states = 46,
	estimator_status = 47,
	estimator_status_flags = 48,
	estimator_visual_odometry_aligned = 49,
	estimator_wind = 50,
	follow_target = 51,
	fw_virtual_attitude_setpoint = 52,
	generator_status = 53,
	geofence_result = 54,
	gimbal_device_attitude_status = 55,
	gimbal_device_information = 56,
	gimbal_device_set_attitude = 57,
	gimbal_manager_information = 58,
	gimbal_manager_set_attitude = 59,
	gimbal_manager_set_manual_control = 60,
	gimbal_manager_status = 61,
	gps_dump = 62,
	gps_inject_data = 63,
	heater_status = 64,
	home_position = 65,
	hover_thrust_estimate = 66,
	input_rc = 67,
	iridiumsbd_status = 68,
	irlock_report = 69,
	landing_gear = 70,
	landing_target_innovations = 71,
	landing_target_pose = 72,
	led_control = 73,
	log_message = 74,
	logger_status = 75,
	mag_worker_data = 76,
	manual_control_setpoint = 77,
	manual_control_switches = 78,
	mavlink_log = 79,
	mc_virtual_attitude_setpoint = 80,
	mission = 81,
	mission_result = 82,
	mount_orientation = 83,
	multirotor_motor_limits = 84,
	navigator_mission_item = 85,
	obstacle_distance = 86,
	obstacle_distance_fused = 87,
	offboard_control_mode = 88,
	onboard_computer_status = 89,
	optical_flow = 90,
	orbit_status = 91,
	parameter_update = 92,
	ping = 93,
	position_controller_landing_status = 94,
	position_controller_status = 95,
	position_setpoint = 96,
	position_setpoint_triplet = 97,
	power_button_state = 98,
	power_monitor = 99,
	pwm_input = 100,
	px4io_status = 101,
	qshell_req = 102,
	qshell_retval = 103,
	radio_status = 104,
	rate_ctrl_status = 105,
	rc_channels = 106,
	rc_parameter_map = 107,
	rpm = 108,
	rtl_flight_time = 109,
	safety = 110,
	satellite_info = 111,
	sensor_accel = 112,
	sensor_accel_fifo = 113,
	sensor_baro = 114,
	sensor_combined = 115,
	sensor_correction = 116,
	sensor_gps = 117,
	sensor_gyro = 118,
	sensor_gyro_fft = 119,
	sensor_gyro_fifo = 120,
	sensor_mag = 121,
	sensor_preflight_mag = 122,
	sensor_selection = 123,
	sensors_status_imu = 124,
	system_power = 125,
	takeoff_status = 126,
	task_stack_info = 127,
	tecs_status = 128,
	telemetry_status = 129,
	test_motor = 130,
	timesync = 131,
	timesync_status = 132,
	trajectory_bezier = 133,
	trajectory_setpoint = 134,
	trajectory_waypoint = 135,
	transponder_report = 136,
	tune_control = 137,
	uavcan_parameter_request = 138,
	uavcan_parameter_value = 139,
	ulog_stream = 140,
	ulog_stream_ack = 141,
	vehicle_acceleration = 142,
	vehicle_actuator_setpoint = 143,
	vehicle_air_data = 144,
	vehicle_angular_acceleration = 145,
	vehicle_angular_acceleration_setpoint = 146,
	vehicle_angular_velocity = 147,
	vehicle_angular_velocity_groundtruth = 148,
	vehicle_attitude = 149,
	vehicle_attitude_groundtruth = 150,
	vehicle_attitude_setpoint = 151,
	vehicle_command = 152,
	vehicle_command_ack = 153,
	vehicle_constraints = 154,
	vehicle_control_mode = 155,
	vehicle_global_position = 156,
	vehicle_global_position_groundtruth = 157,
	vehicle_gps_position = 158,
	vehicle_imu = 159,
	vehicle_imu_status = 160,
	vehicle_land_detected = 161,
	vehicle_local_position = 162,
	vehicle_local_position_groundtruth = 163,
	vehicle_local_position_setpoint = 164,
	vehicle_magnetometer = 165,
	vehicle_mocap_odometry = 166,
	vehicle_odometry = 167,
	vehicle_rates_setpoint = 168,
	vehicle_roi = 169,
	vehicle_status = 170,
	vehicle_status_flags = 171,
	vehicle_thrust_setpoint = 172,
	vehicle_torque_setpoint = 173,
	vehicle_trajectory_bezier = 174,
	vehicle_trajectory_waypoint = 175,
	vehicle_trajectory_waypoint_desired = 176,
	vehicle_vision_attitude = 177,
	vehicle_visual_odometry = 178,
	vtol_vehicle_status = 179,
	wheel_encoders = 180,
	wind = 181,
	yaw_estimator_status = 182,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
