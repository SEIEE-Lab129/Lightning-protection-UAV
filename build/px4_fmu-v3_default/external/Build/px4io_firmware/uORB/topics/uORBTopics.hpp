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

static constexpr size_t ORB_TOPICS_COUNT{179};
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
	differential_pressure = 25,
	distance_sensor = 26,
	ekf2_timestamps = 27,
	ekf_gps_drift = 28,
	esc_report = 29,
	esc_status = 30,
	estimator_attitude = 31,
	estimator_event_flags = 32,
	estimator_global_position = 33,
	estimator_innovation_test_ratios = 34,
	estimator_innovation_variances = 35,
	estimator_innovations = 36,
	estimator_local_position = 37,
	estimator_odometry = 38,
	estimator_optical_flow_vel = 39,
	estimator_selector_status = 40,
	estimator_sensor_bias = 41,
	estimator_states = 42,
	estimator_status = 43,
	estimator_status_flags = 44,
	estimator_visual_odometry_aligned = 45,
	estimator_wind = 46,
	follow_target = 47,
	fw_virtual_attitude_setpoint = 48,
	generator_status = 49,
	geofence_result = 50,
	gimbal_device_attitude_status = 51,
	gimbal_device_information = 52,
	gimbal_device_set_attitude = 53,
	gimbal_manager_information = 54,
	gimbal_manager_set_attitude = 55,
	gimbal_manager_set_manual_control = 56,
	gimbal_manager_status = 57,
	gps_dump = 58,
	gps_inject_data = 59,
	heater_status = 60,
	home_position = 61,
	hover_thrust_estimate = 62,
	input_rc = 63,
	iridiumsbd_status = 64,
	irlock_report = 65,
	landing_gear = 66,
	landing_target_innovations = 67,
	landing_target_pose = 68,
	led_control = 69,
	log_message = 70,
	logger_status = 71,
	mag_worker_data = 72,
	manual_control_setpoint = 73,
	manual_control_switches = 74,
	mavlink_log = 75,
	mc_virtual_attitude_setpoint = 76,
	mission = 77,
	mission_result = 78,
	mount_orientation = 79,
	multirotor_motor_limits = 80,
	navigator_mission_item = 81,
	obstacle_distance = 82,
	obstacle_distance_fused = 83,
	offboard_control_mode = 84,
	onboard_computer_status = 85,
	optical_flow = 86,
	orbit_status = 87,
	parameter_update = 88,
	ping = 89,
	position_controller_landing_status = 90,
	position_controller_status = 91,
	position_setpoint = 92,
	position_setpoint_triplet = 93,
	power_button_state = 94,
	power_monitor = 95,
	pwm_input = 96,
	px4io_status = 97,
	qshell_req = 98,
	qshell_retval = 99,
	radio_status = 100,
	rate_ctrl_status = 101,
	rc_channels = 102,
	rc_parameter_map = 103,
	rpm = 104,
	rtl_flight_time = 105,
	safety = 106,
	satellite_info = 107,
	sensor_accel = 108,
	sensor_accel_fifo = 109,
	sensor_baro = 110,
	sensor_combined = 111,
	sensor_correction = 112,
	sensor_gps = 113,
	sensor_gyro = 114,
	sensor_gyro_fft = 115,
	sensor_gyro_fifo = 116,
	sensor_mag = 117,
	sensor_preflight_mag = 118,
	sensor_selection = 119,
	sensors_status_imu = 120,
	system_power = 121,
	takeoff_status = 122,
	task_stack_info = 123,
	tecs_status = 124,
	telemetry_status = 125,
	test_motor = 126,
	timesync = 127,
	timesync_status = 128,
	trajectory_bezier = 129,
	trajectory_setpoint = 130,
	trajectory_waypoint = 131,
	transponder_report = 132,
	tune_control = 133,
	uavcan_parameter_request = 134,
	uavcan_parameter_value = 135,
	ulog_stream = 136,
	ulog_stream_ack = 137,
	vehicle_acceleration = 138,
	vehicle_actuator_setpoint = 139,
	vehicle_air_data = 140,
	vehicle_angular_acceleration = 141,
	vehicle_angular_acceleration_setpoint = 142,
	vehicle_angular_velocity = 143,
	vehicle_angular_velocity_groundtruth = 144,
	vehicle_attitude = 145,
	vehicle_attitude_groundtruth = 146,
	vehicle_attitude_setpoint = 147,
	vehicle_command = 148,
	vehicle_command_ack = 149,
	vehicle_constraints = 150,
	vehicle_control_mode = 151,
	vehicle_global_position = 152,
	vehicle_global_position_groundtruth = 153,
	vehicle_gps_position = 154,
	vehicle_imu = 155,
	vehicle_imu_status = 156,
	vehicle_land_detected = 157,
	vehicle_local_position = 158,
	vehicle_local_position_groundtruth = 159,
	vehicle_local_position_setpoint = 160,
	vehicle_magnetometer = 161,
	vehicle_mocap_odometry = 162,
	vehicle_odometry = 163,
	vehicle_rates_setpoint = 164,
	vehicle_roi = 165,
	vehicle_status = 166,
	vehicle_status_flags = 167,
	vehicle_thrust_setpoint = 168,
	vehicle_torque_setpoint = 169,
	vehicle_trajectory_bezier = 170,
	vehicle_trajectory_waypoint = 171,
	vehicle_trajectory_waypoint_desired = 172,
	vehicle_vision_attitude = 173,
	vehicle_visual_odometry = 174,
	vtol_vehicle_status = 175,
	wheel_encoders = 176,
	wind = 177,
	yaw_estimator_status = 178,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
