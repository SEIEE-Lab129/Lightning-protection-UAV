/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file mission_result.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define MISSION_RESULT_MISSION_EXECUTION_MODE_NORMAL 0
#define MISSION_RESULT_MISSION_EXECUTION_MODE_REVERSE 1
#define MISSION_RESULT_MISSION_EXECUTION_MODE_FAST_FORWARD 2

#endif


#ifdef __cplusplus
struct __EXPORT mission_result_s {
#else
struct mission_result_s {
#endif
	uint64_t timestamp;
	uint32_t instance_count;
	int32_t seq_reached;
	uint16_t seq_current;
	uint16_t seq_total;
	uint16_t item_changed_index;
	uint16_t item_do_jump_remaining;
	bool valid;
	bool warning;
	bool finished;
	bool failure;
	bool stay_in_failsafe;
	bool flight_termination;
	bool item_do_jump_changed;
	uint8_t execution_mode;


#ifdef __cplusplus
	static constexpr uint8_t MISSION_EXECUTION_MODE_NORMAL = 0;
	static constexpr uint8_t MISSION_EXECUTION_MODE_REVERSE = 1;
	static constexpr uint8_t MISSION_EXECUTION_MODE_FAST_FORWARD = 2;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(mission_result);


#ifdef __cplusplus
void print_message(const mission_result_s& message);
#endif
