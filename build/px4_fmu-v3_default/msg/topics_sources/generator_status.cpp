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

/* Auto-generated by genmsg_cpp from file generator_status.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/generator_status.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

constexpr char __orb_generator_status_fields[] = "uint64_t timestamp;uint64_t status;float battery_current;float load_current;float power_generated;float bus_voltage;float bat_current_setpoint;uint32_t runtime;int32_t time_until_maintenance;uint16_t generator_speed;int16_t rectifier_temperature;int16_t generator_temperature;uint8_t[6] _padding0;";

ORB_DEFINE(generator_status, struct generator_status_s, 50, __orb_generator_status_fields, static_cast<uint8_t>(ORB_ID::generator_status));


void print_message(const generator_status_s &message)
{

	PX4_INFO_RAW(" generator_status_s\n");

	const hrt_abstime now = hrt_absolute_time();

	if (message.timestamp != 0) {
		PX4_INFO_RAW("\ttimestamp: %" PRIu64 "  (%.6f seconds ago)\n", message.timestamp, (now - message.timestamp) / 1e6);
	} else {
		PX4_INFO_RAW("\n");
	}
	PX4_INFO_RAW("\tstatus: %" PRIu64 "\n", message.status);
	PX4_INFO_RAW("\tbattery_current: %.4f\n", (double)message.battery_current);
	PX4_INFO_RAW("\tload_current: %.4f\n", (double)message.load_current);
	PX4_INFO_RAW("\tpower_generated: %.4f\n", (double)message.power_generated);
	PX4_INFO_RAW("\tbus_voltage: %.4f\n", (double)message.bus_voltage);
	PX4_INFO_RAW("\tbat_current_setpoint: %.4f\n", (double)message.bat_current_setpoint);
	PX4_INFO_RAW("\truntime: %" PRIu32 "\n", message.runtime);
	PX4_INFO_RAW("\ttime_until_maintenance: %" PRId32 "\n", message.time_until_maintenance);
	PX4_INFO_RAW("\tgenerator_speed: %u\n", message.generator_speed);
	PX4_INFO_RAW("\trectifier_temperature: %d\n", message.rectifier_temperature);
	PX4_INFO_RAW("\tgenerator_temperature: %d\n", message.generator_temperature);
	
}