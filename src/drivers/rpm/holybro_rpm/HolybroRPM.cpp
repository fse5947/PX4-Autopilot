/****************************************************************************
 *
 *   Copyright (c) 2014-2019, 2021 PX4 Development Team. All rights reserved.
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


/**
 * @file HolybroRPM.h
 * @author Brendan Patience <brendan.patience@shearwater.ai>
 *
 * Driver for the Holybro RPM sensor connected via PWM.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */

#include "HolybroRPM.h"

#include <px4_arch/io_timer.h>

HolybroRPM::HolybroRPM() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	param_get(param_find("SENS_NUM_POLES"), &_num_poles);

	float max_rpm = 0.0;
	param_get(param_find("SENS_MAX_RPM"), &max_rpm);

	_min_period = convert(max_rpm);

	PX4_INFO("Minimum Period set at %f", _min_period);

}

HolybroRPM::~HolybroRPM()
{
	stop();
}

int
HolybroRPM::init()
{
	start();

	return PX4_OK;
}

void
HolybroRPM::start()
{
	ScheduleOnInterval(get_measure_interval());
}

void
HolybroRPM::stop()
{
	ScheduleClear();
}

void
HolybroRPM::Run()
{
	measure();
}


double
HolybroRPM::convert(double value) const {

	if (value < DBL_EPSILON)
	{
		return INFINITY;
	}

	return 2*60*1e6 / (value * _num_poles);
}

int
HolybroRPM::measure()
{
	rpm_s measured_rpm{};
	if (PX4_OK != collect()) {
		PX4_DEBUG("collection error");
		return PX4_ERROR;
	}

	if (_pwm.period > _min_period) {
		measured_rpm.indicated_frequency_rpm = convert(_pwm.period);

		measured_rpm.timestamp = hrt_absolute_time();
		_rpm_pub.publish(measured_rpm);
	}

	return PX4_OK;
}

int
HolybroRPM::collect()
{
	pwm_input_s pwm_input;

	if (_sub_pwm_input.update(&pwm_input)) {

		_pwm = pwm_input;

		return PX4_OK;
	}

	return EAGAIN;
}

void
HolybroRPM::print_info()
{
	printf("Hello, this is the print_info() function");
}
