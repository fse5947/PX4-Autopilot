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
#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/pwm_input.h>
#include <uORB/topics/rpm.h>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <board_config.h>
#include <drivers/device/device.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

// Normal conversion wait time.
static constexpr uint32_t HOLYBRORPM_CONVERSION_INTERVAL{50_ms};

class HolybroRPM : public px4::ScheduledWorkItem
{
public:
	HolybroRPM();
	virtual ~HolybroRPM();

	int init();
	void start();
	void stop();

	void print_info();

protected:

	int collect();
	int measure();

	void Run() override;

private:

	float convert(int value) const;
	uint32_t get_measure_interval() const { return HOLYBRORPM_CONVERSION_INTERVAL; };
	int32_t _num_poles{2};
	int32_t _min_rpm{100};
	double _min_period{0.0};

	int _count{0};

	uORB::Subscription _sub_pwm_input{ORB_ID(pwm_input)};
	uORB::PublicationMulti<rpm_s> _rpm_pub{ORB_ID(rpm)};

	pwm_input_s _pwm{};
	rpm_s _rpm {};
};
