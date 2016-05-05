/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file sideslip_fusion.cpp
 * sideslip fusion methods.
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */
#include "ekf.h"
#include "mathlib.h"

void Ekf::fuseSideslip()
{ 
	// get latest estimated orientation
	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	float vn = _state.vel(0);
	float ve = _state.vel(1);
	float vd = _state.vel(2);

	// get latest wind velocity in earth frame
	float vwn = _state.wind_vel(0);
	float vwe = _state.wind_vel(1);

	float SH_BETA[?] = {}; // Varialbe used to optimise calculations of measurement jacobian
	float H_BETA[?] = {}; // Observation Jacobian
	float SK_BETA[?] = {}; // Varialbe used to optimise calculations of the Kalman gain vector
	float Kfusion[24] = {}; // Kalman gain vector

	// relative wind velocity in earth frame
    float rel_vwn = vn - vwn;
    float rel_vwe = ve - vwe;
    float rel_vwd = vd;

    // rotate into body axes
    vel_rel_wind = prevTnb * vel_rel_wind;
}