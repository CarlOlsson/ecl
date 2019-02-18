/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ecl_l1_pos_controller.h
 * Implementation of L1 position control.
 * Authors and acknowledgements in header.
 *
 */

#include <float.h>

#include "ecl_l1_pos_controller.h"

float ECL_L1_Pos_Controller::nav_roll()
{
	float ret = atanf(_lateral_accel * 1.0f / CONSTANTS_ONE_G);
	ret = math::constrain(ret, -_roll_lim_rad, _roll_lim_rad);
	return ret;
}

float ECL_L1_Pos_Controller::nav_lateral_acceleration_demand()
{
	return _lateral_accel;
}

float ECL_L1_Pos_Controller::nav_bearing()
{
	return matrix::wrap_pi(_nav_bearing);
}

float ECL_L1_Pos_Controller::bearing_error()
{
	return _bearing_error;
}

float ECL_L1_Pos_Controller::target_bearing()
{
	return _target_bearing;
}

float ECL_L1_Pos_Controller::switch_distance(float wp_radius)
{
	/* following [2], switching on L1 distance */
	return math::min(wp_radius, _L1_distance);
}

bool ECL_L1_Pos_Controller::reached_loiter_target(void)
{
	return _circle_mode;
}

float ECL_L1_Pos_Controller::crosstrack_error(void)
{
	return _crosstrack_error;
}

void ECL_L1_Pos_Controller::navigate_waypoints(const matrix::Vector2f &vector_A, const matrix::Vector2f &vector_B,
		const matrix::Vector2f &vector_curr_position,
		const matrix::Vector2f &ground_speed_vector, float airspeed, const float heading)
{

	/* this follows the logic presented in [1] */

	float eta = 0.0f;
	float xtrack_vel;
	float ltrack_vel;

	/* get the direction between the last (visited) and next waypoint */
	_target_bearing = get_bearing_to_next_waypoint(vector_curr_position(0), vector_curr_position(1), vector_B(0),
			  vector_B(1));

	/* calculate ground speed */
	float ground_speed = ground_speed_vector.length();

	/* Reset L1 ratio as it might have been changed during loitering */
	_L1_ratio = 1.0f / M_PI_F * _L1_damping * _L1_period;

	/* calculate the L1 length required for the desired period */
	_L1_distance = _L1_ratio * ground_speed;

	/* calculate vector from A to B */
	matrix::Vector2f vector_AB = get_local_planar_vector(vector_A, vector_B);

	/*
	 * check if waypoints are on top of each other. If yes,
	 * skip A and directly continue to B
	 */
	if (vector_AB.length() < 1.0e-6f) {
		vector_AB = get_local_planar_vector(vector_curr_position, vector_B);
	}

	/* WINGTRA: Protection against producing NaNs.
	 * When triggering RTH, current waypoint is set to current global position,
	 * previous waypoint is reset to current waypoint and we have all three equal
	 * which leads to NaN at normalization as vector's length is zero.
	 * The same can happen for certain types of waypoints
	 */
	if (vector_AB.length() > 1.0e-6f) {
		vector_AB.normalize();

	}
	/* WINGTRA: End */

	/* calculate the vector from waypoint A to the aircraft */
	matrix::Vector2f vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position);

	/* calculate crosstrack error (output only) */
	_crosstrack_error = vector_AB % vector_A_to_airplane;

	/*
	 * If the current position is in a +-135 degree angle behind waypoint A
	 * and further away from A than the L1 distance, then A becomes the L1 point.
	 * If the aircraft is already between A and B normal L1 logic is applied.
	 */
	float distance_A_to_airplane = vector_A_to_airplane.length();
	float alongTrackDist = vector_A_to_airplane * vector_AB;

	/* estimate airplane position WRT to B */
	matrix::Vector2f vector_B_to_P_unit = get_local_planar_vector(vector_B, vector_curr_position).normalized();

	/* calculate angle of airplane position vector relative to line) */

	// XXX this could probably also be based solely on the dot product
	float AB_to_BP_bearing = atan2f(vector_B_to_P_unit % vector_AB, vector_B_to_P_unit * vector_AB);

	if (ground_speed < FLT_EPSILON) {
		eta = 0.0f;  // already facing wind

	} else {
		/* extension from [2], fly directly to A */
		if (distance_A_to_airplane > _L1_distance && alongTrackDist / math::max(distance_A_to_airplane, 1.0f) < -1.0f * M_SQRT1_2_F) {  // correstponds to sin(pi/4)

			/* calculate eta to fly to waypoint A */

			/* unit vector from waypoint A to current position */
			matrix::Vector2f vector_A_to_airplane_unit = vector_A_to_airplane.normalized();
			/* velocity across / orthogonal to line */
			xtrack_vel = ground_speed_vector % (-vector_A_to_airplane_unit);
			/* velocity along line */
			ltrack_vel = ground_speed_vector * (-vector_A_to_airplane_unit);
			eta = atan2f(xtrack_vel, ltrack_vel);
			/* bearing from current position to L1 point */
			_nav_bearing = atan2f(-vector_A_to_airplane_unit(1) , -vector_A_to_airplane_unit(0));

			/*
			 * If the AB vector and the vector from B to airplane point in the same
			 * direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
			 */

		} else if (fabsf(AB_to_BP_bearing) < math::radians(100.0f)) {
			/*
			 * Extension, fly back to waypoint.
			 *
			 * This corner case is possible if the system was following
			 * the AB line from waypoint A to waypoint B, then is
			 * switched to manual mode (or otherwise misses the waypoint)
			 * and behind the waypoint continues to follow the AB line.
			 */

			/* calculate eta to fly to waypoint B */

			/* velocity across / orthogonal to line */
			xtrack_vel = ground_speed_vector % (-vector_B_to_P_unit);
			/* velocity along line */
			ltrack_vel = ground_speed_vector * (-vector_B_to_P_unit);
			eta = atan2f(xtrack_vel, ltrack_vel);
			/* bearing from current position to L1 point */
			_nav_bearing = atan2f(-vector_B_to_P_unit(1) , -vector_B_to_P_unit(0));

		} else {

			/* calculate eta to fly along the line between A and B */

			/* velocity across / orthogonal to line */
			xtrack_vel = ground_speed_vector % vector_AB;
			/* velocity along line */
			ltrack_vel = ground_speed_vector * vector_AB;
			/* calculate eta2 (angle of velocity vector relative to line) */
			float eta2 = atan2f(xtrack_vel, ltrack_vel);
			/* calculate eta1 (angle to L1 point) */
			float xtrackErr = vector_A_to_airplane % vector_AB;
			/* limit L1 distance to xtrack error. --from [3]
			 * effectively commands a perpendicular approach when:
			 * - the aircraft is far from the track
			 * - when the ground speed is  small
			 * */
			//if (_L1_distance < fabsf(xtrackErr)) _L1_distance = fabsf(xtrackErr); //not necessary if 45 degree approach imposed below
			float sine_eta1 = xtrackErr / _L1_distance;
			/* limit output to 45 degrees */
			sine_eta1 = math::constrain(sine_eta1, -1.0f * M_SQRT1_2_F, M_SQRT1_2_F); //sin(pi/4) = M_SQRT1_2_F
			float eta1 = asin_protected(sine_eta1);  // WINGTRA: using protected function to avoid NaNs
			eta = eta1 + eta2;
			/* bearing from current position to L1 point */
			_nav_bearing = atan2f(vector_AB(1), vector_AB(0)) + eta1;

		}

		/* extension from [3] */
		/* calculate wind
		 * NOTES:
		 * -this assumes no sideslip
		 * -should probably be done with an observer if no EKF estimates of these values are available
		 * */
		if (airspeed < 1.0f) {
			airspeed = 1.0f;  // just a safe guard in case the loop is run while on ground
		}

		matrix::Vector2f wind_speed_vector(ground_speed_vector(0) - airspeed * cosf(heading),
						  ground_speed_vector(1) - airspeed * sinf(heading));
		float wind_bearing = atan2f(wind_speed_vector(1), wind_speed_vector(0));
		float wind_speed = wind_speed_vector.length();

		if (wind_speed > airspeed) {
			/* calculate ground tracking bounds */
			float ground_speed_bearing_bnd = asin_protected(airspeed / wind_speed);  // WINGTRA: using protected function to avoid NaNs
			float ground_speed_bnd_min = matrix::wrap_pi(wind_bearing - ground_speed_bearing_bnd);
			float ground_speed_bnd_max = matrix::wrap_pi(wind_bearing + ground_speed_bearing_bnd);

			/* check L1 bearing feasibility */
			if (checkBearingTarget(_nav_bearing, ground_speed_bnd_min, ground_speed_bnd_max)) {
				/* bearing is feasible, but must command heading to avoid multiple ground speed vector solutions */
				float eta_wind = matrix::wrap_pi(_nav_bearing - wind_bearing);
				float eta_airspeed = asin_protected(wind_speed * sinf(fabsf(eta_wind)) / airspeed);  // WINGTRA: using protected function to avoid NaNs
				float L1_heading = matrix::wrap_pi(_nav_bearing + (fabsf(eta_wind) < FLT_EPSILON ? 0.0f : eta_wind / fabsf(
								    eta_wind)) * eta_airspeed);
				eta = matrix::wrap_pi(L1_heading - heading);

			} else {
				/* turn into wind. now a heading controller */
				_nav_bearing = matrix::wrap_pi(wind_bearing - (fabsf(wind_bearing) < FLT_EPSILON ? 0.0f : wind_bearing / fabsf(
						wind_bearing)) * M_PI_F);
				eta = matrix::wrap_pi(_nav_bearing - heading);
			}
		}
	}

	/* limit angle to +-90 degrees */
	eta = math::constrain(eta, (-M_PI_F) / 2.0f, +M_PI_F / 2.0f);
	_lateral_accel = _K_L1 * ground_speed / _L1_ratio * sinf(eta);

	/* flying to waypoints, not circling them */
	_circle_mode = false;

	/* the bearing angle, in NED frame */
	_bearing_error = eta; // NOTE: this actually becomes heading errors in the windy scenarios
}

void ECL_L1_Pos_Controller::navigate_loiter(const matrix::Vector2f &vector_A,
		const matrix::Vector2f &vector_curr_position, float radius, int8_t loiter_direction,
		const matrix::Vector2f &ground_speed_vector, float airspeed, const float heading)
{
	/* from [1] and [2] and modified/extended by [3] */
	float eta = 0.0f;

	/* update bearing to next waypoint */
	_target_bearing = get_bearing_to_next_waypoint(vector_curr_position(0), vector_curr_position(1), vector_A(0),
			  vector_A(1));

	/* calculate ground speed */
	float ground_speed = ground_speed_vector.length();

	/* calculate the L1 length required for the desired period */
	_L1_distance = _L1_ratio * ground_speed;

	/* check circle tracking feasibility */
	if (_L1_distance / radius > 1.0f  && ground_speed > 0.0f) {
		/* reduce period, recalculate L1 ratio & distance */
		_L1_ratio = radius / ground_speed;
		_L1_distance = _L1_ratio * ground_speed;
	}

	/* calculate the vector from waypoint A to current position */
	matrix::Vector2f vector_A_to_airplane = get_local_planar_vector(vector_A, vector_curr_position);
	matrix::Vector2f vector_A_to_airplane_unit;

	/* prevent NaN when normalizing */
	if (vector_A_to_airplane.length() > FLT_EPSILON) {
		/* store the normalized vector from waypoint A to current position */
		vector_A_to_airplane_unit = vector_A_to_airplane.normalized();

	} else {
		vector_A_to_airplane_unit = vector_A_to_airplane;
	}

	/* calculate the distance from the aircraft to the circle */
	float dist_to_circle = vector_A_to_airplane.length() - radius;

	/* check that L1 vector does not exceed reasonable bounds */
	if (_L1_distance >= (2.0f * radius + dist_to_circle)) {
		_L1_distance = 2.0f * radius + dist_to_circle;

	} else if (_L1_distance < fabsf(dist_to_circle)) {
		_L1_distance = fabsf(dist_to_circle);
	}

	if (ground_speed < FLT_EPSILON) {
		eta = 0.0f;  // already facing wind

	} else {
		/* calculate L1 bearing */
		/* WINGTRA: fly to tangent: if far away, calculate the angle between the connection to the midpoint and the circle tangent. then add it to the already
		computed target bearing (to circle center) */
		if ((radius + dist_to_circle) * (radius + dist_to_circle) - radius * radius > _L1_distance * _L1_distance) {
			/* Head directly to the tangent point */
			_nav_bearing = matrix::wrap_pi(asin_protected(-float(loiter_direction) * math::constrain(radius / (radius + dist_to_circle), -1.0f, 1.0f))
				+ _target_bearing);

		} else {
		/* WINGTRA END */
			// Use cosine law to calculate gamma, the angle between the connection to the midpoint of the circle and the intersection of l1 with the circle
			float cos_gam = (_L1_distance * _L1_distance + (dist_to_circle + radius) * (dist_to_circle + radius) - radius * radius)
					/ 2.0f / _L1_distance / (dist_to_circle + radius);
			cos_gam = math::constrain(cos_gam, -1.0f, 1.0f);
			float gam = acos_protected(cos_gam);  // WINGTRA: using protected function to avoid NaNs
			_nav_bearing = matrix::wrap_pi(atan2f(-vector_A_to_airplane(1), -vector_A_to_airplane(0)) - float(loiter_direction) * gam);
		}

		/* estimate wind */ //NOTE: this assumes no sideslip

		if (airspeed < 1.0f) {
			airspeed = 1.0f;  // just a safe guard in case the loop is run while on ground
		}

		matrix::Vector2f wind_speed_vector(ground_speed_vector(0) - airspeed * cosf(heading),
						  ground_speed_vector(1) - airspeed * sinf(heading));
		float wind_bearing = atan2f(wind_speed_vector(1), wind_speed_vector(0));
		float wind_speed = wind_speed_vector.length();

		/* calculate error angle eta */
		if (wind_speed > airspeed) {
			/* calculate ground tracking bounds */
			float ground_speed_bearing_bnd = asin_protected(airspeed / wind_speed);  // WINGTRA: using protected function to avoid NaNs
			float ground_speed_bnd_min = matrix::wrap_pi(wind_bearing - ground_speed_bearing_bnd);
			float ground_speed_bnd_max = matrix::wrap_pi(wind_bearing + ground_speed_bearing_bnd);

			/* check L1 bearing feasibility */
			if (checkBearingTarget(_nav_bearing, ground_speed_bnd_min, ground_speed_bnd_max)) {
				/* bearing is feasible, but must command heading to avoid multiple ground speed vector solutions */
				float eta_wind = matrix::wrap_pi(_nav_bearing - wind_bearing);
				float eta_airspeed = asin_protected(wind_speed * sinf(fabsf(eta_wind)) / airspeed);  // sine law, WINGTRA: using protected function to avoid NaNs
				float L1_heading = matrix::wrap_pi(_nav_bearing + (fabsf(eta_wind) < FLT_EPSILON ? 0.0f : eta_wind / fabsf(
								    eta_wind)) * eta_airspeed);
				eta = matrix::wrap_pi(L1_heading - heading);

			} else {
				/* turn into wind. now a heading controller */
				_nav_bearing = matrix::wrap_pi(wind_bearing - (fabsf(wind_bearing) < FLT_EPSILON ? 0.0f : wind_bearing / fabsf(
						wind_bearing)) * M_PI_F);
				eta = matrix::wrap_pi(_nav_bearing - heading);
			}

		} else {
			float ground_speed_bearing = atan2f(ground_speed_vector(1), ground_speed_vector(0));
			eta = matrix::wrap_pi(_nav_bearing - ground_speed_bearing);
		}
	}

	/* limit angle to +-90 degrees */
	eta = math::constrain(eta, (-1.0f * M_PI_F) / 2.0f, M_PI_F / 2.0f);
	_lateral_accel = _K_L1 * ground_speed / _L1_ratio * sinf(eta);

	_circle_mode = true;
	_bearing_error = eta; // NOTE: this actually becomes heading errors in the windy scenarios
}


void ECL_L1_Pos_Controller::navigate_heading(float navigation_heading, float current_heading,
		const matrix::Vector2f &ground_speed_vector)
{
	/* the complete guidance logic in this section was proposed by [2] */

	float eta;

	/*
	 * As the commanded heading is the only reference
	 * (and no crosstrack correction occurs),
	 * target and navigation bearing become the same
	 */
	_target_bearing = _nav_bearing = matrix::wrap_pi(navigation_heading);
	eta = _target_bearing - matrix::wrap_pi(current_heading);
	eta = matrix::wrap_pi(eta);

	/* consequently the bearing error is exactly eta: */
	_bearing_error = eta;

	/* ground speed is the length of the ground speed vector */
	float ground_speed = ground_speed_vector.length();

	/* adjust L1 distance to keep constant frequency */
	_L1_distance = ground_speed / _heading_omega;
	float omega_vel = ground_speed * _heading_omega;

	/* not circling a waypoint */
	_circle_mode = false;

	/* navigating heading means by definition no crosstrack error */
	_crosstrack_error = 0;

	/* limit eta to 90 degrees */
	eta = math::constrain(eta, (-M_PI_F) / 2.0f, +M_PI_F / 2.0f);
	_lateral_accel = 2.0f * sinf(eta) * omega_vel;
}

void ECL_L1_Pos_Controller::navigate_level_flight(float current_heading)
{
	/* the logic in this section is trivial, but originally proposed by [2] */

	/* reset all heading / error measures resulting in zero roll */
	_target_bearing = current_heading;
	_nav_bearing = current_heading;
	_bearing_error = 0;
	_crosstrack_error = 0;
	_lateral_accel = 0;

	/* not circling a waypoint when flying level */
	_circle_mode = false;

}


matrix::Vector2f ECL_L1_Pos_Controller::get_local_planar_vector(const matrix::Vector2f &origin,
		const matrix::Vector2f &target) const
{
	/* this is an approximation for small angles, proposed by [2] */

	matrix::Vector2f out(math::radians((target(0) - origin(0))),
			    math::radians((target(1) - origin(1))*cosf(math::radians(origin(0)))));

	return out * static_cast<float>(CONSTANTS_RADIUS_OF_EARTH);
}

bool ECL_L1_Pos_Controller::checkBearingTarget(float bearing, float bnd_min, float bnd_max)
{
	/* ASSUMES POSITIVE (CLOCKWISE) BND SWEEP FROM MIN TO MAX AND ALL BEARINGS BETWEEN -PI AND PI */
	if (bnd_min > bnd_max) {
		return (bearing >= -1.0f * M_PI_F && bearing <= bnd_max) || (bearing >= bnd_min && bearing <= M_PI_F);

	} else {
		return (bearing >= bnd_min && bearing <= bnd_max);
	}
}
