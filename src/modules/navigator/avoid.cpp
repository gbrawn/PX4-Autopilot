/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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

#include "avoid.h"
#include "navigator.h"

Avoid::Avoid(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	rep = _navigator->get_reposition_triplet();
	pos_sp_triplet = _navigator->get_position_setpoint_triplet();
}

void
Avoid::on_activation()
{
	reposition();
}

void
Avoid::on_active()
{
	if (_navigator->get_reposition_triplet()->current.valid) {
		reposition();
	}

	avoid_complete = is_avoid_complete();
	if (avoid_complete)
	{
		vehicle_command_s vcmd = {};
		mavlink_log_info(&_mavlink_log_pub, "Conflict cleared, restarting mission");
		vcmd.command = vehicle_command_s::VEHICLE_CMD_MISSION_START;
		_navigator->publish_vehicle_cmd(&vcmd);
	}

	_navigator->check_traffic();
}

void
Avoid::reposition()
{
	// we can't reposition if we are not armed yet
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	//struct position_setpoint_triplet_s *rep = _navigator->get_reposition_triplet();

	if (rep->current.valid) {
		// set position based on reposition command

		// convert mission item to current setpoint
		pos_sp_triplet->current.velocity_valid = false;
		pos_sp_triplet->previous.yaw = _navigator->get_local_position()->heading;
		pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
		memcpy(&pos_sp_triplet->current, &rep->current, sizeof(rep->current));
		pos_sp_triplet->next.valid = false;

		_navigator->set_can_loiter_at_sp(false);
		_navigator->set_position_setpoint_triplet_updated();

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}
}

bool
Avoid::is_avoid_complete()
{

	int current_lat = (int)((_navigator->get_global_position()->lat)*1E4);
	int current_lon = (int)((_navigator->get_global_position()->lon)*1E4);

	int target_lat = (int)((pos_sp_triplet->current.lat)*1E4);
	int target_lon = (int)((pos_sp_triplet->current.lon)*1E4);

	//get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

	if ((current_lat == target_lat) && (current_lon == target_lon))
	{
		avoid_complete = true;
		return avoid_complete;

	} else {

		avoid_complete = false;
		return avoid_complete;
	}
}