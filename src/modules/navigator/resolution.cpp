#include "resolution.h"
#include <systemlib/mavlink_log.h>

void resolution::resolve_predicted_conflict(double *avoidance_lat, double *avoidance_lon)
{   
    //calculate distance to avoid conf
    float distance_to_avoid = _horizontal_separation + buffer - _encroachment;
    //Calculate change of heading
    float min_heading_change_to_avoid = atan(distance_to_avoid/_conflict_distance);

    float new_heading = _heading + min_heading_change_to_avoid;

    float dist_to_wp = sqrt(distance_to_avoid*distance_to_avoid + _conflict_distance*_conflict_distance);

    double self_lat_target, self_lon_target;
    waypoint_from_heading_and_distance(_self_pos[0], _self_pos[1], new_heading, dist_to_wp,
                                        &self_lat_target, &self_lon_target);

    *avoidance_lat = self_lat_target;
    *avoidance_lon = self_lon_target;
}
