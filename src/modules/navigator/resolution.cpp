#include "resolution.h"
#include <systemlib/mavlink_log.h>

void resolution::resolve_predicted_conflict(double *avoidance_lat, double *avoidance_lon)
{   
    
    //calculate bearing to traffic
    float bearing_to_traf = get_bearing_to_next_waypoint(self_pos[0], self_pos[1], traff_pos[0], traff_pos[1]);

    float relative_bearing = fabs(heading - bearing_to_traf);
    
    //calculate distance to avoid conf
    float distance_to_avoid = horizontal_separation + buffer - encroachment;
    //Calculate change of heading
    float min_heading_change_to_avoid = atan(distance_to_avoid/conflict_distance);

    if (relative_bearing > math::radians(180)) 
    {

        float new_heading = heading - min_heading_change_to_avoid;

        float dist_to_wp = sqrt(distance_to_avoid*distance_to_avoid + conflict_distance*conflict_distance);

        double self_lat_target, self_lon_target;
        waypoint_from_heading_and_distance(self_pos[0], self_pos[1], new_heading, dist_to_wp,
                                            &self_lat_target, &self_lon_target);

        *avoidance_lat = self_lat_target;
        *avoidance_lon = self_lon_target;

    } else {

        float new_heading = heading + min_heading_change_to_avoid;

        float dist_to_wp = sqrt(distance_to_avoid*distance_to_avoid + conflict_distance*conflict_distance);

        double self_lat_target, self_lon_target;
        waypoint_from_heading_and_distance(self_pos[0], self_pos[1], new_heading, dist_to_wp,
                                            &self_lat_target, &self_lon_target);

        *avoidance_lat = self_lat_target;
        *avoidance_lon = self_lon_target;

    }
}
