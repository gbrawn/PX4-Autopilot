#include "resolution.h"
#include <systemlib/mavlink_log.h>

void resolution::resolve_predicted_conflict(double *avoidance_lat, double *avoidance_lon, double *heading_delta)
{   

    float relative_dist = sqrt(pow((self_pos[0] - traf_pos[0]),2) + pow((self_pos[1] - traf_pos[1]),2));

    float lookahead_dist = sqrt(pow(relative_dist,2) + pow(horizontal_protection_zone,2));

    //Calculate change of heading
    float min_heading_change_to_avoid = atan((horizontal_protection_zone / (relative_dist)));

    if (min_heading_change_to_avoid > 0.79f)
    {
        //command heading change against the direction of relative velocity
        float new_heading = self_heading + min_heading_change_to_avoid;

        double self_lat_target, self_lon_target;
        waypoint_from_heading_and_distance(self_pos[0], self_pos[1], new_heading, lookahead_dist,
                                            &self_lat_target, &self_lon_target);

        *avoidance_lat = self_lat_target;
        *avoidance_lon = self_lon_target;

    } else {

        //command heading change in the direction of relative velocity
        float new_heading = self_heading - min_heading_change_to_avoid;

        double self_lat_target, self_lon_target;
        waypoint_from_heading_and_distance(self_pos[0], self_pos[1], new_heading, lookahead_dist,
                                            &self_lat_target, &self_lon_target);

        *avoidance_lat = self_lat_target;
        *avoidance_lon = self_lon_target;
        *heading_delta = min_heading_change_to_avoid;
        
    }
}
