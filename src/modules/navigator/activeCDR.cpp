#include "activeCDR.h"
#include <systemlib/mavlink_log.h>

/*
void MVP::set_params()
{
    sw_horiz_reso = true;
    sw_speed_reso = false;
    sw_hdg_reso = false;
    sw_vert_reso = false;
}*/

double MVP::degrees_to_radians(double degrees)
{
    return degrees*180*3.1415926535;
}

void MVP::resolve()
{   
    /*double xpos, ypos;
    double xvel, yvel;
    //change heading to avoid 
    double conflict_heading_rad = degrees_to_radians(conflict_heading);

    xpos = conflict_distance*sin(conflict_heading_rad);
    ypos = conflict_distance*cos(conflict_heading_rad);

    xvel = self_cruising_speed*sin(conflict_heading_rad);

    std::vector<double> rel_pos{xpos, ypos};
    std::vector<double> rel_vel{xvel, yvel};

    //decompose relative position vector into components
    rel_pos.push_back(conflict_distance*sin(conflict_heading_rad));
    rel_pos.push_back(conflict_distance*cos(conflict_heading_rad));

    //calculate relative velocity vectors 
    //Since traffic is calculated relative to the ownship aircraft, traffic heading can be used to determine relative velocity

    rel_vel.push_back(self_cruising_speed*sin(conflict_heading_rad) - traffic_cruising_velocity*sin(traffic_heading));
    rel_vel.push_back(self_cruising_speed*cos(conflict_heading_rad) - traffic_cruising_velocity*cos(traffic_heading));*/
}