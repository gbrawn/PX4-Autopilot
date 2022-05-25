#include "math.h"
#include <lib/geo/geo.h>

#include <array>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>

class resolution
{
    public:

    double _avoidance_lat;
    double _avoidance_lon;

    private:

    crosstrack_error_s crosstrack;
    float lookahead_time;
    float horizontal_protection_zone;
    float self_heading;
    float traf_heading;

    std::array<double, 3> traf_pos;
    std::array<double, 3> self_pos;

    public:
        resolution(crosstrack_error_s cr,
                   std::array<double, 3> tr_pos,
                   std::array<double, 3> slf_pos,
                   float horizontal_separation,
                   float lkahd_time,
                   float slf_heading,
                   float traffic_heading)
        {
            crosstrack = cr;
            horizontal_protection_zone = horizontal_separation;
            lookahead_time = lkahd_time;
            traf_pos = tr_pos;
            self_pos = slf_pos;
            self_heading = slf_heading;
            traf_heading = traffic_heading;
        }
        

        void resolve_predicted_conflict(double *avoidance_lat, double *avoidance_lon, double *heading_delta);

};