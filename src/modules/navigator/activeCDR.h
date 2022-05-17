#include "math.h"
#include <vector>

class MVP
{
    public:

    private:

        int     conflict_distance;
        int     conflict_heading;
        
        double   self_cruising_speed;
        double   self_heading;
        double   traf_cruising_speed;
        double   traf_heading;

        bool    sw_horiz_reso;
        bool    sw_speed_reso;
        bool    sw_hdg_reso;
        bool    sw_vert_reso;

    public:

        MVP(int distance, int heading, double slf_cruising_speed, double slf_heading, double trf_cruising_speed, double trf_heading)
        {
            conflict_distance = distance;
            conflict_heading = heading;
            self_cruising_speed = slf_cruising_speed;
            self_heading = slf_heading;
            traf_cruising_speed = trf_cruising_speed;
            traf_heading = trf_heading;
        }

        void resolve();

        double degrees_to_radians(double degrees);
};