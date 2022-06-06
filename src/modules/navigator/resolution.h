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

    std::array<double, 3> self_pos;
    std::array<double, 3> traff_pos;
    std::array<float, 3> tr_vel;
    std::array<float, 3> self_vel;

    float conflict_distance;
    float horizontal_separation;
    float encroachment;
    float buffer=10.0;
    float heading;


    public:
        resolution(std::array<double, 3> _self_pos, std::array<double, 3> _traff_pos,
                   float _conflict_distance, float _encroachment, float _horizontal_separation, float _heading)
        {
            self_pos = _self_pos;
            traff_pos = _traff_pos;

            conflict_distance = _conflict_distance;
            horizontal_separation = _horizontal_separation; 
            encroachment = _encroachment;
            heading = _heading;
        }
        

        void resolve_predicted_conflict(double *avoidance_lat, double *avoidance_lon);

};