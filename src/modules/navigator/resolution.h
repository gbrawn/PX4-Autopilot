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

    float _conflict_distance;
    std::array<double, 3> _self_pos;
    std::array<double, 3> _traff_pos;
    float _horizontal_separation;
    float _encroachment;
    float buffer=5.0;
    float _heading;

    public:
        resolution(std::array<double, 3> self_pos, std::array<double, 3> traff_pos, float conflict_distance, float encroachment, float horizontal_separation, float heading)
        {
            _self_pos = self_pos;
            _traff_pos = traff_pos;
            _conflict_distance = conflict_distance;
            _horizontal_separation = horizontal_separation; 
            _encroachment = encroachment;
            _heading = heading;
        }
        

        void resolve_predicted_conflict(double *avoidance_lat, double *avoidance_lon);

};