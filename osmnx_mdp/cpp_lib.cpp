#include "cpp_lib.hpp"
#include <cmath>


float get_angle(
        float p1_x,
        float p1_y,
        float p2_x,
        float p2_y,
        float origin_x,
        float origin_y) {
    p1_x -= origin_x;
    p1_y -= origin_y;
    p2_x -= origin_x;
    p2_y -= origin_y;

    // arctan2 gives us the angle between the ray from the origin to the point
    // and the x axis.
    // Thus, to get the angle between two points, simply get the difference
    // between their two angles to the x axis.
    float angle = atan2(p2_x, p2_y) - atan2(p1_x, p1_y);
    return 180 * angle / M_PI;
}

double aerial_dist(double lat1, double lon1, double lat2, double lon2, double R) {
    lon1 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;
    lat1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    double d = pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2);
    return R * 2 * asin(pow(d, .5));
}
