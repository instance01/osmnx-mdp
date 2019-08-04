#include "cpp_lib.hpp"
#include <cmath>


std::vector<std::pair<std::pair<long, long>, std::pair<long, long>>> combinations(
        long origin_node,
        std::vector<long> successors)
{
    std::vector<std::pair<std::pair<long, long>, std::pair<long, long>>> out;

    for (unsigned long i = 0; i < successors.size(); ++i) {
        for (unsigned long j = 0; j < successors.size(); ++j) {
            if (i >= j)
                continue;
            std::pair<long, long> edge1(origin_node, successors[i]);
            std::pair<long, long> edge2(origin_node, successors[j]);
            out.push_back({edge1, edge2});
        }
    }

    return out;
}

double get_angle(
        double p1_x,
        double p1_y,
        double p2_x,
        double p2_y,
        double origin_x,
        double origin_y) {
    p1_x -= origin_x;
    p1_y -= origin_y;
    p2_x -= origin_x;
    p2_y -= origin_y;

    // arctan2 gives us the angle between the ray from the origin to the point
    // and the x axis.
    // Thus, to get the angle between two points, simply get the difference
    // between their two angles to the x axis.
    double angle = atan2(p2_x, p2_y) - atan2(p1_x, p1_y);
    return 180 * angle / M_PI;
}

double aerial_dist(double lat1, double lon1, double lat2, double lon2, double R) {
    // Since all maps are already converted to UTM coordinates, we can simply
    // use euclidian distance.
    // For reference, a map based on LAT/LON would have to use Haversine formula
    // for aerial distance.
    // UTM coords are in metres. Thus divide by 1000 for kilometres.
    return sqrt(pow(lat1 - lat2, 2) + pow(lon1 - lon2, 2)) / 1000;
}
