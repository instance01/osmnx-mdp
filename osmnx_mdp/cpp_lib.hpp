#ifndef LIB_INCLUDE
#define LIB_INCLUDE

#include <vector>
// TODO: Move to ../cpp_lib.hpp
// Basically create a C++ version of lib.

// TODO Move comments to cpp

// TODO: Explain why collisions probably won't happen with this.
// We assume a 64 bit system.
// Node numbers sometimes exceed the maximum 32 bit number, which is around
// 2.1 billion, by a small margin (e.g. 3, 4 or 5 billion). Thus by shifting
// We lose one number at the front, e.g. 3.1 billion becomes 100 million
// (before shifting).
// Since we use this hashing for actions, i.e. the two longs denote the ids of
// two adjacent nodes, to get a collision, we need to find a node that has as
// neighbors two numbers that are exactly the same up to their last (most
// signifcant) digit, the billion digit.
// This is highly unlikely. // TODO WHY
// Since we simply add the second number, none of its digits get lost.
// And with just two ops it's super efficient. Just uses a lot of space.
struct pair_hash {
    long long operator () (const std::pair<long, long> &p) const {
        long long ret = p.first;
        ret <<= 32;
        return ret + p.second;
    }
};

std::vector<std::pair<std::pair<long, long>, std::pair<long, long>>> combinations(
        long origin_node,
        std::vector<long> successors);

double get_angle(
        double p1_x,
        double p1_y,
        double p2_x,
        double p2_y,
        double origin_x,
        double origin_y);

// Returns aerial distance in km.
//
// @param R Radius of earth in km.
//
// Source:
// R. W. Sinnott, "Virtues of the Haversine", Sky and Telescope 68 (2), p.159 (1984).
double aerial_dist(double lat1, double lon1, double lat2, double lon2, double R=6356.8);

#endif
