#include "cpp_improved_dstar_lite.hpp"
#include <cmath> // sin, asin, cos, pow, M_PI
#include <cfloat> // DBL_EPSILON
#include <algorithm> // min_element
#include <stdexcept> // runtime_error
#include <unordered_set>

#include "../cpp_queue_util.hpp" // queue_decrease_priority, queue_pop

#ifdef TESTS
#include "../serialize_util.hpp"
#endif


ImprovedDStarLite::ImprovedDStarLite () {}
ImprovedDStarLite::~ImprovedDStarLite () {}


int ImprovedDStarLite::init(
        google::dense_hash_map<long, std::vector<long>> *predecessors,
        google::dense_hash_map<long, std::vector<long>> *successors,
        google::dense_hash_map<std::pair<long, long>, double, pair_hash> *cost,
        google::dense_hash_map<long, std::pair<double, double>> *data,
        google::dense_hash_map<long, google::dense_hash_map<std::pair<long, long>, double, pair_hash>> *num_outcomes)
{
    DStarLite::init(predecessors, successors, cost, data);
    this->num_outcomes = num_outcomes;
    return 0;
}

double ImprovedDStarLite::get_penalty(long predecessor, std::pair<long, long> action) {
    double penalty = 0.0;

    if (predecessor == action.second) {
        penalty += U_TURN_PENALTY;
    }

    if ((*this->num_outcomes)[predecessor][action] > 1.5) {
        penalty += 10 / 3600.;
    }

    return penalty;
}
