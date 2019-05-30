#include <google/dense_hash_map>
#include <unordered_map>
#include <vector>

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

int Xd(float gamma, int state);

void cpp_get_Q_value(
        google::dense_hash_map<std::pair<long, long>, float> C,
        google::dense_hash_map<std::pair<long, long>, float> P,
        google::dense_hash_map<int, int> prev_V,
        float gamma,
        long state,
        std::pair<long, long> action);

//void test(google::dense_hash_map<int, int> a);

//struct pair_hash {
//    template <class T1, class T2>
//    std::size_t operator () (const std::pair<T1, T2> &p) const {
//        auto h1 = std::hash<T1>{}(p.first);
//        auto h2 = std::hash<T2>{}(p.second);
//
//        long long ret = h1;
//        ret <<= 32;
//        return ret + h2;
//    }
//};

// TODO Rename CPP_* things
// C++ has camelcase style

struct CPP_Intersection {
    long left_node;
    long right_node;
    long straight_on_node;
    std::pair<long, long> origin_edge;
};

int CPP_get_normal_intersections(
        google::dense_hash_map<std::pair<long, long>, CPP_Intersection, pair_hash> &out,
        google::dense_hash_map<long, std::vector<long>> &successors,
        google::dense_hash_map<long, std::pair<float, float>> &data);

// TODO: lmao
int solve(
        google::dense_hash_map<long, float> &V,
        std::vector<long> &S,
        google::dense_hash_map<long, std::vector<std::pair<long, long>>> &A,
        google::dense_hash_map<std::pair<long, long>, float, pair_hash> &C,
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, float>>,
                pair_hash
            >
        > &P,
        int max_iter);

int CPP_make_low_angle_intersections_uncertain(
        std::vector<long> *angle_nodes,
        google::dense_hash_map<
            long,
            google::dense_hash_map<
                std::pair<long, long>,
                std::vector<std::pair<long, double>>,
                pair_hash
            >
        > *P,
        google::dense_hash_map<long, std::vector<long>> &successors,
        google::dense_hash_map<long, std::pair<float, float>> &data,
        float max_angle);
