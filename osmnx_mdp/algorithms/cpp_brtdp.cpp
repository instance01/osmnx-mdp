#include "cpp_brtdp.hpp"
#include <cmath>
#include <stack>
#include <unordered_set>

#include <iostream>

// TODO: Rename t to tau

CPP_BRTDP::CPP_BRTDP() {};
CPP_BRTDP::~CPP_BRTDP() {};

double CPP_BRTDP::get_Q_value(
        google::dense_hash_map<long, double> &v,
        const long &s,
        const std::pair<long, long> &a) {
    double immediate_cost = (*this->C)[a];
    double future_cost = 0;
    for (auto &outcome : (*this->P)[s][a]) {
        future_cost += outcome.second * v[outcome.first];
    }
    return immediate_cost + future_cost;
}

// TODO: COPIED from cpp_dstar_lite.cpp
double aerial_dist(double lat1, double lon1, double lat2, double lon2, double R=6356.8) {
    lon1 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;
    lat1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    double d = pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2);
    return R * 2 * asin(pow(d, .5));
}

int CPP_BRTDP::init(
    std::vector<long> *S,
    google::dense_hash_map<long, std::vector<std::pair<long, long>>> *A,
    google::dense_hash_map<std::pair<long, long>, float, pair_hash> *C,
    google::dense_hash_map<
        long,
        google::dense_hash_map<
            std::pair<long, long>,
            std::vector<std::pair<long, double>>,
            pair_hash
        >
    > *P,
    google::dense_hash_map<long, std::pair<float, float>> *data) {
    this->S = S;
    this->A = A;
    this->C = C;
    this->P = P;

    this->data = data;

    this->vl.set_empty_key(0);
    this->vu.set_empty_key(0);

    return 0;
}

int CPP_BRTDP::setup(long start, long goal) {
    this->start = start;
    this->goal = goal;

    std::default_random_engine rd;
    this->random_generator = std::default_random_engine(rd());

    double lat2 = (*this->data)[this->goal].first;
    double lon2 = (*this->data)[this->goal].second;

    for (auto &x : *data) {
        double lat1 = x.second.first;
        double lon1 = x.second.second;

        this->vl[x.first] = aerial_dist(lat1, lon1, lat2, lon2) / 50;
        this->vu[x.first] = aerial_dist(lat1, lon1, lat2, lon2) / 10;
    }

    return 0;
}

// TODO RENAME
std::pair<long, long> CPP_BRTDP::update(google::dense_hash_map<long, double> &v, const long &x) {
    std::pair<long, long> curr_min_action;
    double curr_min = INFINITY;
    for (auto &a : (*this->A)[x]) {
        double q_val = get_Q_value(v, x, a);
        if (q_val < curr_min) {
            curr_min = q_val;
            curr_min_action = a;
        }
    }
    v[x] = curr_min;
    return curr_min_action;
}

// TODO See header for default values alpha=0.001, t=10
int CPP_BRTDP::run_trials(double alpha, double t) {
    int i = 0;
    while (this->vu[this->start] - this->vl[this->start] > alpha) {
        this->run_trial(t);
        i += 1;
    }
    std::cout << "CONVERGED " << i << std::endl;
    return 0;
}

int CPP_BRTDP::run_trial(double t) {
    long x = this->start;

    std::stack<long> traj;

    while (true) {
        if (x == this->goal)
            break;

        traj.push(x);

        this->update(this->vu, x);
        std::pair<long, long> curr_min_action = this->update(this->vl, x);

        std::vector<float> distribution_param;
        double B = 0;
        for (auto &outcome : (*this->P)[x][curr_min_action]) {
            double cost = outcome.second * (this->vu[outcome.first] - this->vl[outcome.first]);
            distribution_param.push_back(cost);
            B += cost;
        }
        
        if (B < ((this->vu[this->start] - this->vl[this->start]) / t))
            break;

        std::discrete_distribution<int> distribution(distribution_param.begin(), distribution_param.end());
        int n = distribution(this->random_generator);
        x = (*this->P)[x][curr_min_action][n].first;
    }

    while (!traj.empty()) {
        long x = traj.top();
        traj.pop();
        this->update(this->vu, x);
        this->update(this->vl, x);
    }

    return 0;
}

std::vector<long> CPP_BRTDP::get_path() {
    std::vector<long> path;

    long curr_node = this->start;
    while (curr_node != this->goal) {
        path.push_back(curr_node);

        std::pair<long, long> curr_min_action = this->update(this->vl, curr_node);
        curr_node = (*this->P)[curr_node][curr_min_action][0].first;
    }

    return path;
}
