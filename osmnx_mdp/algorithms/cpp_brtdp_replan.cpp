#include "cpp_brtdp_replan.hpp"
#include <unordered_set>
#include <iostream>


BRTDP_REPLAN::BRTDP_REPLAN() {};
BRTDP_REPLAN::~BRTDP_REPLAN() {};


void BRTDP_REPLAN::replan(const long &curr_node) {
    std::cout << "REPLANNING " << std::endl;
    this->start = curr_node;
    this->run_trials();
}

int BRTDP_REPLAN::setup(const long &start, const long &goal, std::unordered_map<std::string, double> cfg) {
    BRTDP::setup(start, goal, cfg);

    this->beta = cfg["beta"];
    this->always_replan = cfg["always_replan"] == 1.0;

    return 0;
}

std::vector<long> BRTDP_REPLAN::get_path(google::dense_hash_map<long, long> &diverge_policy) {
    std::vector<long> path;

    std::unordered_set<long> visited {this->start};

    std::pair<long, long> curr_node_pair = {0, this->start};
    while (curr_node_pair.second != this->goal) {
        const long curr_node = curr_node_pair.second;
        path.push_back(curr_node);

        long diverged_node = diverge_policy[curr_node];
        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            // Current node shall not diverge, or we already visited the
            // candidate before. Let's follow the path calculated by BRTPD.
            const auto min_action = this->get_minimum_action(this->vl, curr_node_pair);

            const std::pair<long, long> last_node_pair = curr_node_pair;
            curr_node_pair = {last_node_pair.second, min_action.first.second};

            if (this->vu[curr_node_pair] - this->vl[curr_node_pair] > this->beta) {
                this->replan(curr_node);
            } else {
                // We're looping between two nodes. This is most likely because
                // we diverged too far and BRTDP wasn't here before.
                // What do?
                //
                // Two possibilities:
                // * Replan
                // * Simply update vl. After a while both looping nodes become
                //   costly and we find a new path. Is the new path good though?
                // Due to this option one is preferred. Replanning doesn't cost
                // much and makes sure that the new path is excellent.
                if (visited.find(curr_node_pair.second) != visited.end()) {
                    if (this->always_replan)
                        this->replan(curr_node);
                   else
                       curr_node_pair = {last_node_pair.second, this->update_v(this->vl, last_node_pair).second};
                }
            }
        } else {
            curr_node_pair = {curr_node_pair.second, diverged_node};
        }

        visited.insert(curr_node);
    }

    return path;
}
