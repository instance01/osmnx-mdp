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


std::vector<long> BRTDP_REPLAN::get_path(
        google::dense_hash_map<long, long> &diverge_policy,
        const float &beta,
        const bool &always_replan) {
    // Defaults:
    //  beta = .02
    //  always_replan = true
    std::vector<long> path;

    std::unordered_set<long> visited {this->start};

    long curr_node = this->start;
    while (curr_node != this->goal) {
        path.push_back(curr_node);

        long diverged_node = diverge_policy[curr_node];
        if (diverged_node == 0 || visited.find(diverged_node) != visited.end()) {
            // Current node shall not diverge, or we already visited the
            // candidate before. Let's follow the path calculated by BRTPD.
            const auto min_action = this->get_minimum_action(this->vl, curr_node);

            const long last_node = curr_node;
            curr_node = min_action.first.second;

            if (this->vu[curr_node] - this->vl[curr_node] > beta) {
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
                if (visited.find(curr_node) != visited.end()) {
                    if (always_replan)
                        this->replan(curr_node);
                    else
                        curr_node = this->update_v(this->vl, last_node).second;
                }
            }
        } else {
            curr_node = diverged_node;
        }

        visited.insert(curr_node);
    }

    return path;
}
