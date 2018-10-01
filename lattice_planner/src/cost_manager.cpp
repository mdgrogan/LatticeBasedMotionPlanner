#include <stdexcept>
#include <lattice_planner/cost_manager.h>

/********************************************************************
 * Basically all this is doing is evaluating the cost of a states
 * trajectory. In simulation, currently using ideal localization so
 * the main function is commented out. If trajectories were being
 * pre-computed, their costs could be sort of built in and this would
 * be totally obsolete. As it is, I was borrowing from an implementa-
 * tion I'd found on github and this made sense at the time. Leaning 
 * more towards pre-computing trajectories at this point.
 ********************************************************************/

namespace lattice_planner {

double CostManager::getTrajectoryCost(State *state) {
    double costs = 0.0;
    State *parent = state->parent;
    std::vector<unsigned int> map_indices = state->trajectory;
    double cell_distance = 
        state->state_i.getDiagonalDistance(parent->state_i);
    double rot_distance = 
        state->pose.getRotDistance(parent->pose);
    int cell_cost = 0;

    /*
    for (int i=0; i<map_indices.size(); i++) {
        unsigned int mx, my;
        costmap_->getCostmap()->indexToCells(map_indices[i], mx, my);
        cell_cost = costmap_->getCostmap()->getCost(mx, my);

        // ignoring costs that aren't lethal - this approach
        // would be flawed if we weren't using ideal localization
        if (cell_cost >= cost_factors_.lethal_cost) {
            return -1;
        } else {
            cell_cost = 0;
        }
        costs += cost_factors_.environment_cost * cell_cost;
    }
    */

    costs += cost_factors_.time_cost;
    costs += cell_distance * cost_factors_.step_cost;
    costs += rot_distance * cost_factors_.rotation_cost;
    if (costs < 0) {
        ROS_WARN("CostManager::getTrajectoryCost");
        ROS_WARN("cell_distance = %f", cell_distance);
        ROS_WARN("rot_distance = %f", rot_distance);
        throw std::runtime_error("(cost_manager.cpp) negative cost");
    }
    return costs;
}

CostFactors CostManager::getCostFactors() {
    return cost_factors_;
}

} /* namespace lattice_planner */
