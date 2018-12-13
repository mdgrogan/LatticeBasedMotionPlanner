#ifndef COST_MANAGER_H
#define COST_MANAGER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <pseudo_lattice_planner/state.h>
#include <pseudo_lattice_planner/motion_constraints.h>

namespace pseudo_lattice_planner {

struct CostFactors {
    double time_cost;
    double step_cost;
    double environment_cost;
    double rotation_cost;
    int lethal_cost;
};

class CostManager {
    public:

        CostManager(costmap_2d::Costmap2DROS *costmap,
                    MotionConstraints motion_constraints,
                    CostFactors cost_factors) :
                    costmap_(costmap),
                    motion_constraints_(motion_constraints),
                    cost_factors_(cost_factors) {}

        double getTrajectoryCost(State *state);

        CostFactors getCostFactors();

    private:

        CostManager();

        costmap_2d::Costmap2DROS *costmap_;
        MotionConstraints motion_constraints_;
        CostFactors cost_factors_;
};

} /* namespace pseudo_lattice_planner */

#endif /* COST_MANAGER_H */

