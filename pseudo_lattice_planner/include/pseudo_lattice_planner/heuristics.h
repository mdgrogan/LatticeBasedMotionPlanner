#ifndef HEURISTICS_H
#define HEURISTICS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <pseudo_lattice_planner/state.h>
#include <pseudo_lattice_planner/motion_constraints.h>
#include <pseudo_lattice_planner/dijkstra.h>
#include <pseudo_lattice_planner/cost_manager.h>

#include <cmath>

namespace pseudo_lattice_planner {

class Heuristics {
    public:

        Heuristics(costmap_2d::Costmap2DROS *costmap,
                CostFactors cost_factors,
                MotionConstraints motion_constraints,
                double time_delta);

        ~Heuristics();

        bool dijkstraHeuristic(State *goal);

        bool diagDistanceHeuristic(State *goal);

        double getHeuristic(State *state, Pose goal_pose);

        void setNeutralCost(float neutral_cost) {
            dijkstra_->setNeutralCost(neutral_cost);
        }

        void setHasUnknown(bool allow_unknown) {
            dijkstra_->setAllowUnknown(allow_unknown);
        }

        void setLethalCost(float lethal_cost) {
            dijkstra_->setLethalCost(lethal_cost);
        }

        void initPublisher(std::string topic_name, 
                int publish_scale = 100);

        void publishHeuristic();

        double max_val;

    private:

        costmap_2d::Costmap2DROS *costmap_;
        CostFactors cost_factors_;
        MotionConstraints motion_constraints_;
        DijkstraExpansion *dijkstra_;

        float *heuristics_;
        double resolution_;
        double time_delta_;
        unsigned int size_x_, size_y_;

        bool pub_initialized_;
        int publish_scale_;
        ros::Publisher heuristics_pub_;
};

} /* namespace pseudo_lattice_planner */

#endif /* HEURISTICS_H */

