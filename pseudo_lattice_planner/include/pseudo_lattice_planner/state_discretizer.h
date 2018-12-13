#ifndef STATEDISCRETIZER_H
#define STATEDISCRETIZER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <pseudo_lattice_planner/pose.h>
#include <pseudo_lattice_planner/velocity.h>
#include <pseudo_lattice_planner/discrete_state.h>
#include <pseudo_lattice_planner/motion_constraints.h>

namespace pseudo_lattice_planner {


class StateDiscretizer {
    public:

        StateDiscretizer(costmap_2d::Costmap2DROS *costmap,
                MotionConstraints motion_constraints,
                double time_resolution);

        int getNumDiscreteAngles(double acc_phi, double time_delta);

        DiscreteState discretizeState(Pose pose, Velocity vel);

        bool getCellPosition(double x, double y, 
                unsigned int &x_i, unsigned int &y_i,
                unsigned int &index);

        unsigned int getDiscreteOrientation(double angle);

        void getDiscreteVelocity(Velocity vel, 
                unsigned int &vel_x_i, unsigned int &vel_phi_i);

        void getLimits(int &num_gridcells, int &num_orientations,
                int &num_vels_x, int &num_vels_phi);

        Velocity getVelocity(unsigned int vel_x_i,
                unsigned int vel_phi_i);

    private:

        StateDiscretizer();

        costmap_2d::Costmap2DROS *costmap_;
        MotionConstraints motion_constraints_;
        int num_gridcells_;
        int num_orientations_;
        int num_vels_x_;
        int num_vels_phi_;
        double time_resolution_; 
        double vel_step_x_;
        double vel_step_phi_;
};

} /* namespace pseudo_lattice_planner */

#endif /* STATEDISCRETIZER_H */
