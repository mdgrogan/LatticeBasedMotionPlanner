#include <lattice_planner/state_discretizer.h>

/********************************************************************
 * State discretization functions. Another class that I think can
 * be done without if motion primitives are being pre-computed.
 ********************************************************************/

namespace lattice_planner {

StateDiscretizer::StateDiscretizer(costmap_2d::Costmap2DROS *costmap,
            MotionConstraints motion_constraints,
            double time_resolution) :
            costmap_(costmap),
            motion_constraints_(motion_constraints),
            time_resolution_(time_resolution) {
    unsigned int nx = costmap_->getCostmap()->getSizeInCellsX();
    unsigned int ny = costmap_->getCostmap()->getSizeInCellsY();
    num_gridcells_ = nx*ny;

    num_orientations_ = getNumDiscreteAngles(motion_constraints_.acc_phi,
                                             time_resolution_);

    num_vels_x_ = 0;
    for(double vel = motion_constraints.min_vel_x;
            vel <= motion_constraints_.max_vel_x;
            vel += motion_constraints.acc_x * time_resolution_) {
        num_vels_x_++;
    }
    num_vels_phi_ = 0;
    for(double vel = motion_constraints.min_vel_phi;
            vel <= motion_constraints_.max_vel_phi;
            vel += motion_constraints.acc_phi * time_resolution_) {
        num_vels_phi_++;
    }

    vel_step_x_ = motion_constraints_.acc_x * time_resolution_;
    vel_step_phi_ = motion_constraints_.acc_phi * time_resolution_;
}

int StateDiscretizer::getNumDiscreteAngles(double acc_phi, 
                                           double time_delta) {
    int max_angle = (int)((4*M_PI)/(acc_phi*time_delta) + 1);
    int num_angles = max_angle + 1;
    return num_angles;
}

DiscreteState StateDiscretizer::discretizeState(Pose pose,
                                                Velocity vel) {
    DiscreteState state_i;
    state_i.in_map = getCellPosition(pose.getX(), pose.getY(),
                                     state_i.x_i, state_i.y_i,
                                     state_i.grid_cell);
    state_i.angle_i = getDiscreteOrientation(pose.getTheta());
    getDiscreteVelocity(vel, state_i.vel_x_i, state_i.vel_phi_i);
    return state_i;
}

bool StateDiscretizer::getCellPosition(double x, double y,
                                       unsigned int &x_i,
                                       unsigned int &y_i,
                                       unsigned int &index) {
    if (costmap_->getCostmap()->worldToMap(x, y, x_i, y_i)) {
        index = costmap_->getCostmap()->getIndex(x_i, y_i);
        return true;
    } else {
        return false;
    }
}

unsigned int StateDiscretizer::getDiscreteOrientation(double angle) {
    if (angle < 0 - ROUNDED_ZERO || angle > 2*M_PI + ROUNDED_ZERO) {
        ROS_ERROR("StateDiscretizer: bad angle %f", angle);
        return 0;
    }

    unsigned int orientation = angle/(2*M_PI/(num_orientations_-1)) + 0.5;
    if (orientation == num_orientations_) {
        orientation = 0;
    }

    return orientation;
}

void StateDiscretizer::getDiscreteVelocity(Velocity vel,
                                           unsigned int &vel_x_i,
                                           unsigned int &vel_phi_i) {
    vel_x_i = (int)((vel.vel_x/vel_step_x_) - 
                    (motion_constraints_.min_vel_x/vel_step_x_) + 0.5);
    vel_phi_i = (int)((vel.vel_phi/vel_step_phi_) - 
                    (motion_constraints_.min_vel_phi/vel_step_phi_) + 0.5);
}

void StateDiscretizer::getLimits(int &num_gridcells, 
                                 int &num_orientations,
                                 int &num_vels_x,
                                 int &num_vels_phi) {
    num_gridcells = num_gridcells_;
    num_orientations = num_orientations_;
    num_vels_x = num_vels_x_;
    num_vels_phi = num_vels_phi_;
}

Velocity StateDiscretizer::getVelocity(unsigned int vel_x_i,
                                       unsigned int vel_phi_i) {
    return Velocity(vel_x_i*vel_step_x_, vel_phi_i*vel_step_phi_);
}

} /* namespace lattice_planner */
