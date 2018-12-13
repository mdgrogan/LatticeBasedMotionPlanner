#include <pseudo_lattice_planner/lattice_primitives.h>

/********************************************************************
 * Functions for generating feasible motions and poses
 ********************************************************************/

namespace pseudo_lattice_planner {
    
// for floating point stuff
#define ROUNDED_ZERO 1e-6

LatticePrims::LatticePrims(MotionConstraints m) :
    motion_constraints_(m) {}

/********************************************************************
 * Return a vector of up to nine velocities which can be reached 
 * within the current time slice.
 * Protects against transitions from a motionless state to itself.
 ********************************************************************/
std::vector<Velocity> LatticePrims::getReachableVelocities(
        Velocity current_velocity,
        double time_delta) {
    std::vector<Velocity> vels;
    Velocity vel;

    for (int i=0; i<3; i++) {
        if (i==0) {
            vel.vel_x = current_velocity.vel_x;
        } else if (i==1) {
            vel.vel_x = current_velocity.vel_x + 
                        motion_constraints_.acc_x*time_delta;
        } else if (i==2) {
            vel.vel_x = current_velocity.vel_x - 
                        motion_constraints_.acc_x*time_delta;
        }
        for (int j=0; j<3; j++) {
            if (j==0) {
                vel.vel_phi = current_velocity.vel_phi;
            } else if (j==1) {
                vel.vel_phi = current_velocity.vel_phi +
                              motion_constraints_.acc_phi*time_delta;
            } else if (j==2) {
                vel.vel_phi = current_velocity.vel_phi -
                              motion_constraints_.acc_phi*time_delta;
            }
            // validate
            if (vel.vel_x >= motion_constraints_.min_vel_x - ROUNDED_ZERO &&
                vel.vel_x <= motion_constraints_.max_vel_x + ROUNDED_ZERO &&
                vel.vel_phi >= motion_constraints_.min_vel_phi - ROUNDED_ZERO &&
                vel.vel_phi <= motion_constraints_.max_vel_phi + ROUNDED_ZERO) {
                // prevent 0 vel -> 0 vel and turn in place
                if (!(fabs(vel.vel_x) < ROUNDED_ZERO && 
                      fabs(current_velocity.vel_x) < ROUNDED_ZERO)) {
                    vels.push_back(vel);
                }
            }
        }
    }
    return vels;
}

MotionConstraints LatticePrims::getMotionConstraints() {
    return motion_constraints_;
}

/********************************************************************
 * Return the pose that will be reached by traveling at the current
 * velocity for time_delta seconds.
 ********************************************************************/
Pose LatticePrims::getNextPose(Pose current,
                               Velocity vel,
                               double time_delta) {
    double x, y, theta;
    if (fabs(vel.vel_phi) > ROUNDED_ZERO) {
        // arced trajectory
        double arc_radius = vel.vel_x / vel.vel_phi;
        x = current.getX() + arc_radius*(-sin(current.getTheta())
                                         + sin(current.getTheta()
                                             + vel.vel_phi*time_delta));
        y = current.getY() + arc_radius*(cos(current.getTheta())
                                         - cos(current.getTheta()
                                             + vel.vel_phi*time_delta));
        theta = current.getTheta() + vel.vel_phi*time_delta;
    } else {
        // straight trajectory
        x = current.getX() + vel.vel_x*cos(current.getTheta())*time_delta;
        y = current.getY() + vel.vel_x*sin(current.getTheta())*time_delta;
        theta = current.getTheta();
    }
    return Pose(x, y, theta);
}

/********************************************************************
 * Return a vector of the cell indices traversed by current trajectory
 ********************************************************************/
std::vector<unsigned int> LatticePrims::getTrajectoryIndices(
                                State *begin,
                                DiscreteState end_i,
                                Velocity vel,
                                StateDiscretizer *discretizer,
                                double time_incr,
                                double time_delta) {
    std::vector<unsigned int> indices;
    indices.push_back(begin->state_i.grid_cell);
    double time = time_incr;
    Pose next = begin->pose;
    unsigned int x_i, y_i, index;
    while (time < time_delta) {
        next = getNextPose(next, vel, time_incr);
        if (discretizer->getCellPosition(next.getX(), next.getY(), 
                                         x_i, y_i, index)) {
            indices.push_back(index);
        }
        time += time_incr;
    }
    indices.push_back(end_i.grid_cell);
    return indices;
}

} /* namespace pseudo_lattice_planner */

