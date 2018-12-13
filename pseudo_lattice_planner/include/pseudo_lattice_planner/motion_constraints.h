#ifndef MOTIONCONSTRAINTS_
#define MOTIONCONSTRAINTS_

namespace pseudo_lattice_planner {

struct MotionConstraints {
    MotionConstraints() :
        min_vel_x(0.0),
        max_vel_x(0.0),
        acc_x(0.0),
        min_vel_phi(0.0),
        max_vel_phi(0.0),
        acc_phi(0.0) {}

    MotionConstraints(double min_vel_x,
                      double max_vel_x,
                      double acc_x,
                      double min_vel_phi,
                      double max_vel_phi,
                      double acc_phi) :
        min_vel_x(min_vel_x),
        max_vel_x(max_vel_x),
        acc_x(acc_x),
        min_vel_phi(min_vel_phi),
        max_vel_phi(max_vel_phi),
        acc_phi(acc_phi) {}


    double min_vel_x;
    double max_vel_x;
    double acc_x;
    double min_vel_phi;
    double max_vel_phi;
    double acc_phi;
};

} /* namespace pseudo_lattice_planner */

#endif /* MOTIONCONSTRAINTS */

