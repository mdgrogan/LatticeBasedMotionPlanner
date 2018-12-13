#ifndef VELOCITY_H
#define VELOCITY_H

namespace pseudo_lattice_planner {

struct Velocity {
    Velocity() : vel_x(0.0), vel_phi(0.0) {}

    Velocity(double vel_x, double vel_phi) :
        vel_x(vel_x), vel_phi(vel_phi) {}

    double vel_x;
    double vel_phi;
};

} /* namespace pseudo_lattice_planner */

#endif /* VELOCITY_H */

