#ifndef DISCRETESTATE_H
#define DISCRETESTATE_H

#include <stdexcept>

namespace lattice_planner {

struct DiscreteState {
    DiscreteState() :
        grid_cell(0),
        x_i(0),
        y_i(0),
        angle_i(0),
        vel_x_i(0),
        vel_phi_i(0),
        in_map(false) {}

    DiscreteState(int grid_cell,
                  int grid_x,
                  int grid_y,
                  int angle_i,
                  int vel_x_i,
                  int vel_phi_i,
                  bool in_map) :
        grid_cell(grid_cell),
        x_i(grid_x),
        y_i(grid_y),
        angle_i(angle_i),
        vel_x_i(vel_x_i),
        vel_phi_i(vel_phi_i),
        in_map(in_map) {}

    bool operator==(DiscreteState const &s) const {
        return grid_cell == s.grid_cell
            && angle_i == s.angle_i
            && vel_x_i == s.vel_x_i
            && vel_phi_i == s.vel_phi_i;
    }

    double getDiagonalDistance(DiscreteState const &s) {
        // getting negative values, think it's got to do with
        // substracting unsigned ints
        int num_x = abs((int)x_i - (int)s.x_i);
        int num_y = abs((int)y_i - (int)s.y_i);
        int max = std::max(num_x, num_y);
        int min = std::min(num_x, num_y);
        if ((max - min + sqrt(2) * min) < 0) {
            throw std::runtime_error("(discrete_state.h) negative diagonal distance");
        }
        return (max - min) + sqrt(2)*min;
    }

    double getDiagonalDistance(unsigned int const &x, unsigned int const &y) {
        int num_x = abs(x_i - x);
        int num_y = abs(y_i - y);
        int max = std::max(num_x, num_y);
        int min = std::min(num_x, num_y);
        return (max - min) + sqrt(2)*min;
    }

    unsigned int grid_cell;
    unsigned int x_i;
    unsigned int y_i;
    unsigned int angle_i;
    unsigned int vel_x_i;
    unsigned int vel_phi_i;
    bool in_map;
};

} /* namespace lattice_planner */

#endif /* DISCRETESTATE_H */
