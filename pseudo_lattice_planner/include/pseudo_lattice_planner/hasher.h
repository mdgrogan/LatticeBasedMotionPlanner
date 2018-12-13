#ifndef HASHER_H
#define HASHER_H

namespace pseudo_lattice_planner {

struct Hasher {
    std::size_t operator()(const DiscreteState &index) const {
        return getHash(index);
    }

    static void setLimits(int n_gridcells, int n_orientations,
                          int n_vels_x, int n_vels_phi) {
        num_gridcells_ = n_gridcells;
        num_orientations_ = n_orientations;
        num_vels_x_ = n_vels_x;
        num_vels_phi_ = n_vels_phi;
    }

    static size_t getHash(const DiscreteState &index) {
        size_t hash = index.grid_cell
            + index.angle_i * Hasher::num_gridcells_
            + index.vel_x_i * Hasher::num_gridcells_ * Hasher::num_orientations_
            + index.vel_phi_i * Hasher::num_gridcells_ * Hasher::num_orientations_
            * Hasher::num_vels_x_;
        return hash;
    }

    static int num_gridcells_;
    static int num_orientations_;
    static int num_vels_x_;
    static int num_vels_phi_;
};

} /* namespace pseudo_lattice_planner */

#endif /* HASHER_H */

