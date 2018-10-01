#ifndef STATE_H
#define STATE_H

#include <lattice_planner/pose.h>
#include <lattice_planner/velocity.h>
#include <lattice_planner/discrete_state.h>

#include <float.h>

/********************************************************************
 * This state implementation and the headers/discretization borrow
 * heavily from Marina Kollmitz implementation. That said, I'd like 
 * to scrap all of it in favor of pre-computing the motion primitives
 ********************************************************************/

#define MAX 1e10

namespace lattice_planner {

struct State {
    int stateID; 
    int heapIndex;
    struct ListElement *listElement;
    int iterationClosed;
    int callNumberAccessed;
    int numExpands;

    DiscreteState state_i;
    Pose pose;
    Velocity vel;
    std::vector<unsigned int> trajectory;
    State *parent;
    State *bestSucc;
    double costToBestSucc;
    double g;
    double v;
    double h;

};

} /* namespace lattice_planner */

#endif /* STATE_H */
