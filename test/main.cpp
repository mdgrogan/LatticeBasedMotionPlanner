#include "environment.h"

int main()
{
    Environment env;
    env.initializeEnv('m', "unicycle_2p5cm.mprim");
    DiscreteCell current_state(-23,10,13,2);
    std::vector<DiscreteCell> next_states;
    std::vector<Action> next_actions;
    env.getNextStates(current_state, next_states, next_actions);
    for (int i=0; i<next_states.size(); i++) {
        std::cout<<"["<<
                   current_state.x_i<<" "<<
                   current_state.y_i<<" "<<
                   current_state.theta_i<<" "<<
                   current_state.vel_i<<"] --> ["<<
                   next_states[i].x_i<<" "<<
                   next_states[i].y_i<<" "<<
                   next_states[i].theta_i<<" "<<
                   next_states[i].vel_i<<"] "<<
                   std::endl;
    }

    return 0;
}
