#include "environment.h"

Environment::Environment() {
    // hmm
}

bool Environment::initialize(char map_placeholder, const std::string primFile) {
    resolution_ = 0.025;
    return readMotionPrimitives(primFile);
}

bool Environment::setPlanningParams(DiscreteCell goal_state) {
    goal_state_ = goal_state;
    return calculateHeuristic();
}

bool Environment::readMotionPrimitives(const std::string primFile) {
    std::ifstream in;
    in.open(primFile);
    if (!in) {
        return false;
    }

    std::string line;
    size_t pos = std::string::npos;
    MotionPrimitive mprim;

    while (std::getline(in, line)) {
        while ((pos=line.find_first_of(" ")) != std::string::npos) {
            std::string tmp = line.substr(0, pos);
            line.erase(0, pos+1);
            if (tmp == "startangle_i:") {
                mprim.start_theta_i = std::stoi(line);
                break;
            }
            if (tmp == "startvel_i:") {
                mprim.start_vel_i = std::stoi(line);
                break;
            }
            if (tmp == "exectime:") {
                mprim.exec_time = std::stod(line);
                break;
            }
            if (tmp == "endpose_i:") {
                pos = line.find_first_of(" ");
                tmp = line.substr(0, pos);
                line.erase(0, pos+1);
                mprim.end_cell_i.x_i = std::stoi(tmp);
                pos = line.find_first_of(" ");
                tmp = line.substr(0, pos);
                line.erase(0, pos+1);
                mprim.end_cell_i.y_i = std::stoi(tmp);
                pos = line.find_first_of(" ");
                tmp = line.substr(0, pos);
                line.erase(0, pos+1);
                mprim.end_cell_i.theta_i = std::stoi(tmp);
                mprim.end_cell_i.vel_i = std::stoi(line);
            }
            if (tmp == "intermediateposes:") {
                int numLines = std::stoi(line);
                for (int i=0; i<numLines; i++) {
                    std::getline(in, line);
                    pos=line.find_first_of(" ");
                    tmp = line.substr(0, pos);
                    line.erase(0, pos+1);
                    Point p(std::stod(tmp), std::stod(line));
                    mprim.intermediatePoints.push_back(p);
                }
                mprims_.push_back(mprim);
            }
        }
    }
    in.close();

    return true;
}
    
void Environment::getNextCells(DiscreteCell current_cell,
                                std::vector<DiscreteCell> &cells,
                                std::vector<Action> &actions) {
   for (int i=0; i<mprims_.size(); i++) {
      if (mprims_[i].start_theta_i==current_cell.theta_i &&
          mprims_[i].start_vel_i==current_cell.vel_i) {

          DiscreteCell next_cell;
          Action next_action;

          next_cell.x_i = current_cell.x_i + mprims_[i].end_cell_i.x_i;
          next_cell.y_i = current_cell.y_i + mprims_[i].end_cell_i.y_i;
          next_cell.theta_i = mprims_[i].end_cell_i.theta_i;
          next_cell.vel_i = mprims_[i].end_cell_i.vel_i;

          next_action.start_theta_i = mprims_[i].start_theta_i;
          next_action.start_vel_i = mprims_[i].start_vel_i;
          next_action.end_theta_i = mprims_[i].end_cell_i.theta_i;
          next_action.end_vel_i = mprims_[i].end_cell_i.vel_i;
          next_action.dx = mprims_[i].end_cell_i.x_i;
          next_action.dy = mprims_[i].end_cell_i.y_i;
          next_action.exec_time = mprims_[i].exec_time;
          for (int j=0; j<mprims_[i].intermediatePoints.size(); j++) {
              Point p(mprims_[i].intermediatePoints[j].x+current_cell.x_i*resolution_,
                      mprims_[i].intermediatePoints[j].y+current_cell.y_i*resolution_);
              next_action.intermediatePoints.push_back(p);
          }

          cells.push_back(next_cell);
          actions.push_back(next_action);
      }
   }
}

double Environment::getActionCost(Action action) {

}
    
double Environment::getHeuristicCost(DiscreteCell cell) {

}
