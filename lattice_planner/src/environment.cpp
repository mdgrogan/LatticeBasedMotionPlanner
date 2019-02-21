#include <lattice_planner/environment.h>

namespace lattice_planner {

#define MAX_VEL 0.5

Environment::Environment(costmap_2d::Costmap2DROS *costmap, 
                         CostFactors cost_factors,
                         const std::string primFile) {
    
    if (!readMotionPrimitives(primFile)) {
        ROS_ERROR("failed to read motion primitives");
    }

    costmap_ = costmap;

    cost_factors_ = cost_factors;

    if (fabs(resolution_ - costmap_->getCostmap()->getResolution()) > ROUNDED_ZERO) {
        ROS_ERROR("map resolution (%.4f) does not match primitive resolution (%.4f)",
                  resolution_, costmap_->getCostmap()->getResolution());
    }
    origin_x_ = costmap_->getCostmap()->getOriginX();
    origin_y_ = costmap_->getCostmap()->getOriginY();
    nx_ = costmap_->getCostmap()->getSizeInCellsX();
    ny_ = costmap_->getCostmap()->getSizeInCellsY();
    
    heuristics_ = new float[nx_ * ny_];
    float time_factor = (float)(resolution_/MAX_VEL * cost_factors_.time_cost);
    ROS_INFO("time_factor: %f", time_factor);
    dijkstra_ = new DijkstraExpansion(nx_, ny_);
    dijkstra_->setNeutralCost(cost_factors_.step_cost + time_factor);
    //dijkstra_->setNeutralCost(cost_factors_.step_cost);
    dijkstra_->setLethalCost(cost_factors_.lethal_cost);
    initPublisher("lattice_planner/heuristics", 100);
}

Environment::~Environment() {
    delete dijkstra_;
    delete heuristics_;
}

bool Environment::setPlanningParams(DiscreteCell goal_cell) {
    goal_cell_ = goal_cell;
    bool ret = calculateHeuristic();

    if (ret) {
        publishHeuristic();
    }

    return ret;
}

bool Environment::calculateHeuristic() {
    unsigned char *char_costmap = costmap_->getCostmap()->getCharMap();
    unsigned int num_cells = nx_ * ny_;
    unsigned int goal_x = goal_cell_.x_i;
    unsigned int goal_y = goal_cell_.y_i;
    bool ret = dijkstra_->expand(char_costmap, goal_x, goal_y,
                                  num_cells, heuristics_);
    //max_val = dijkstra_->max_potential;
    return ret;
}

double Environment::getActionCost(Action action) {

    double costs = 0;

    for (int i=0; i<action.intmPoints.size(); i++) {
        unsigned int x_i = 0;
        unsigned int y_i = 0;

        if (!costmap_->getCostmap()->worldToMap(action.intmPoints[i].x,
                                           action.intmPoints[i].y,
                                           x_i, y_i)) {
            ROS_ERROR("worldToMap failed: %.4f, %.4f",
                      action.intmPoints[i].x,
                      action.intmPoints[i].y);
            return -1;
        }

        int cell_cost = costmap_->getCostmap()->getCost(x_i, y_i);

        // ignoring costs that aren't lethal - this approach
        // would be flawed if we weren't using ideal localization
        if (cell_cost >= cost_factors_.lethal_cost) {
            return -1;
        } else {
            cell_cost = 0;
        }
        costs += cost_factors_.environment_cost * cell_cost;
    }
        
    //costs += cost_factors_.time_cost * 
    //         action.exec_time/(hypot((float)action.dx, (float)action.dy)*resolution_);
    costs += cost_factors_.time_cost * action.exec_time; 
    

    int max = std::max(action.dx, action.dy);
    int min = std::min(action.dx, action.dy);
    costs += cost_factors_.step_cost * ((max-min) + sqrt(2)*min);

    if (action.end_vel_i == -1 || action.start_vel_i == -1) {
        costs += cost_factors_.direction_cost * ((max-min) + sqrt(2)*min);
    }

    // stupid behavior that should be discouraged, maybe just remove prim
    if (action.end_vel_i == 0 && action.start_vel_i == 0) {
        costs += 1000;
    }

    costs += cost_factors_.rotation_cost * 
             ((action.end_theta_i - action.start_theta_i)%nangles_ *
             2*M_PI/nangles_);

    //ROS_INFO("getActionCost");
    return costs;
}
    
double Environment::getHeuristicCost(DiscreteCell cell) {
    if (heuristics_ == NULL) {
        ROS_ERROR("heuristic not yet calculated!");
        return -1;
    }

    int cell_index = costmap_->getCostmap()->getIndex(cell.x_i, cell.y_i);
    if (cell_index >= nx_*ny_) {
        ROS_ERROR("getHeuristic: index out of bounds");
        ROS_ERROR("index %d, max %d", cell_index, nx_*ny_);
        return -1;
    }

    double rot_cost = ((goal_cell_.theta_i - cell.theta_i)%nangles_ * 
                       2*M_PI/nangles_ * cost_factors_.rotation_cost);
    // the (1/4) scaling results in a better heuristic
    //ROS_INFO("rot_cost/100 = %f", rot_cost/10);
    double ret = (double)heuristics_[cell_index] + rot_cost;

    /*
    if (ret > 10e9) {
        ROS_WARN("heuristic[%d]_ = %f", cell_index, heuristics_[cell_index]);
        ROS_WARN("max index = %d", nx_*ny_);
        ROS_WARN("rot_cost = %f", rot_cost);
    }
    */

    return ret;
}

void Environment::mapToWorld(unsigned int mx, unsigned int my,
                double &wx, double &wy) {
    wx = origin_x_ + mx*resolution_;
    wy = origin_y_ + my*resolution_;
}

bool Environment::worldToMap(double wx, double wy,
                unsigned int &mx, unsigned int &my) {
    if (wx < origin_x_ || wy < origin_y_ ) {
        return false;
    }

    mx = (int)((wx - origin_x_)/resolution_);
    my = (int)((wy - origin_y_)/resolution_);

    if (mx < nx_ && my < ny_) {
        return true;
    }

    return false;
}
        
DiscreteCell Environment::discretizePose(geometry_msgs::PoseStamped pose, int vel) {
    DiscreteCell ret;
    unsigned int x_i = 0;
    unsigned int y_i = 0;

    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf::getYaw(pose.pose.orientation);
    ROS_INFO("theta %.3f", theta);
    while (theta < 0 - ROUNDED_ZERO) {
        theta += 2*M_PI;
    }
    while (theta >= 2*M_PI + ROUNDED_ZERO) {
        theta -= 2*M_PI;
    }

    worldToMap(x, y, x_i, y_i);
    ret.x_i = (int)x_i;
    ret.y_i = (int)y_i;
    ret.theta_i = theta/(2*M_PI/(nangles_-1)) + 0.5;
    if (ret.theta_i == nangles_) {
        ret.theta_i = 0;
    }
    ret.vel_i = vel;

    return ret;
}

/*
geometry_msgs::PoseStamped Environment::getPose(DiscreteCell cell) {
    geometry_msgs::PoseStamped ret;
    double x,y;
    costmap_->getCostmap()->mapToWorld(cell.x_i, cell.y_i, x, y);
    tf::Quaternion tmp;
    tmp.setEulerZYX(cell.theta_i*(2*M_PI/nangles_), 0, 0);

    ret.pose.position.x = x;
    ret.pose.position.y = y;
    //ret.pose.position.z = hopefully the default is sane?
    ret.pose.orientation.x = tmp.getX();
    ret.pose.orientation.y = tmp.getY();
    ret.pose.orientation.z = tmp.getZ();
    ret.pose.orientation.w = tmp.getW();

    return ret;
}
*/

geometry_msgs::PoseStamped Environment::getPose(Point point) {
    geometry_msgs::PoseStamped ret;
    tf::Quaternion tmp;
    tmp.setEulerZYX(point.theta, 0, 0);

    ret.pose.position.x = point.x;
    ret.pose.position.y = point.y;
    //ret.pose.position.z = hopefully the default is sane?
    ret.pose.orientation.x = tmp.getX();
    ret.pose.orientation.y = tmp.getY();
    ret.pose.orientation.z = tmp.getZ();
    ret.pose.orientation.w = tmp.getW();

    return ret;
}

void Environment::getNextCells(DiscreteCell current_cell,
                                std::vector<DiscreteCell> &cells,
                                std::vector<Action> &actions) {

    //ROS_INFO("current_cell.theta_i = %d", current_cell.theta_i);
    //ROS_INFO("current_cell.vel_i = %d", current_cell.vel_i);
    //ROS_INFO("mprims_.size() = %d", mprims_.size());
    
    for (int i=0; i<mprims_.size(); i++) {
        //ROS_INFO("mprims_[i].start_theta_i = %d", mprims_[i].start_theta_i);
        //ROS_INFO("mprims_[i].start_vel_i = %d", mprims_[i].start_vel_i);
        if (mprims_[i].start_theta_i==current_cell.theta_i &&
             mprims_[i].start_vel_i==current_cell.vel_i) {

            //ROS_INFO("i: %d", i);

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
            next_action.dx = abs(mprims_[i].end_cell_i.x_i);
            next_action.dy = abs(mprims_[i].end_cell_i.y_i);
            next_action.exec_time = mprims_[i].exec_time;

            double world_x = 0;
            double world_y = 0;
            mapToWorld(current_cell.x_i, current_cell.y_i,
                       world_x, world_y);

            //ROS_INFO("mprims_[%d].intmPoints.size() = %d", i, mprims_[i].intmPoints.size());
            for (int j=0; j<mprims_[i].intmPoints.size(); j++) {
                Point p(mprims_[i].intmPoints[j].x + world_x,
                        mprims_[i].intmPoints[j].y + world_y,
                        mprims_[i].intmPoints[j].theta);
                next_action.intmPoints.push_back(p);
                next_action.intmTimes.push_back(mprims_[i].intmTimes[j]);
            }

            if (getActionCost(next_action) < 0) {
                continue;
            }

            cells.push_back(next_cell);
            actions.push_back(next_action);
        }
    }
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
            if (tmp == "resolution_m:") {
                resolution_ = std::stod(line);
                //std::cout<<tmp<<" "<<resolution_<<std::endl;
                break;
            }
            if (tmp == "numberofangles:") {
                nangles_ = std::stoi(line);
                /*
                //std::cout<<tmp<<" "<<nangles_<<std::endl;
                std::getline(in, line);
                //std::cout<<"original:"<<line<<std::endl;
                std::stringstream ss(line);
                double d;
                while (ss>>d) {
                    angles_.push_back(d);
                    if (ss.peek() == ' ')
                        ss.ignore();
                }
                //std::cout<<angles_.size()<<std::endl;
                */
                break;
            }
            if (tmp == "numberofvelocities:") {
                nvels_ = std::stoi(line);
                //std::cout<<tmp<<" "<<nvels_<<std::endl;
                break;
            }
            if (tmp == "timetoturn22.5deginplace:") {
                turn_in_place_time_ = std::stod(line);
                //std::cout<<tmp<<" "<<turn_in_place_time_<<std::endl;
                break;
            }
            if (tmp == "startangle_i:") {
                mprim.start_theta_i = std::stoi(line);
                //std::cout<<tmp<<" "<<mprim.start_theta_i<<std::endl;
                break;
            }
            if (tmp == "startvel_i:") {
                mprim.start_vel_i = std::stoi(line);
                //std::cout<<tmp<<" "<<resolution_<<std::endl;
                break;
            }
            if (tmp == "exectime:") {
                mprim.exec_time = std::stod(line);
                //std::cout<<tmp<<" "<<resolution_<<std::endl;
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
                //std::cout<<tmp<<" "<<mprim.end_cell_i.x_i<<" "<<mprim.end_cell_i.y_i<<" "<<mprim.end_cell_i.theta_i<<std::endl;
            }
            if (tmp == "intermediateposes:") {
                int numLines = std::stoi(line);
                //std::cout<<tmp<<std::endl;
                for (int i=0; i<numLines; i++) {

                    std::getline(in, line);
                    //std::cout<<"original:"<<line<<std::endl;
                    std::vector<double> vals;
                    std::stringstream ss(line);
                    double d;
                    while (ss>>d) {
                        vals.push_back(d);
                        if (ss.peek() == ' ')
                            ss.ignore();
                    }

                    //std::cout<<"vals:"<<vals[0]<<" "<<vals[1]<<" "<<vals[2]<<" "<<vals[3]<<std::endl;

                    Point p(vals[0], vals[1], vals[2]);
                    mprim.intmPoints.push_back(p);
                    mprim.intmTimes.push_back(vals[3]);
                    //std::cout<<"mprim:"<<p.x<<" "<<p.y<<" "<<p.theta<<" "<<vals[3]<<std::endl;
                }
                //std::cout<<"mprim.intmPoints.size() = "<<mprim.intmPoints.size()<<std::endl;
                mprims_.push_back(mprim);
                mprim.intmPoints.clear();
                mprim.intmTimes.clear();
            }
        }
    }
    in.close();
    //ROS_INFO("mprims_.size() = %d", mprims_.size());

/*
    for (int i=0; i<15; i++) {
        ROS_INFO("start angle: %d", mprims_[i].start_theta_i);
        ROS_INFO("start vel: %d", mprims_[i].start_vel_i);
        ROS_INFO("end pose: %d %d %d %d", mprims_[i].end_cell_i.x_i,
                                          mprims_[i].end_cell_i.y_i,
                                          mprims_[i].end_cell_i.theta_i);
        ROS_INFO("exec time: %.4f", mprims_[i].exec_time);
        ROS_INFO("intermediate points");
        for (int j=0; j<mprims_[i].intmPoints.size(); j++) {
            ROS_INFO("%.4f %.4f %.4f %.4f", mprims_[i].intmPoints[j].x,
                                            mprims_[i].intmPoints[j].y,
                                            mprims_[i].intmPoints[j].theta,
                                            mprims_[i].intmTimes[j]);
        }
    }
*/

    return true;
}

void Environment::initPublisher(std::string topic_name, int publish_scale) {
    ros::NodeHandle nh;
    heuristics_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);
    publish_scale_ = publish_scale;
    pub_initialized_ = true;
}

void Environment::publishHeuristic() {
    if (!pub_initialized_) {
        ROS_ERROR("heuristics publisher not initialized");
        return;
    }
 
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = costmap_->getGlobalFrameID();
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = costmap_->getCostmap()->getResolution();
    grid.info.width = nx_;
    grid.info.height = ny_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.05;
    grid.info.origin.orientation.w = 1.0; 
    grid.data.resize(grid.info.width * grid.info.height);

    int max = 0;
    for (int i=0; i<grid.data.size(); i++) {
        int potential = heuristics_[i];
        if (potential > max && potential < POT_HIGH) {
            max = potential;
        }
    }
    // scale all entries according to max value
    for (int i=0; i<grid.data.size(); i++) {
        if (heuristics_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else {
            grid.data[i] = float(heuristics_[i] * publish_scale_/max);
        }
    }
    heuristics_pub_.publish(grid);
}
    
} /* namespace lattice_planner */
