#include <pseudo_lattice_planner/heuristics.h>

/********************************************************************
 * Heuristic class functions. Currently missing a Dijkstra heuristic.
 * Should get around to adding that.
 ********************************************************************/

namespace pseudo_lattice_planner {

Heuristics::Heuristics(costmap_2d::Costmap2DROS *costmap,
                       CostFactors cost_factors,
                       MotionConstraints motion_constraints,
                       double time_delta) :
                       costmap_(costmap),
                       pub_initialized_(false),
                       heuristics_(NULL),
                       cost_factors_(cost_factors),
                       motion_constraints_(motion_constraints),
                       time_delta_(time_delta) {
    size_x_= costmap_->getCostmap()->getSizeInCellsX();
    size_y_ = costmap_->getCostmap()->getSizeInCellsY();
    resolution_ = costmap_->getCostmap()->getResolution();
    heuristics_ = new float[size_x_ * size_y_];
    // for dijkstra
    double time_factor = (resolution_ /
                          (motion_constraints_.max_vel_x * 
                           time_delta_)) * cost_factors_.time_cost;
    dijkstra_ = new DijkstraExpansion(size_x_, size_y_);
    setLethalCost(cost_factors_.lethal_cost);
    setNeutralCost(cost_factors_.step_cost + time_factor);
    max_val = 1000.0; // default for now
}

Heuristics::~Heuristics() {
    delete heuristics_;
}

/********************************************************************
 ********************************************************************/
bool Heuristics::dijkstraHeuristic(State *goal) {
    unsigned char *char_costmap = costmap_->getCostmap()->getCharMap();
    int num_cells = size_x_ * size_y_;
    int goal_x = goal->state_i.x_i;
    int goal_y = goal->state_i.y_i;
    try {
        bool ret = dijkstra_->expand(char_costmap, goal_x, goal_y,
                                  num_cells, heuristics_);
        max_val = dijkstra_->max_potential;
        return ret;
    } catch (std::exception ex) {
        ROS_ERROR_STREAM("error: " << ex.what());
        return false;
    }
}

/********************************************************************
 * Set the heuristic value for each cell to be the diagonal distance
 * from that cell to the goal cell plus the time cost if moving at 
 * the maximum velocity
 ********************************************************************/
bool Heuristics::diagDistanceHeuristic(State *goal) {
    unsigned char *char_costmap = costmap_->getCostmap()->getCharMap();
    int num_cells = size_x_ * size_y_;
    int goal_x = goal->state_i.x_i;
    int goal_y = goal->state_i.y_i;

    for (int i=0; i<num_cells; i++) {
        // get coords from index
        int my = i / size_x_;
        int mx = i - (my * size_x_);
        int num_x = abs(goal_x - mx);
        int num_y = abs(goal_y - my);
        int max = std::max(num_x, num_y);
        int min = std::min(num_x, num_y);
        // get diagonal distance
        float cell_dist = (max - min + sqrt(2) * min);
        // estimate time steps to goal
        float m_dist = cell_dist * resolution_;
        float time_estimate = m_dist / motion_constraints_.max_vel_x;

        heuristics_[i] = (cost_factors_.step_cost * cell_dist +
                         cost_factors_.time_cost * time_estimate);    
    }
    if (size_x_ == size_y_) {
        double max_cell_dist = size_x_*sqrt(2);
        double max_m_dist = max_cell_dist * resolution_;
        double time_estimate = max_m_dist / motion_constraints_.max_vel_x;
        max_val = cost_factors_.step_cost * max_cell_dist + 
                   cost_factors_.time_cost * time_estimate;
    }
    return true;
}

double Heuristics::getHeuristic(State *state, Pose goal_pose) {
    if (heuristics_ == NULL) {
        ROS_ERROR("heuristic not yet calculated!");
        return -1;
    }
    int num_cells = size_x_ * size_y_;
    if (state->state_i.grid_cell >= num_cells) {
        ROS_ERROR("getHeuristic: index out of bounds");
        ROS_ERROR("index %d, max %d", state->state_i.grid_cell, num_cells);
        ROS_ERROR("xy pose %f, %f", state->pose.getX(), state->pose.getY());
        return -1;
    }

    double rot_cost = (fabs(state->pose.getRotDistance(goal_pose)) / 
                      (std::max(fabs(motion_constraints_.min_vel_phi),
                                fabs(motion_constraints_.max_vel_phi)) *
                      time_delta_)) * cost_factors_.time_cost;
    // the (1/4) scaling results in a better heuristic
    double ret = (double)heuristics_[state->state_i.grid_cell] + rot_cost/8;

    if (ret > 10e9) {
        ROS_WARN("heuristic_ = %f", heuristics_[state->state_i.grid_cell]);
        ROS_WARN("rot_cost = %f", rot_cost);
    }
    return ret;
}

void Heuristics::initPublisher(std::string topic_name, int publish_scale) {
    ROS_INFO("Heuristics::initPublisher");
    ros::NodeHandle nh;
    heuristics_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, 1);
    publish_scale_ = publish_scale;
    pub_initialized_ = true;
}

void Heuristics::publishHeuristic() {
    ROS_INFO("Heuristics::publishHeuristic");
    if (!pub_initialized_) {
        ROS_ERROR("heuristics publisher not initialized");
        return;
    }
 
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = costmap_->getGlobalFrameID();
    grid.info.resolution = resolution_;
    grid.info.width = size_x_;
    grid.info.height = size_y_;
    grid.info.origin.position.x = costmap_->getCostmap()->getOriginX();
    grid.info.origin.position.y = costmap_->getCostmap()->getOriginY();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0; 
    grid.data.resize(grid.info.width * grid.info.height);

    int max = 0;
    for (int i=0; i<grid.data.size(); i++) {
        int potential = heuristics_[i];
        if (potential > max) {
            max = potential;
        }
    }
    // scale all entries according to max value
    for (int i=0; i<grid.data.size(); i++) {
        grid.data[i] = float(heuristics_[i] * publish_scale_/max);
    }
    heuristics_pub_.publish(grid);
}

} /* namespace pseudo_lattice_planner */
