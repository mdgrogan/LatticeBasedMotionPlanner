#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <lattice_planner/planner.h>



/********************************************************************
 * Planner implementation. 
 ********************************************************************/

namespace lattice_planner {

#define ROUNDED_ZERO 1e-6
std::string PRIMFILE("/home/grogan/catkin_ws/src/LatticeBasedMotionPlanner/prim_gen/unicycle_2p5cm.mprim");

int Hasher::num_x_ = 0;
int Hasher::num_y_ = 0;
int Hasher::num_theta_ = 0;
int Hasher::num_vel_ = 0;

/********************************************************************
 * Constructor
 ********************************************************************/
LatticePlanner::LatticePlanner(std::string name, 
                           costmap_2d::Costmap2DROS *costmap) {
    ROS_INFO("LatticePlanner::LatticePlanner");
    ros::NodeHandle private_nh("~/" + name);

    expanded_paths_pub_ = 
        private_nh.advertise<nav_msgs::Path>("expanded_paths", 1);

    path_pub_ = private_nh.advertise<nav_msgs::Path>("path", 1);

    waypoint_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("path_pose", 1);

    CostFactors cost_factors;
    private_nh.param("lethal_cost", cost_factors.lethal_cost,
                         (int)costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    private_nh.param("time_cost_factor", cost_factors.time_cost, 200.0);
    private_nh.param("step_cost_factor", cost_factors.step_cost, 80.0);
    private_nh.param("dir_cost_factor", cost_factors.direction_cost, 5.0);
    private_nh.param("rotation_cost_factor", cost_factors.rotation_cost, 5.0);
    private_nh.param("environment_cost_factor", cost_factors.environment_cost, 1.0);
    
    private_nh.param("planning_timeout", planning_timeout_, 5.0);
    private_nh.param("publish_expanded", publish_expanded_, false);
    private_nh.param("debug", debug_, false);

    private_nh.param("heuristic_inflation", eps_, 3.0);

    ROS_INFO("time cost: %f, step cost: %f", cost_factors.time_cost,
                                             cost_factors.step_cost);

    costmap_ = costmap;
    env_ = new Environment(costmap, cost_factors, PRIMFILE);
    
    // set up hasher for state_map
    int nx = costmap_->getCostmap()->getSizeInCellsX();
    int ny = costmap_->getCostmap()->getSizeInCellsY();
    int ntheta = 16;
    int nvel = 4;
    Hasher::setLimits(nx, ny, ntheta, nvel);
    
    // set up Q-learning thing
    // alpha, gamma
    double discount, Qlr;
    int greediness;
    private_nh.param("discount", discount, 0.5);
    private_nh.param("Qlr", Qlr, 0.1);
    private_nh.param("greediness", greediness, 90);

    QT_ = new QStuff::QTable(greediness, Qlr, discount);
    QT_->loadTable("/home/grogan/Qtable_vals.txt");
    //QSD_ = new QStuff::QStateDiscretizer(-0.08887, 0.82106,
    //                                      //1.44389, 0.31415,
    //                                      4.54869, 0.65246,
    //                                      eps_);


    open_list_ = NULL;
    incons_list_ = NULL;
    all_expanded_.rehash(1000000); //million good?
    all_expanded_.reserve(1000000);

    start_state_ = NULL;
    goal_state_ = NULL;
    search_iteration_ = 0;
    call_number_ = 0;

    next_start_ = env_->getPose(Point(0, 0, 0));
}

/********************************************************************
 * Destructor
 ********************************************************************/
LatticePlanner::~LatticePlanner() {
    ROS_INFO("LatticePlanner::~LatticePlanner");
    reset();
    delete env_;
    delete open_list_;
    delete incons_list_;
    delete QT_;
    //delete QSD_;
}

/********************************************************************
 * Free up containers.
 * Deleting states here.
 ********************************************************************/
void LatticePlanner::reset() {
    //ROS_INFO("LatticePlanner::reset");
    if (open_list_ != NULL) {
        if (!open_list_->isEmpty()) {
            open_list_->makeEmpty();
        }
        //delete open_list_;
        //open_list_ = NULL;
    }
    if (incons_list_ != NULL) {
        if (!incons_list_->isEmpty()) {
            incons_list_->makeEmpty();
        }
        //delete incons_list_;
        //incons_list_ = NULL;
    }
    state_map::iterator it;
    for (it=all_expanded_.begin(); it != all_expanded_.end(); it++) {
        if (it->second != NULL) {
            delete it->second;
        }
        else
            ROS_WARN("NULL state in reset()");
    }
    all_expanded_.clear();

    expanded_paths_.poses.clear();

    //clearRobotFootprint();
}

/********************************************************************
 * Timer callback. This is not set up currently. Will ultimately be 
 * used to set a time limit on planning.
 ********************************************************************/
void LatticePlanner::plannerTimeoutCallback(const ros::TimerEvent &event) {
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("LatticePlanner::plannerTimeoutCallback");
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    planning_timed_out_ = true;
}

/********************************************************************
 * Clear the ole robot footprint. Haven't really validated this...
 ********************************************************************/
void LatticePlanner::clearRobotFootprint() {
    //ROS_INFO("LatticePlanner::clearRobotFootprint");
    std::vector<geometry_msgs::Point> footprint = costmap_->getRobotFootprint();
    costmap_->getCostmap()->setConvexPolygonCost(footprint, costmap_2d::FREE_SPACE);
}

/********************************************************************
 * Allocate memory for a new state and assign default values.
 * Add to the state_map of expanded states.
 ********************************************************************/
State* LatticePlanner::createState(State state) {
    State *new_state = new State;
    new_state->stateID = state.stateID;
    new_state->heapIndex = 0;
    new_state->listElement = NULL;
    new_state->iterationClosed = 0;
    new_state->callNumberAccessed = call_number_;
    new_state->numExpands = 0;
    new_state->cell_i = state.cell_i;
    new_state->action = state.action;
    new_state->parent = state.parent;
    new_state->g = MAX;
    new_state->v = MAX;
    new_state->h = env_->getHeuristicCost(new_state->cell_i);

    std::pair<DiscreteCell, State*> pair(new_state->cell_i, new_state);
    all_expanded_.insert(pair);

    return new_state;
}

/********************************************************************
 * Return a state pointer for a state with the key state_i. If such a
 * state has already been created, it will be in the state_map. 
 * Otherwise, create it.
 ********************************************************************/
State* LatticePlanner::getState(State state) {
    state_map::const_iterator it = all_expanded_.find(state.cell_i);
    if (it != all_expanded_.end()) { // already exists
        //ROS_INFO("getState: already exists");
        return it->second;
    } else {                        // doesn't exist, create it
        //ROS_INFO("getState: creating state");
        return createState(state);
    }
}

/********************************************************************
 * Generate the set of feasible states that can be reached from 
 * current state
 ********************************************************************/
std::vector<State> LatticePlanner::getNextStates(State *current) {
    std::vector<State> ret;
    std::vector<DiscreteCell> next_cells;
    std::vector<Action> next_actions;
    env_->getNextCells(current->cell_i, next_cells, next_actions);

    //ROS_INFO("next_cells.size() = %d", next_cells.size());
    for (int i=0; i<next_cells.size(); i++) {
        State next;
        next.stateID = Hasher::getHash(next_cells[i]);
        next.cell_i = next_cells[i];
        next.action = next_actions[i];
        next.parent = current; 

        ret.push_back(next);
    }

    return ret;
}

/********************************************************************
 * Improve on the current path. Reference ARA* paper for more.
 ********************************************************************/
int LatticePlanner::improvePath(double cur_eps, State *&best, bool &goal_found,
                                 double end_time) {
    ROS_INFO("LatticePlanner::improvePath");
    int expands = 0;
    State *current = NULL;
    double goal_key = goal_state_->g;
    double min_key = open_list_->getMinKey();
    double old_key = min_key;
    //ROS_INFO("goal_key = %.3f", goal_key);
    //ROS_INFO("min_key = %.3f", min_key);

    while (!open_list_->isEmpty() && min_key < MAX && goal_key > min_key &&
           (double)clock()/CLOCKS_PER_SEC < end_time) {
        if (expands%50000 == 0) {
            ROS_INFO("%f < %f", (double)clock()/CLOCKS_PER_SEC, end_time);
        }
        // pop heap
        //ROS_INFO("pop heap");
        current = open_list_->deleteMinElement();
        if (best == NULL) {
            best = current;
        } else if (current->h < best->h) {
            best = current;
        }
        
        if (current->listElement != NULL) {
            ROS_ERROR("state is already in incons_list_");
        }
        if (current->v == current->g) {
            ROS_ERROR("consistent state being expanded");
        }

        // state is now closed
        current->iterationClosed = search_iteration_;
        current->v = current->g;
        expands++;
        current->numExpands++;
        /*
        ROS_INFO("current [x y theta vel] = [%d %d %d %d]",
                 current->cell_i.x_i,
                 current->cell_i.y_i,
                 current->cell_i.theta_i,
                 current->cell_i.vel_i);
        ROS_INFO("current [g h] = [%.3f %.3f]",
                 current->g,
                 current->h);
        ROS_INFO("openlist: %d, all: %d",
                    open_list_->cur_size,
                    all_expanded_.size());
        */

        // get successor states
        std::vector<State> next_states = getNextStates(current);
        //ROS_INFO("next_states.size() = %d", next_states.size());
        for (int i=0; i<next_states.size(); i++) {
            if (next_states[i].parent == NULL) {
                ROS_ERROR("next_states[%d].parent == NULL", i);
            }

            State *succ = getState(next_states[i]);
            //ROS_INFO("next_states[%d]", i);
            //ROS_INFO("succ.cell_i.x_i= = %d", succ->cell_i.x_i);
            //ROS_INFO("succ.cell_i.y_i= = %d", succ->cell_i.y_i);
            //ROS_INFO("succ.cell_i.theta_i= = %d", succ->cell_i.theta_i);
            //ROS_INFO("succ.cell_i.vel_i= = %d", succ->cell_i.vel_i);
            // if state is the goal, set the goal->parent pointer accordingly
            if (succ->cell_i == goal_state_->cell_i && succ->parent == NULL) {
                ROS_INFO("found goal");
                succ->parent = current;
                succ->action = next_states[i].action;
                goal_found = true;
            }
            // if state is the start, ignore - can't be improved
            if (succ->cell_i == start_state_->cell_i) {
                continue;
            }

            if (succ->parent == NULL) { // shouldn't happen
                ROS_ERROR("succ.parent == NULL");
                ROS_ERROR("succ->stateID = %d", succ->stateID);
            }

            //double cost = env_->getActionCost(succ->action);
            double cost = env_->getActionCost(next_states[i].action);
            if (cost <= 0 || succ->h == POT_HIGH) { // invalid action
                //ROS_INFO("invalid action");
                continue;
            }

            // improve state if possible
            if (succ->g > current->v + cost) {
                //ROS_INFO("improve state");
                //double tmp = succ->g;
                succ->g = current->v + cost;
                succ->parent = current;
                succ->action = next_states[i].action;

                // if state was not previously closed, heap it
                // else it belongs in inconsistent list (if not already there)
                if (succ->iterationClosed != search_iteration_) {
                    double key = succ->g + cur_eps*succ->h;
                    /*
                    if (key+0.0001 < start_state_->h) {
                        ROS_WARN("WTF");
                        ROS_INFO("next_states[%d]", i);
                        ROS_INFO("goal.cell_i.x_i= = %d", goal_state_->cell_i.x_i);
                        ROS_INFO("goal.cell_i.y_i= = %d", goal_state_->cell_i.y_i);
                        ROS_INFO("succ.cell_i.x_i= = %d", succ->cell_i.x_i);
                        ROS_INFO("succ.cell_i.y_i= = %d", succ->cell_i.y_i);
                        ROS_INFO("cur.cell_i.x_i= = %d", current->cell_i.x_i);
                        ROS_INFO("cur.cell_i.y_i= = %d", current->cell_i.y_i);
                        ROS_INFO("succ.g: %f, succ.h: %f", tmp, succ->h);
                        ROS_INFO("current.v: %f, cost: %f", current->v, cost);
                        ROS_INFO("key: %f", key);
                        retracePath(succ, tmp);
                    }
                    */
                    //ROS_INFO("heap");
                    if (succ->heapIndex != 0) {
                        open_list_->updateElement(succ, key);
                    } else {
                        open_list_->insertElement(succ, key);
                    }
                } else if (succ->listElement == NULL) {
                    //ROS_INFO("list");
                    incons_list_->insertElement(succ);
                }
            } //else {
                //ROS_INFO("couldn't improve state");
            //}
        }

        // reset keys
        old_key = min_key;
        min_key = open_list_->getMinKey();
        goal_key = goal_state_->g;

        // debugging stuff set in the params file
        if (publish_expanded_) {
            double tmp;
            std::vector<geometry_msgs::PoseStamped> poses = retracePath(current, tmp);
            expanded_paths_.poses.insert(expanded_paths_.poses.end(),
                    poses.begin(), poses.end());
            std::reverse(poses.begin(), poses.end());
            expanded_paths_.poses.insert(expanded_paths_.poses.end(),
                    poses.begin(), poses.end());
            if (debug_) {
                //ROS_INFO("expanded_paths_ size: %d", expanded_paths_.poses.size());
                expanded_paths_.header.frame_id = costmap_->getGlobalFrameID();
                //expanded_paths_.header.stamp = ros::Time::now();
                expanded_paths_pub_.publish(expanded_paths_);
                ROS_INFO("goal_key = %.3f", goal_key);
                ROS_INFO("min_key = %.3f", old_key);
                //ROS_INFO("old_key = %.3f", old_key);
                ROS_INFO("current [x y theta vel] = [%d %d %d %d]",
                        current->cell_i.x_i,
                        current->cell_i.y_i,
                        current->cell_i.theta_i,
                        current->cell_i.vel_i);
                ROS_INFO("current [g h] = [%.3f %.3f]",
                        current->g,
                        current->h*cur_eps);
                ROS_INFO("openlist: %d, all: %d",
                        open_list_->cur_size,
                        all_expanded_.size());
                std::string input;
                std::cout << "press enter to continue, c + enter to finish planning without interrupt: ";

                std::getline(std::cin, input);
                if(input.compare("c") == 0)
                    debug_ = false;
            }
        }
        ros::spinOnce();
    }
    if (open_list_->isEmpty()) {
        ROS_INFO("improvePath: open list empty");
        return -1;
    } else if (min_key >= MAX) {
        ROS_INFO("improvePath: min_key >= MAX");
        return -1;
    } else if (goal_key <= min_key) {
        ROS_INFO("improvePath: goal_key <= min_key");
        return 0;
    } else {
        ROS_INFO("improvePath: timed out");
        return 1;
    }
}

/********************************************************************
 * Main function. Need to set up planning timeout and set up a way 
 * of signaling the path executer to begin execution without exiting.
 * Or at least without losing information. That might mean a more
 * sophisticated method of evaluating when to reset
 ********************************************************************/
bool LatticePlanner::getPath(geometry_msgs::PoseStamped start,
                             geometry_msgs::PoseStamped goal,
                     std::vector<geometry_msgs::PoseStamped> &path) {
    ROS_INFO("LatticePlanner::getPath");
    //call_number_++;
    // clear containers
    reset();
    // new containers
    open_list_ = new Heap;
    incons_list_ = new List;


    /*
    ROS_INFO("origin = %.5f %.5f", costmap_->getCostmap()->getOriginX(),
                                   costmap_->getCostmap()->getOriginY());

    ROS_INFO("start [x y theta vel] = [%d %d %d %d]",
             start_state_->cell_i.x_i,
             start_state_->cell_i.y_i,
             start_state_->cell_i.theta_i,
             start_state_->cell_i.vel_i);
    ROS_INFO("cont [x y] = %.5f %.5f", start.pose.position.x, start.pose.position.y);
    double world_x = 0;
    double world_y = 0;
    env_->mapToWorld(start_state_->cell_i.x_i,
                                       start_state_->cell_i.y_i,
                                        world_x, world_y);
    ROS_INFO("world_x world_y = %.5f %.5f", world_x, world_y);
    unsigned int x_i = 0;
    unsigned int y_i = 0;
    env_->worldToMap(start.pose.position.x, start.pose.position.y, 
            x_i, y_i);
    ROS_INFO("map_x map_y = %d %d", x_i, y_i);

    ROS_INFO("goal [x y theta vel] = [%d %d %d %d]",
             goal_state_->cell_i.x_i,
             goal_state_->cell_i.y_i,
             goal_state_->cell_i.theta_i,
             goal_state_->cell_i.vel_i);
    ROS_INFO("cont [x y] = %.5f %.5f", goal.pose.position.x, goal.pose.position.y);
    costmap_->getCostmap()->mapToWorld(goal_state_->cell_i.x_i,
                                       goal_state_->cell_i.y_i,
                                        world_x, world_y);
    ROS_INFO("world_x world_y = %.5f %.5f", world_x, world_y);
    */

    TimingInfo timing_info;
    // set heuristic inside clock

    if (next_start_.pose.position.x == goal.pose.position.x &&
        next_start_.pose.position.y == goal.pose.position.y) {
        ROS_INFO("ignore");
        return true;
    }

    goal_state_ = setGoalState(goal);

    if (!env_->setPlanningParams(goal_state_->cell_i)) {
        ROS_ERROR("heuristic calculation failed");
        return false;
    }
    
    //start_state_ = setStartState(start);
    start_state_ = setStartState(next_start_);

    if (start_state_ == NULL || goal_state_ == NULL) {
        ROS_INFO("Bad start or goal state");
        return false;
    }

    // Set start state cost
    start_state_->g = 0.0;

    // insert start state into open list
    double key = start_state_->g + eps_*start_state_->h;
    open_list_->insertElement(start_state_, key);

    call_number_++;

    State *current_best = NULL;
    bool found_goal = false;
    double cur_eps = eps_;

    // main loop
    search_iteration_ = 1;
    double time_limit = (double)clock()/CLOCKS_PER_SEC+planning_timeout_;
    int ret = improvePath(cur_eps, current_best, 
            found_goal, time_limit);
    expanded_paths_.poses.clear();

    if (ret == -1) {
        ROS_WARN("bad");
        return false;
    }
    if (ret == 1 || !found_goal) {
        ROS_WARN("timed out on first search");
        return false;
    }


    //std::string input;
    //std::getline(std::cin, input);
    
    double exec_time = publishPlan(goal_state_);
    timing_info.update(exec_time);
    double reward = 0.0;
    double rewardSum = reward;
    double eps_bound = getBoundedEps(cur_eps);
    /////////////////////////////////////////////////////
    int prev_open_size = open_list_->cur_size;
    int prev_incons_size = incons_list_->cur_size;
    int prev_expanded_size = (int)all_expanded_.size();
    double prev_eps_bound = eps_bound;
    double prev_g = goal_state_->g;
    FILE *fp;
    fp = fopen("/home/grogan/planner_output.txt", "a");
    fprintf(fp, "%d, %.3f, %.3f, %.3f, %.3f, %.1f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %d, %d, %d, %d\n",
            call_number_,
            timing_info.plan_time_,
            timing_info.exec_time_,
            reward,
            rewardSum,
            cur_eps,
            eps_bound,
            eps_bound - prev_eps_bound,
            goal_state_->g,
            goal_state_->g - prev_g,
            start_state_->h,
            open_list_->cur_size,
            open_list_->cur_size - prev_open_size,
            incons_list_->cur_size,
            incons_list_->cur_size - prev_incons_size,
            all_expanded_.size(),
            (int)(all_expanded_.size()) - prev_expanded_size);
    fclose(fp);
    /////////////////////////////////////////////////////

    QStuff::QState q = QT_->discretize(
            std::vector<double> {timing_info.plan_time_,
            exec_time, 
            0, 
            0,
            0});
    int q_action = QT_->getAction(q);
    ROS_INFO("Q state = [%d %d %d %d]", 
            q.features[0], q.features[1], q.features[2], q.features[3]);
    ROS_INFO("Action %d", q_action);

    while (q_action != ACTION_SIZE-1 && 
           (double)clock()/CLOCKS_PER_SEC < time_limit &&
           (cur_eps > 1+ROUNDED_ZERO || ret == 1)) {
        double new_plan_time = (double)clock()/CLOCKS_PER_SEC + 
                               QT_->indexToAction(q_action);
        while ((double)clock()/CLOCKS_PER_SEC < new_plan_time && 
               (double)clock()/CLOCKS_PER_SEC < time_limit &&
               (cur_eps > 1+ROUNDED_ZERO || ret == 1)) {
            if (ret == 0) {
                search_iteration_++;
                cur_eps -= 0.2;
                buildOpenList(cur_eps);
            }
            ret = improvePath(cur_eps, current_best, 
                              found_goal, new_plan_time);
            expanded_paths_.poses.clear();
            // this can be done better
            double tmp;
            retracePath(goal_state_, tmp);
            if (tmp < exec_time) {
                exec_time = publishPlan(goal_state_);
            }

            //printStuff(timing_info, cur_eps, eps_bound, reward);

            //std::string input;
            //std::getline(std::cin, input);
            //expanded_paths_.poses.clear();
        }
        reward = timing_info.update(exec_time);
        rewardSum += reward;
        eps_bound = getBoundedEps(cur_eps);

        /////////////////////////////////////////////////////
        fp = fopen("/home/grogan/planner_output.txt", "a");
        fprintf(fp, "%d, %.3f, %.3f, %.3f, %.3f, %.1f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %d, %d, %d, %d\n",
                call_number_,
                timing_info.plan_time_,
                timing_info.exec_time_,
                reward,
                rewardSum,
                cur_eps,
                eps_bound,
                eps_bound - prev_eps_bound,
                goal_state_->g,
                goal_state_->g - prev_g,
                start_state_->h,
                open_list_->cur_size,
                open_list_->cur_size - prev_open_size,
                incons_list_->cur_size,
                incons_list_->cur_size - prev_incons_size,
                all_expanded_.size(),
                (int)(all_expanded_.size()) - prev_expanded_size);
        fclose(fp);

        /////////////////////////////////////////////////////

        double d_open = open_list_->cur_size - prev_open_size;
        double d_incons = incons_list_->cur_size - prev_incons_size;
        double d_eps = eps_bound-prev_eps_bound;
        QStuff::QState q_new = QT_->discretize(
                std::vector<double> {timing_info.plan_time_,
                                     exec_time, 
                                     d_open, 
                                     d_incons,
                                     d_eps});

        prev_expanded_size = (int)all_expanded_.size();
        prev_eps_bound = eps_bound;
        prev_g = goal_state_->g;

        QT_->update(q, q_new, q_action, reward);
        ROS_INFO("Updated table value: %.3f", QT_->getTableVal(q, q_action));
        QT_->saveTable("/home/grogan/Qtable_vals.txt");
        q = q_new;
        if (fabs(cur_eps - eps_) < ROUNDED_ZERO && ret == 0) {
            //ROS_INFO("(1)");
            //ROS_INFO("%d", q.eps_i);
            q_action = ACTION_SIZE-1;
        } else {
            //ROS_INFO("(2)");
            //ROS_INFO("%d", q.eps_i);
            q_action = QT_->getAction(q);
        }

        ROS_INFO("-------------------");
        ROS_INFO("Q state = [%d %d %d %d]", 
                q.features[0], q.features[1], q.features[2], q.features[3], q.features[4]);
        ROS_INFO("Action %d", q_action);
    }
    QT_->updateTerminal(q, q_action, rewardSum);
    QT_->saveTable("/home/grogan/Qtable_vals.txt");

    // Write the start/goal sequence plus times for posterity
    //FILE *fp;
    fp = fopen("/home/grogan/start_goal_sequence.txt", "a");
    fprintf(fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            next_start_.pose.position.x, next_start_.pose.position.y, 
            tf::getYaw(next_start_.pose.orientation),
            goal.pose.position.x, goal.pose.position.y, 
            tf::getYaw(goal.pose.orientation),
            timing_info.plan_time_, timing_info.exec_time_);
    fclose(fp);

    // done with planning by here
    
    //if (found_goal) {
    //    current_plan_ = retracePath(goal_state_);
    //} else if (current_best != NULL) {
    //    current_plan_ = retracePath(current_best);
    //}


    ros::Duration offset = ros::Time::now() - current_plan_[0].header.stamp;
    for (int i=0; i<current_plan_.size(); i++) {
        //current_plan_[i].header.frame_id = costmap_->getGlobalFrameID();
        current_plan_[i].header.stamp += offset;
        //ROS_INFO("t = %.4f", poses[i].header.stamp.toSec());
    }

    //path = current_plan_;
    std::vector<geometry_msgs::PoseStamped> p;
    path = p;
    next_start_ = goal;
    return false;

    if (!path.empty()) {
        return true;
    } else {
        return false;
    }
}

/**/
#define FIXED 0.2

bool LatticePlanner::getPathFixedPolicy(geometry_msgs::PoseStamped start,
                             geometry_msgs::PoseStamped goal,
                     std::vector<geometry_msgs::PoseStamped> &path) {
    ROS_INFO("LatticePlanner::getPath");
    //call_number_++;
    // clear containers
    reset();
    // new containers
    open_list_ = new Heap;
    incons_list_ = new List;

    TimingInfo timing_info;
    // set heuristic inside clock

    if (next_start_.pose.position.x == goal.pose.position.x &&
        next_start_.pose.position.y == goal.pose.position.y) {
        ROS_INFO("ignore");
        return true;
    }

    goal_state_ = setGoalState(goal);

    if (!env_->setPlanningParams(goal_state_->cell_i)) {
        ROS_ERROR("heuristic calculation failed");
        return false;
    }
    
    //start_state_ = setStartState(start);
    start_state_ = setStartState(next_start_);

    if (start_state_ == NULL || goal_state_ == NULL) {
        ROS_INFO("Bad start or goal state");
        return false;
    }

    // Set start state cost
    start_state_->g = 0.0;

    // insert start state into open list
    double key = start_state_->g + eps_*start_state_->h;
    open_list_->insertElement(start_state_, key);

    call_number_++;

    State *current_best = NULL;
    bool found_goal = false;
    double cur_eps = eps_;

    // main loop
    double time_limit = (double)clock()/CLOCKS_PER_SEC+planning_timeout_;
    search_iteration_ = 1;
    int ret = improvePath(cur_eps, current_best, 
            found_goal, time_limit);
    expanded_paths_.poses.clear();

    if (ret == -1) {
        ROS_WARN("bad");
        return false;
    }
    if (ret == 1 || !found_goal) {
        ROS_WARN("timed out on first search");
        return false;
    }

    double exec_time = publishPlan(goal_state_);
    timing_info.update(exec_time);
    double reward = 0.0;
    double rewardSum = reward;
    double eps_bound = getBoundedEps(cur_eps);

    //printStuff(timing_info, cur_eps, eps_bound, reward);

    /////////////////////////////////////////////////////
    int prev_open_size = open_list_->cur_size;
    int prev_incons_size = incons_list_->cur_size;
    int prev_expanded_size = (int)all_expanded_.size();
    double prev_eps_bound = eps_bound;
    double prev_g = goal_state_->g;
    FILE *fp;
    fp = fopen("/home/grogan/planner_output.txt", "a");
    fprintf(fp, "%d, %.3f, %.3f, %.3f, %.3f, %.1f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %d, %d, %d, %d\n",
            call_number_,
            timing_info.plan_time_,
            timing_info.exec_time_,
            reward,
            rewardSum,
            cur_eps,
            eps_bound,
            eps_bound - prev_eps_bound,
            goal_state_->g,
            goal_state_->g - prev_g,
            start_state_->h,
            open_list_->cur_size,
            open_list_->cur_size - prev_open_size,
            incons_list_->cur_size,
            incons_list_->cur_size - prev_incons_size,
            all_expanded_.size(),
            (int)(all_expanded_.size()) - prev_expanded_size);
    fclose(fp);
    /////////////////////////////////////////////////////

    double new_plan_time = (double)clock()/CLOCKS_PER_SEC + FIXED;
    while ((double)clock()/CLOCKS_PER_SEC < time_limit &&
           (cur_eps > 1+ROUNDED_ZERO || ret == 1)) {
        double new_plan_time = (double)clock()/CLOCKS_PER_SEC + FIXED;
        while ((double)clock()/CLOCKS_PER_SEC < new_plan_time && 
               (double)clock()/CLOCKS_PER_SEC < time_limit &&
               (cur_eps > 1+ROUNDED_ZERO || ret == 1)) {
            if (ret == 0) {
                search_iteration_++;
                cur_eps -= 0.2;
                buildOpenList(cur_eps);
            }
            ret = improvePath(cur_eps, current_best, 
                              found_goal, new_plan_time);
            expanded_paths_.poses.clear();
            // this can be done better
            double tmp;
            retracePath(goal_state_, tmp);
            if (tmp < exec_time) {
                exec_time = publishPlan(goal_state_);
            }
        }
        reward = timing_info.update(exec_time);
        rewardSum += reward;
        eps_bound = getBoundedEps(cur_eps);

        /////////////////////////////////////////////////////
        fp = fopen("/home/grogan/planner_output.txt", "a");
        fprintf(fp, "%d, %.3f, %.3f, %.3f, %.3f, %.1f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %d, %d, %d, %d\n",
                call_number_,
                timing_info.plan_time_,
                timing_info.exec_time_,
                reward,
                rewardSum,
                cur_eps,
                eps_bound,
                eps_bound - prev_eps_bound,
                goal_state_->g,
                goal_state_->g - prev_g,
                start_state_->h,
                open_list_->cur_size,
                open_list_->cur_size - prev_open_size,
                incons_list_->cur_size,
                incons_list_->cur_size - prev_incons_size,
                all_expanded_.size(),
                (int)(all_expanded_.size()) - prev_expanded_size);
        fclose(fp);
        prev_expanded_size = (int)all_expanded_.size();
        prev_eps_bound = eps_bound;
        prev_g = goal_state_->g;
        /////////////////////////////////////////////////////
    }

    // Write the start/goal sequence plus times for posterity
    //FILE *fp;
    fp = fopen("/home/grogan/start_goal_sequence.txt", "a");
    fprintf(fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            next_start_.pose.position.x, next_start_.pose.position.y, 
            tf::getYaw(next_start_.pose.orientation),
            goal.pose.position.x, goal.pose.position.y, 
            tf::getYaw(goal.pose.orientation),
            timing_info.plan_time_, timing_info.exec_time_);
    fclose(fp);

    // done with planning by here
    
    //if (found_goal) {
    //    current_plan_ = retracePath(goal_state_);
    //} else if (current_best != NULL) {
    //    current_plan_ = retracePath(current_best);
    //}


    ros::Duration offset = ros::Time::now() - current_plan_[0].header.stamp;
    for (int i=0; i<current_plan_.size(); i++) {
        //current_plan_[i].header.frame_id = costmap_->getGlobalFrameID();
        current_plan_[i].header.stamp += offset;
        //ROS_INFO("t = %.4f", poses[i].header.stamp.toSec());
    }

    //path = current_plan_;
    std::vector<geometry_msgs::PoseStamped> p;
    path = p;
    next_start_ = goal;
    return false;

    if (!path.empty()) {
        return true;
    } else {
        return false;
    }
}
/**/
/*
#define FIXED 6.0
bool LatticePlanner::getPathFixedPolicy(geometry_msgs::PoseStamped start,
                             geometry_msgs::PoseStamped goal,
                     std::vector<geometry_msgs::PoseStamped> &path) {
    ROS_INFO("LatticePlanner::getPath");
    //call_number_++;
    // clear containers
    reset();
    // new containers
    open_list_ = new Heap;
    incons_list_ = new List;

    TimingInfo timing_info;
    // set heuristic inside clock

    if (next_start_.pose.position.x == goal.pose.position.x &&
        next_start_.pose.position.y == goal.pose.position.y) {
        ROS_INFO("ignore");
        return true;
    }

    goal_state_ = setGoalState(goal);

    if (!env_->setPlanningParams(goal_state_->cell_i)) {
        ROS_ERROR("heuristic calculation failed");
        return false;
    }
    
    //start_state_ = setStartState(start);
    start_state_ = setStartState(next_start_);

    if (start_state_ == NULL || goal_state_ == NULL) {
        ROS_INFO("Bad start or goal state");
        return false;
    }

    // Set start state cost
    start_state_->g = 0.0;

    // insert start state into open list
    double key = start_state_->g + eps_*start_state_->h;
    open_list_->insertElement(start_state_, key);

    call_number_++;

    State *current_best = NULL;
    bool found_goal = false;
    double cur_eps = eps_;

    // main loop
    search_iteration_ = 1;
    int ret = improvePath(cur_eps, current_best, 
            found_goal, (double)clock()/CLOCKS_PER_SEC+planning_timeout_);
    expanded_paths_.poses.clear();

    if (ret == -1) {
        ROS_WARN("bad");
        return false;
    }
    if (ret == 1 || !found_goal) {
        ROS_WARN("timed out on first search");
        return false;
    }

    double exec_time = publishPlan(goal_state_);
    timing_info.update(exec_time);
    double reward = 0.0;
    double rewardSum = reward;
    double eps_bound = getBoundedEps(cur_eps);
    printStuff(timing_info, cur_eps, eps_bound, reward);

    double new_plan_time = (double)clock()/CLOCKS_PER_SEC + FIXED;
    while (cur_eps > 1+ROUNDED_ZERO && 
            (double)clock()/CLOCKS_PER_SEC < new_plan_time) {

        search_iteration_++;
        cur_eps -= 0.2;
        buildOpenList(cur_eps);
        ret = improvePath(cur_eps, current_best, 
                found_goal, new_plan_time);
        double tmp;
        retracePath(goal_state_, tmp);
        if (tmp < exec_time) {
            exec_time = publishPlan(goal_state_);
        }

        reward = timing_info.update(exec_time);
        rewardSum += reward;
        eps_bound = getBoundedEps(cur_eps);


        printStuff(timing_info, cur_eps, eps_bound, rewardSum);
        ROS_INFO("-------------------");
    }
    //QT_->updateTerminal(q, qaction, reward);

    // Write the start/goal sequence plus times for posterity
    FILE *fp;
    fp = fopen("/home/grogan/start_goal_sequence.txt", "a");
    fprintf(fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            next_start_.pose.position.x, next_start_.pose.position.y, 
            tf::getYaw(next_start_.pose.orientation),
            goal.pose.position.x, goal.pose.position.y, 
            tf::getYaw(goal.pose.orientation),
            timing_info.plan_time_, timing_info.exec_time_);
    fclose(fp);

    // done with planning by here
    
    //if (found_goal) {
    //    current_plan_ = retracePath(goal_state_);
    //} else if (current_best != NULL) {
    //    current_plan_ = retracePath(current_best);
    //}


    ros::Duration offset = ros::Time::now() - current_plan_[0].header.stamp;
    for (int i=0; i<current_plan_.size(); i++) {
        //current_plan_[i].header.frame_id = costmap_->getGlobalFrameID();
        current_plan_[i].header.stamp += offset;
        //ROS_INFO("t = %.4f", poses[i].header.stamp.toSec());
    }

    //path = current_plan_;
    std::vector<geometry_msgs::PoseStamped> p;
    path = p;
    next_start_ = goal;
    return false;

    if (!path.empty()) {
        return true;
    } else {
        return false;
    }
}
*/

/********************************************************************
 * Get lower bound on cost of a solution for suboptimality bound (ARA*)
 * I absolutely hate the way I'm doing this. And it might be buggy
 ********************************************************************/
double LatticePlanner::getBoundedEps(double cur_eps) {
    //ROS_INFO("AStarLattice::getLowerBound");
    double lower_bound = DBL_MAX;
    State *state = NULL;
    //ROS_INFO("open_list size: %d", open_list_.size());
    for (int i=1; i<=open_list_->cur_size; i++) {
        state = open_list_->heap[i].heapState;
        double tmp = state->g + state->h;
        if (tmp < lower_bound) {
            lower_bound = tmp;
        }
    }

    state = incons_list_->getFirst();
    while (state != NULL) {  
        double tmp = state->g + state->h;
        if (tmp < lower_bound) {
            lower_bound = tmp;
        }
        state = incons_list_->getNext(state);
    }
    return std::min(cur_eps, goal_state_->g/lower_bound);
}

/********************************************************************
 * Move states from the inconsistent list to the open list and 
 * reevaluate their f values for the new heuristic inflation
 ********************************************************************/
void LatticePlanner::buildOpenList(double cur_eps) {
    //ROS_INFO("LatticePlanner::buildOpenList");
    // move from incons list
    while (incons_list_->firstElement != NULL) {
        State *state = incons_list_->firstElement->listState;
        double key = state->g + cur_eps*state->h;
        open_list_->insertElement(state, key);
        incons_list_->deleteElement(state);
    }
    // reevaluate keys
    for (int i=1; i<=open_list_->cur_size; i++) {
        State *state = open_list_->heap[i].heapState;
        open_list_->heap[i].key = state->g + cur_eps*state->h;
    }
    open_list_->makeHeap();
}

bool LatticePlanner::initStartAndGoal(geometry_msgs::PoseStamped start,
                                      geometry_msgs::PoseStamped goal) {
    // set up start and goal states
    /*
    start_pose_.setX(start.pose.position.x);
    start_pose_.setY(start.pose.position.y);
    start_pose_.setTheta(tf::getYaw(start.pose.orientation));
    goal_pose_.setX(goal.pose.position.x);
    goal_pose_.setY(goal.pose.position.y);
    goal_pose_.setTheta(tf::getYaw(goal.pose.orientation));
    */

    // Allocate new start and goal states. 
    start_state_ = setStartState(start);
    goal_state_ = setGoalState(goal);

    if (start_state_ == NULL || goal_state_ == NULL) {
        ROS_INFO("Bad start or goal state");
        return false;
    }

    // Set start state cost
    start_state_->g = 0.0;
    //start_state_->h = env_->getHeuristicCost(start_state_->cell_i);

    // insert start state into open list
    double key = start_state_->g + eps_*start_state_->h;
    open_list_->insertElement(start_state_, key);

    return true;
}

/********************************************************************
 * Does what it says
 ********************************************************************/
State* LatticePlanner::setStartState(geometry_msgs::PoseStamped start) {
    State new_state;
    DiscreteCell cell_i = env_->discretizePose(start, 0);
    new_state.stateID = Hasher::getHash(new_state.cell_i);
    new_state.cell_i = cell_i;
    new_state.parent = NULL;
    return getState(new_state);
}

/********************************************************************
 * Does what it says
 ********************************************************************/
State* LatticePlanner::setGoalState(geometry_msgs::PoseStamped goal) {
    State new_state;
    DiscreteCell cell_i = env_->discretizePose(goal, 0);
    new_state.stateID = Hasher::getHash(new_state.cell_i);
    new_state.cell_i = cell_i;
    new_state.parent = NULL;
    return getState(new_state);
}

/********************************************************************
 * Retrace path from state using parent pointers. Return a Path
 ********************************************************************/
std::vector<geometry_msgs::PoseStamped> LatticePlanner::retracePath(
                                        State *state, double &time) {
    //ROS_INFO("LatticePlanner::retracePath");
    std::vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;
    std::string fixed_frame = costmap_->getGlobalFrameID();

    //ROS_INFO("1");
    ros::Time t = ros::Time::now() + ros::Duration(1000);
    int count = 0;
    if (start_state_->parent != NULL) {
        ROS_ERROR("woah there");
    }
    //int dx = 0;
    //int dy = 0;
    //double cost = 0.0;
    while (state->parent != NULL) {
        //ROS_INFO("action size = %d", state->action.intmPoints.size());
        for (int i=state->action.intmPoints.size()-1; i>0; i--) {
            pose = env_->getPose(state->action.intmPoints[i]);
            pose.header.frame_id = fixed_frame;
            pose.header.stamp = t;
            poses.push_back(pose);
            //ROS_INFO("pose.x = %.4f", pose.pose.position.x);
            //ROS_INFO("pose.y = %.4f", pose.pose.position.y);
            //ROS_INFO("pose.theta = %.4f", tf::getYaw(pose.pose.orientation));

            t -= ros::Duration(
                    state->action.intmTimes[i]-state->action.intmTimes[i-1]);

            //ROS_INFO("t = %.4f", t.toSec());
            //geometry_msgs::PoseStamped pubpose = pose;
            //pubpose.header.stamp = ros::Time::now();
            //waypoint_pub_.publish(pubpose);
            //std::string input;
            //std::getline(std::cin, input);
        }
        //dx += state->action.dx;
        //dy += state->action.dy;
        //cost += env_->getActionCost(state->action);
        /*
        if (state->parent == NULL) {
            pose = env_->getPose(state->action.intmPoints[0]);
            pose.header.frame_id = fixed_frame;
            pose.header.stamp = t;
            poses.push_back(pose);
        } else {
            state = state->parent;
        }
        */
        //ROS_INFO("1.1");
        //std::string input;
        //std::getline(std::cin, input);
        state = state->parent;
        //ROS_INFO("1.2");
        //ROS_INFO("count = %d", count++);
    }

    /*
    int max = std::max(dx, dy);
    int min = std::min(dx, dy);
    double dist = ((max-min) + sqrt(2)*min);
    ROS_INFO("path distance %f", dist);
    ROS_INFO("path cost %f", cost);
    */

    //ROS_INFO("2");
    if (poses.empty()) {
        ROS_WARN("empty path");
        return poses;
    }

    std::reverse(poses.begin(), poses.end());

    //ROS_INFO("3");
    ros::Duration offset = ros::Time::now() - poses[0].header.stamp;
    for (int i=0; i<poses.size(); i++) {
        //current_plan_[i].header.frame_id = costmap_->getGlobalFrameID();
        poses[i].header.stamp += offset;
        //ROS_INFO("t = %.4f", poses[i].header.stamp.toSec());
    }
    time = (poses[poses.size()-1].header.stamp.toSec() -
                poses[0].header.stamp.toSec());
 
    //ROS_INFO("4");
    return poses;
}

/********************************************************************
 * Publish the ole plannerooski
 * return projected execution time
 ********************************************************************/
double LatticePlanner::publishPlan(State *state) {
    ROS_INFO("LatticePlanner::publishPlan");
    double ret;
    current_plan_ = retracePath(state, ret);
    nav_msgs::Path path;

    /*
    for (int i=0; i<current_plan_.size(); i++) {
        ROS_INFO("[x y theta t] = %.3f %.3f %.3f %.3f",
                 current_plan_[i].pose.position.x,
                 current_plan_[i].pose.position.y,
                 tf::getYaw(current_plan_[i].pose.orientation),
                 current_plan_[i].header.stamp.toSec());
    }
    */

    if (!current_plan_.empty()) {
        path.header.stamp = current_plan_[0].header.stamp;
        path.header.frame_id = current_plan_[0].header.frame_id;
        path.poses = current_plan_;
        ROS_INFO("publishing plan of size %d", path.poses.size());
        path_pub_.publish(path);

        return ret;
    } else {
        ROS_WARN("current plan is empty");
        return -1;
    }
}

void LatticePlanner::printStuff(TimingInfo ti, double cur_eps, 
        double bounded_eps, double reward) {
    //ROS_INFO("-------------------------");
    ROS_INFO("goal_state cost: %f", goal_state_->g);
    ROS_INFO("planning time: %f", ti.plan_time_);
    ROS_INFO("estimated exec time: %f", ti.exec_time_);
    ROS_INFO("reward: %f", reward);
    //ROS_INFO("wall planning time %f", ti.plan_time_wall_);
    //ROS_INFO("wall improve time %f", ti.improve_time_wall_);
    ROS_INFO("current eps: %f", cur_eps);
    ROS_INFO("bounded eps: %f", bounded_eps);
    //ROS_INFO("eps_bound %f", eps_bound);
    //ROS_INFO("Start heuristic = %f/%f", 
    //        heuristic_calc_->getHeuristic(start_state_, goal_pose_),
    //        heuristic_calc_->max_val);
    //ROS_INFO("-------------------------");


    FILE *fp;
    fp = fopen("/home/grogan/planner_output.txt", "a");
    fprintf(fp, "%d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.1f, %.3f, %.3f, %.3f, %d, %d, %d\n",
            call_number_,
            ti.plan_time_,
            ti.improve_time_,
            reward,
            ti.plan_time_wall_,
            ti.improve_time_wall_,
            ti.exec_time_,
            cur_eps,
            bounded_eps,
            goal_state_->g,
            start_state_->h,
            //heuristic_calc_->max_val,
            open_list_->cur_size,
            incons_list_->cur_size,
            all_expanded_.size());
    fclose(fp);
} 


} /* namespace lattice_planner */

