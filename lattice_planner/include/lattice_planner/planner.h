#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <lattice_planner/state.h>
#include <lattice_planner/environment.h>
#include <lattice_planner/heap.h>
#include <lattice_planner/list.h>
#include <lattice_planner/timing_info.h>
#include <lattice_planner/Path.h>

#include <lattice_planner/Qtable.h>

#define SQRT2 1.414213562

namespace lattice_planner {

typedef std::unordered_map<DiscreteCell, State*, Hasher> state_map;
typedef std::pair<DiscreteCell, State*> state_pair;

class LatticePlanner {
    public:

        LatticePlanner(std::string name, costmap_2d::Costmap2DROS *costmap);

        ~LatticePlanner();

        bool getPath(geometry_msgs::PoseStamped start,
                     geometry_msgs::PoseStamped goal,
                     std::vector<geometry_msgs::PoseStamped> &path);

        bool getPathFixedPolicy(geometry_msgs::PoseStamped start,
                     geometry_msgs::PoseStamped goal,
                     std::vector<geometry_msgs::PoseStamped> &path);

    private:

        void reset();
        void plannerTimeoutCallback(const ros::TimerEvent &event);
        void clearRobotFootprint();
        State *createState(State state);
        State *getState(State state);
        std::vector<State> getNextStates(State *current);
        int improvePath(double cur_eps, State *&best, bool &found_goal, 
                         double end_time);
        double getBoundedEps(double cur_eps);
        void buildOpenList(double cur_eps);
        bool initStartAndGoal(geometry_msgs::PoseStamped start,
                              geometry_msgs::PoseStamped goal);
        State *setStartState(geometry_msgs::PoseStamped start);
        State *setGoalState(geometry_msgs::PoseStamped goal);
        std::vector<geometry_msgs::PoseStamped> retracePath(State *state, double &time);
        void publishExpanded(State *state);
        double publishPlan(State *state);
        void printStuff(TimingInfo ti, double cur_eps, double bounded_eps, double reward);

        double eps_; // inflation
        //properties of current path finding run
        int search_iteration_;  //number of improvePaths
        int call_number_;   // not really using for now
        State *start_state_;    //start state
        State *goal_state_;     //goal state
        std::vector<geometry_msgs::PoseStamped> current_plan_;    //for visualizing 
        nav_msgs::Path expanded_paths_;     //for visualizing
        bool planning_timed_out_;   //time out flag for planning
        
        //auxiliary classes
        costmap_2d::Costmap2DROS *costmap_; 
        Environment *env_;

        //Q-learning stuff
        //QStuff::QStateDiscretizer *QSD_;
        QStuff::QTable *QT_;

       
        //containers
        Heap *open_list_; // open list
        List *incons_list_; // inconsistent states
        state_map all_expanded_; // all previously expanded states
       
        //ros stuff
        ros::NodeHandle nh_; 
        ros::Publisher expanded_paths_pub_; 
        ros::Publisher path_pub_;
        ros::Publisher waypoint_pub_;
        
        //planner preferences
        double planning_timeout_; 
        bool publish_expanded_; //for visualizing
        bool debug_; //lots of info printed

        // for training/evaluation
        geometry_msgs::PoseStamped next_start_;
        
};

} /* namespace lattice_planner */

#endif /* PLANNER_H */
