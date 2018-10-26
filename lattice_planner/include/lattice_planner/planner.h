#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <lattice_planner/pose.h>
#include <lattice_planner/velocity.h>
#include <lattice_planner/discrete_state.h>
#include <lattice_planner/state.h>
#include <lattice_planner/heuristics.h>
#include <lattice_planner/state_discretizer.h>
#include <lattice_planner/lattice_primitives.h>
#include <lattice_planner/hasher.h>
#include <lattice_planner/heap.h>
#include <lattice_planner/list.h>
#include <lattice_planner/cost_manager.h>
#include <lattice_planner/timing_info.h>
#include <lattice_planner/Path.h>

#include <lattice_planner/Qstate_discretizer.h>
#include <lattice_planner/Qtable.h>

#define SQRT2 1.414213562

namespace lattice_planner {

typedef std::unordered_map<DiscreteState, State*, Hasher> state_map;
typedef std::pair<DiscreteState, State*> state_pair;

class LatticePlanner {
    public:

        LatticePlanner(std::string name, costmap_2d::Costmap2DROS *costmap);

        ~LatticePlanner();

        bool getPath(geometry_msgs::PoseStamped start,
                     geometry_msgs::PoseStamped goal,
                     std::vector<geometry_msgs::PoseStamped> &path);

    private:

        void reset();
        void plannerTimeoutCallback(const ros::TimerEvent &event);
        void clearRobotFootprint();
        State *createState(State state);
        State *getState(State state);
        bool isValidTrajectory(std::vector<unsigned int> indices);
        std::vector<State> getNextStates(State *current);
        int improvePath(double cur_eps, State *&best, bool &found_goal, 
                         double end_time);
        double getBoundedEps(double cur_eps);
        void buildOpenList(double cur_eps);
        bool initStartAndGoal(geometry_msgs::PoseStamped start,
                              geometry_msgs::PoseStamped goal);
        State *setStartState();
        State *setGoalState();
        lattice_planner::Path retracePath(State *state);
        void publishExpanded(State *state);
        int publishPlan(State *state);
        void printStuff(TimingInfo ti, double cur_eps, double bounded_eps, double reward);

        double eps_; // inflation
        //properties of current path finding run
        int search_iteration_;  //number of improvePaths
        int call_number_;   // not really using for now
        Pose start_pose_;   // start pose
        State *start_state_;    //start state
        Pose goal_pose_;    // goal pose
        State *goal_state_;     //goal state
        lattice_planner::Path current_plan_;    //for visualizing 
        nav_msgs::Path expanded_paths_;     //for visualizing
        bool planning_timed_out_;   //time out flag for planning
        
        //auxiliary classes
        costmap_2d::Costmap2DROS *costmap_; 
        StateDiscretizer *discretizer_; 
        LatticePrims *primgen_; 
        Heuristics *heuristic_calc_; 
        CostManager *cost_calc_; 
        MotionConstraints motion_constraints_;
        CostFactors cost_factors_;

        //Q-learning stuff
        QStuff::QStateDiscretizer *QSD_;
        QStuff::QTable *QT_;

       
        //containers
        Heap *open_list_; // open list
        List *incons_list_; // inconsistent states
        state_map all_expanded_; // all previously expanded states
       
        //ros stuff
        ros::NodeHandle nh_; 
        ros::Publisher expanded_paths_pub_; 
        ros::Publisher vel_path_pub_; 
        ros::Publisher path_pub_;
        
        //debug things
        ros::Publisher current_node_pub_; 
        ros::Publisher new_node_pub_; 
        ros::Publisher all_expanded_pub_; 

        //planner preferences
        bool allow_unknown_; // unused for now and maybe ever
        double time_resolution_; // length of time slices
        double map_index_check_time_incr_; 
        double planning_timeout_; // not used yet
        bool use_dijkstra_; // Dijkstra not implemented
        bool publish_expanded_; //for visualizing
        bool debug_; //lots of info printed
        
};

} /* namespace lattice_planner */

#endif /* PLANNER_H */
