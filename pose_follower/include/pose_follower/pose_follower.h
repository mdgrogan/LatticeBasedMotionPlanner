#ifndef SIMPLE_CONTROLLER_HEADER_H
#define SIMPLE_CONTROLLER_HEADER_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>


namespace pose_follower {

class PoseFollower : public nav_core::BaseLocalPlanner {
    public:

        PoseFollower();

        void initialize(std::string name, tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

        bool isGoalReached();

    private:

        double sign(double n) {
            return n < 0.0 ? -1.0 : 1.0;
        }

        double headingDiff(double x, double y, double pt_x, double pt_y, double heading);
        geometry_msgs::Twist diff2D(const tf::Stamped<tf::Pose> &pose1, 
                                    const tf::Stamped<tf::Pose> &pose2);
        geometry_msgs::Twist limitTwist(const geometry_msgs::Twist &twist);

        bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
                      const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
                      std::vector<geometry_msgs::PoseStamped>& transformed_plan);

        std::vector<geometry_msgs::PoseStamped> global_plan_;
        costmap_2d::Costmap2DROS *costmap_ros_;
        double K_trans_, K_rot_, tol_trans_, tol_rot_;
        unsigned int current_waypoint_;


  // controller preferences
        double max_vel_x_; ///< maximum forward velocity
        double max_vel_phi_; ///< maximum rotation velocity
        double xy_goal_tolerance_; ///< euclidian goal distance tolerance
        double yaw_goal_tolerance_; ///< rotational goal distance tolerance
        bool allow_backwards_; ///< whether backwards motion is allowed for the controller

  //local planner status flags
        bool initialized_; ///< whether the planner is initialized
        bool goal_reached_; ///< whether the robot has reached the goal
        bool replanning_requested_; ///< whether replanning for the global path was requested

  //dynamically feasible, time dependent global plan
        std::vector<geometry_msgs::PoseStamped> global_plan_waypoints_; ///< global plan waypoints
        std::vector<geometry_msgs::Twist> global_plan_velocities_; ///< global plan velocities
        geometry_msgs::PoseStamped goal_; ///< navigation goal

  //controller gains:
        double k_rho_; ///< controller gain for euclidian distance
        double k_alpha_; ///< controller gain for heading difference to face path
        double k_beta_; ///< controller gain for desired orientation

  //ros related stuff

        ros::ServiceClient replan_client_; ///< service client to make a replanning request to the global planner
        tf::TransformListener* tfl_; ///< tf transform listener
  //dynamic_reconfigure::Server<path_executer::PathExecuterConfig> *dsrv_; ///< dynamic reconfigure server
        ros::Publisher current_waypoint_pub_; ///< ros publisher to visualize the currently scheduled waypoint

};


} //namespace path_executer

#endif
