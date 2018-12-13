#ifndef PLANNER_ROS_H
#define PLANNER_ROS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pseudo_lattice_planner/planner.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

namespace pseudo_lattice_planner {

class PseudoLatticePlannerROS : public nav_core::BaseGlobalPlanner {
    public:

        PseudoLatticePlannerROS();

        PseudoLatticePlannerROS(std::string name, 
                       costmap_2d::Costmap2DROS *costmap,
                       std::string frame_id);

        void initialize(std::string name,
                        costmap_2d::Costmap2DROS *costmap);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

        bool makePlanService(nav_msgs::GetPlan::Request &req,
                             nav_msgs::GetPlan::Response &resp);


    protected:

        void initialize(std::string name,
                        costmap_2d::Costmap2DROS *costmap,
                        std::string frame_id);
        
        void outlineMap(costmap_2d::Costmap2D *costmap, unsigned char val);

        bool replanServiceCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &resp);

        ros::ServiceServer make_plan_srv_;
        ros::ServiceServer replan_service_;
        ros::Publisher path_pub_;

        costmap_2d::Costmap2DROS *costmap_;
        LatticePlanner *planner_;
        geometry_msgs::PoseStamped goal_pose_;
        std::string frame_id_;
        bool replanning_requested_;
        boost::mutex mutex_;
        bool initialized_;
};

} /* namespace pseudo_lattice_planner */

#endif /* PLANNER_ROS_H */
