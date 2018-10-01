#include <lattice_planner/planner_ros.h>

#include <pluginlib/class_list_macros.h>

/********************************************************************
 * Interface between move_base and the planner
 ********************************************************************/

namespace lattice_planner {

PLUGINLIB_EXPORT_CLASS(lattice_planner::LatticePlannerROS,
                       nav_core::BaseGlobalPlanner)


/********************************************************************
 * Default constructor
 ********************************************************************/
LatticePlannerROS::LatticePlannerROS() :
    initialized_(false) {}

/********************************************************************
 * Constructor
 ********************************************************************/
LatticePlannerROS::LatticePlannerROS(std::string name,
                               costmap_2d::Costmap2DROS *costmap,
                               std::string frame_id) :
    initialized_(false) {
        initialize(name, costmap, frame_id);
}

/********************************************************************
 ********************************************************************/
void LatticePlannerROS::initialize(std::string name,
                                costmap_2d::Costmap2DROS *costmap) {
    initialize(name, costmap, costmap->getGlobalFrameID());
}

/********************************************************************
 ********************************************************************/
void LatticePlannerROS::initialize(std::string name,
                                costmap_2d::Costmap2DROS *costmap,
                                std::string frame_id) {
    ROS_INFO("LatticePlannerROS::initialize");
    if (!initialized_) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~/" + name);

        make_plan_srv_ = private_nh.advertiseService("make_plan",
                                        &LatticePlannerROS::makePlanService,
                                        this);

        replan_service_ = nh.advertiseService("move_base/replan",
                                        &LatticePlannerROS::replanServiceCallback,
                                        this);

        //path_pub_ = private_nh.advertise<nav_msgs::Path>("path", 1);

        

        costmap_ = costmap;
        //outlineMap(costmap_->getCostmap(), costmap_2d::LETHAL_OBSTACLE);
        frame_id_ = frame_id;
        planner_ = new LatticePlanner(name, costmap_);
        replanning_requested_ = false;
        initialized_ = true;
    } else {
        ROS_WARN("This planner has already been initialized");
    }
}

/********************************************************************
 ********************************************************************/
bool LatticePlannerROS::replanServiceCallback(std_srvs::Empty::Request &req,
                                           std_srvs::Empty::Response &resp) {
    ROS_INFO("LatticePlannerROS::replanServiceCallback");
    ros::Duration(0.5).sleep(); //?
    replanning_requested_ = true;
    return true;
}

/********************************************************************
 ********************************************************************/
bool LatticePlannerROS::makePlanService(nav_msgs::GetPlan::Request &req,
                                     nav_msgs::GetPlan::Response &resp) {
    ROS_INFO("LatticePlannerROS::makePlanService");
    makePlan(req.start, req.goal, resp.plan.poses);
    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;
    return true;
}

/********************************************************************
 ********************************************************************/
bool LatticePlannerROS::makePlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan) {
    ROS_INFO("LatticePlannerROS::makePlan");
    try {

    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR("This planner has not yet been initialized");
        return false;
    }
    
    plan.clear();
    geometry_msgs::PoseStamped start_fixed_frame;
    geometry_msgs::PoseStamped goal_fixed_frame;
    tf::TransformListener tfl;
    try {
        ros::Duration timeout(0.5);
        tfl.waitForTransform(frame_id_,
                             start.header.frame_id,
                             start.header.stamp,
                             timeout);
        tfl.transformPose(frame_id_, start, start_fixed_frame);
        tfl.waitForTransform(frame_id_,
                             goal.header.frame_id,
                             goal.header.stamp,
                             timeout);
        tfl.transformPose(frame_id_, goal, goal_fixed_frame);
    } catch (tf::TransformException ex) {
        ROS_ERROR("lattice_planner: could not transform pose into fixed"
                  "frame: %s", ex.what());
    }
    start_fixed_frame.header.stamp = ros::Time::now();

    ros::spinOnce();

    goal_pose_ = goal_fixed_frame;


    bool planning_ok = planner_->getPath(start_fixed_frame,
                                         goal_fixed_frame,
                                         plan);

    if (!planning_ok) {
        ROS_INFO("planning not ok");
        plan.clear();
    }

    //ROS_INFO("plan start time = %f", plan.front().header.stamp.toSec());
    //ROS_INFO("plan end time = %f", plan.back().header.stamp.toSec());
    //for (int i=0; i<plan.size(); i++) {
    //    ROS_INFO("plan[%d].header.stamp = %f", i, plan[i].header.stamp.toSec());
    //}

    } catch (std::runtime_error re) {
        ROS_ERROR("%s", re);
    }

    return true;
}

/********************************************************************
 ********************************************************************/
void LatticePlannerROS::outlineMap(costmap_2d::Costmap2D *costmap,
                                       unsigned char value) {
    unsigned int nx = costmap->getSizeInCellsX();
    unsigned int ny = costmap->getSizeInCellsY();
    unsigned int x = 0;
    unsigned int y = 0;

    for (int i=0; i < nx; i++) {
        costmap->setCost(x, y, value);
        x++;
    }
    for (int i=0; i < ny; i++) {
        costmap->setCost(x, y, value);
        y++;
    }
    for (int i=nx-1; i >= 0; i--) {
        costmap->setCost(x, y, value);
        x--;
    }
    for (int i=ny-1; i >= 0; i--) {
        costmap->setCost(x, y, value);
        y--;
    }
}

} /* namespace lattice_planner */
