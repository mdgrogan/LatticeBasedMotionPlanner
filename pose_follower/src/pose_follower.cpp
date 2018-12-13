#include <pose_follower/pose_follower.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(pose_follower::PoseFollower, nav_core::BaseLocalPlanner)

namespace pose_follower {

  //check for zero with rounding effects
#define ROUNDED_ZERO 1e-4

PoseFollower::PoseFollower() :
    initialized_(false),
    goal_reached_(false),
    costmap_ros_(NULL) {}

void PoseFollower::initialize(std::string name, tf::TransformListener *tf,
                              costmap_2d::Costmap2DROS *costmap_ros) {

    ROS_INFO("PoseFollower::initialize");
    if (!initialized_) {
        //create ros node handles
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~/" + name);

        //initialize ros publisher and service client
        current_waypoint_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("waypoint", 1);
        replan_client_ = nh.serviceClient<std_srvs::Empty>("move_base/replan");

        //collect transform listener and the ros costmap from the nav stack
        tfl_ = tf;
        costmap_ros_ = costmap_ros;

        //get the planner preferences from ros params
        private_nh.param("allow_backwards", allow_backwards_, true);
        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.05);
        private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);
        private_nh.param("max_vel_x", max_vel_x_, 6.5);
        private_nh.param("max_vel_phi", max_vel_phi_, 1.57);
        private_nh.param("k_rho", k_rho_, 2.8);
        private_nh.param("k_alpha", k_alpha_, 8.0);
        ROS_INFO("k_alpha_ = %f", k_alpha_);
        private_nh.param("k_beta", k_beta_, -1.8);

        K_trans_ = 2.0;
        K_rot_ = 2.0;
        tol_trans_ = 0.02;
        tol_rot_ = 0.04;

        current_waypoint_ = 0;

        initialized_ = true;
    }
}

double PoseFollower::headingDiff(double x, double y, 
                                 double pt_x, double pt_y, double heading) {
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);
    double perp_dot = (v1_x * v2_y) - (v1_y * v2_x);
    double dot = (v1_x * v2_x) + (v1_y * v2_y);
    double vector_angle = atan2(perp_dot, dot);
    return -1.0 * vector_angle;
    //return vector_angle;
}

geometry_msgs::Twist PoseFollower::diff2D(const tf::Stamped<tf::Pose> &pose1,
                                          const tf::Stamped<tf::Pose> &pose2) {

    double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(),
                                  pose2.getOrigin().x(), pose2.getOrigin().y(),
                                  tf::getYaw(pose2.getRotation()));
    tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
    tf::Quaternion rot = pose2.getRotation() * rot_diff;
    tf::Pose new_pose = pose1;
    new_pose.setRotation(rot);
    tf::Pose diff = pose2.inverse() * new_pose;
    double time_diff= (pose2.stamp_ - pose1.stamp_).toSec();
    ROS_INFO(" time diff %f", time_diff);
    geometry_msgs::Twist res;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());
    //res.linear.x = diff.getOrigin().x()/time_diff;
    //res.linear.y = diff.getOrigin().y()/time_diff;
    //res.angular.z = tf::getYaw(diff.getRotation())/time_diff;
    return res;
}


bool PoseFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    //ROS_INFO("PoseFollower::computeVelocityCommands");
    geometry_msgs::Twist zero_vel;
    tf::Stamped<tf::Pose> robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_ERROR("can't get robot pose");
        geometry_msgs::Twist ret;
        cmd_vel = ret;
        return false;
    }

    //check if we are already within the goal tolerance
    tf::Pose goal;
    tf::poseMsgToTF(goal_.pose, goal);

    //calculate the transformation between the robot and the goal pose
    tf::Transform robot_in_goal = goal.inverse() * robot_pose;

    //calculate the euclidian distance between the current robot pose and the goal
    double goal_distance =
        hypot(robot_in_goal.getOrigin().getX(), robot_in_goal.getOrigin().getY());

    //calculate the angular distance between the current robot pose and the goal
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(robot_pose.getRotation(), quat);
    double angular_goal_distance =
        angles::shortest_angular_distance(tf::getYaw(goal_.pose.orientation),
                                          tf::getYaw(quat));

    //check if robot is within the goal distance
    if(goal_distance < tol_trans_ && fabs(angular_goal_distance) < tol_rot_)
    {
      goal_reached_ = true;
      cmd_vel = zero_vel;
      return true;
    }
    // not strictly accurate...
    if (current_waypoint_ >= global_plan_.size()) {
        goal_reached_ = true;
        cmd_vel = zero_vel;
        return true;
    }

    ros::Time time = ros::Time::now();
    while(current_waypoint_ < global_plan_.size()-1 && 
          global_plan_[current_waypoint_].header.stamp <= time) {
        current_waypoint_++;
    }

    tf::Pose waypnt;
    tf::poseMsgToTF(global_plan_[current_waypoint_].pose, waypnt);
    current_waypoint_pub_.publish(global_plan_[current_waypoint_]);

    //calculate transformation between the robot position and the desired position
    tf::Pose robot_in_wpnt = waypnt.inverse() * robot_pose;

    double delta_x = robot_in_wpnt.getOrigin().getX();
    double delta_y = robot_in_wpnt.getOrigin().getY();
    double phi = tf::getYaw(robot_in_wpnt.getRotation());
    
    //It is not possible to set the velocities v_x, v_y and omega directly to
    //eliminate the pose error between the current position of the robot and
    //the goal because of the differential constraints.
    //Therefore, we use rho, alpha and beta as control values. The
    //controller design follows the feedback motion controller for differential
    //drive robots suggested in the book "Introduction to Autonomous Mobile Robots"
    //by Siegwart et al. (MIT Press 2011).
    double rho = hypot(delta_x, delta_y);
    double alpha = atan2(-1*delta_y, -1*delta_x) - phi;

    //if delta_x and delta_y are both very small, the angle is not properly
    //defined. hence, only take orientation difference phi into account.
    //also, if the desired forward vel is zero, only turn in place
    /*
    if(fabs(rho) < 0.001) {
        //only turn in place
        ROS_INFO("turn in place");
        alpha = 0;
        rho = 0;
    }
    */

    //if the distance between the robot and the desired waypoint is too big,
    //we should trigger replanning
    if(rho > 0.4)
    {
        ROS_INFO("distance %f to path is too big, triggering replanning", rho);
        std_srvs::Empty srv;
        //call the planner with the replanning request
        replan_client_.call(srv);
        //set the velocity to zero and wait for the new plan
        cmd_vel = zero_vel;
        replanning_requested_ = true;
        return false;
    }

    double beta = -1* alpha - phi;

    if (allow_backwards_) {
        if (fabs(alpha) > M_PI_2) {
            rho *= -1;
            if (alpha > 0)
                alpha -= M_PI;
            else
                alpha += M_PI;

            if (beta > 0)
                beta -= M_PI;
            else
                beta += M_PI;
        }
    }

    int tmp = current_waypoint_;
    /*
    ROS_INFO("global_plan_[%d] x y theta = %f %f %f",
            tmp, global_plan_[tmp].pose.position.x,
            global_plan_[tmp].pose.position.y,
            tf::getYaw(global_plan_[tmp].pose.orientation));

    //controll values
    ROS_INFO("delta_x: %f, delta_y: %f, phi: %f", delta_x, delta_y, phi);
    ROS_INFO("rho: %f, alpha: %f, beta: %f", rho, alpha, beta);
    */

    //calculate control velocities with the control values and the
    //controller gains. Always make sure that the velocity is within the bounds
    cmd_vel.linear.x = k_rho_ * rho;
    cmd_vel.angular.z = k_alpha_ * alpha + k_beta_ * beta;

    //ROS_INFO("cmd_vel.linear.x: %f, cmd_vel.angular.z: %f",
    //        cmd_vel.linear.x, cmd_vel.angular.z);

    if(cmd_vel.linear.x > max_vel_x_)
        cmd_vel.linear.x = max_vel_x_;

    else if(cmd_vel.linear.x < -max_vel_x_)
        cmd_vel.linear.x = 0;

    if(cmd_vel.angular.z > max_vel_phi_)
        cmd_vel.angular.z = max_vel_phi_;

    else if(cmd_vel.angular.z < -max_vel_phi_)
        cmd_vel.angular.z = -max_vel_phi_;
    
    /*
    ros::Time time = ros::Time::now();
    while(current_waypoint_ < global_plan_.size() && 
          global_plan_[current_waypoint_].header.stamp <= time) {
        current_waypoint_++;
    }

    ROS_INFO("waypoint = %d", current_waypoint_);
    tf::Stamped<tf::Pose> target_pose;
    tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);

    ROS_INFO("current pose %f %f %f", 
            robot_pose.getOrigin().x(),
            robot_pose.getOrigin().y(),
            tf::getYaw(robot_pose.getRotation()));
    ROS_INFO("target pose %f %f %f", 
            target_pose.getOrigin().x(),
            target_pose.getOrigin().y(),
            tf::getYaw(target_pose.getRotation()));

    geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
    ROS_INFO("dx dy dtheta = %.5f %.5f %.5f", diff.linear.x, diff.linear.y,
                                              diff.angular.z);

    geometry_msgs::Twist vel = limitTwist(diff);
    cmd_vel = vel;

    if (current_waypoint_ == global_plan_.size()-1) {
        goal_reached_ = true;
    }
    */

    /*
    while (fabs(diff.linear.x) <= tol_trans_ &&
           fabs(diff.angular.z) <= tol_rot_) {
        if (current_waypoint_ < global_plan_.size()-1) {
            current_waypoint_++;
            tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
            diff = diff2D(target_pose, robot_pose);
        }
        else {
            ROS_INFO("In goal position");
            goal_reached_ = true;
            break;
        }
    }
    */

    return true;
}
    
geometry_msgs::Twist PoseFollower::limitTwist(const geometry_msgs::Twist &twist) {
    geometry_msgs::Twist res = twist;
    res.linear.y = 0;
    if (res.linear.x > max_vel_x_) {
        res.linear.x = max_vel_x_;
    }
    if (fabs(res.angular.z) > max_vel_phi_) {
        res.angular.z = max_vel_phi_ * sign(res.angular.z);
    }
    return res;
}

bool PoseFollower::isGoalReached() {
    return goal_reached_;
}

bool PoseFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>&
                           global_plan) {
    ROS_INFO("PoseFollower::setPlan");
    current_waypoint_ = 0;
    goal_reached_ = false;
    /*
    if(!transformGlobalPlan(*tfl_, global_plan, *costmap_ros_, 
                            costmap_ros_->getGlobalFrameID(), global_plan_)) {
        ROS_ERROR("Could not transform the global plan to the frame of the controller");
        return false;
    }
    */
    global_plan_ = global_plan;

    // I'm doing this is global planner as well...........
    ros::Time t = ros::Time::now();
    ros::Duration offset = t - global_plan_[0].header.stamp;
    for (int i=0; i<global_plan_.size(); i++) {
        global_plan_[i].header.stamp += offset;
    }
    //make sure planner had been initialized
    if(!initialized_) {
        ROS_ERROR("path executer: planner has not been initialized");
        return false;
    }

    goal_ = global_plan.back();
    /*
    for (int i=0; i<global_plan_.size(); i++) {
        ROS_INFO("global_plan_[%d] x y theta = %f %f %f",
                i, global_plan_[i].pose.position.x,
                global_plan_[i].pose.position.y,
                tf::getYaw(global_plan_[i].pose.orientation));
    }
    */

    return true;
}


bool PoseFollower::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
              const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
                    std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try{
        if (global_plan.empty())
        {
            ROS_ERROR("Recieved plan with zero length");
            return false;
        }

        tf::StampedTransform transform;
        tf.lookupTransform(global_frame, ros::Time(), 
                plan_pose.header.frame_id, plan_pose.header.stamp, 
                plan_pose.header.frame_id, transform);

        tf::Stamped<tf::Pose> tf_pose;
        geometry_msgs::PoseStamped newer_pose;
        // now we'll transform until points are outside of our distance threshold
            for(unsigned int i = 0; i < global_plan.size(); ++i){
                const geometry_msgs::PoseStamped& pose = global_plan[i];
                poseStampedMsgToTF(pose, tf_pose);
                tf_pose.setData(transform * tf_pose);
                tf_pose.stamp_ = transform.stamp_;
                tf_pose.frame_id_ = global_frame;
                poseStampedTFToMsg(tf_pose, newer_pose);

                transformed_plan.push_back(newer_pose);
            }
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        if (!global_plan.empty())
            ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

        return false;
    }

    return true;
}

} //namespace path_executer
