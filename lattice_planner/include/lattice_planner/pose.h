#ifndef POSE_H
#define POSE_H

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#define ROUNDED_ZERO 1e-6

namespace lattice_planner {

class Pose {
    // theta in radians
    public:
        Pose() : x_(0.0), y_(0.0) {
            setTheta(0.0);
        }

        Pose(double x, double y, double theta) : x_(x), y_(y) {
            setTheta(theta);
        }

        void setX(double x) {
            this->x_ = x;
        }

        void setY(double y) {
            this->y_ = y;
        }

        void setTheta(double theta) {
            this->theta_ = theta;
            while (this->theta_ < 0 - ROUNDED_ZERO) {
                this->theta_ += 2*M_PI;
            }
            while (this->theta_ >= 2*M_PI + ROUNDED_ZERO) {
                this->theta_ -= 2*M_PI;
            }
        }

        double getX() {
            return x_;
        }

        double getY() {
            return y_;
        }

        double getTheta() {
            return theta_;
        }

        geometry_msgs::Pose getGeometryPose() {
            geometry_msgs::Pose pose;
            pose.position.x = x_;
            pose.position.y = y_;
            pose.position.z = 0.0;
            tf::quaternionTFToMsg(
                    tf::createQuaternionFromYaw(theta_),
                    pose.orientation);
            return pose;
        }

        geometry_msgs::PoseStamped getStampedPose(
                std::string frame_id, ros::Time time) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose = getGeometryPose();
            pose_stamped.header.frame_id = frame_id;
            pose_stamped.header.stamp = time;
            return pose_stamped;
        }

        double getDistance(Pose &p) {
            return hypot(x_ - p.getX(), y_ - p.getY());
        }

        double getRotDistance(Pose &p) {
            return std::min(fabs(theta_ - p.getTheta()),
                    fabs(fabs(theta_ - p.getTheta()) - 2*M_PI));
        }

    private:
        double x_;
        double y_;
        double theta_;
};

} /* namespace lattice_planner */

#endif /* POSE_H */
