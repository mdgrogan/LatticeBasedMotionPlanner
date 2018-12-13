#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <lattice_planner/dijkstra.h>


#define ROUNDED_ZERO 1e-6


namespace lattice_planner {

/********************************************************************
 * Discrete cell 
 ********************************************************************/
struct DiscreteCell {
    public:
        DiscreteCell() {
            x_i = 0;
            y_i = 0;
            theta_i = 0;
            vel_i = 0;
        }

        DiscreteCell(int x_, int y_, int theta_, int vel_) {
            x_i = x_;
            y_i = y_;
            theta_i = theta_;
            vel_i = vel_;
        }

        bool operator==(const DiscreteCell cell) const {
            return (x_i==cell.x_i && y_i==cell.y_i && 
                    theta_i==cell.theta_i && vel_i==cell.vel_i);
        }

        bool operator<(const DiscreteCell cell) const {
            return (x_i<cell.x_i || 
                    (x_i==cell.x_i && y_i<cell.y_i) || 
                    (y_i==cell.y_i && theta_i<cell.theta_i) ||
                    (theta_i==cell.theta_i && vel_i<cell.vel_i));
        }

        int x_i;
        int y_i;
        int theta_i;
        int vel_i;
};

struct Hasher {
    std::size_t operator()(const DiscreteCell &index) const {
        return getHash(index);
    }

    static void setLimits(int n_x, int n_y,
                          int n_theta, int n_vel) {
        num_x_ = n_x;
        num_y_ = n_y;
        num_theta_ = n_theta;
        num_vel_ = n_vel;
    }

    static size_t getHash(const DiscreteCell &index) {
        size_t hash = index.x_i
            + index.y_i * Hasher::num_x_
            + index.theta_i * Hasher::num_x_ * Hasher::num_y_
            + index.vel_i * Hasher::num_x_ * Hasher::num_y_ * Hasher::num_theta_;
        return hash;
    }

    static int num_x_;
    static int num_y_;
    static int num_theta_;
    static int num_vel_;
};

/********************************************************************
 * Real point coordinates. See no reason to include theta/vel 
 ********************************************************************/
struct Point {
    public:
        Point() {
            x = 0;
            y = 0;
            theta = 0;
        }

        Point(double x_, double y_, double theta_) {
            x = x_;
            y = y_;
            theta = theta_;
        }
        
        double x;
        double y;
        double theta;
};

/********************************************************************
 * Motion primitives read from .mprim file
 ********************************************************************/
struct MotionPrimitive {
    int start_theta_i;
    int start_vel_i;
    DiscreteCell end_cell_i;
    double exec_time;
    std::vector<Point> intmPoints;
    std::vector<double> intmTimes;
};

/********************************************************************
 * Feasible action
 ********************************************************************/
struct Action {
    int start_theta_i;
    int start_vel_i;
    int end_theta_i;
    int end_vel_i;
    int dx;
    int dy;
    double exec_time;
    std::vector<Point> intmPoints;
    std::vector<double> intmTimes;
    //std::vector<DiscreteCell> intmCells;
};

struct CostFactors {
    double time_cost;
    double step_cost;
    double direction_cost;
    double environment_cost;
    double rotation_cost;
    int lethal_cost;
};

class Environment {
    public:
        Environment(costmap_2d::Costmap2DROS *costmap,
                    CostFactors cost_factors, 
                    const std::string primFile);
        ~Environment();

        bool setPlanningParams(DiscreteCell goal_cell);
        void initPublisher(std::string topic_name, int publish_scale = 100);
        void publishHeuristic();

        void getNextCells(DiscreteCell current_cell,
                           std::vector<DiscreteCell> &cells,
                           std::vector<Action> &actions);
        double getActionCost(Action action);
        double getHeuristicCost(DiscreteCell cell);

        DiscreteCell discretizePose(geometry_msgs::PoseStamped pose, int vel);
        //geometry_msgs::PoseStamped getPose(DiscreteCell cell);
        geometry_msgs::PoseStamped getPose(Point point);
        void mapToWorld(unsigned int mx, unsigned int my,
                        double &wx, double &wy);
        bool worldToMap(double wx, double wy,
                        unsigned int &mx, unsigned int &my);

    private:
        bool readMotionPrimitives(const std::string primFile);
        bool calculateHeuristic();

        costmap_2d::Costmap2DROS *costmap_;
        double resolution_;
        double origin_x_;
        double origin_y_;
        int nx_;
        int ny_;

        int nangles_;
        //std::vector<double> angles_;
        int nvels_;
        double turn_in_place_time_;

        DiscreteCell goal_cell_;
        std::vector<MotionPrimitive> mprims_;
        CostFactors cost_factors_;

        DijkstraExpansion *dijkstra_;
        float *heuristics_;
        bool pub_initialized_;
        int publish_scale_;
        ros::Publisher heuristics_pub_;
};

} /* namespace lattice_planner */

#endif /* ENVIRONMENT_H */
