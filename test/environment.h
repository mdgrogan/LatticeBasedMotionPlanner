#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <cstdio>
#include <cstdlib>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>

//namespace lattice_planner {

/********************************************************************
 * Discrete cell 
 ********************************************************************/
class DiscreteCell {
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

/********************************************************************
 * Real point coordinates. See no reason to include theta/vel 
 ********************************************************************/
class Point {
    public:
        Point() {
            x = 0;
            y = 0;
        }

        Point(double x_, double y_) {
            x = x_;
            y = y_;
        }
        
        double x;
        double y;
};

/********************************************************************
 * Motion primitives read from .mprim file
 ********************************************************************/
struct MotionPrimitive {
    int start_theta_i;
    int start_vel_i;
    DiscreteCell end_cell_i;
    double exec_time;
    std::vector<Point> intermediatePoints;
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
    //double turning_radius;
    double exec_time;
    std::vector<Point> intermediatePoints;
    //std::vector<DiscreteCell> intermediateCells;
};

class Environment {
    public:
        Environment();
        bool initialize(char map_placeholder, const std::string primFile);
        bool setPlanningParams(DiscreteCell goal_cell);
        void getNextCells(DiscreteCell current_cell,
                           std::vector<DiscreteCell> &cells,
                           std::vector<Action> &actions);
        double getActionCost(Action action);
        double getHeuristicCost(DiscreteCell cell);

    private:
        bool readMotionPrimitives(const std::string primFile);
        bool calculateHeuristic();

        // costmap_
        DiscreteCell goal_cell_;
        std::vector<MotionPrimitive> mprims_;
        double resolution_;
};

//} /* namespace lattice_planner */

#endif /* ENVIRONMENT_H */
