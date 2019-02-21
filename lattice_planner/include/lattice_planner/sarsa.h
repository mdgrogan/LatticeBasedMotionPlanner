#ifndef SARSA_H
#define SARSA_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <lattice_planner/Qstate.h>
//#include "Qstate.h"

namespace TD {

struct RBF {
        std::vector<double> means;
        std::vector<double> stds;
};

class Sarsa {
    public:
        Sarsa(int greediness, double lr, double discount, double lambda);
        QState quantize(std::vector<double> s);
        void presentFeatures(QState qs, std::vector<double> &features);
        double getAction(const std::vector<double> &features);
        double getWeightedValue(const std::vector<double> &features);
        void updateWeights(const double &Q, const double &Q_old, 
                           const std::vector<double> &features);
        void updateTrace(const std::vector<double> &features);
        void updateDelta(const double &R, const double &Q, const double &Q_new);
        void reset();

        void save(std::string filename);
        void load(std::string filename);

    private:
        double gaussian(double x, double mean, double stdev);
        double dot(const std::vector<double> &x, const std::vector<double> &y);

        int greediness_;
        double lr_;
        double discount_;
        double lambda_;
        double delta_;

        std::vector<RBF> dimension_;
        std::vector<double> w_ = std::vector<double>(NUM_DIMENSIONS*FEATURES_PER_DIMENSION, 0);
        std::vector<double> z_ = std::vector<double>(NUM_DIMENSIONS*FEATURES_PER_DIMENSION, 0);

};

} /* namespace SARSA */

#endif /* SARSA_H */
