#ifndef QSTATE_DISCRETIZER_H
#define QSTATE_DISCRETIZER_H

#include <vector>
#include <boost/random/normal_distribution.hpp>
#include <boost/math/distributions/normal.hpp>
#include <lattice_planner/Qstate.h>


namespace QStuff {

class QStateDiscretizer {
    public:
        QStateDiscretizer(double mu_t_cpu, double sigma_t_cpu,
                          //double mu_t_exec, double sigma_t_exec,
                          double mu_open_size, double sigma_open_size,
                          double start_eps);
        QState discretize(double t_cpu, //double t_exec,
                          double open_size, double eps);
        double indexToAction(int i);

    private:
        void calcBins(boost::math::normal nd, 
                        std::vector<double> &bins,
                        int numBins);
        int discretize(double x, std::vector<double> bins);
        int discretize(double x);

        boost::math::normal nd_t_cpu_;
        //boost::math::normal nd_t_exec_;
        boost::math::normal nd_open_size_;
        std::vector<double> bins_t_cpu_;
        //std::vector<double> bins_t_exec_;
        std::vector<double> bins_open_size_;
        double start_eps_;
};

} /* namespace QStuff */

#endif /* QSTATE_DISCRETIZER_H */
