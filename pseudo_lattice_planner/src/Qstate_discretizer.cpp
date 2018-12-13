#include <stdio.h>
#include <pseudo_lattice_planner/Qstate_discretizer.h>

namespace QStuff {

double QStateDiscretizer::indexToAction(int i) {
    if (i < 0 || i > ACTION_SIZE-1) {
        printf("QStateDiscretizer::indexToAction: bad index\n");
        return 0;
    }
    return (0.1*pow(10, i/2.5));
}

QStateDiscretizer::QStateDiscretizer(double mu_t_cpu, double sigma_t_cpu,
                          //double mu_t_exec, double sigma_t_exec,
                          double mu_open_size, double sigma_open_size,
                          double start_eps) {
    nd_t_cpu_ = boost::math::normal(mu_t_cpu, sigma_t_cpu);
    calcBins(nd_t_cpu_, bins_t_cpu_, FEATURE_SIZE);
    //nd_t_exec_ = boost::math::normal(mu_t_exec, sigma_t_exec);
    //calcBins(nd_t_exec_, bins_t_exec_, FEATURE_SIZE);
    nd_open_size_ = boost::math::normal(mu_open_size, sigma_open_size);
    calcBins(nd_open_size_, bins_open_size_, FEATURE_SIZE);
    //start_eps_ = start_eps;
}

void QStateDiscretizer::calcBins(boost::math::normal nd, 
                                 std::vector<double> &bins,
                                 int numBins) {
    for (int i=1; i<numBins; i++) {
        double bin = boost::math::quantile(nd, i*(1.0/numBins));
        bins.push_back(bin);
    }
}

int QStateDiscretizer::discretize(double x, std::vector<double> bins) {
    int ret = 0;
    int i = 0;
    while ((i < bins.size()) && (log10(x) > bins[i])) {
        ret = i++;
    }
    if (i == bins.size() && log10(x) > bins[i-1]) {
        ret = i;
    }
    return ret;
}
        
int QStateDiscretizer::discretize(double x) {
    // assuming 0.2 decrements
    return round(2.5*(START_EPS - x));
}

QState QStateDiscretizer::discretize(double t_cpu, //double t_exec,
                                     double open_size, double eps) {
    QState ret; 
    ret.t_cpu_i = discretize(t_cpu, bins_t_cpu_);
    //ret.t_exec_i = discretize(t_exec, bins_t_exec_);
    ret.open_size_i = discretize(open_size, bins_open_size_);
    ret.eps_i = discretize(eps);
    return ret;
}

} /* namespace QStuff */
