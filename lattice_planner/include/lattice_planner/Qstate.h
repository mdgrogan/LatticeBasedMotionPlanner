#ifndef QSTATE_H
#define QSTATE_H

#include <vector>

// having a dynamic number of features is tripping me up
// some I'm harcoding it for now
#define NUM_FEATURES 7
#define F0 5
#define F1 5
#define F2 5
#define F3 5
#define F4 5
#define F5 5
#define F6 5
#define ACTION_SIZE 2
#define ACTION_E 1


namespace QStuff {

class QState {
    public:
        //QState(double t_cpu, double t_exec, double eps, double open_size, double expanded_size);
        //bool isTerminal() {
        //    return (eps_i == EPS_SIZE-1);
        //}

        std::vector<int> features;
    //private:
        //discretize(double val, double mu, double sigma);
        //int t_cpu_i;
        //int t_exec_i;
        //int eps_i;
        //int open_size_i;
};

} /* namespace QStuff */

namespace TD {

#define NUM_DIMENSIONS 7
#define FEATURES_PER_DIMENSION 30

class QState {
    public:
        //QState(double t_cpu, double t_exec, double eps, double open_size, double expanded_size);
        //bool isTerminal() {
        //    return (eps_i == EPS_SIZE-1);
        //}

        std::vector<double> dimension;
    //private:
        //discretize(double val, double mu, double sigma);
        //int t_cpu_i;
        //int t_exec_i;
        //int eps_i;
        //int open_size_i;
};

} /* namespace TD */

#endif /* QSTATE_H */
