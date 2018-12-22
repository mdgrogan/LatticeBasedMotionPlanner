#ifndef QSTATE_H
#define QSTATE_H

#include <vector>

// having a dynamic number of features is tripping me up
// some I'm harcoding it for now
#define NUM_FEATURES 5
#define F0 4
#define F1 4
#define F2 4
#define F3 4
#define F4 4
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

#endif /* QSTATE_H */
