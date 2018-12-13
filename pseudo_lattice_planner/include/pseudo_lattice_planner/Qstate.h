#ifndef QSTATE_H
#define QSTATE_H

#define START_EPS 3.0
#define EPS_SIZE 6
#define FEATURE_SIZE 5
// hack job: 10 actions in the sense of commit times and 1 for execution, so 11
#define ACTION_SIZE 7
#define ACTION_E 6


namespace QStuff {

class QState {
    public:
        //QState(double t_cpu, double t_exec, double eps, double open_size, double expanded_size);
        bool isTerminal() {
            return (eps_i == EPS_SIZE-1);
        }

    //private:
        //discretize(double val, double mu, double sigma);
        int t_cpu_i;
        //int t_exec_i;
        int eps_i;
        int open_size_i;
};

} /* namespace QStuff */

#endif /* QSTATE_H */
