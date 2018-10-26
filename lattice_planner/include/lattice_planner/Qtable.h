#ifndef QTABLE_H
#define QTABLE_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <lattice_planner/Qstate.h>

namespace QStuff {

class QTable {
    public:
        QTable(double alpha, double gamma);
        void update(QState q, QState qnew, int action, double R);
        void updateTerminal(QState q, int action, double R);
        double getTableVal(QState q, int a);
        int getAction(QState q);
        void saveTable(std::string filename);
        void loadTable(std::string filename);

    private:
        void setTableVal(QState q, int a, double val);
        double minOverA(QState q);
        int argminOverA(QState q);

        double alpha_;
        double gamma_;
        // 4 features, 1 action
        const int eps_col = 0;
        const int t_cpu_col = 1;
        //const int t_exec_col = 2;
        const int open_size_col = 3;
        const int action_col = 4;
        //double table[EPS_SIZE][FEATURE_SIZE][FEATURE_SIZE][FEATURE_SIZE][ACTION_SIZE] = {0};
        double table[EPS_SIZE][FEATURE_SIZE][FEATURE_SIZE][ACTION_SIZE] = {0};
        int table_update_count[EPS_SIZE][FEATURE_SIZE][FEATURE_SIZE][ACTION_SIZE] = {0};
};

} /* namespace QStuff */

#endif /* QTABLE_H */
