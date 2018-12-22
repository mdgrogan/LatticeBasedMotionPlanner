#ifndef QTABLE_H
#define QTABLE_H

#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <lattice_planner/Qstate.h>

namespace QStuff {

// this all requires some consistency on feature indices
class Feature {
    public:
        int discretize(int input);

        double Flr_; // feature learning rate
        std::vector<double> means;
};

class QTable {
    public:
        QTable(int greediness, double lr, double discount);
        ~QTable();
        void update(QState q, QState qnew, int action, double R);
        void updateTerminal(QState q, int action, double R);
        double getTableVal(QState q, int a);
        int getAction(QState q);
        double indexToAction(int a);
        QState discretize(std::vector<double> input);
        void saveTable(std::string filename);
        void loadTable(std::string filename);
        void initFeature(int featureNum, double lr, std::vector<double> means);

    private:
        void setTableVal(QState q, int a, double val);
        double minOverA(QState q);
        int argminOverA(QState q);

        double Qlr_; // Q table learning rate
        double discount_; // discount
        int greediness_; // how willing to explore = 1 - g/100
        std::vector<Feature> features_;
        double table_[F0][F1][F2][F3][F4][ACTION_SIZE] = {0};
        //std::vector<std::vector<double>> table_;
        int table_update_count_[F0][F1][F2][F3][F4][ACTION_SIZE] = {0};
};

} /* namespace QStuff */

#endif /* QTABLE_H */
