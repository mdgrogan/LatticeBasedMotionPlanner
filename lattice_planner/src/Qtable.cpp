#include <ros/ros.h>
#include <cstdio>
#include <lattice_planner/Qtable.h>

namespace QStuff {

QTable::~QTable() {
}

QTable::QTable(int greediness, double lr, double discount) {
    srand(time(0));
    features_.resize(NUM_FEATURES);
    //table_.resize(numFeatures + 1);
    //table_[numFeatures].resize(numActions);
    greediness_ = greediness;
    Qlr_ = lr;
    discount_ = discount;

    // hardcoding this in. Everything is about halfway to having variable
    // features, just not sure how to handle the table
    // planning time
    initFeature(0, 0, std::vector<double> {0.6, 1.8, 4.0, 6.0});
    // exec time
    initFeature(1, 0, std::vector<double> {12.0, 20.0, 30.0, 36.0});
    // delta open size
    initFeature(2, 0, std::vector<double> {20000, 50000, 120000, 200000});
    // delta inconsistent size
    initFeature(3, 0, std::vector<double> {-5000, 0, 15000, 25000});
    // delta epsilon bound
    initFeature(4, 0, std::vector<double> {0.0, -0.05, -0.15, -0.3});
    /*
    //time
    features_[0].means.push_back(0.1);
    features_[0].means.push_back(0.3);
    features_[0].means.push_back(0.8);
    features_[0].means.push_back(1.9);
    features_[0].means.push_back(5.4);
    features_[0].Flr_ = 0.01;
    table_[0].resize(features_[0].size());

    //expanded size
    features_[1].means.push_back(6000);
    features_[1].means.push_back(21000);
    features_[1].means.push_back(40000);
    features_[1].means.push_back(70000);
    features_[1].means.push_back(160000);
    features_[1].Flr_ = 0.01;
    table_[1].resize(features_[1].size());

    //heuristic bound
    features_[2].means.push_back(1);
    features_[2].means.push_back(1.2);
    features_[2].means.push_back(1.3);
    features_[2].means.push_back(1.6);
    features_[2].Flr_ = 0;
    table_[2].resize(features_[2].size());
    */


}

void QTable::initFeature(int featureNum, double lr, std::vector<double> means) {
    features_[featureNum].means = means;
    features_[featureNum].Flr_ = lr;
}

/*
int QTable::getAction(QState q) {
    // eps greedy
    int x = rand()%100;
    if (x>greediness_) { // 2 is exploratory val, should be variable
        ROS_INFO("exploring");
        return rand()%(ACTION_SIZE-1);
    } else {
        return argminOverA(q);
    }
} 

double QTable::indexToAction(int i) {
    if (i < 0 || i > ACTION_SIZE-1) {
        printf("QStateDiscretizer::indexToAction: bad index\n");
        return 0;
    }
    return (0.2*pow(10, i/3.5));
}
*/

int QTable::getAction(QState q) {
    // eps greedy
    int x = rand()%100;
    if (x>greediness_) { // 2 is exploratory val, should be variable
        ROS_INFO("exploring");
        return 0;
        //return rand()%ACTION_SIZE;
    } else {
        return argminOverA(q);
    }
} 
double QTable::indexToAction(int i) {
    if (i < 0 || i > ACTION_SIZE-1) {
        printf("QStateDiscretizer::indexToAction: bad index\n");
        return 0;
    }
    if (i == 0)
        return 0.2;
    //if (i == 1)
    //    return 4.2;
}
    
QState QTable::discretize(std::vector<double> input) {
    QState ret;
    for (int i=0; i<input.size(); i++) {
        // find closest mean
        double d = fabs(input[i] - features_[i].means[0]);
        int index = 0;
        for (int j=1; j<features_[i].means.size(); j++) {
            if (fabs(input[i] - features_[i].means[j]) < d) {
                d = fabs(input[i] - features_[i].means[j]);
                index = j;
            }
        }
        //update means
        features_[i].means[index] += features_[i].Flr_*
                                 (input[i] - features_[i].means[index]);
        ret.features.push_back(index);
    }
    return ret;
}

// how to genericize?
double QTable::getTableVal(QState q, int a) {
    //return table[q.eps_i][q.t_cpu_i][q.t_exec_i][q.open_size_i][a];
    return table_[q.features[0]][q.features[1]][q.features[2]][q.features[3]][q.features[4]][a];
}

void QTable::setTableVal(QState q, int a, double val) {
    //table[q.eps_i][q.t_cpu_i][q.t_exec_i][q.open_size_i][a] = val;
    if (table_[q.features[0]][q.features[1]][q.features[2]][q.features[3]][q.features[4]][a]==0) {
        ROS_INFO("\nNEW STATE ENCOUNTERED\n");
    }
    table_[q.features[0]][q.features[1]][q.features[2]][q.features[3]][q.features[4]][a] = val;
    //table_update_count_[q.features[0]][q.features[1]][q.features[2]][q.features[3]][q.features[4]][a]++;
}

void QTable::update(QState q, QState qnew, int action, double R) {
    double update = (1-Qlr_)*getTableVal(q, action) + 
                       Qlr_*(R + discount_*minOverA(qnew));
    setTableVal(q, action, update);
}

void QTable::updateTerminal(QState q, int a, double R) {
    //double update= (1-Qlr_)*getTableVal(q, a) + 
    //                   Qlr_*R;
    //setTableVal(q, a, update);
    //table_update_count_[q.features[0]][q.features[1]][q.features[2]][q.features[3]][q.features[4]][a]++;
}

double QTable::minOverA(QState q) {
    double min = 10e-9;
    for (int i=0; i<ACTION_SIZE; i++) {
        if (getTableVal(q, i) < min) {
            min = getTableVal(q, i);
        }
    }
    return min;
}

int QTable::argminOverA(QState q) {
    double min = 10e-9;
    int argmin = 0;
    for (int i=0; i<ACTION_SIZE; i++) {
        if (getTableVal(q, i) < min) {
            min = getTableVal(q, i);
            argmin = i;
        }
    }
    return argmin;
}

void QTable::saveTable(std::string filename) {
    std::ofstream out1, out2;
    out1.open(filename);
    for (int i=0; i<features_.size(); i++) {
        out1 << "feature:" << std::endl;
        for (int j=0; j<features_[i].means.size(); j++) {
            out1 << features_[i].means[j] << " ";
        }
        out1 << ":" << features_[i].Flr_ << std::endl;
    }
    
    out2.open("/home/grogan/Qtable_count.txt");
    // how to make arbitrary?
    out1 << "table:" << std::endl;
    for (int i=0; i<F0; i++) {
        for (int j=0; j<F1; j++) {
            for (int k=0; k<F2; k++) {
                for (int l=0; l<F3; l++) {
                    for (int m=0; m<F4; m++) {
                        for(int n=0; n<ACTION_SIZE; n++) {
                            out1 << i << " " << j << " " << k << " " <<
                                l << " " << m << " " << n << " " <<
                                table_[i][j][k][l][m][n] << std::endl;
                            //out2 << i << " " << j << " " << k << " " <<
                                //l << " " << m << " " <<
                                //table_update_count_[i][j][k][l][m][n] << std::endl;
                            //out2 << i << "," << k << "," << l << "," << m << "," 
                            //    << table_update_count[i][k][l][m] << std::endl;
                        }
                    }
                }
            }
        }
        //}
    }
    out1.close(); 
    //out2.close(); 
}

void QTable::loadTable(std::string filename) {
    std::ifstream in;
    in.open(filename);
    if (!in) {
        /*
        for (int i=0; i<F0; i++) {
            for (int j=0; j<F1; j++) {
                for (int k=0; k<F2; k++) {
                    for (int l=0; l<F3; l++) {
                        for (int m=0; m<ACTION_SIZE-1; m++) {
                            table_[i][j][k][l][m] = 0.0001;
                        }
                    }
                }
            }
        }
        */
        return;
    }
    int count = 0;
    std::string line;
    size_t pos = std::string::npos;
    while (std::getline(in, line)) {
        if (line == "feature:") {
            features_[count].means.clear();
            std::getline(in, line);
            std::stringstream ss(line);
            double m;
            while (ss>>m) {
                features_[count].means.push_back(m);
                if (ss.peek() == ' ') {
                    ss.ignore();
                }
                if (ss.peek() == ':') {
                    ss.ignore();
                    ss>>m;
                    features_[count].Flr_ = m;
                    break;
                }
            }
            printf("features %d\n", count);
            for (int i=0; i<features_[count].means.size(); i++) {
                printf("%f ", features_[count].means[i]);
            }
            printf(": %f\n", features_[count].Flr_);
            count++;
        }

        if (line == "table:") {
            while (std::getline(in, line)) {
                std::stringstream ss(line);
                std::vector<double> vals;
                double d;
                while (ss>>d) {
                    vals.push_back(d);
                    if (ss.peek() == ' ') {
                        ss.ignore();
                    }
                }
                //std::cout<<vals[5]<<std::endl;
                // dumb
                //if (fabs(vals[4]) < 1e-6)
                //    vals[4] = 0.0;

                table_[(int)vals[0]][(int)vals[1]][(int)vals[2]][(int)vals[3]][(int)vals[4]][(int)vals[5]] = vals[6];
                //if ((int)vals[4] == 5) {
                //    table_[(int)vals[0]][(int)vals[1]][(int)vals[2]][(int)vals[3]][(int)vals[4]] = 0;
                //}
            }
        }
    }
    in.close();

    /*
    in.open("/home/grogan/Qtable_count.txt");
    if (!in) {
        return;
    }
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::vector<int> vals;
        int d;
        while (ss>>d) {
            vals.push_back(d);
            if (ss.peek() == ' ') {
                ss.ignore();
            }
        }
        table_update_count_[vals[0]][vals[1]][vals[2]][vals[3]][vals[4]][vals[5]] = vals[6];
    }
    in.close();
    */
}

} /* namespace QStuff */
