#include <cstdio>
#include <pseudo_lattice_planner/Qtable.h>

namespace QStuff {


QTable::QTable(double alpha, double gamma) {
    srand(time(0));
    alpha_ = alpha;
    gamma_ = gamma;
}

int QTable::getAction(QState q) {
    // eps greedy
    int x = rand()%10;
    if (x<2) {
        return rand()%ACTION_SIZE;
    } else {
        return argminOverA(q);
    }
    return 0;
} 
    

double QTable::getTableVal(QState q, int a) {
    //return table[q.eps_i][q.t_cpu_i][q.t_exec_i][q.open_size_i][a];
    return table[q.eps_i][q.t_cpu_i][q.open_size_i][a];
}

void QTable::setTableVal(QState q, int a, double val) {
    //table[q.eps_i][q.t_cpu_i][q.t_exec_i][q.open_size_i][a] = val;
    table[q.eps_i][q.t_cpu_i][q.open_size_i][a] = val;
    table_update_count[q.eps_i][q.t_cpu_i][q.open_size_i][a]++;
}

void QTable::update(QState q, QState qnew, int action, double R) {
    double update= (1-alpha_)*getTableVal(q, action) + 
                       alpha_*(R + gamma_*minOverA(qnew));
    setTableVal(q, action, update);
}

void QTable::updateTerminal(QState q, int action, double R) {
    //double update= (1-alpha_)*getTableVal(q, action) + 
    //                   alpha_*R;
    //setTableVal(q, action, update);
    table_update_count[q.eps_i][q.t_cpu_i][q.open_size_i][action]++;
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
    out2.open("/home/grogan/Qtable_count.txt");
    for (int i=0; i<EPS_SIZE; i++) {
        //for (int j=0; j<FEATURE_SIZE; j++) {
            for (int k=0; k<FEATURE_SIZE; k++) {
                for (int l=0; l<FEATURE_SIZE; l++) {
                    for (int m=0; m<ACTION_SIZE; m++) {
                        out1 << i << "," << k << "," << l << "," <<
                            //m << "," << table[i][j][k][l][m] << std::endl;
                            m << "," << table[i][k][l][m] << std::endl;
                        out2 << i << "," << k << "," << l << "," << m << "," 
                            << table_update_count[i][k][l][m] << std::endl;
                    }
                }
            }
        //}
    }
    out1.close(); 
    out2.close(); 
}

void QTable::loadTable(std::string filename) {
    std::ifstream in;
    in.open(filename);
    if (!in) {
        return;
    }
    std::string line;
    size_t pos = std::string::npos;
    while (std::getline(in, line)) {
        std::vector<double> vals;
        while ((pos=line.find_first_of(",")) != std::string::npos) {
            std::string tmp = line.substr(0, pos);
            vals.push_back(std::stod(tmp));
            line.erase(0, pos+1);
        }
        //std::cout<<vals[5]<<std::endl;
        // dumb
        if (fabs(vals[4]) < 1e-6)
            vals[4] = 0.0;
        table[(int)vals[0]][(int)vals[1]][(int)vals[2]][(int)vals[3]] = vals[4];
    }
    in.close();

    in.open("/home/grogan/Qtable_count.txt");
    if (!in) {
        return;
    }
    pos = std::string::npos;
    while (std::getline(in, line)) {
        std::vector<int> vals;
        while ((pos=line.find_first_of(",")) != std::string::npos) {
            std::string tmp = line.substr(0, pos);
            vals.push_back(std::stoi(tmp));
            line.erase(0, pos+1);
            if (line.length() == 1) {
                vals.push_back(std::stoi(line));
            }
        }
        table_update_count[vals[0]][vals[1]][vals[2]][vals[3]] = vals[4];
    }
    in.close();
}

} /* namespace QStuff */
