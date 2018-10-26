#ifndef TIMING_INFO_H
#define TIMING_INFO_H

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

namespace lattice_planner {

class TimingInfo {
    public:
        TimingInfo(double time_resolution);
        double update(int plan_len);
        //double getPlanTime();
        //double getExecTime()
        double getWallTime();

    //private:
        
        clock_t start_time_;
        double plan_time_, prev_plan_time_;
        double improve_time_;

        double start_time_wall_;
        double plan_time_wall_, prev_plan_time_wall_;
        double improve_time_wall_;
        
        double exec_time_;
        double prev_exec_time_;

        double time_resolution_;
};

} /* namespace lattice_planner */

#endif /* TIMING_INFO_H */
