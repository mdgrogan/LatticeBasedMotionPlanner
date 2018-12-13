#include <lattice_planner/timing_info.h>

namespace lattice_planner {

TimingInfo::TimingInfo() {
    start_time_ = clock();
    plan_time_ = 0.0;
    prev_plan_time_ = 0.0;
    improve_time_ = 0.0;

    start_time_wall_ = getWallTime();
    plan_time_wall_ = 0.0;
    prev_plan_time_wall_ = 0.0;
    improve_time_wall_ = 0.0;

    exec_time_ = 0.0;
    prev_exec_time_ = 0.0;
}

double TimingInfo::update(double exec_time) {
    plan_time_ = (double)(clock() - start_time_)/((double)CLOCKS_PER_SEC);
    plan_time_wall_ = getWallTime() - start_time_wall_;
    improve_time_ = (double)(clock() - start_time_)/((double)CLOCKS_PER_SEC) - prev_plan_time_;
    improve_time_wall_ = getWallTime() - start_time_wall_ - prev_plan_time_wall_;
    prev_plan_time_ = plan_time_;
    prev_plan_time_wall_ = plan_time_wall_;
    exec_time_ = exec_time;
    
    double ret = improve_time_ + exec_time_ - prev_exec_time_;
    prev_exec_time_ = exec_time_;

    return ret;
}   

double TimingInfo::getWallTime() {
    struct timeval time;
    gettimeofday(&time, NULL);
    return (double)time.tv_sec + (double)time.tv_usec * 0.000001;
}

} /* namespace lattice_planner */
