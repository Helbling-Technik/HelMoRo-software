#pragma once

#include <chrono>
#include <limits>

#define SECS_TO_USECS(seconds)          ((USecs)((seconds)*1e6))
#define USECS_TO_SECS(microseconds)     ((double)(microseconds)*1e-6)
#define ROSTIME_TO_USECS(rostime)       ((USecs) (1e6*rostime.sec) + (USecs) (rostime.nsec/1000))

class Timer {
  public:
    using usecs_t = uint16_t;

    // constructor & destructor
    explicit Timer(bool autostart = true) {
        if (autostart) {
            Reset();
        }
        else {
            init_ = false;
        }
    }

    // timer functions
    double ElapsedSeconds(void) {
        using namespace std::chrono;

        if (!init_) return std::numeric_limits<double>::max();

        return USECS_TO_SECS(duration_cast<microseconds>(system_clock::now() - timer_start_).count());
    }

    usecs_t ElapsedUSeconds(void) {
        using namespace std::chrono;

        if (!init_) return std::numeric_limits<usecs_t>::max();

        return (usecs_t) duration_cast<microseconds>(system_clock::now() - timer_start_).count();
    }

    void Reset(void) {
        using namespace std::chrono;

        timer_start_ = system_clock::now();
        init_ = true;
    }


    // public get functions
    bool IsInit(void) {return init_;};

private:
    // general timer variables
    volatile bool init_;
    std::chrono::system_clock::time_point timer_start_;
};