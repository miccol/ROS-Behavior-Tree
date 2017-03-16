#ifndef TICK_ENGINE_H
#define TICK_ENGINE_H

//#include <boost/thread.hpp>


#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

class TickEngine
{
private:
    int value_;
    std::mutex mutex_;
    std::condition_variable condition_variable_;
    //    boost::mutex mutex_;
//    boost::condition_variable condition_variable_;
public:
    TickEngine(int initial_value);
    ~TickEngine();
    void Wait();
    void Tick();
};

#endif
