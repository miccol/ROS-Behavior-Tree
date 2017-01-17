#ifndef TICK_ENGINE_H
#define TICK_ENGINE_H

#include <boost/thread.hpp>

class TickEngine
{
private:
    int value_;
    boost::mutex mutex_;
    boost::condition_variable condition_variable_;
public:
    TickEngine(int initial_value);
    ~TickEngine();
    void wait();
    void tick();
};

#endif
