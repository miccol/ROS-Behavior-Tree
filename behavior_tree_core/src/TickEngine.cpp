#include <TickEngine.h>

TickEngine::TickEngine(int initial_value)
{
    value_ = initial_value;
}

TickEngine::~TickEngine() {}

void TickEngine::wait()
{
    // Lock acquire (need a unique lock for the condition variable usage)
    boost::unique_lock<boost::mutex> UniqueLock(mutex_);

    // If the state is 0 then we have to wait for a signal
    if (value_ == 0)
        condition_variable_.wait(UniqueLock);

    // Once here we decrement the state
    value_--;
}

void TickEngine::tick()
{
    // Lock acquire
    boost::lock_guard<boost::mutex> LockGuard(mutex_);

    // State increment
    value_++;

    // Notification
    condition_variable_.notify_all();
}
