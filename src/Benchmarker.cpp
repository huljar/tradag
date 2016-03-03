#include <TraDaG/Benchmarker.h>

#include <iostream>

using namespace TraDaG;

Benchmarker* Benchmarker::msSingleton = nullptr;

Benchmarker& Benchmarker::getSingleton() {
    if(!msSingleton)
        msSingleton = new Benchmarker();
    return *msSingleton;
}

Benchmarker* Benchmarker::getSingletonPtr() {
    if(!msSingleton)
        msSingleton = new Benchmarker();
    return msSingleton;
}

Benchmarker::Benchmarker()
    : mEnabled(false)
{
}

void Benchmarker::checkpoint(const std::string& description, bool resetTimer) {
    if(mEnabled) {
        Duration currentTime = Clock::now() - mStart;
        std::cout << "Benchmark Checkpoint: " << description << " @ " << currentTime.count() * 1000 * Duration::period::num / Duration::period::den << "ms";
        if(resetTimer) {
            std::cout << " (resetting timer)";
            mStart = Clock::now();
        }
        std::cout << std::endl;
    }
}

bool Benchmarker::isEnabled() const {
    return mEnabled;
}

void Benchmarker::setEnabled(bool enabled) {
    mEnabled = enabled;
    mStart = Clock::now();
}
