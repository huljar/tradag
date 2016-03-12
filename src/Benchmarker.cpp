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
{
}

size_t Benchmarker::checkpoint(const std::string& description) {
    size_t newIdx = mStart.size();
    std::cout << "Benchmark Checkpoint: " << newIdx << ": " << description << " @ 0 ms (creating new)" << std::endl;
    mStart.push_back(Clock::now());
    return newIdx;
}

void Benchmarker::checkpoint(size_t idx, const std::string& description, bool resetTimer, bool suppressCurrentTime) {
    Duration currentTime = Clock::now() - mStart[idx];
    std::cout << "Benchmark Checkpoint: " << idx << ": " << description << " @ ";
    if(resetTimer && suppressCurrentTime)
        std::cout << '0';
    else
        std::cout << currentTime.count() * 1000 * Duration::period::num / Duration::period::den;
    std::cout << " ms";
    if(resetTimer){
        std::cout << " (resetting timer)";
        mStart[idx] = Clock::now();
    }
    std::cout << std::endl;
}
