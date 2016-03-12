/************************************************************//**
 * @file
 *
 * @brief Benchmarker class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef BENCHMARKER_H
#define BENCHMARKER_H

#include <chrono>
#include <string>
#include <vector>

namespace TraDaG {
    class Benchmarker;
}

/**
 * @brief Class for doing benchmarks and performance testing.
 */
class TraDaG::Benchmarker
{
public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;
    typedef Clock::duration Duration;

    static Benchmarker& getSingleton();
    static Benchmarker* getSingletonPtr();

    virtual size_t checkpoint(const std::string& description);
    virtual void checkpoint(size_t idx, const std::string& description, bool resetTimer = false, bool suppressCurrentTime = false);

protected:
    std::vector<TimePoint> mStart;

private:
    static Benchmarker* msSingleton;

    Benchmarker();
};

#endif // BENCHMARKER_H
