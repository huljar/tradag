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
 * @brief Very simple class for doing benchmarks and performance testing.
 */
class TraDaG::Benchmarker
{
public:
    /// Clock type used for the benchmarks.
    typedef std::chrono::high_resolution_clock Clock;
    /// Time point type used for the benchmarks.
    typedef std::chrono::time_point<Clock> TimePoint;
    /// Duration type used for the benchmarks.
    typedef Clock::duration Duration;

    /// Get a reference to this Singleton (will be created on first call).
    static Benchmarker& getSingleton();
    /// Get a pointer to this Singleton (will be created on first call).
    static Benchmarker* getSingletonPtr();

    /// Get a checkpoint at the current point in time for a new measurement ID (current time for this ID will start at 0 now).
    virtual size_t checkpoint(const std::string& description);
    /// Get a checkpoint at the current point in time for an existing measurement ID (current time since creation or last reset of this ID).
    virtual void checkpoint(size_t idx, const std::string& description, bool resetTimer = false, bool suppressCurrentTime = false);

protected:
    std::vector<TimePoint> mStart;

private:
    static Benchmarker* msSingleton;

    Benchmarker();
};

#endif // BENCHMARKER_H
