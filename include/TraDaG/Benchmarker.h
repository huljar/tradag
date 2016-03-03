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

    virtual void checkpoint(const std::string& description, bool resetTimer = false);

    bool isEnabled() const;
    void setEnabled(bool enabled);

protected:
    TimePoint mStart;

    bool mEnabled;

private:
    static Benchmarker* msSingleton;

    Benchmarker();
};

#endif // BENCHMARKER_H
