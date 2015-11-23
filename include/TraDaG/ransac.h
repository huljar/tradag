#ifndef RANSAC_H
#define RANSAC_H

#include <chrono>
#include <functional>
#include <vector>
#include <array>
#include <random>
#include <limits>
#include <cmath>

namespace TraDaG {
    template<class P, class M, size_t d> class Ransac;
}

template<class P, class M, size_t d>
class TraDaG::Ransac
{
public:
    typedef std::pair<M, std::vector<const P*>> result_type;

    Ransac(const std::function<M(const std::array<P, d>&)>& modelFunc, const std::function<float(const P&, const M&)>& evalFunc)
        : mModelFunc(modelFunc)
        , mEvalFunc(evalFunc)
        , mGenerator(std::chrono::system_clock::now().time_since_epoch().count())
    {
    }

    result_type operator() (const std::vector<P>& dataPoints) {
        // Initialize random number distribution
        std::uniform_int_distribution<size_t> distribution(0, dataPoints.size() - 1);

        unsigned int n = std::numeric_limits<unsigned int>::max(); // Number of samples (start with large value)
        float e = 0.9; // Outlier probability (start with large value)

        M bestModel; // Store best current model here
        float bestModelEval = std::numeric_limits<float>::max(); // Store evaluation result of best current model here

        std::vector<const P*> bestModelInliers; // Store inliers of the best current model here

        // Perform standard RANSAC with adaptive number of samples
        for(unsigned int i = 0; i < n; ++i) {
            // Sample d points
            std::array<P, d> sample;
            for(size_t j = 0; j < d; ++j) {
                sample[j] = dataPoints[distribution(mGenerator)];
            }

            // Compute model from sample
            M currentModel = mModelFunc(sample);

            // Evaluate current model
            float currentModelEval = 0;
            std::vector<const P*> currentModelInliers;
            for(typename std::vector<P>::const_iterator it = dataPoints.cbegin(); it != dataPoints.cend(); ++it) {
                float eval = mEvalFunc(*it, currentModel);
                currentModelEval += eval;
                if(eval < 1.0) currentModelInliers.push_back(&(*it));
            }

            // Compare current to best model
            if(currentModelEval < bestModelEval) {
                // Replace best model with current one
                bestModel = currentModel;
                bestModelEval = currentModelEval;
                bestModelInliers = currentModelInliers;

                // Recompute e and n
                e = (float)(dataPoints.size() - currentModelInliers.size()) / (float)dataPoints.size();
                n = std::round(std::log(1.0 - p) / std::log(1.0 - std::pow(1 - e, d)));
            }
        }

        return std::make_pair(bestModel, bestModelInliers);
    }

protected:
    // Function to compute the model from the smallest number of points possible
    std::function<M(const std::array<P, d>&)> mModelFunc;

    // Function to evaluate a model with a single point
    std::function<float(const P&, const M&)> mEvalFunc;

    // Random engine
    std::default_random_engine mGenerator;

    // Probability to sample a right tuple
    float p = 0.999;

};

#endif // RANSAC_H
