/************************************************************//**
 * @file
 *
 * @brief Ransac class header file.
 *
 * @author Julian Harttung
 *//************************************************************/

#ifndef RANSAC_H
#define RANSAC_H

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <vector>

namespace TraDaG {
    template<class P, class M, size_t d> class Ransac;
}

/**
 * @brief Simple implementation of the RANSAC (<b>Ran</b>dom <b>Sa</b>mple <b>C</b>onsensus).
 * @tparam P The type of a single data point.
 * @tparam M The model type.
 * @tparam d The number of data points required to construct a model.
 *
 * This implementation of RANSAC aims to provide a very generic version of the algorithm,
 * with the user being able to define the important functions himself.
 */
template<class P, class M, size_t d>
class TraDaG::Ransac
{
public:
    /// Iterator pointing into a vector of data points.
    typedef typename std::vector<P>::const_iterator const_point_iterator;
    /// RANSAC return type, containing the model and a vector of inliers (the iterators point into the vector passed to \ref operator()() "operator()").
    typedef std::pair<M, std::vector<const_point_iterator>> result_type;

    /**
     * @brief Constructor
     * @param modelFunc Function that constructs a model from a fixed number of data points.
     * @param evalFunc Function that evaluates a data point with a model.
     *
     * @a modelFunc is supposed to construct and return a model from the least amount of data points
     * necessary (e.g. if you want to fit a line, this function would take 2 data points and the
     * constructed line would pass exactly through these 2 data points.
     *
     * @a evalFunc should classify points as inliers or outliers when given a specific model. This
     * classification can either be binary (e.g. by returning 0 for inliers and 1 for outliers) or
     * be combined with a penalty that increases with the distance of the point from the model. In
     * any case, the lower the returned value, the better the point fits the given model.
     */
    Ransac(const std::function<M(const std::array<P, d>&)>& modelFunc, const std::function<float(const P&, const M&)>& evalFunc)
        : mModelFunc(modelFunc)
        , mEvalFunc(evalFunc)
        , mGenerator(std::chrono::system_clock::now().time_since_epoch().count())
    {
    }

    /**
     * @brief Execute the RANSAC algorithm for the given data points.
     * @param dataPoints Vector of data points for which a model will be searched.
     * @return The best model found, together with the inlier data points.
     *
     * @remarks The inliers returned are iterators pointing into the vector passed
     * to this operator.
     */
    result_type operator() (const std::vector<P>& dataPoints) {
        // Check if there are enough data points
        if(dataPoints.size() < d)
            return std::make_pair(M(), std::vector<const_point_iterator>());

        // Initialize random number distribution
        std::uniform_int_distribution<size_t> distribution(0, dataPoints.size() - 1);

        unsigned int n = std::numeric_limits<unsigned int>::max(); // Number of samples (start with large value)
        float e = 0.9; // Outlier probability (start with large value)

        M bestModel; // Store best current model here
        float bestModelEval = std::numeric_limits<float>::infinity(); // Store evaluation result of best current model here

        std::vector<const_point_iterator> bestModelInliers; // Store inliers of the best current model here

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
            std::vector<const_point_iterator> currentModelInliers;
            for(const_point_iterator it = dataPoints.cbegin(); it != dataPoints.cend(); ++it) {
                float eval = mEvalFunc(*it, currentModel);
                currentModelEval += eval;
                if(eval < 1.0) currentModelInliers.push_back(it);
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
