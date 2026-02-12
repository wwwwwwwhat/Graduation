#ifndef THESIS_RCSPP_NORMALIZER_H
#define THESIS_RCSPP_NORMALIZER_H

#include "graph/Graph.h"
#include "graph/Roadmap.h"
#include "algorithm/Pulse.h"
#include "algorithm/ERCA.h"
#include <vector>

namespace rcspp {

// Per-iteration result for detailed analysis
struct NormIterDetail {
    int iteration = 0;
    bool foundSolution = false;       // did ERCA* on normalized graph find a path?
    bool feasibleOnOriginal = false;  // is that path feasible on original constraints?
    long cost = -1;                   // primary cost on original graph (-1 if no solution)
    std::vector<double> resourceRatios; // resource usage ratios (usage[r]/limit[r])
    double iterTimeMs = 0.0;         // time for this iteration alone (ms)
    double cumTimeMs = 0.0;          // cumulative time up to this iteration (ms)
};

struct NormResult {
    bool feasible = false;        // found a feasible solution?
    CostVector upperBound;        // upper bound cost vector
    std::vector<long> path;       // corresponding path
    int iterations = 0;           // actual iterations used
    double rtTotal = 0.0;         // total time for normalization + Pulse iterations
    std::vector<NormIterDetail> iterDetails; // per-iteration details
};

// Resource normalization module.
// Implements the user's pruning strategy:
//   1. Normalize all resources using quintile-based coefficients
//   2. Run Pulse on normalized single-resource graph
//   3. Validate solution, adjust weights, repeat up to maxIter times
//   4. Return upper bound for ERCA* pruning
class Normalizer {
public:
    Normalizer();
    ~Normalizer();

    void SetGraph(Graph* g);
    void SetResourceLimits(const std::vector<long>& limits);

    // Compute upper bound using normalization + Pulse iteration (DFS-based)
    // source, target: start and end nodes
    // maxIter: max number of weight adjustment iterations (default 5)
    NormResult ComputeUpperBound(long source, long target, int maxIter = 5);

    // Compute upper bound using normalization + ERCA* (more reliable)
    // Uses ERCA* on the normalized single-resource graph instead of Pulse DFS
    NormResult ComputeUpperBoundERCA(long source, long target, int maxIter = 5);

    // Same as above but records per-iteration details for analysis
    NormResult ComputeUpperBoundERCA_Detailed(long source, long target, int maxIter = 5);

private:
    // Compute normalization coefficients for each resource dimension
    // Uses quintile-based averaging (20%, 40%, 60%, 80% percentiles)
    std::vector<double> computeCoefficients();

    // Build a normalized single-resource graph from the original graph
    // weights: per-resource weights (initially 1.0)
    // coefficients: normalization coefficients
    // Returns a Roadmap with costDim=2 (primary cost, normalized resource)
    Roadmap buildNormalizedGraph(const std::vector<double>& weights,
                                const std::vector<double>& coefficients);

    // Compute normalized resource limit
    long computeNormalizedLimit(const std::vector<double>& weights,
                                const std::vector<double>& coefficients);

    // Validate a path against original multi-resource constraints
    // Returns: for each resource, the usage ratio (usage/limit)
    std::vector<double> validatePath(const std::vector<long>& path);

    Graph* graph_ = nullptr;
    std::vector<long> resourceLimits_;
};

} // namespace rcspp

#endif // THESIS_RCSPP_NORMALIZER_H
