#include "algorithm/Normalizer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace rcspp {

Normalizer::Normalizer() {}
Normalizer::~Normalizer() {}

void Normalizer::SetGraph(Graph* g) {
    graph_ = g;
}

void Normalizer::SetResourceLimits(const std::vector<long>& limits) {
    resourceLimits_ = limits;
}

std::vector<double> Normalizer::computeCoefficients() {
    size_t cdim = graph_->GetCostDim();
    // Resource dimensions are indices 1..cdim-1
    size_t numRes = cdim - 1;
    std::vector<double> coeffs(numRes, 1.0);

    // Collect resource costs from all edges
    std::vector<std::vector<long>> resCosts(numRes);

    auto nodes = graph_->GetNodes();
    for (auto v : nodes) {
        auto succs = graph_->GetSuccs(v);
        for (auto u : succs) {
            CostVector c = graph_->GetCost(v, u);
            for (size_t r = 0; r < numRes; r++) {
                resCosts[r].push_back(c[r + 1]);
            }
        }
    }

    // For each resource, compute quintile-based coefficient
    for (size_t r = 0; r < numRes; r++) {
        auto& vals = resCosts[r];
        if (vals.empty()) {
            coeffs[r] = 1.0;
            continue;
        }
        std::sort(vals.begin(), vals.end());
        size_t n = vals.size();

        // 5-quantile points: 20%, 40%, 60%, 80%
        double sum = 0;
        int count = 0;
        for (int q = 1; q <= 4; q++) {
            size_t idx = static_cast<size_t>(n * q / 5.0);
            if (idx >= n) idx = n - 1;
            sum += vals[idx];
            count++;
        }
        double avg = sum / count;
        coeffs[r] = (avg > 0) ? avg : 1.0;
    }

    return coeffs;
}

Roadmap Normalizer::buildNormalizedGraph(const std::vector<double>& weights,
                                          const std::vector<double>& coefficients) {
    Roadmap normGraph;
    normGraph.Init(graph_->GetNumberOfNodes(), 2); // dim 0: primary cost, dim 1: normalized resource

    auto nodes = graph_->GetNodes();
    for (auto v : nodes) {
        normGraph.AddNode(v);
    }

    for (auto v : nodes) {
        auto succs = graph_->GetSuccs(v);
        for (auto u : succs) {
            CostVector origCost = graph_->GetCost(v, u);

            CostVector normCost(0, 2);
            normCost[0] = origCost[0]; // keep primary cost

            // Normalized resource = weighted sum of (resource_i / coeff_i)
            double normRes = 0;
            for (size_t r = 0; r < coefficients.size(); r++) {
                normRes += weights[r] * (origCost[r + 1] / coefficients[r]);
            }
            normCost[1] = static_cast<long>(std::round(normRes * 100)); // scale to integer

            normGraph.AddEdge(v, u, normCost);
        }
    }

    return normGraph;
}

long Normalizer::computeNormalizedLimit(const std::vector<double>& weights,
                                         const std::vector<double>& coefficients) {
    double normLimit = 0;
    for (size_t r = 0; r < coefficients.size(); r++) {
        normLimit += weights[r] * (resourceLimits_[r] / coefficients[r]);
    }
    return static_cast<long>(std::round(normLimit * 100));
}

std::vector<double> Normalizer::validatePath(const std::vector<long>& path) {
    size_t numRes = graph_->GetCostDim() - 1;
    std::vector<long> usage(numRes, 0);

    for (size_t i = 0; i + 1 < path.size(); i++) {
        CostVector c = graph_->GetCost(path[i], path[i + 1]);
        for (size_t r = 0; r < numRes; r++) {
            usage[r] += c[r + 1];
        }
    }

    std::vector<double> ratios(numRes);
    for (size_t r = 0; r < numRes; r++) {
        ratios[r] = static_cast<double>(usage[r]) / resourceLimits_[r];
    }
    return ratios;
}

NormResult Normalizer::ComputeUpperBound(long source, long target, int maxIter) {
    auto tstart = std::chrono::steady_clock::now();
    NormResult result;

    // Step 1: Compute normalization coefficients
    std::vector<double> coeffs = computeCoefficients();
    size_t numRes = coeffs.size();

    // Initial weights: all 1.0
    std::vector<double> weights(numRes, 1.0);

    for (int iter = 0; iter < maxIter; iter++) {
        result.iterations = iter + 1;

        // Step 2: Build normalized graph and run Pulse
        Roadmap normGraph = buildNormalizedGraph(weights, coeffs);
        long normLimit = computeNormalizedLimit(weights, coeffs);

        Pulse pulse;
        pulse.SetGraph(&normGraph);
        pulse.SetResourceLimit(normLimit, 1);
        pulse.Search(source, target, 0.5); // 0.5s per iteration

        PulseResult pulseRes = pulse.GetResult();
        if (!pulseRes.feasible) {
            // Pulse didn't find a solution, try next iteration with adjusted weights
            // Increase all weights slightly
            for (size_t r = 0; r < numRes; r++) {
                weights[r] *= 1.5;
            }
            continue;
        }

        // Step 3: Validate against original constraints
        std::vector<double> ratios = validatePath(pulseRes.bestPath);

        bool allFeasible = true;
        for (size_t r = 0; r < numRes; r++) {
            if (ratios[r] > 1.0) {
                allFeasible = false;
            }
        }

        if (allFeasible) {
            // Found a feasible solution - use as upper bound
            result.feasible = true;
            result.path = pulseRes.bestPath;

            // Compute full cost vector on original graph
            result.upperBound = CostVector(0, graph_->GetCostDim());
            for (size_t i = 0; i + 1 < result.path.size(); i++) {
                result.upperBound += graph_->GetCost(result.path[i], result.path[i + 1]);
            }
            break;
        }

        // Step 3b: Adjust weights - double weights for exceeded resources,
        //          halve weights for under-utilized resources
        for (size_t r = 0; r < numRes; r++) {
            if (ratios[r] > 1.0) {
                weights[r] *= 2.0; // exceeded: increase importance
            } else if (ratios[r] < 0.5) {
                weights[r] *= 0.5; // under-utilized: decrease importance
            }
        }
    }

    auto tend = std::chrono::steady_clock::now();
    result.rtTotal = std::chrono::duration<double>(tend - tstart).count();
    return result;
}

NormResult Normalizer::ComputeUpperBoundERCA(long source, long target, int maxIter) {
    auto tstart = std::chrono::steady_clock::now();
    NormResult result;

    // Step 1: Compute normalization coefficients
    std::vector<double> coeffs = computeCoefficients();
    size_t numRes = coeffs.size();

    // Initial weights: all 1.0
    std::vector<double> weights(numRes, 1.0);

    for (int iter = 0; iter < maxIter; iter++) {
        result.iterations = iter + 1;

        // Step 2: Build normalized graph and run ERCA* (single resource)
        Roadmap normGraph = buildNormalizedGraph(weights, coeffs);
        long normLimit = computeNormalizedLimit(weights, coeffs);

        ERCA erca;
        erca.SetGraph(&normGraph);
        erca.SetResourceLimits({normLimit}); // single normalized resource
        erca.Search(source, target, 5.0); // 5s timeout

        SearchResult ercaRes = erca.GetResult();
        if (ercaRes.paths.empty()) {
            // No feasible solution found on normalized graph
            for (size_t r = 0; r < numRes; r++) {
                weights[r] *= 1.5;
            }
            continue;
        }

        // Get the path from ERCA* result
        std::vector<long> path = ercaRes.paths.begin()->second;

        // Step 3: Validate against original constraints
        std::vector<double> ratios = validatePath(path);

        bool allFeasible = true;
        for (size_t r = 0; r < numRes; r++) {
            if (ratios[r] > 1.0) {
                allFeasible = false;
            }
        }

        if (allFeasible) {
            // Found a feasible solution - use as upper bound
            result.feasible = true;
            result.path = path;

            // Compute full cost vector on original graph
            result.upperBound = CostVector(0, graph_->GetCostDim());
            for (size_t i = 0; i + 1 < result.path.size(); i++) {
                result.upperBound += graph_->GetCost(result.path[i], result.path[i + 1]);
            }
            break;
        }

        // Step 3b: Adjust weights
        for (size_t r = 0; r < numRes; r++) {
            if (ratios[r] > 1.0) {
                weights[r] *= 2.0;
            } else if (ratios[r] < 0.5) {
                weights[r] *= 0.5;
            }
        }
    }

    auto tend = std::chrono::steady_clock::now();
    result.rtTotal = std::chrono::duration<double>(tend - tstart).count();
    return result;
}

NormResult Normalizer::ComputeUpperBoundERCA_Detailed(long source, long target, int maxIter) {
    auto tstart = std::chrono::steady_clock::now();
    NormResult result;

    std::vector<double> coeffs = computeCoefficients();
    size_t numRes = coeffs.size();
    std::vector<double> weights(numRes, 1.0);

    for (int iter = 0; iter < maxIter; iter++) {
        auto iterStart = std::chrono::steady_clock::now();
        result.iterations = iter + 1;

        NormIterDetail detail;
        detail.iteration = iter + 1;

        Roadmap normGraph = buildNormalizedGraph(weights, coeffs);
        long normLimit = computeNormalizedLimit(weights, coeffs);

        ERCA erca;
        erca.SetGraph(&normGraph);
        erca.SetResourceLimits({normLimit});
        erca.Search(source, target, 5.0);

        SearchResult ercaRes = erca.GetResult();
        if (ercaRes.paths.empty()) {
            detail.foundSolution = false;
            for (size_t r = 0; r < numRes; r++) weights[r] *= 1.5;
            auto iterEnd = std::chrono::steady_clock::now();
            detail.iterTimeMs = std::chrono::duration<double>(iterEnd - iterStart).count() * 1000;
            detail.cumTimeMs = std::chrono::duration<double>(iterEnd - tstart).count() * 1000;
            result.iterDetails.push_back(detail);
            continue;
        }

        std::vector<long> path = ercaRes.paths.begin()->second;
        detail.foundSolution = true;

        // Compute cost on original graph
        CostVector fullCost(0, graph_->GetCostDim());
        for (size_t i = 0; i + 1 < path.size(); i++) {
            fullCost += graph_->GetCost(path[i], path[i + 1]);
        }
        detail.cost = fullCost[0];

        std::vector<double> ratios = validatePath(path);
        detail.resourceRatios = ratios;

        bool allFeasible = true;
        for (size_t r = 0; r < numRes; r++) {
            if (ratios[r] > 1.0) allFeasible = false;
        }
        detail.feasibleOnOriginal = allFeasible;

        auto iterEnd = std::chrono::steady_clock::now();
        detail.iterTimeMs = std::chrono::duration<double>(iterEnd - iterStart).count() * 1000;
        detail.cumTimeMs = std::chrono::duration<double>(iterEnd - tstart).count() * 1000;
        result.iterDetails.push_back(detail);

        if (allFeasible) {
            result.feasible = true;
            result.path = path;
            result.upperBound = fullCost;
            break;
        }

        for (size_t r = 0; r < numRes; r++) {
            if (ratios[r] > 1.0) weights[r] *= 2.0;
            else if (ratios[r] < 0.5) weights[r] *= 0.5;
        }
    }

    auto tend = std::chrono::steady_clock::now();
    result.rtTotal = std::chrono::duration<double>(tend - tstart).count();
    return result;
}

} // namespace rcspp
