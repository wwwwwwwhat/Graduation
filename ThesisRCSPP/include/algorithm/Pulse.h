#ifndef THESIS_RCSPP_PULSE_H
#define THESIS_RCSPP_PULSE_H

#include "graph/Graph.h"
#include "algorithm/Dijkstra.h"
#include <vector>
#include <unordered_set>
#include <chrono>

namespace rcspp {

struct PulseResult {
    bool feasible = false;       // found a feasible solution?
    long bestCost = -1;          // best primary cost found
    CostVector bestCostVec;      // full cost vector of best solution
    std::vector<long> bestPath;  // corresponding path
    long nExplored = 0;          // nodes explored
    double rtSearch = 0.0;       // search time
    bool timeout = false;
};

// Pulse algorithm: DFS with pruning for single-resource RCSPP.
// Fast for finding feasible solutions, used as pre-computation for ERCA*.
class Pulse {
public:
    Pulse();
    ~Pulse();

    void SetGraph(Graph* g);

    // Set the single resource limit (resource is at cost dimension resIndex)
    void SetResourceLimit(long limit, size_t resIndex = 1);

    // Initialize heuristic (Dijkstra on primary cost dimension 0)
    void InitHeuristic(long target);

    // Run DFS-based search with pruning
    int Search(long source, long target, double timeLimit);

    PulseResult GetResult() const;

private:
    void pulse(long node, long currentCost, long currentResource,
               std::vector<long>& currentPath, std::unordered_set<long>& visited);

    Graph* graph_ = nullptr;
    long resourceLimit_ = 0;
    size_t resIndex_ = 1;    // which cost dimension is the resource
    long source_ = -1;
    long target_ = -1;
    double timeLimit_ = 0;

    DijkstraScan heuDijkstra_;  // heuristic on primary cost (dim 0)
    PulseResult result_;

    std::chrono::steady_clock::time_point tstart_;
    bool timedOut_ = false;
};

} // namespace rcspp

#endif // THESIS_RCSPP_PULSE_H
