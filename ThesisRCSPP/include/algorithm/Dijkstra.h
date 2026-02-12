#ifndef THESIS_RCSPP_DIJKSTRA_H
#define THESIS_RCSPP_DIJKSTRA_H

#include "graph/Graph.h"
#include <unordered_map>
#include <set>

namespace rcspp {

// Reverse Dijkstra from goal to all nodes on a single cost dimension.
// Used as heuristic for ERCA*.
class DijkstraScan {
public:
    DijkstraScan();
    ~DijkstraScan();

    void SetGraphPtr(Graph* g);

    // Run reverse Dijkstra from goal node on cost dimension cdim
    int Search(long goal, size_t cdim);

    // Get shortest distance from node u to goal on this cost dimension
    // Returns -1 if unreachable
    long GetCost(long u);

    // Get the full cost vector of the cheapest path from v to goal
    CostVector GetCostVec(long v);

    // Get shortest path from v to goal
    std::vector<long> GetPath(long v);

private:
    Graph* graph_ = nullptr;
    std::unordered_map<long, long> dist_;     // node -> single-dim distance
    std::unordered_map<long, long> parent_;   // for path reconstruction
    std::unordered_map<long, CostVector> cvec_; // full cost vector
};

} // namespace rcspp

#endif // THESIS_RCSPP_DIJKSTRA_H
