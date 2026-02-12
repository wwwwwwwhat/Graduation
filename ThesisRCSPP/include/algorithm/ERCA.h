#ifndef THESIS_RCSPP_ERCA_H
#define THESIS_RCSPP_ERCA_H

#include "graph/Graph.h"
#include "frontier/ParetoFrontier.h"
#include "algorithm/Dijkstra.h"
#include <set>
#include <unordered_map>

namespace rcspp {

struct SearchResult {
    std::unordered_map<long, std::vector<long>> paths; // label_id -> path
    std::unordered_map<long, CostVector> costs;        // label_id -> cost
    long nGenerated = 0;
    long nExpanded = 0;
    long nDomCheck = 0;
    long nPrunedByResource = 0;   // pruned by resource constraint alone
    long nPrunedByUpperBound = 0; // pruned by upper bound from Pulse
    long nPrunedByDominance = 0;  // pruned by Pareto dominance
    double rtInitHeu = 0.0;
    double rtSearch = 0.0;
    bool timeout = false;
};

// Enhanced Resource-Constrained A* (ERCA*)
class ERCA {
public:
    ERCA();
    ~ERCA();

    void SetGraph(Graph* g);
    void SetResourceLimits(const std::vector<long>& limits);

    // Optional: set cost upper bound from Pulse pre-computation
    void SetUpperBound(const CostVector& ub);

    // Set heuristic inflation rate (w >= 1.0)
    void SetHeuInflateRate(double w);

    // Initialize heuristic (reverse Dijkstra from target)
    void InitHeuristic(long target);

    // Run search. Returns 1 on success.
    int Search(long source, long target, double timeLimit);

    SearchResult GetResult() const;

private:
    CostVector heuristic(long v);
    bool resourceCheck(const Label& l);
    bool frontierCheck(const Label& l);
    void updateFrontier(const Label& l);
    std::vector<long> buildPath(long labelId);
    long genLabelId();

    Graph* graph_ = nullptr;
    CostVector resourceLimits_;
    CostVector upperBound_;        // optional upper bound from Pulse
    bool hasUpperBound_ = false;
    double w_ = 1.0;              // heuristic inflation

    long vo_ = -1, vd_ = -1;
    long labelIdGen_ = 0;

    std::set<std::pair<CostVector, long>> open_;
    std::unordered_map<long, Label> labels_;
    std::unordered_map<long, long> parent_;
    std::unordered_map<long, ParetoFrontier> alpha_;
    std::vector<DijkstraScan> dijks_;

    SearchResult result_;
};

} // namespace rcspp

#endif // THESIS_RCSPP_ERCA_H
