#include "algorithm/Pulse.h"
#include <chrono>
#include <limits>
#include <algorithm>

namespace rcspp {

Pulse::Pulse() {}
Pulse::~Pulse() {}

void Pulse::SetGraph(Graph* g) {
    graph_ = g;
}

void Pulse::SetResourceLimit(long limit, size_t resIndex) {
    resourceLimit_ = limit;
    resIndex_ = resIndex;
}

void Pulse::InitHeuristic(long target) {
    heuDijkstra_.SetGraphPtr(graph_);
    heuDijkstra_.Search(target, 0); // heuristic on primary cost (dim 0)
}

int Pulse::Search(long source, long target, double timeLimit) {
    source_ = source;
    target_ = target;
    timeLimit_ = timeLimit;
    timedOut_ = false;

    result_ = PulseResult();
    result_.bestCost = std::numeric_limits<long>::max();

    InitHeuristic(target);

    tstart_ = std::chrono::steady_clock::now();

    std::vector<long> currentPath;
    std::unordered_set<long> visited;

    currentPath.push_back(source);
    visited.insert(source);

    pulse(source, 0, 0, currentPath, visited);

    auto tend = std::chrono::steady_clock::now();
    result_.rtSearch = std::chrono::duration<double>(tend - tstart_).count();
    result_.timeout = timedOut_;

    if (result_.bestCost < std::numeric_limits<long>::max()) {
        result_.feasible = true;
    }

    return result_.feasible ? 1 : 0;
}

void Pulse::pulse(long node, long currentCost, long currentResource,
                  std::vector<long>& currentPath, std::unordered_set<long>& visited) {
    // Timeout check (every 1000 nodes to reduce chrono overhead)
    result_.nExplored++;
    if (result_.nExplored % 1000 == 0) {
        auto tnow = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(tnow - tstart_).count() > timeLimit_) {
            timedOut_ = true;
            return;
        }
    }

    // Feasibility pruning: resource exceeded
    if (currentResource > resourceLimit_) {
        return;
    }

    // Bounds pruning: cost lower bound >= best known
    long h = heuDijkstra_.GetCost(node);
    if (h < 0) return; // unreachable
    if (currentCost + h >= result_.bestCost) {
        return;
    }

    // Goal reached
    if (node == target_) {
        if (currentCost < result_.bestCost) {
            result_.bestCost = currentCost;
            result_.bestPath = currentPath;
            // Build full cost vector by walking the path
            result_.bestCostVec = CostVector(0, graph_->GetCostDim());
            for (size_t i = 0; i + 1 < currentPath.size(); i++) {
                result_.bestCostVec += graph_->GetCost(currentPath[i], currentPath[i + 1]);
            }
        }
        return;
    }

    // Expand successors
    auto succs = graph_->GetSuccs(node);
    for (auto u : succs) {
        if (visited.count(u)) continue; // avoid cycles
        if (timedOut_) return;

        CostVector edgeCost = graph_->GetCost(node, u);
        long newCost = currentCost + edgeCost[0];
        long newResource = currentResource + edgeCost[resIndex_];

        visited.insert(u);
        currentPath.push_back(u);

        pulse(u, newCost, newResource, currentPath, visited);

        currentPath.pop_back();
        visited.erase(u);
    }
}

PulseResult Pulse::GetResult() const {
    return result_;
}

} // namespace rcspp
