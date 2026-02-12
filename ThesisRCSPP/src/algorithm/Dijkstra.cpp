#include "algorithm/Dijkstra.h"
#include <limits>

namespace rcspp {

DijkstraScan::DijkstraScan() {}
DijkstraScan::~DijkstraScan() {}

void DijkstraScan::SetGraphPtr(Graph* g) {
    graph_ = g;
}

int DijkstraScan::Search(long goal, size_t cdim) {
    dist_.clear();
    parent_.clear();
    cvec_.clear();

    // min-heap: (distance, node)
    std::set<std::pair<long, long>> open;
    dist_[goal] = 0;
    open.insert({0, goal});

    while (!open.empty()) {
        auto [d, v] = *open.begin();
        open.erase(open.begin());

        if (d > dist_[v]) continue;

        // Reverse: get predecessors of v (these are edges u->v in original graph)
        auto preds = graph_->GetPreds(v);
        for (auto u : preds) {
            CostVector edgeCost = graph_->GetCost(u, v);
            long newDist = d + edgeCost[cdim];

            if (dist_.find(u) == dist_.end() || newDist < dist_[u]) {
                dist_[u] = newDist;
                parent_[u] = v;
                cvec_[u] = edgeCost;
                open.insert({newDist, u});
            }
        }
    }
    return 1;
}

long DijkstraScan::GetCost(long u) {
    if (dist_.find(u) == dist_.end()) return -1;
    return dist_[u];
}

CostVector DijkstraScan::GetCostVec(long v) {
    if (cvec_.find(v) == cvec_.end()) return CostVector();
    return cvec_[v];
}

std::vector<long> DijkstraScan::GetPath(long v) {
    std::vector<long> path;
    long cur = v;
    path.push_back(cur);
    while (parent_.find(cur) != parent_.end()) {
        cur = parent_[cur];
        path.push_back(cur);
    }
    return path;
}

} // namespace rcspp
