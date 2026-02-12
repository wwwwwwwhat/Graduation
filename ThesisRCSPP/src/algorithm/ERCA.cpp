#include "algorithm/ERCA.h"
#include <chrono>
#include <stdexcept>

namespace rcspp {

static long G_DOM_CHECK_COUNT = 0;

ERCA::ERCA() {}
ERCA::~ERCA() {}

void ERCA::SetGraph(Graph* g) {
    graph_ = g;
}

void ERCA::SetResourceLimits(const std::vector<long>& limits) {
    resourceLimits_ = CostVector(limits);
}

void ERCA::SetUpperBound(const CostVector& ub) {
    upperBound_ = ub;
    hasUpperBound_ = true;
}

void ERCA::SetHeuInflateRate(double w) {
    w_ = w;
}

void ERCA::InitHeuristic(long target) {
    auto tstart = std::chrono::steady_clock::now();
    size_t cdim = graph_->GetCostDim();
    dijks_.resize(cdim);
    for (size_t i = 0; i < cdim; i++) {
        dijks_[i].SetGraphPtr(graph_);
        dijks_[i].Search(target, i);
    }
    auto tend = std::chrono::steady_clock::now();
    result_.rtInitHeu = std::chrono::duration<double>(tend - tstart).count();
    G_DOM_CHECK_COUNT = 0;
}

CostVector ERCA::heuristic(long v) {
    CostVector h(0, graph_->GetCostDim());
    for (size_t i = 0; i < h.size(); i++) {
        h[i] = dijks_[i].GetCost(v);
        if (h[i] < 0) {
            throw std::runtime_error("[ERCA] Unreachable node in heuristic");
        }
    }
    h[0] = static_cast<long>(h[0] * w_); // inflate primary cost heuristic
    return h;
}

bool ERCA::resourceCheck(const Label& l) {
    for (size_t i = 0; i < resourceLimits_.size(); i++) {
        if (l.f[i + 1] > resourceLimits_[i]) {
            result_.nPrunedByResource++;
            return true; // resource violated
        }
    }
    // Optional upper bound check on primary cost
    if (hasUpperBound_ && l.f[0] > upperBound_[0]) {
        result_.nPrunedByUpperBound++;
        return true;
    }
    return false;
}

bool ERCA::frontierCheck(const Label& l) {
    if (alpha_.find(l.v) == alpha_.end()) return false;
    return alpha_[l.v].Check(l.g);
}

void ERCA::updateFrontier(const Label& l) {
    if (alpha_.find(l.v) == alpha_.end()) {
        alpha_[l.v] = ParetoFrontier();
    }
    alpha_[l.v].Update(l);
}

long ERCA::genLabelId() {
    return labelIdGen_++;
}

std::vector<long> ERCA::buildPath(long labelId) {
    std::vector<long> reversed;
    reversed.push_back(labels_[labelId].v);
    while (parent_.find(labelId) != parent_.end()) {
        labelId = parent_[labelId];
        reversed.push_back(labels_[labelId].v);
    }
    std::vector<long> path(reversed.rbegin(), reversed.rend());
    return path;
}

int ERCA::Search(long source, long target, double timeLimit) {
    InitHeuristic(target);

    auto tstart = std::chrono::steady_clock::now();
    vo_ = source;
    vd_ = target;

    // Clear state
    open_.clear();
    labels_.clear();
    parent_.clear();
    alpha_.clear();
    labelIdGen_ = 0;
    result_ = SearchResult();
    result_.rtInitHeu = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - tstart).count();

    // Re-init heuristic timing
    auto theuStart = std::chrono::steady_clock::now();
    // (heuristic already initialized above)
    auto theuEnd = std::chrono::steady_clock::now();

    tstart = std::chrono::steady_clock::now();

    // Init start label
    CostVector zeroVec(0, graph_->GetCostDim());
    Label l0(genLabelId(), source, zeroVec, heuristic(source));
    labels_[l0.id] = l0;
    result_.nGenerated++;
    open_.insert({l0.f, l0.id});

    // Main search loop
    while (!open_.empty()) {
        // Timeout check
        auto tnow = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(tnow - tstart).count() > timeLimit) {
            result_.timeout = true;
            break;
        }

        // Pop label with smallest f
        Label l = labels_[open_.begin()->second];
        open_.erase(open_.begin());

        // Lazy dominance check + resource check (with detailed counting)
        if (frontierCheck(l)) {
            result_.nPrunedByDominance++;
            continue;
        }
        if (resourceCheck(l)) {
            continue; // counted inside resourceCheck
        }

        updateFrontier(l);

        // Goal reached
        if (l.v == vd_) {
            break; // ERCA* finds single optimal solution
        }

        // Expand
        result_.nExpanded++;
        auto succs = graph_->GetSuccs(l.v);
        for (auto u : succs) {
            CostVector gu = l.g + graph_->GetCost(l.v, u);
            CostVector fu = gu + heuristic(u);
            Label l2(genLabelId(), u, gu, fu);
            labels_[l2.id] = l2;
            parent_[l2.id] = l.id;

            if (frontierCheck(l2)) {
                result_.nPrunedByDominance++;
                continue;
            }
            if (resourceCheck(l2)) {
                continue; // counted inside resourceCheck
            }
            result_.nGenerated++;
            open_.insert({l2.f, l2.id});
        }
    }

    // Post-process results
    if (alpha_.find(vd_) != alpha_.end()) {
        for (auto lid : alpha_[vd_].labelIds) {
            result_.paths[lid] = buildPath(lid);
            result_.costs[lid] = labels_[lid].g;
        }
    }

    auto tend = std::chrono::steady_clock::now();
    result_.nDomCheck = G_DOM_CHECK_COUNT;
    result_.rtSearch = std::chrono::duration<double>(tend - tstart).count();

    return 1;
}

SearchResult ERCA::GetResult() const {
    return result_;
}

} // namespace rcspp
