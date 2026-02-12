#include "graph/Graph.h"
#include <sstream>
#include <algorithm>

namespace rcspp {

// --- CostVector ---

CostVector::CostVector() : std::vector<long>() {}

CostVector::CostVector(long val, size_t dim) : std::vector<long>(dim, val) {}

CostVector::CostVector(const std::vector<long>& v) : std::vector<long>(v) {}

CostVector CostVector::operator+(const CostVector& rhs) const {
    CostVector result(0, size());
    for (size_t i = 0; i < size(); i++) {
        result[i] = (*this)[i] + rhs[i];
    }
    return result;
}

CostVector& CostVector::operator+=(const CostVector& rhs) {
    for (size_t i = 0; i < size(); i++) {
        (*this)[i] += rhs[i];
    }
    return *this;
}

CostVector CostVector::operator-(const CostVector& rhs) const {
    CostVector result(0, size());
    for (size_t i = 0; i < size(); i++) {
        result[i] = (*this)[i] - rhs[i];
    }
    return result;
}

bool CostVector::operator==(const CostVector& rhs) const {
    if (size() != rhs.size()) return false;
    for (size_t i = 0; i < size(); i++) {
        if ((*this)[i] != rhs[i]) return false;
    }
    return true;
}

int CostVector::CompareLexico(const CostVector& v) const {
    for (size_t i = 0; i < size(); i++) {
        if ((*this)[i] < v[i]) return -1;
        if ((*this)[i] > v[i]) return 1;
    }
    return 0;
}

CostVector CostVector::ElemWiseMin(const CostVector& rhs) const {
    CostVector result(0, size());
    for (size_t i = 0; i < size(); i++) {
        result[i] = std::min((*this)[i], rhs[i]);
    }
    return result;
}

std::string CostVector::ToStr() const {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < size(); i++) {
        if (i > 0) oss << ",";
        oss << (*this)[i];
    }
    oss << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const CostVector& c) {
    os << c.ToStr();
    return os;
}

bool EpsDominance(const CostVector& v1, const CostVector& v2, double eps) {
    for (size_t i = 0; i < v1.size(); i++) {
        if (v1[i] > (long)((1.0 + eps) * v2[i])) {
            return false;
        }
    }
    return true;
}

} // namespace rcspp
