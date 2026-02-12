#ifndef THESIS_RCSPP_AVLTREE_H
#define THESIS_RCSPP_AVLTREE_H

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rcspp {

struct AVLNode {
    long id = -1;
    AVLNode* left = nullptr;
    AVLNode* right = nullptr;
    size_t h = 0;
};

inline size_t MaxH(size_t a, size_t b) { return (a > b) ? a : b; }
inline size_t H(AVLNode* n) { return n ? n->h : 0; }

AVLNode* NewAVLNode(long id);
AVLNode* RightRotate(AVLNode* y);
AVLNode* LeftRotate(AVLNode* x);
int GetBalanceFactor(AVLNode* n);

// Generic AVL tree with extended search operations
template <typename KeyType>
class AVLTree {
public:
    AVLTree();
    virtual ~AVLTree();

    virtual void Add(KeyType k, long id = -1);
    virtual AVLNode Find(KeyType k);

    // Find largest key < k (if_equal: <=)
    virtual int FindMaxLess(KeyType k, KeyType* out, bool if_equal = false, long* out_id = nullptr);
    // Find smallest key > k (if_equal: >=)
    virtual int FindMinMore(KeyType k, KeyType* out, bool if_equal = false, long* out_id = nullptr);

    virtual void Delete(KeyType k);
    virtual void Clear();
    virtual size_t Size() const;

    virtual void ToSortedVector(
        std::vector<KeyType>* out, std::vector<long>* out_id = nullptr,
        std::unordered_set<long>* skip_set = nullptr);

protected:
    virtual AVLNode* _insert(AVLNode* n, KeyType k, long id0);
    virtual AVLNode* _find(AVLNode* n, KeyType k);
    virtual void _findMaxLess(AVLNode* n, KeyType k, long* ref, bool if_equal);
    virtual void _findMinMore(AVLNode* n, KeyType k, long* ref, bool if_equal);
    virtual AVLNode* _findMin(AVLNode* n);
    virtual AVLNode* _delete(AVLNode* n, KeyType k);
    virtual void _deleteAll(AVLNode* n);
    virtual void _inOrder(AVLNode* n, std::vector<KeyType>* out,
                          std::vector<long>* out_id, std::unordered_set<long>* skip_set);

    inline long _genId() { return idGen_++; }

    std::unordered_map<long, KeyType> keys_;
    AVLNode* root_ = nullptr;
    long idGen_ = 0;
    size_t size_ = 0;
};

} // namespace rcspp

#endif // THESIS_RCSPP_AVLTREE_H
