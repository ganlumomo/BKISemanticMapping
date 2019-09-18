#pragma once

#include <stack>
#include <vector>

#include "point3f.h"
#include "bkioctree_node.h"

namespace semantic_bki {

    /// Hash key to index OcTree nodes given depth and the index in that layer.
    typedef int OcTreeHashKey;

    /// Convert from node to hask key.
    OcTreeHashKey node_to_hash_key(unsigned short depth, unsigned short index);

    /// Convert from hash key to node.
    void hash_key_to_node(OcTreeHashKey key, unsigned short &depth, unsigned short &index);

    /*
     * @brief A simple OcTree to organize occupancy data in one block.
     *
     * OcTree doesn't store positions of nodes in order to reduce memory usage.
     * The nodes in OcTrees are indexed by OcTreeHashKey which can be used to
     * retrieve positions later (See Block).
     * For the purpose of mapping, this OcTree has fixed depth which should be
     * set before using OcTrees.
     */
    class SemanticOcTree {
        friend class SemanticBKIOctoMap;

    public:
        SemanticOcTree();

        ~SemanticOcTree();

        SemanticOcTree(const SemanticOcTree &other);

        SemanticOcTree &operator=(const SemanticOcTree &other);

        /*
         * @brief Rursively pruning OcTreeNodes with the same state.
         *
         * Prune nodes by setting nodes to PRUNED.
         * Delete the layer if all nodes are pruned.
         */
        //bool prune();

        /// @return true if this node is a leaf node.
        bool is_leaf(OcTreeHashKey key) const;

        /// @return true if this node is a leaf node.
        bool is_leaf(unsigned short depth, unsigned short index) const;

        /// @return true if this node exists and is not pruned.
        bool search(OcTreeHashKey key) const;

        /// @return Occupancy of the node (without checking if it exists!)
        SemanticOcTreeNode &operator[](OcTreeHashKey key) const;

        /// Leaf iterator for OcTrees: iterate all leaf nodes not pruned.
        class LeafIterator : public std::iterator<std::forward_iterator_tag, SemanticOcTreeNode> {
        public:
            LeafIterator() : tree(nullptr) { }

            LeafIterator(const SemanticOcTree *tree)
                    : tree(tree != nullptr && tree->node_arr != nullptr ? tree : nullptr) {
                if (tree != nullptr) {
                    stack.emplace(0, 0);
                    stack.emplace(0, 0);
                    ++(*this);
                }
            }

            LeafIterator(const LeafIterator &other) : tree(other.tree), stack(other.stack) { }

            LeafIterator &operator=(const LeafIterator &other) {
                tree = other.tree;
                stack = other.stack;
                return *this;
            }

            bool operator==(const LeafIterator &other) const {
                return (tree == other.tree) &&
                       (stack.size() == other.stack.size()) &&
                       (stack.size() == 0 || (stack.size() > 0 &&
                                              (stack.top().depth == other.stack.top().depth) &&
                                              (stack.top().index == other.stack.top().index)));
            }

            bool operator!=(const LeafIterator &other) const {
                return !(this->operator==(other));
            }

            LeafIterator operator++(int) {
                LeafIterator result(*this);
                ++(*this);
                return result;
            }

            LeafIterator &operator++() {
                if (stack.empty()) {
                    tree = nullptr;
                } else {
                    stack.pop();
                    while (!stack.empty() && !tree->is_leaf(stack.top().depth, stack.top().index))
                        single_inc();
                    if (stack.empty())
                        tree = nullptr;
                }
                return *this;
            }

            inline SemanticOcTreeNode &operator*() const {
                return (*tree)[get_hash_key()];
            }

            inline SemanticOcTreeNode &get_node() const {
                return operator*();
            }

            inline OcTreeHashKey get_hash_key() const {
                OcTreeHashKey key = node_to_hash_key(stack.top().depth, stack.top().index);
                return key;
            }

        protected:
            void single_inc() {
                StackElement top(stack.top());
                stack.pop();

                for (int i = 0; i < 8; ++i) {
                    stack.emplace(top.depth + 1, top.index * 8 + i);
                }
            }

            struct StackElement {
                unsigned short depth;
                unsigned short index;

                StackElement(unsigned short depth, unsigned short index)
                        : depth(depth), index(index) { }
            };

            const SemanticOcTree *tree;
            std::stack<StackElement, std::vector<StackElement> > stack;
        };

        /// @return the beginning of leaf iterator
        inline LeafIterator begin_leaf() const { return LeafIterator(this); };

        /// @return the end of leaf iterator
        inline LeafIterator end_leaf() const { return LeafIterator(nullptr); };

    private:
        SemanticOcTreeNode **node_arr;
        static unsigned short max_depth;
    };
}
