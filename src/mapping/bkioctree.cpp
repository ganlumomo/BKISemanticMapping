#include <cmath>

#include "bkioctree.h"

namespace semantic_bki {

    unsigned short SemanticOcTree::max_depth = 0;

    OcTreeHashKey node_to_hash_key(unsigned short depth, unsigned short index) {
        return (depth << 16) + index;
    }

    void hash_key_to_node(OcTreeHashKey key, unsigned short &depth, unsigned short &index) {
        depth = (unsigned short) (key >> 16);
        index = (unsigned short) (key & 0xFFFF);
    }

    SemanticOcTree::SemanticOcTree() {
        if (max_depth <= 0)
            node_arr = nullptr;
        else {
            node_arr = new SemanticOcTreeNode *[max_depth]();
            for (unsigned short i = 0; i < max_depth; ++i) {
                node_arr[i] = new SemanticOcTreeNode[(int) pow(8, i)]();
            }
        }
    }

    SemanticOcTree::~SemanticOcTree() {
        if (node_arr != nullptr) {
            for (unsigned short i = 0; i < max_depth; ++i) {
                if (node_arr[i] != nullptr) {
                    delete[] node_arr[i];
                }
            }
            delete[] node_arr;
        }
    }

    SemanticOcTree::SemanticOcTree(const SemanticOcTree &other) {
        if (other.node_arr == nullptr) {
            node_arr = nullptr;
            return;
        }

        node_arr = new SemanticOcTreeNode *[max_depth]();
        for (unsigned short i = 0; i < max_depth; ++i) {
            if (other.node_arr[i] != nullptr) {
                int n = (int) pow(8, i);
                node_arr[i] = new SemanticOcTreeNode[n]();
                std::copy(node_arr[i], node_arr[i] + n, other.node_arr[i]);
            } else
                node_arr[i] = nullptr;
        }
    }

    SemanticOcTree &SemanticOcTree::operator=(const SemanticOcTree &other) {
        SemanticOcTreeNode **local_node_arr = new SemanticOcTreeNode *[max_depth]();
        for (unsigned short i = 0; i < max_depth; ++i) {
            if (local_node_arr[i] != nullptr) {
                int n = (int) pow(8, i);
                local_node_arr[i] = new SemanticOcTreeNode[n]();
                std::copy(local_node_arr[i], local_node_arr[i] + n, other.node_arr[i]);
            } else
                local_node_arr[i] = nullptr;
        }

        node_arr = local_node_arr;
        return *this;
    }

    bool SemanticOcTree::is_leaf(unsigned short depth, unsigned short index) const {
        if (node_arr != nullptr && node_arr[depth] != nullptr && node_arr[depth][index].get_state() != State::PRUNED) {
            if (depth + 1 < max_depth) {
                if (node_arr[depth + 1] == nullptr || node_arr[depth + 1][index * 8].get_state() == State::PRUNED)
                    return true;
            } else {
                return true;
            }
        }
        return false;
    }

    bool SemanticOcTree::is_leaf(OcTreeHashKey key) const {
        unsigned short depth = 0;
        unsigned short index = 0;
        hash_key_to_node(key, depth, index);
        return is_leaf(depth, index);
    }

    bool SemanticOcTree::search(OcTreeHashKey key) const {
        unsigned short depth;
        unsigned short index;
        hash_key_to_node(key, depth, index);

        return node_arr != nullptr &&
               node_arr[depth] != nullptr &&
               node_arr[depth][index].get_state() != State::PRUNED;
    }

    SemanticOcTreeNode &SemanticOcTree::operator[](OcTreeHashKey key) const {
        unsigned short depth;
        unsigned short index;
        hash_key_to_node(key, depth, index);
        return node_arr[depth][index];
    }
}
