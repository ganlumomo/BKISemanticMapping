#pragma once

#include <unordered_map>
#include <array>

#include "point3f.h"
#include "bkioctree_node.h"
#include "bkioctree.h"

namespace semantic_bki {

    /// Hask key to index Block given block's center.
    typedef int64_t BlockHashKey;

    /// Initialize Look-Up Table
    std::unordered_map<OcTreeHashKey, point3f> init_key_loc_map(float resolution, unsigned short max_depth);

    std::unordered_map<unsigned short, OcTreeHashKey> init_index_map(const std::unordered_map<OcTreeHashKey, point3f> &key_loc_map,
                                                                     unsigned short max_depth);

    /// Extended Block
#ifdef PREDICT
    typedef std::array<BlockHashKey, 27> ExtendedBlock;
#else
    typedef std::array<BlockHashKey, 7> ExtendedBlock;
#endif

    /// Convert from block to hash key.
    BlockHashKey block_to_hash_key(point3f center);

    /// Convert from block to hash key.
    BlockHashKey block_to_hash_key(float x, float y, float z);

    /// Convert from hash key to block.
    point3f hash_key_to_block(BlockHashKey key);

    /// Get current block's extended block.
    ExtendedBlock get_extended_block(BlockHashKey key);

    /*
     * @brief Block is built on top of OcTree, providing the functions to locate nodes.
     *
     * Block stores the information needed to locate each OcTreeNode's position:
     * fixed resolution, fixed block_size, both of which must be initialized.
     * The localization is implemented using Loop-Up Table.
     */
    class Block : public SemanticOcTree {
        friend BlockHashKey block_to_hash_key(point3f center);

        friend BlockHashKey block_to_hash_key(float x, float y, float z);

        friend point3f hash_key_to_block(BlockHashKey key);

        friend ExtendedBlock get_extended_block(BlockHashKey key);

        friend class SemanticBKIOctoMap;

    public:
        Block();

        Block(point3f center);

        /// @return location of the OcTreeNode given OcTree's LeafIterator.
        inline point3f get_loc(const LeafIterator &it) const {
            return Block::key_loc_map[it.get_hash_key()] + center;
        }

        /// @return size of the OcTreeNode given OcTree's LeafIterator.
        inline float get_size(const LeafIterator &it) const {
            unsigned short depth, index;
            hash_key_to_node(it.get_hash_key(), depth, index);
            return float(size / pow(2, depth));
        }

        /// @return center of current Block.
        inline point3f get_center() const { return center; }

        /// @return min lim of current Block.
        inline point3f get_lim_min() const { return center - point3f(size / 2.0f, size / 2.0f, size / 2.0f); }

        /// @return max lim of current Block.
        inline point3f get_lim_max() const { return center + point3f(size / 2.0f, size / 2.0f, size / 2.0f); }

        /// @return ExtendedBlock of current Block.
        ExtendedBlock get_extended_block() const;

        OcTreeHashKey get_node(unsigned short x, unsigned short y, unsigned short z) const;

        point3f get_point(unsigned short x, unsigned short y, unsigned short z) const;

        void get_index(const point3f &p, unsigned short &x, unsigned short &y, unsigned short &z) const;

        SemanticOcTreeNode &search(float x, float y, float z) const;

        SemanticOcTreeNode &search(point3f p) const;

    private:
        // Loop-Up Table
        static std::unordered_map<OcTreeHashKey, point3f> key_loc_map;
        static std::unordered_map<unsigned short, OcTreeHashKey> index_map;
        static float resolution;
        static float size;
        static unsigned short cell_num;

        point3f center;
    };
}
