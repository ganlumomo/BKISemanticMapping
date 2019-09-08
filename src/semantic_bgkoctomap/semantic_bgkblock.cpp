#include "semantic_bgkblock.h"
#include <queue>
#include <algorithm>
#include <iostream>

namespace la3dm {

    std::unordered_map<OcTreeHashKey, point3f> init_key_loc_map(float resolution, unsigned short max_depth) {
        std::unordered_map<OcTreeHashKey, point3f> key_loc_map;

        std::queue<point3f> center_q;
        center_q.push(point3f(0.0f, 0.0f, 0.0f));

        for (unsigned short depth = 0; depth < max_depth; ++depth) {
            unsigned short q_size = (unsigned short) center_q.size();
            float half_size = (float) (resolution * pow(2, max_depth - depth - 1) * 0.5f);
            for (unsigned short index = 0; index < q_size; ++index) {
                point3f center = center_q.front();
                center_q.pop();
                key_loc_map.emplace(node_to_hash_key(depth, index), center);

                if (depth == max_depth - 1)
                    continue;
                for (unsigned short i = 0; i < 8; ++i) {
                    float x = (float) (center.x() + half_size * (i & 4 ? 0.5 : -0.5));
                    float y = (float) (center.y() + half_size * (i & 2 ? 0.5 : -0.5));
                    float z = (float) (center.z() + half_size * (i & 1 ? 0.5 : -0.5));
                    center_q.emplace(x, y, z);
                }
            }
        }
        return key_loc_map;
    }

    std::unordered_map<unsigned short, OcTreeHashKey> init_index_map(
            const std::unordered_map<OcTreeHashKey, point3f> &key_loc_map, unsigned short max_depth) {
        std::vector<std::pair<OcTreeHashKey, point3f>> temp;
        for (auto it = key_loc_map.begin(); it != key_loc_map.end(); ++it) {
            unsigned short depth, index;
            hash_key_to_node(it->first, depth, index);
            if (depth == max_depth - 1)
                temp.push_back(std::make_pair(it->first, it->second));
        }

        std::stable_sort(temp.begin(), temp.end(),
                         [](const std::pair<OcTreeHashKey, point3f> &p1,
                            const std::pair<OcTreeHashKey, point3f> &p2) {
                             return p1.second.x() < p2.second.x();
                         });
        std::stable_sort(temp.begin(), temp.end(),
                         [](const std::pair<OcTreeHashKey, point3f> &p1,
                            const std::pair<OcTreeHashKey, point3f> &p2) {
                             return p1.second.y() < p2.second.y();
                         });
        std::stable_sort(temp.begin(), temp.end(),
                         [](const std::pair<OcTreeHashKey, point3f> &p1,
                            const std::pair<OcTreeHashKey, point3f> &p2) {
                             return p1.second.z() < p2.second.z();
                         });

        std::unordered_map<unsigned short, OcTreeHashKey> index_map;
        int index = 0;
        for (auto it = temp.cbegin(); it != temp.cend(); ++it, ++index) {
            index_map.insert(std::make_pair(index, it->first));
        }

        return index_map;
    };

    BlockHashKey block_to_hash_key(point3f center) {
        return block_to_hash_key(center.x(), center.y(), center.z());
    }

    BlockHashKey block_to_hash_key(float x, float y, float z) {
        return (int64_t(x / (double) Block::size + 524288.5) << 40) |
               (int64_t(y / (double) Block::size + 524288.5) << 20) |
               (int64_t(z / (double) Block::size + 524288.5));
    }

    point3f hash_key_to_block(BlockHashKey key) {
        return point3f(((key >> 40) - 524288) * Block::size,
                       (((key >> 20) & 0xFFFFF) - 524288) * Block::size,
                       ((key & 0xFFFFF) - 524288) * Block::size);
    }

    ExtendedBlock get_extended_block(BlockHashKey key) {
        ExtendedBlock blocks;
        point3f center = hash_key_to_block(key);
        float x = center.x();
        float y = center.y();
        float z = center.z();
        blocks[0] = key;

        float ex, ey, ez;
        for (int i = 0; i < 6; ++i) {
            ex = (i / 2 == 0) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ey = (i / 2 == 1) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ez = (i / 2 == 2) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            blocks[i + 1] = block_to_hash_key(ex + x, ey + y, ez + z);
        }
        return blocks;
    }

    float Block::resolution = 0.1f;
    float Block::size = 0.8f;
    unsigned short Block::cell_num = static_cast<unsigned short>(round(Block::size / Block::resolution));

    std::unordered_map<OcTreeHashKey, point3f> Block::key_loc_map;
    std::unordered_map<unsigned short, OcTreeHashKey> Block::index_map;

    Block::Block() : SemanticOcTree(), center(0.0f, 0.0f, 0.0f) { }

    Block::Block(point3f center) : SemanticOcTree(), center(center) { }

    ExtendedBlock Block::get_extended_block() const {
        ExtendedBlock blocks;
        float x = center.x();
        float y = center.y();
        float z = center.z();
        blocks[0] = block_to_hash_key(x, y, z);

        float ex, ey, ez;
        for (int i = 0; i < 6; ++i) {
            ex = (i / 2 == 0) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ey = (i / 2 == 1) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ez = (i / 2 == 2) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            blocks[i + 1] = block_to_hash_key(ex + x, ey + y, ez + z);
        }

        return blocks;
    }

    OcTreeHashKey Block::get_node(unsigned short x, unsigned short y, unsigned short z) const {
        unsigned short index = x + y * Block::cell_num + z * Block::cell_num * Block::cell_num;
        return Block::index_map[index];
    }

    point3f Block::get_point(unsigned short x, unsigned short y, unsigned short z) const {
        return Block::key_loc_map[get_node(x, y, z)] + center;
    }

    void Block::get_index(const point3f &p, unsigned short &x, unsigned short &y, unsigned short &z) const {
        int xx = static_cast<int>((p.x() - center.x()) / resolution + Block::cell_num / 2);
        int yy = static_cast<int>((p.y() - center.y()) / resolution + Block::cell_num / 2);
        int zz = static_cast<int>((p.z() - center.z()) / resolution + Block::cell_num / 2);
        auto clip = [](int a) -> int { return std::max(0, std::min(a, Block::cell_num - 1)); };
        x = static_cast<unsigned short>(clip(xx));
        y = static_cast<unsigned short>(clip(yy));
        z = static_cast<unsigned short>(clip(zz));
    }

    SemanticOcTreeNode& Block::search(float x, float y, float z) const {
        return search(point3f(x, y, z));
    }

    SemanticOcTreeNode& Block::search(point3f p) const {
        unsigned short x, y, z;
        get_index(p, x, y, z);
        return operator[](get_node(x, y, z));
    }
}
