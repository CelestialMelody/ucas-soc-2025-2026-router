#pragma once

/**
 * 按块分配的稀疏网格。将整平面按 64×64×层 分块，按需分配块，只分配被访问到的块，
 * 用于 occupancy、history_cost、is_pin_net_id 及 A* 的 visited，以降低大图内存。
 */
#include <vector>
#include "config.hpp"

template <typename T>
class PagedGrid {
public:
    static constexpr int BLOCK_SIZE = PAGED_GRID_BLOCK_SIZE;
    static constexpr int BLOCK_SHIFT = PAGED_GRID_BLOCK_SHIFT;
    static constexpr int MASK = PAGED_GRID_MASK;
    static constexpr int LAYER_COUNT = METAL_LAYER_COUNT;

    struct Block {
        T data[BLOCK_SIZE][BLOCK_SIZE][LAYER_COUNT];

        Block(T default_val) {
            for (int i = 0; i < BLOCK_SIZE; ++i)
                for (int j = 0; j < BLOCK_SIZE; ++j)
                    for (int k = 0; k < LAYER_COUNT; ++k)
                        data[i][j][k] = default_val;
        }
    };

    int width_blocks;
    int height_blocks;
    std::vector<Block*> table;
    T default_val;
    std::vector<int> allocated_indices;  /* 已分配块下标，供 freeAll 快速清空 */

    PagedGrid(int width, int height, T def_val) : default_val(def_val) {
        width_blocks = (width + BLOCK_SIZE - 1) / BLOCK_SIZE;
        height_blocks = (height + BLOCK_SIZE - 1) / BLOCK_SIZE;
        table.resize(width_blocks * height_blocks, nullptr);
    }

    ~PagedGrid() {
        freeAll();
    }

    void freeAll() {
        for (int idx : allocated_indices) {
            delete table[idx];
            table[idx] = nullptr;
        }
        allocated_indices.clear();
    }

    /** 取 (x,y,l) 的指针；force_create 为 true 时未分配则分配块。块号 (x>>6, y>>6)，块内 (x&63, y&63, l)。 */
    T* getPtr(int x, int y, int l, bool force_create = false) {
        if (x < 0 || y < 0 || l < 0 || l >= LAYER_COUNT) return nullptr;

        int bx = x >> BLOCK_SHIFT;
        int by = y >> BLOCK_SHIFT;
        int idx = by * width_blocks + bx;

        if ((size_t)idx >= table.size()) return nullptr;

        if (!table[idx]) {
            if (!force_create) return nullptr;
            table[idx] = new Block(default_val);
            allocated_indices.push_back(idx);
        }

        return &table[idx]->data[x & MASK][y & MASK][l];
    }

    /** 读取 (x,y,l) 的值，未分配块则返回 default_val。 */
    T get(int x, int y, int l) {
        T* ptr = getPtr(x, y, l, false);
        return ptr ? *ptr : default_val;
    }

    /** 将 (x,y,l) 设为 val，必要时分配块。 */
    void set(int x, int y, int l, T val) {
        T* ptr = getPtr(x, y, l, true);
        if (ptr) *ptr = val;
    }

    /** 将 (x,y,l) 的值加上 val，常用于 occupancy += 1 或 history_cost += factor。 */
    void add(int x, int y, int l, T val) {
        T* ptr = getPtr(x, y, l, true);
        if (ptr) *ptr += val;
    }
};
