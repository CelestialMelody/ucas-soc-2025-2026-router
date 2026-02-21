#pragma once

/** 全局配置：金属层数、PagedGrid 块大小、PathFinder/Cleanup 迭代与代价、A* 参数、Key 编解码。改用例时主要改本文件。 */
using Key = long long;

// -----------------------------------------------------------------------------
// 金属层与引脚层
// -----------------------------------------------------------------------------
/** 金属布线层总数（如 M1~M4 为 4）。必须 >= 1，否则无有效格点、所有线网布不通且 overuse 恒为 0 不收敛。典型 2.5D 至少为 2（C4 一层 + Chiplet 一层）。 */
constexpr int METAL_LAYER_COUNT = 4;
/** 底层引脚（C4）的虚拟层号，仅用于 calculatePinMapKey 内部映射；固定 -1，不随 METAL_LAYER_COUNT 变 */
constexpr int PIN_BOTTOM_DUMMY_LAYER = -1;
/** 顶层引脚的虚拟层号，仅用于 calculatePinMapKey；设为 METAL_LAYER_COUNT 以便随层数缩放（与 CHIPLET 层号区分） */
constexpr int PIN_TOP_DUMMY_LAYER = METAL_LAYER_COUNT;
/** C4 引脚对应的金属层索引（底层，固定 0），不随 METAL_LAYER_COUNT 变 */
constexpr int C4_METAL_LAYER_IDX = 0;
/** Chiplet 引脚对应的金属层索引（顶层），随 METAL_LAYER_COUNT 变：= METAL_LAYER_COUNT - 1 */
constexpr int CHIPLET_METAL_LAYER_IDX = METAL_LAYER_COUNT - 1;

// -----------------------------------------------------------------------------
// PagedGrid 分块参数
// -----------------------------------------------------------------------------
/** 块大小 = 2^BLOCK_SHIFT，与金属层数无关，固定 64 以平衡内存与访问 */
constexpr int PAGED_GRID_BLOCK_SHIFT = 6;
/** 每块边长（x/y 方向格点数） */
constexpr int PAGED_GRID_BLOCK_SIZE = 1 << PAGED_GRID_BLOCK_SHIFT;
/** 块内坐标掩码，用于 x & MASK、y & MASK 得到块内偏移 */
constexpr int PAGED_GRID_MASK = (1 << PAGED_GRID_BLOCK_SHIFT) - 1;

// -----------------------------------------------------------------------------
// PathFinder 迭代与代价
// -----------------------------------------------------------------------------
/** PathFinder 主循环最大迭代次数，超过则进入 Cleanup（若仍有冲突） */
constexpr int MAX_PATHFINDER_ITERATIONS = 50;
/** Cleanup 阶段最大迭代次数（仅当主循环未完全收敛时执行） */
constexpr int MAX_CLEANUP_ITERATIONS = 100;
/** PathFinder 初始 history 因子，用于对历史上拥塞过的格子加价 */
constexpr double INITIAL_HISTORY_FACTOR = 0.5;
/** PathFinder 初始 present 因子，用于对当前已占用格子加价 */
constexpr double INITIAL_PRESENT_FACTOR = 0.5;
/** 每轮 PathFinder 迭代 history 因子的增加量 */
constexpr double HISTORY_FACTOR_INCREMENT = 0.5;
/** 每轮 PathFinder 迭代 present 因子的乘数 */
constexpr double PRESENT_FACTOR_MULTIPLIER = 1.5;
/** Cleanup 阶段使用的 history 因子（较大以强烈避开冲突格） */
constexpr double CLEANUP_HISTORY_FACTOR = 50.0;
/** Cleanup 阶段使用的 present 因子 */
constexpr double CLEANUP_PRESENT_FACTOR = 50.0;

// -----------------------------------------------------------------------------
// A* 路由代价与启发式
// -----------------------------------------------------------------------------
/** A* 中 f = g + weight*h 的启发式权重，略大于 1 可加速搜索 */
constexpr double ASTAR_HEURISTIC_WEIGHT = 1.2;
/** 水平/垂直移动一格的代价 */
constexpr double MOVE_COST_XY = 1.0;
/** 换层（过孔）一步的代价 */
constexpr double MOVE_COST_VIA = 5.0;
/** 启发式函数中层差对应的权重（与 MOVE_COST_VIA 一致较合理） */
constexpr double LAYER_HEURISTIC_WEIGHT = 5.0;
/** net_type == 1 的线网在 M1/M2（layer 0、1）上的额外惩罚，引导走高层 */
constexpr double NET_TYPE1_LAYER_PENALTY = 5.0;
/** A* 中判断 f 值“过期”的浮点容差 */
constexpr double ASTAR_F_EPSILON = 1e-9;

// -----------------------------------------------------------------------------
// 输出与日志格式
// -----------------------------------------------------------------------------
/** 控制台输出中指标名称列宽度（如 "wirelength"、"disconnected_nets"） */
constexpr int OUTPUT_LABEL_WIDTH = 22;
/** 控制台输出中数值列宽度（右对齐） */
constexpr int OUTPUT_VALUE_WIDTH = 14;
/** 本路由器不产生非法线段，报告中的 invalid_segments 固定为此值 */
constexpr int OUTPUT_INVALID_SEGMENTS_COUNT = 0;
/** 控制台输出中时间列宽度（右对齐） */
constexpr int TIME_WIDTH = 12;

/** 将网格坐标 (x, y, l) 编码为单值 Key，用于 occupancy / history 等一维索引；非法层返回 -1。 */
inline Key calculateCellKey(int x, int y, int l, int grid_width, int grid_height) {
    if (l < 0 || l >= (int)METAL_LAYER_COUNT) return -1;
    return (long long)(l * grid_height + y) * grid_width + x;
}

/** 将 (x, y, l) 编码为引脚映射用 Key，支持虚拟层号，用于 pin_output_layers / pin_original_coords 等。 */
inline long long calculatePinMapKey(int x, int y, int l, int grid_width, int grid_height) {
    int normalized_l = l;
    if (l == PIN_BOTTOM_DUMMY_LAYER) normalized_l = -1;
    if (l == PIN_TOP_DUMMY_LAYER) normalized_l = METAL_LAYER_COUNT;
    int mapped_l = normalized_l + 1;
    return (long long)x + (long long)y * grid_width + (long long)mapped_l * grid_width * grid_height;
}
