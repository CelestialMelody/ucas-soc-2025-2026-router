#pragma once

#include <memory>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include "types.hpp"
#include "config.hpp"
#include "paged_grid.hpp"

using namespace std;

class PathFinderRouter {
private:
    Input& in;
    int width, height, layers;
    long long total_nodes;

    unique_ptr<PagedGrid<int>> occupancy;
    unique_ptr<PagedGrid<double>> history_cost;
    unique_ptr<PagedGrid<int>> is_pin_net_id;

    double history_factor = INITIAL_HISTORY_FACTOR;
    double present_factor = INITIAL_PRESENT_FACTOR;

    /** A* 启发式：曼哈顿距离 + 层差×权重。 */
    double heuristic(int x1, int y1, int l1, int x2, int y2, int l2);
    /** 将 (x,y,l) 编码为本路由器网格下的 Key。 */
    Key getKey(int x, int y, int l);
    /** 将 Key 解码为 (x, y, layer)。 */
    Coord getCoord(Key k);
    /** 对单条线网做 A* 布线并更新 occupancy；多引脚时先 MST 排序再逐段连接。 */
    bool routeNetAStar(Net& net, int net_id);
    /** 从 occupancy 中移除该线网占用，并清空其 steps/occupied_nodes。 */
    void ripUpNet(Net& net);

    /** 对多引脚线网用 Prim MST 确定连接顺序，以减小总线长。 */
    vector<Bump> mstOrderBumps(const vector<Bump>& bumps);

    /** 轻量级冲突消除：仅扫描已布线单元，对冲突线网用高代价重布，直至无冲突或达迭代上限。 */
    void runCleanup(int max_cleanup_iters = MAX_CLEANUP_ITERATIONS);

public:
    /** 根据 Input 初始化网格尺寸与 PagedGrid，并填写 is_pin_net_id。 */
    PathFinderRouter(Input& input);
    /** 执行 PathFinder 主循环（rip-up & reroute + 历史代价），未收敛时再执行 Cleanup。 */
    void run(int max_iterations = MAX_PATHFINDER_ITERATIONS);
};
