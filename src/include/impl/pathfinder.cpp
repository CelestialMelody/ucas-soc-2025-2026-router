/**
 * PathFinder 布线核心：rip-up & reroute、A* 单线网、MST 多引脚顺序、轻量 Cleanup。
 * 网格状态使用 PagedGrid（occupancy、history_cost、is_pin_net_id）；A* 的 visited 亦为 PagedGrid 以省内存。
 */
#include <iostream>
#include <queue>
#include <algorithm>
#include <functional>
#include <limits>
#include <iomanip>
#include <climits>
#include "pathfinder.hpp"
#include "config.hpp"

/* 启发式：曼哈顿距离 + 层差×LAYER_HEURISTIC_WEIGHT，与 config 一致 */
double PathFinderRouter::heuristic(int x1, int y1, int l1, int x2, int y2, int l2) {
    return abs(x1 - x2) + abs(y1 - y2) + abs(l1 - l2) * LAYER_HEURISTIC_WEIGHT;
}

/** 网格格点 (x,y,l) → 一维 Key，与 calculateCellKey 约定一致 */
Key PathFinderRouter::getKey(int x, int y, int l) {
    return (long long)(l * height + y) * width + x;
}

/** Key → (x, y, layer) */
Coord PathFinderRouter::getCoord(Key k) {
    long long wh = (long long)width * height;
    int l = k / wh;
    long long rem = k % wh;
    int y = rem / width;
    int x = rem % width;
    return Coord(x, y, l);
}

/* 多引脚线网按 MST（Prim）确定连接顺序以减小总线长 */
vector<Bump> PathFinderRouter::mstOrderBumps(const vector<Bump>& bumps) {
    if (bumps.size() <= 2) return bumps;

    int n = (int)bumps.size();
    vector<int> order;
    order.reserve(n);
    order.push_back(0);
    vector<bool> used(n, false);
    used[0] = true;

    while ((int)order.size() < n) {
        int best_j = -1;
        int best_d = INT_MAX;
        for (int i : order) {
            for (int j = 0; j < n; ++j) {
                if (used[j]) continue;
                int d = abs(bumps[i].x - bumps[j].x) + abs(bumps[i].y - bumps[j].y)
                      + abs(bumps[i].layer - bumps[j].layer) * (int)LAYER_HEURISTIC_WEIGHT;
                if (d < best_d) {
                    best_d = d;
                    best_j = j;
                }
            }
        }
        if (best_j < 0) break;
        used[best_j] = true;
        order.push_back(best_j);
    }

    vector<Bump> result;
    result.reserve(n);
    for (int i : order) result.push_back(bumps[i]);
    return result;
}

/**
 * 单线网 A* 布线。多引脚时先 MST 排序再逐段 (start,end) 做 A*；每段用 6 邻域扩展，代价含 base/history/occupancy/net_type。
 * 路径回填到 new_steps/new_occupied，最后去重 occupied、写回 net 并更新 occupancy。
 */
bool PathFinderRouter::routeNetAStar(Net& net, int net_id) {
    vector<RouteStep> new_steps;
    vector<Key> new_occupied;

    net.steps.clear();
    net.occupied_nodes.clear();

    struct NodeState {
        double g;
        Key parent;
        bool visited;

        NodeState() : g(numeric_limits<double>::max()), parent(-1), visited(false) {}
        NodeState(double) : g(numeric_limits<double>::max()), parent(-1), visited(false) {}
    };

    static PagedGrid<NodeState> visited(width, height, NodeState(0.0));
    visited.freeAll();

    /* 多引脚时使用 MST 排序。 */
    const vector<Bump>& raw_bumps = net.bumps;
    vector<Bump> ordered_bumps = (raw_bumps.size() > 2) ? mstOrderBumps(raw_bumps) : raw_bumps;

    for (size_t i = 0; i < ordered_bumps.size() - 1; ++i) {
        Coord start = ordered_bumps[i];
        Coord end = ordered_bumps[i+1];

        if (start.layer < 0 || end.layer < 0) return false;

        visited.freeAll();

        using PQElement = pair<double, Key>;
        priority_queue<PQElement, vector<PQElement>, greater<PQElement>> pq;

        Key startKey = getKey(start.x, start.y, start.layer);
        Key endKey = getKey(end.x, end.y, end.layer);

        NodeState* startNode = visited.getPtr(start.x, start.y, start.layer, true);
        startNode->g = 0;
        startNode->parent = -1;
        startNode->visited = true;

        pq.push({1.2 * heuristic(start.x, start.y, start.layer, end.x, end.y, end.layer), startKey});

        bool found = false;

        /* 6 邻域：左右、上下、上下层；水平/竖直 1.0，过孔 5.0 */
        int dx[] = {1, -1, 0, 0, 0, 0};
        int dy[] = {0, 0, 1, -1, 0, 0};
        int dl[] = {0, 0, 0, 0, 1, -1};
        double base_costs[] = {1.0, 1.0, 1.0, 1.0, 5.0, 5.0};

        while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();
            double f = top.first;
            Key u = top.second;

            if (u == endKey) {
                found = true;
                break;
            }

            Coord uc = getCoord(u);
            NodeState* uNode = visited.getPtr(uc.x, uc.y, uc.layer, true);

            if (uNode->g + 1.2 * heuristic(uc.x, uc.y, uc.layer, end.x, end.y, end.layer) < f - 1e-9) continue;

            for (int k = 0; k < 6; ++k) {
                int nx = uc.x + dx[k];
                int ny = uc.y + dy[k];
                int nl = uc.layer + dl[k];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height || nl < 0 || nl >= layers) continue;

                /* 非本线网的引脚格不可经过，除非是当前目标点 */
                int pin_net = is_pin_net_id->get(nx, ny, nl);
                if (pin_net != -1 && pin_net != net_id) {
                    if (nx != end.x || ny != end.y || nl != end.layer) continue;
                }

                double step_cost = base_costs[k];

                step_cost += history_cost->get(nx, ny, nl) * history_factor;

                int occ = occupancy->get(nx, ny, nl);
                if (occ >= 1) {
                    step_cost += present_factor * (occ + 1);
                }

                /* net_type==1 在 M1/M2 上加惩罚，引导走高层 */
                if (net.net_type == 1 && (nl == 0 || nl == 1)) {
                    step_cost += 5.0;
                }

                double new_g = uNode->g + step_cost;

                NodeState* vNode = visited.getPtr(nx, ny, nl, true);

                if (!vNode->visited || new_g < vNode->g) {
                    vNode->visited = true;
                    vNode->g = new_g;
                    vNode->parent = u;
                    double new_f = new_g + 1.2 * heuristic(nx, ny, nl, end.x, end.y, end.layer);
                    pq.push({new_f, getKey(nx, ny, nl)});
                }
            }
        }

        if (!found) {
            return false;
        }

        vector<Coord> path_segment;
        Key curr = endKey;
        while (curr != -1) {
            path_segment.push_back(getCoord(curr));
            Coord c = getCoord(curr);
            NodeState* node = visited.getPtr(c.x, c.y, c.layer, false);
            curr = node ? node->parent : -1;
        }
        reverse(path_segment.begin(), path_segment.end());

        /* 将路径转为 steps 并收集占用格点 */
        for (size_t j = 0; j < path_segment.size() - 1; ++j) {
            new_steps.push_back({path_segment[j], path_segment[j+1]});
            Key k1 = getKey(path_segment[j].x, path_segment[j].y, path_segment[j].layer);
            Key k2 = getKey(path_segment[j+1].x, path_segment[j+1].y, path_segment[j+1].layer);
            new_occupied.push_back(k1);
            new_occupied.push_back(k2);
        }
    }

    /* 去重 occupied，写回 net 并更新全局 occupancy */
    sort(new_occupied.begin(), new_occupied.end());
    new_occupied.erase(unique(new_occupied.begin(), new_occupied.end()), new_occupied.end());

    net.steps = new_steps;
    net.occupied_nodes = new_occupied;

    for (Key k : net.occupied_nodes) {
        Coord c = getCoord(k);
        occupancy->add(c.x, c.y, c.layer, 1);
    }

    net.routed_successfully = true;
    return true;
}

/** 从 occupancy 中减去该线网占用，清空 steps/occupied_nodes，标记未布通 */
void PathFinderRouter::ripUpNet(Net& net) {
    if (!net.routed_successfully) return;
    for (Key k : net.occupied_nodes) {
        Coord c = getCoord(k);
        occupancy->add(c.x, c.y, c.layer, -1);
    }
    net.occupied_nodes.clear();
    net.steps.clear();
    net.routed_successfully = false;
}

/* 轻量 Cleanup：仅扫描已布线单元（O(布线长度)），不扫全图；通过 cell→nets 反向索引找冲突线网，再用高代价重布。 */
void PathFinderRouter::runCleanup(int max_cleanup_iters) {
    /* 仅从已布线线网构建 cell→nets 反向索引 */
    unordered_map<Key, vector<int>> cell_to_nets;
    for (size_t i = 0; i < in.nets.size(); ++i) {
        if (!in.nets[i].routed_successfully) continue;
        for (Key k : in.nets[i].occupied_nodes) {
            cell_to_nets[k].push_back((int)i);
        }
    }

    /* 仅扫描已布线单元统计当前 overuse 数量 */
    auto countOveruse = [&]() -> int {
        unordered_set<Key> overused;
        for (const auto& net : in.nets) {
            if (!net.routed_successfully) continue;
            for (Key k : net.occupied_nodes) {
                Coord c = getCoord(k);
                if (occupancy->get(c.x, c.y, c.layer) > 1)
                    overused.insert(k);
            }
        }
        return (int)overused.size();
    };

    int best_overuse = countOveruse();
    if (best_overuse == 0) return;

    cout << "Starting cleanup pass (overuse=" << best_overuse << ")..." << endl;

    double saved_hf = history_factor;
    double saved_pf = present_factor;
    history_factor = CLEANUP_HISTORY_FACTOR;
    present_factor = CLEANUP_PRESENT_FACTOR;

    for (int cleanup = 0; cleanup < max_cleanup_iters && best_overuse > 0; ++cleanup) {
        /* 找出过载格（仅扫描已布线单元）。 */
        unordered_set<Key> overused_cells;
        for (const auto& net : in.nets) {
            if (!net.routed_successfully) continue;
            for (Key k : net.occupied_nodes) {
                Coord c = getCoord(k);
                if (occupancy->get(c.x, c.y, c.layer) > 1)
                    overused_cells.insert(k);
            }
        }
        if (overused_cells.empty()) break;

        /* 通过反向索引找出涉及冲突的线网及冲突数 */
        unordered_map<int, int> conflict_count;
        for (Key k : overused_cells) {
            auto it = cell_to_nets.find(k);
            if (it == cell_to_nets.end()) continue;
            for (int n : it->second)
                conflict_count[n]++;
        }

        /* 按冲突数从多到少排序，优先重布冲突多的线网 */
        vector<int> candidates;
        candidates.reserve(conflict_count.size());
        for (auto& p : conflict_count) candidates.push_back(p.first);
        sort(candidates.begin(), candidates.end(), [&](int a, int b) {
            return conflict_count[a] > conflict_count[b];
        });

        bool improved = false;
        for (int net_idx : candidates) {
            Net& net = in.nets[net_idx];
            vector<RouteStep> old_steps = net.steps;
            vector<Key> old_occupied = net.occupied_nodes;

            /* 从索引中移除该线网 */
            for (Key k : net.occupied_nodes) {
                auto& v = cell_to_nets[k];
                v.erase(remove(v.begin(), v.end(), net_idx), v.end());
            }

            ripUpNet(net);

            if (routeNetAStar(net, net_idx)) {
                /* 将新路径加入索引 */
                for (Key k : net.occupied_nodes) {
                    cell_to_nets[k].push_back(net_idx);
                }

                int new_overuse = countOveruse();
                if (new_overuse <= best_overuse) {
                    if (new_overuse < best_overuse) improved = true;
                    best_overuse = new_overuse;
                    cout << "Cleanup " << cleanup + 1 << ": rerouted net " << net_idx
                         << ", overuse=" << best_overuse << endl;
                    break;
                }

                /* 重布未改善：恢复旧路径 */
                for (Key k : net.occupied_nodes) {
                    auto& v = cell_to_nets[k];
                    v.erase(remove(v.begin(), v.end(), net_idx), v.end());
                }
                ripUpNet(net);
                net.steps = old_steps;
                net.occupied_nodes = old_occupied;
                net.routed_successfully = true;
                for (Key k : net.occupied_nodes) {
                    Coord c = getCoord(k);
                    occupancy->add(c.x, c.y, c.layer, 1);
                    cell_to_nets[k].push_back(net_idx);
                }
            } else {
                /* 布线失败：恢复旧路径 */
                net.steps = old_steps;
                net.occupied_nodes = old_occupied;
                net.routed_successfully = true;
                for (Key k : net.occupied_nodes) {
                    Coord c = getCoord(k);
                    occupancy->add(c.x, c.y, c.layer, 1);
                    cell_to_nets[k].push_back(net_idx);
                }
            }
        }

        if (!improved && cleanup > 10) break;
    }

    history_factor = saved_hf;
    present_factor = saved_pf;

    if (best_overuse == 0) {
        cout << "Cleanup: all conflicts resolved!" << endl;
    } else {
        cout << "Cleanup done, remaining overuse=" << best_overuse << endl;
    }
}

/** 初始化网格尺寸与三张 PagedGrid；is_pin_net_id 记录每格被哪条线网引脚占用（-1 表示非引脚）。 */
PathFinderRouter::PathFinderRouter(Input& input) : in(input) {
    width = in.grid.width;
    height = in.grid.height;
    layers = METAL_LAYER_COUNT;
    total_nodes = (long long)width * height * layers;

    occupancy = make_unique<PagedGrid<int>>(width, height, 0);
    history_cost = make_unique<PagedGrid<double>>(width, height, 0.0);
    is_pin_net_id = make_unique<PagedGrid<int>>(width, height, -1);

    for (size_t i = 0; i < in.nets.size(); ++i) {
        for (const auto& b : in.nets[i].bumps) {
            is_pin_net_id->set(b.x, b.y, b.layer, (int)i);
        }
    }
}

void PathFinderRouter::run(int max_iterations) {
    cout << "Starting PathFinder Routing (Single-Threaded, Paged Grid)..." << endl;

    history_factor = 0.5;
    present_factor = 0.5;

    vector<int> net_order(in.nets.size());
    for (size_t i = 0; i < in.nets.size(); ++i) net_order[i] = i;

    bool converged = false;

    for (int iter = 0; iter < max_iterations; ++iter) {

        /* 每轮打乱线网顺序并提高 history/present，加强收敛 */
        if (iter > 0) {
            shuffle(net_order.begin(), net_order.end(), std::default_random_engine(iter));
            history_factor += 0.5;
            present_factor *= 1.5;
        }

        int routed_cnt = 0;

        for (size_t idx = 0; idx < net_order.size(); ++idx) {
            int net_idx = net_order[idx];
            Net& net = in.nets[net_idx];

            ripUpNet(net);

            if (routeNetAStar(net, net_idx)) {
                routed_cnt++;
            }
        }

        /* 统计本轮回溯格，并对其加 history_cost，下一轮 A* 会绕开 */
        unordered_set<Key> overused_cells;
        for (const auto& net : in.nets) {
            if (net.routed_successfully) {
                for (Key k : net.occupied_nodes) {
                    Coord c = getCoord(k);
                    int occ = occupancy->get(c.x, c.y, c.layer);

                    if (occ > 1) {
                        overused_cells.insert(k);
                        history_cost->add(c.x, c.y, c.layer, history_factor);
                    }
                }
            }
        }
        int overuse_count = (int)overused_cells.size();

        cout << "-- Iteration ";
        cout.width(3);
        cout << left << iter + 1 << right << " Done. ";
        cout.width(6);
        cout << left << "overuse = " << right;
        cout.width(6);
        cout << overuse_count << endl;

        /* 若本轮全部布通，检查是否仍有冲突；无冲突则收敛退出 */
        if ((size_t)routed_cnt == in.nets.size()) {
            bool has_conflict = false;
            for (const auto& net : in.nets) {
                for (Key k : net.occupied_nodes) {
                    Coord c = getCoord(k);
                    if (occupancy->get(c.x, c.y, c.layer) > 1) {
                        has_conflict = true;
                        break;
                    }
                }
                if (has_conflict) break;
            }

            if (!has_conflict) {
                cout << "Success! No conflicts." << endl;
                converged = true;
                break;
            }
        }
    }

    /* PathFinder 未完全收敛时，执行轻量 Cleanup */
    if (!converged) {
        runCleanup(MAX_CLEANUP_ITERATIONS);
    }
}
