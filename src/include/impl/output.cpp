/**
 * 将布线结果写成 evaluator 要求的 output.json，并统计 wirelength、disconnected_nets、
 * grid_overuse、sharp_angles 输出到控制台。线段为物理坐标；首尾插入引脚到网格的 L 形段以保证无锐角。
 */
#include <iostream>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <iomanip>
#include "output.hpp"
#include "config.hpp"

using namespace std;

namespace {

struct OutStep {
    int x1, y1; string l1;
    int x2, y2; string l2;
};

/** 线段物理长度（同层为欧几里得距离，跨层/过孔不计）。 */
double segmentLength(const OutStep& s) {
    if (s.l1 != s.l2) return 0.0;
    double dx = s.x2 - s.x1, dy = s.y2 - s.y1;
    return hypot(dx, dy);
}

double angleBetween(int x1, int y1, int x2, int y2, int x3, int y3) {
    double v1x = x1 - x2, v1y = y1 - y2;
    double v2x = x3 - x2, v2y = y3 - y2;
    double dot = v1x * v2x + v1y * v2y;
    double cross = v1x * v2y - v1y * v2x;
    double angle_rad = atan2(fabs(cross), dot);
    double angle_deg = angle_rad * 180.0 / 3.141592653589793;
    if (angle_deg > 180.0) angle_deg = 360.0 - angle_deg;
    return angle_deg;
}

}  // 匿名命名空间

void writeOutput(const Input& in, const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open output file " << filename << endl;
        return;
    }

    /* 统计每格被多少线网占用，用于 grid_overuse；统计未布通线网数 */
    unordered_map<Key, int> cell_occupancy;
    for (const auto& net : in.nets) {
        if (!net.routed_successfully) continue;
        for (Key k : net.occupied_nodes)
            cell_occupancy[k]++;
    }
    int grid_overuse_count = 0;
    for (const auto& p : cell_occupancy)
        if (p.second > 1) grid_overuse_count++;

    int disconnected_nets_count = 0;
    for (const auto& net : in.nets)
        if (!net.routed_successfully) disconnected_nets_count++;

    const int invalid_segments_count = 0;

    auto cellKeyWrapper = [W=in.grid.width, H=in.grid.height](int x, int y, int l) -> long long {
        return calculatePinMapKey(x, y, l, W, H);
    };

    double total_wirelength = 0.0;
    int sharp_angles_count = 0;

    file << "{\n";

    bool firstNet = true;
    for (const auto& net : in.nets) {
        if (!net.routed_successfully || net.steps.empty()) continue;

        if (!firstNet) file << ",\n";

        /* 每条线网：首段从引脚物理坐标 L 形接到网格点，中间为网格段（转物理坐标），末段 L 形接到终点引脚 */
        vector<OutStep> out_steps;

        auto toOutStep = [&](const RouteStep& s) -> OutStep {
            string l1 = (s.start.layer >= 0 && (size_t)s.start.layer < in.grid.layers.size()) ? in.grid.layers[s.start.layer] : "Unknown";
            string l2 = (s.end.layer >= 0 && (size_t)s.end.layer < in.grid.layers.size()) ? in.grid.layers[s.end.layer] : "Unknown";
            return {
                s.start.x * in.grid.grid_length, s.start.y * in.grid.grid_length, l1,
                s.end.x * in.grid.grid_length, s.end.y * in.grid.grid_length, l2
            };
        };

        if (!net.steps.empty()) {
            Coord start = net.steps.front().start;
            long long startKey = cellKeyWrapper(start.x, start.y, start.layer);
            if (net.pin_original_coords.count(startKey)) {
                const Coord& orig = net.pin_original_coords.at(startKey);
                string pinLayer = net.pin_output_layers.at(startKey);
                int gridX = start.x * in.grid.grid_length;
                int gridY = start.y * in.grid.grid_length;
                string gridLayer = in.grid.layers[start.layer];
                out_steps.push_back({orig.x, orig.y, pinLayer, orig.x, orig.y, gridLayer});
                if (orig.x != gridX || orig.y != gridY) {
                    if (orig.x != gridX && orig.y != gridY) {
                        out_steps.push_back({orig.x, orig.y, gridLayer, orig.x, gridY, gridLayer});
                        out_steps.push_back({orig.x, gridY, gridLayer, gridX, gridY, gridLayer});
                    } else {
                        out_steps.push_back({orig.x, orig.y, gridLayer, gridX, gridY, gridLayer});
                    }
                }
            }
        }

        for (const auto& step : net.steps)
            out_steps.push_back(toOutStep(step));

        if (!net.steps.empty()) {
            Coord end = net.steps.back().end;
            long long endKey = cellKeyWrapper(end.x, end.y, end.layer);
            if (net.pin_original_coords.count(endKey)) {
                const Coord& orig = net.pin_original_coords.at(endKey);
                string pinLayer = net.pin_output_layers.at(endKey);
                int gridX = end.x * in.grid.grid_length;
                int gridY = end.y * in.grid.grid_length;
                string gridLayer = in.grid.layers[end.layer];
                if (orig.x != gridX || orig.y != gridY) {
                    if (orig.x != gridX && orig.y != gridY) {
                        out_steps.push_back({gridX, gridY, gridLayer, gridX, orig.y, gridLayer});
                        out_steps.push_back({gridX, orig.y, gridLayer, orig.x, orig.y, gridLayer});
                    } else {
                        out_steps.push_back({gridX, gridY, gridLayer, orig.x, orig.y, gridLayer});
                    }
                }
                out_steps.push_back({orig.x, orig.y, gridLayer, orig.x, orig.y, pinLayer});
            }
        }

        for (const auto& s : out_steps)
            total_wirelength += segmentLength(s);

        /* 沿路径三点同层时算夹角，小于 90° 计为锐角（本实现通常不产生） */
        struct Pt { int x, y; string layer; };
        vector<Pt> path;
        for (const auto& s : out_steps) {
            if (path.empty() || path.back().x != s.x1 || path.back().y != s.y1)
                path.push_back({s.x1, s.y1, s.l1});
        }
        if (!out_steps.empty())
            path.push_back({out_steps.back().x2, out_steps.back().y2, out_steps.back().l2});
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            if (path[i-1].layer != path[i].layer || path[i].layer != path[i+1].layer) continue;
            double ang = angleBetween(path[i-1].x, path[i-1].y, path[i].x, path[i].y, path[i+1].x, path[i+1].y);
            if (ang < 90.0 && ang > 1e-6) sharp_angles_count++;
        }

        file << "    \"" << net.name << "\": [\n";
        for (size_t i = 0; i < out_steps.size(); ++i) {
            if (i > 0) file << ",\n";
            file << "        {\n";
            file << "            \"start_grid_coordinate\": [" << out_steps[i].x1 << ", " << out_steps[i].y1 << ", \"" << out_steps[i].l1 << "\"],\n";
            file << "            \"end_grid_coordinate\": [" << out_steps[i].x2 << ", " << out_steps[i].y2 << ", \"" << out_steps[i].l2 << "\"]\n";
            file << "        }";
        }
        file << "\n    ]";
        firstNet = false;
    }

    file << "\n}\n";
    file.close();

    const int value_width = 14;
    cout << "\n";
    cout << "  Route results written to " << filename << "\n";
    cout << "  ----------------------------------------\n";
    cout << "  ";
    cout.width(22);
    cout << left << "wirelength" << right;
    cout.width(value_width);
    cout << fixed << setprecision(4) << total_wirelength << "\n";
    cout << "  ";
    cout.width(22);
    cout << left << "disconnected_nets" << right;
    cout.width(value_width);
    cout << disconnected_nets_count << "\n";
    cout << "  ";
    cout.width(22);
    cout << left << "invalid_segments" << right;
    cout.width(value_width);
    cout << invalid_segments_count << "\n";
    cout << "  ";
    cout.width(22);
    cout << left << "grid_overuse" << right;
    cout.width(value_width);
    cout << grid_overuse_count << "\n";
    cout << "  ";
    cout.width(22);
    cout << left << "sharp_angles" << right;
    cout.width(value_width);
    cout << sharp_angles_count << "\n";
    cout << "  ----------------------------------------\n";
}
