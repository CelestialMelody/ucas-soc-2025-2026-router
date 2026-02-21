#pragma once

/** 公共数据结构：Coord/Bump、RouteStep、Net、GridLayout、Input、PinCoordType；Key 为格点一维编码。 */
#include <string>
#include <vector>
#include <unordered_map>

using namespace std;

/** 网格坐标：x/y 为格点索引，layer 为金属层索引。 */
struct Coord {
    int x, y, layer;
    Coord(int x_val, int y_val, int l_val) : x(x_val), y(y_val), layer(l_val) {}
    Coord() : x(0), y(0), layer(0) {}

    bool operator==(const Coord& other) const {
        return x == other.x && y == other.y && layer == other.layer;
    }
};

/** 引脚：继承 Coord，并带输出用层名（如 "Bottom"/"Top"）。 */
struct Bump : public Coord {
    using Coord::Coord;
    Bump(const Coord& c, const string& type) : Coord(c.x, c.y, c.layer), output_layer_name(type) {}
    string output_layer_name;
};

/** 一段布线：起点与终点的网格坐标。 */
struct RouteStep {
    Coord start;
    Coord end;
};

using Key = long long;

/** 线网：名称、类型、引脚列表、布线结果；pin_output_layers/pin_original_coords 用于输出时写引脚 L 形段；occupied_nodes 为占用的格点 Key。 */
struct Net {
    string name;
    int net_type = 0;
    vector<Bump> bumps;
    vector<RouteStep> steps;
    unordered_map<long long, string> pin_output_layers;
    bool routed_successfully = false;
    unordered_map<long long, Coord> pin_original_coords;
    vector<Key> occupied_nodes;
};

/** 网格布局：宽高（格点数）、grid_length（物理/网格换算）、层数与层名列表。 */
struct GridLayout {
    int width, height;
    int grid_length;
    int layer_count;
    vector<string> layers;
};

/** 解析结果：网格 + 线网列表，作为 PathFinder 与输出的输入。 */
struct Input {
    GridLayout grid;
    vector<Net> nets;
};

/** 解析时使用的引脚信息：网格坐标、输出层名、物理坐标。 */
struct PinCoordType {
    Coord coord;
    string type_name;
    int original_x;
    int original_y;
};
