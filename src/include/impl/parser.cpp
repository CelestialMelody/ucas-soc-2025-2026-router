/**
 * 解析 grid_layout.json 与 netlist.json，填充网格尺寸、层名、引脚表、线网列表。
 * 引脚物理坐标经 grid_length 换算为网格坐标；线网通过引脚名与 bump_info_map 关联。
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <unordered_set>
#include "parser.hpp"
#include "json.hpp"
#include "config.hpp"

using namespace std;

string readFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Error: Could not open file " + filename);
    }
    stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

Input parseInput(const string& glStr, const string& nlStr) {
    Input in;
    nlohmann::json gridJson = nlohmann::json::parse(glStr);
    nlohmann::json netlistJson = nlohmann::json::parse(nlStr);

    unordered_map<string, PinCoordType> bump_info_map;
    unordered_set<long long> used_pin_keys;

    /* 层名：M1, M2, ..., Mn，与 config 中 METAL_LAYER_COUNT 一致 */
    in.grid.layers.clear();
    in.grid.layers.reserve(METAL_LAYER_COUNT);
    for (int i = 1; i <= METAL_LAYER_COUNT; ++i)
        in.grid.layers.push_back("M" + std::to_string(i));
    in.grid.layer_count = METAL_LAYER_COUNT;

    int grid_len = 5;
    auto grid_info = gridJson.value("grid_info", nlohmann::json::object());
    if (!grid_info.empty()) {
        grid_len = grid_info.value("grid_length", 5);
        int max_width = grid_info.value("grid_max_width", 4560);
        int max_height = grid_info.value("grid_max_height", 3840);
        in.grid.width = max_width / grid_len;
        in.grid.height = max_height / grid_len;
        in.grid.grid_length = grid_len;
    } else {
        in.grid.width = 912;
        in.grid.height = 768;
        in.grid.grid_length = 5;
    }

    /* 解析底层（C4）引脚：c4_name, grid_coord_x/y → 网格坐标，层 0 */
    auto bottom_layer = gridJson.value("bottom_layer", nlohmann::json::array());
    for (const auto& pin_entry : bottom_layer) {
        if (!pin_entry.is_object()) continue;
        string name = pin_entry.value("c4_name", "");
        int x = pin_entry.value("grid_coord_x", 0);
        int y = pin_entry.value("grid_coord_y", 0);
        int grid_x = (int)lround((double)x / grid_len);
        int grid_y = (int)lround((double)y / grid_len);

        if (!name.empty() && grid_x >= 0 && grid_x < in.grid.width && grid_y >= 0 && grid_y < in.grid.height) {
            long long key = calculatePinMapKey(grid_x, grid_y, C4_METAL_LAYER_IDX, in.grid.width, in.grid.height);
            if (used_pin_keys.count(key)) {
                cout << "WARNING: Pin collision at " << grid_x << "," << grid_y << " layer " << C4_METAL_LAYER_IDX << " (" << name << ")" << endl;
            }
            used_pin_keys.insert(key);
            bump_info_map[name] = PinCoordType{Coord(grid_x, grid_y, C4_METAL_LAYER_IDX), "Bottom", x, y};
        }
    }

    /* 解析顶层（Chiplet）引脚：bump_name, grid_coord_x/y → 网格坐标，顶层金属层 */
    auto top_layer = gridJson.value("top_layer", nlohmann::json::array());
    for (const auto& pin_entry : top_layer) {
        if (!pin_entry.is_object()) continue;
        string name = pin_entry.value("bump_name", "");
        int x = pin_entry.value("grid_coord_x", 0);
        int y = pin_entry.value("grid_coord_y", 0);
        int grid_x = (int)lround((double)x / grid_len);
        int grid_y = (int)lround((double)y / grid_len);

        if (!name.empty() && grid_x >= 0 && grid_x < in.grid.width && grid_y >= 0 && grid_y < in.grid.height) {
            long long key = calculatePinMapKey(grid_x, grid_y, CHIPLET_METAL_LAYER_IDX, in.grid.width, in.grid.height);
            if (used_pin_keys.count(key)) {
                cout << "WARNING: Pin collision at " << grid_x << "," << grid_y << " layer " << CHIPLET_METAL_LAYER_IDX << " (" << name << ")" << endl;
            }
            used_pin_keys.insert(key);
            bump_info_map[name] = PinCoordType{Coord(grid_x, grid_y, CHIPLET_METAL_LAYER_IDX), "Top", x, y};
        }
    }
    cout << "Loaded " << bump_info_map.size() << " Pin coordinates from grid_layout.json." << endl;

    /* 解析线网：net_name, net_type, pins/bumps；用引脚名查 bump_info_map 填 bumps、pin_output_layers、pin_original_coords */
    auto netsArray = netlistJson.value("nets", nlohmann::json::array());
    for (const auto& ne : netsArray) {
        Net net;
        net.name = ne.value("net_name", "net_unknown");
        net.net_type = ne.value("net_type", 0);

        auto pins = ne.contains("pins") ? ne["pins"] : ne.value("bumps", nlohmann::json::array());
        for (const auto& p : pins) {
            string bump_name = p.value("pin_name", "");
            if (bump_name.empty()) bump_name = p.value("bump_name", "");

            if (!bump_name.empty() && bump_info_map.count(bump_name)) {
                const auto& pin_info = bump_info_map.at(bump_name);
                net.bumps.push_back(Bump(pin_info.coord, pin_info.type_name));

                long long pin_key = calculatePinMapKey(pin_info.coord.x, pin_info.coord.y, pin_info.coord.layer, in.grid.width, in.grid.height);
                net.pin_output_layers[pin_key] = pin_info.type_name;
                net.pin_original_coords[pin_key] = Coord(pin_info.original_x, pin_info.original_y, 0);
            }
        }

        if (net.bumps.size() >= 2) {
            in.nets.push_back(move(net));
        }
    }
    cout << "Parsed " << in.nets.size() << " nets and grid size " << in.grid.width << "x" << in.grid.height << " with " << in.grid.layer_count << " layers." << endl;
    return in;
}
