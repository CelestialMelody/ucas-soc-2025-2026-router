/**
 * 基板布线程序入口。
 * 读入 grid_layout.json、netlist.json，运行 PathFinder 布线，写出 output.json 并打印统计与运行时间。
 */
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include "config.hpp"
#include "parser.hpp"
#include "pathfinder.hpp"
#include "output.hpp"

using namespace std;

/** 程序入口：读入 grid_layout + netlist，运行 PathFinder 布线，写出 output JSON。 */
int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " [grid_layout.json path] [netlist.json path] [output.json path]" << endl;
        return 1;
    }

    string grid_file = argv[1];
    string net_file = argv[2];
    string output_file = argv[3];

    try {
        auto t_start = chrono::steady_clock::now();

        string grid_content = readFile(grid_file);
        string net_content = readFile(net_file);

        Input input = parseInput(grid_content, net_content);

        /* 层数至少为 1，否则无有效格点、线网无法布通。 */
        if (input.grid.layer_count < 1) {
            cerr << "Error: METAL_LAYER_COUNT must be >= 1 (got " << input.grid.layer_count
                 << " layers). No valid grid cells for routing; nets will never route and overuse stays 0 without converging." << endl;
            return 1;
        }

        PathFinderRouter router(input);
        router.run(MAX_PATHFINDER_ITERATIONS);  /* 未收敛时内部会调用 Cleanup */
        writeOutput(input, output_file);

        auto t_end = chrono::steady_clock::now();
        double sec = chrono::duration<double>(t_end - t_start).count();
        double min = sec / 60.0;
        double hr = sec / 3600.0;
        cout << fixed << setprecision(2);
        const int time_width = TIME_WIDTH;
        cout << "  Total time (s):   " << right << setw(time_width) << sec << "\n";
        cout << "  Total time (min): " << right << setw(time_width) << min << "\n";
        cout << "  Total time (h):   " << right << setw(time_width) << hr << "\n";

    } catch (const exception& e) {
        cerr << "An error occurred: " << e.what() << endl;
        return 1;
    }

    return 0;
}
