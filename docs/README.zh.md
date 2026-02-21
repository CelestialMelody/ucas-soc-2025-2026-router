# 基板布线器

用于 LEC 基板布线作业的粗网格 PathFinder 布线器。实现 PathFinder 迭代（rip-up & reroute）、A* 单线网布线、MST 多引脚连接顺序、PagedGrid 稀疏网格与轻量 Cleanup；主循环单线程，参数集中在 `config.hpp`。

## 快速开始

### 编译

```bash
mkdir -p build && cd build
cmake ..
cmake --build .
```

### 运行

```bash
# 从 build 目录运行
./router <grid_layout.json> <netlist.json> <output.json>

# 示例（C2IO1）
./router ../benchmark/C2IO1_grid_layout.json ../benchmark/C2IO1_netlist.json ../benchmark/output.C2IO1.json
```

运行后会打印解析信息、每轮 overuse、收敛提示及 wirelength / grid_overuse / 运行时间等统计。

### 评测

```bash
# 在 router 目录下运行评测脚本
python benchmark/evaluator.py benchmark/C2IO1_grid_layout.json benchmark/C2IO1_netlist.json benchmark/output.C2IO1.json

# 查看报告
tail -20 report.json
```

### 完整示例

```bash
cd router
mkdir -p build && cd build
cmake ..
cmake --build .

./router ../benchmark/C2IO1_grid_layout.json ../benchmark/C2IO1_netlist.json ../benchmark/output.C2IO1.json

cd ..
python benchmark/evaluator.py benchmark/C2IO1_grid_layout.json benchmark/C2IO1_netlist.json benchmark/output.C2IO1.json
tail -20 report.json
```

## 项目结构

- **src/main.cpp** – 入口：读文件、parseInput、PathFinderRouter::run、writeOutput，并输出运行时间。
- **src/include/** – 头文件与实现。
  - **config.hpp** – 金属层数、PagedGrid 块大小、PathFinder/Cleanup 迭代与代价、A* 参数、Key 编解码；改用例时主要改此文件。
  - **types.hpp** – Coord、Bump、RouteStep、Net、GridLayout、Input、PinCoordType、Key。
  - **parser.hpp**、**impl/parser.cpp** – readFile、parseInput（grid_layout + netlist → Input）。
  - **pathfinder.hpp**、**impl/pathfinder.cpp** – PathFinderRouter：occupancy/history_cost/is_pin_net_id（PagedGrid）、routeNetAStar、ripUpNet、mstOrderBumps、run、runCleanup。
  - **output.hpp**、**impl/output.cpp** – writeOutput：写出 evaluator 所需 JSON，并统计 wirelength、grid_overuse、sharp_angles 等。
  - **paged_grid.hpp** – PagedGrid<T>：64×64×层 分块、按需分配，用于大图省内存。
- **external/json.hpp** – nlohmann/json（单头文件）。
- **benchmark/** – 测试用例与评测脚本（C2IO1、C8IO1、C4M4 等；evaluator.py）。
- **docs/** – 说明与版本记录（如 提交文档.md、v1～v6 脉络）。

## 依赖

- C++17
- nlohmann/json（已放在 `external/`）
