# Substrate Router

Coarse-grid PathFinder router for the LEC substrate routing assignment. Implements PathFinder iteration (rip-up & reroute), A* per-net routing, MST ordering for multi-pin nets, PagedGrid sparse grid, and lightweight Cleanup; main loop is single-threaded, with parameters in `config.hpp`.

## Quick Start

### Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build .
```

### Run

```bash
# From build directory
./router <grid_layout.json> <netlist.json> <output.json>

# Example (C2IO1)
./router ../benchmark/C2IO1_grid_layout.json ../benchmark/C2IO1_netlist.json ../benchmark/output.C2IO1.json
```

The program prints parse info, per-iteration overuse, convergence message, and stats (wirelength, grid_overuse, runtime).

### Evaluate

```bash
# From router directory
python benchmark/evaluator.py benchmark/C2IO1_grid_layout.json benchmark/C2IO1_netlist.json benchmark/output.C2IO1.json

# View report
tail -20 report.json
```

### Full Example

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

## Project Structure

- **src/main.cpp** – Entry: read files, parseInput, PathFinderRouter::run, writeOutput; prints runtime.
- **src/include/** – Headers and implementation.
  - **config.hpp** – Metal layer count, PagedGrid block size, PathFinder/Cleanup iterations and costs, A* parameters, Key encode/decode; main place to change for different cases.
  - **types.hpp** – Coord, Bump, RouteStep, Net, GridLayout, Input, PinCoordType, Key.
  - **parser.hpp**, **impl/parser.cpp** – readFile, parseInput (grid_layout + netlist → Input).
  - **pathfinder.hpp**, **impl/pathfinder.cpp** – PathFinderRouter: occupancy/history_cost/is_pin_net_id (PagedGrid), routeNetAStar, ripUpNet, mstOrderBumps, run, runCleanup.
  - **output.hpp**, **impl/output.cpp** – writeOutput: write evaluator JSON and stats (wirelength, grid_overuse, sharp_angles, etc.).
  - **paged_grid.hpp** – PagedGrid<T>: 64×64×layer blocks, on-demand allocation for large designs.
- **external/json.hpp** – nlohmann/json (single-header).
- **benchmark/** – Test cases and evaluator (C2IO1, C8IO1, C4M4; evaluator.py).
- **docs/** – Documentation and version notes (e.g. 提交文档.md, v1–v6).

## Dependencies

- C++17
- nlohmann/json (in `external/`)
