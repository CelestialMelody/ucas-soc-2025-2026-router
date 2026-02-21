#pragma once

/** 读文件与解析 grid_layout / netlist JSON，产出 Input（网格、引脚表、线网列表）。 */
#include <string>
#include "types.hpp"

using namespace std;

/** 将整个文件读入并返回内容字符串，打开失败时抛异常。 */
string readFile(const string& filename);

/** 解析 grid_layout 与 netlist 的 JSON 字符串，填充网格尺寸、引脚表、线网列表，返回 Input。 */
Input parseInput(const string& glStr, const string& nlStr);
