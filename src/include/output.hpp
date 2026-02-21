#pragma once

#include <string>
#include "types.hpp"

using namespace std;

/** 将布线结果按赛题要求写成 JSON 文件，并统计 wirelength / overuse / sharp_angles 等输出到控制台。 */
void writeOutput(const Input& in, const string& filename = "result.json");
