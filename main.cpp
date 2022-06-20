#include <string>
#include "TimeModel.h"
#include "SingleModel.h"

int main() {
    std::string Pointpath = "../PointData/d6.txt"; //点云数据
    std::string path = "../obj/SingleModel"; //SingleModel输出位置，输出准备
    std::string pathT = "../obj/TimeModel"; //TimeModel输出位置，输出准备

    SingleModel(Pointpath, path);

    TimeModel(Pointpath, pathT);
    return 0;
}