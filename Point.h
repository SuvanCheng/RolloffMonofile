//
// Created by Suvan Cheng on 2022/6/20.
//

#ifndef ROLLOFFMONOFILE_POINT_H
#define ROLLOFFMONOFILE_POINT_H

#include <iostream>

#include <iostream>
#include <stdio.h>

#ifdef WIN32
#include <io.h>
#else
#if (defined __ARM__) || (defined __APPLE__)

#include <sys/uio.h>

#else
#include <sys/io.h>
#endif
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cstring>
#include <string>
#include <chrono>
#include <cstdio>
#include <math.h>
#include <dirent.h>

//#pragma warning(disable:4996)
using namespace std;//double比较精度问题
using std::stringstream;
#define EPS 1e-6

vector<double> xx, yy, zz, tt;
//读取点云文件
void readFile(string Pointpath){
    cout << "读取点云文件：" << Pointpath << endl;
    cout << "..." << endl;
    ifstream file(Pointpath);

    string lineStr;
    while (getline(file, lineStr)) {
        stringstream ss(lineStr);
        string out;
        ss >> out;
        xx.push_back(std::stod(out));
        ss >> out;
        yy.push_back(std::stod(out));
        ss >> out;
        zz.push_back(std::stod(out));
        ss >> out;
        tt.push_back(std::stod(out));
    }
    cout << "读取点云文件完毕：" << endl;
    cout << "X：" << xx.size() << endl;
    cout << "Y：" << yy.size() << endl;
    cout << "Z：" << zz.size() << endl;
    cout << "T：" << tt.size() << endl;
}

//点结构
class Point {
public:
    double x, y, z, time;
    int num;

    Point(double x = 0, double y = 0, double z = 0, double time = 0, int num = 0) : x(x), y(y), z(z), time(time),
                                                                                    num(num) {}

    Point operator+(Point a) { return Point(a.x + x, a.y + y); }

    Point operator-(Point a) { return Point(x - a.x, y - a.y); }

    bool operator<(const Point &a) const {
        if (x == a.x) return y < a.y;
        return x < a.x;
    }
};

//面结构（三点构成面）
struct Tri {
    int a;
    int b;
    int c;
};

//时间和模型绑定结构体
struct num_day {
    int num_dd;
    double day;
};

//实层结构（记录相连实面）
struct floatz {
    double under;
    double up;
};

typedef Point Vector;

//计算凸包前置函数
double cross(Vector a, Vector b) {
    return a.x * b.y - a.y * b.x;
}

double dot(Vector a, Vector b) {
    return a.x * b.x + a.y * b.y;
}

//判断连线是否组成凸包
bool isclock(Point p0, Point p1, Point p2) {
    Vector a = p1 - p0;
    Vector b = p2 - p0;
    if (cross(a, b) < -EPS) return true;
    return false;
}

double getDistance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

typedef vector<Point> Polygo;

//计算凸包点
Polygo andrewScan(Polygo s) {
    Polygo u, l;
    if (s.size() < 3) return s;
    sort(s.begin(), s.end());
    u.push_back(s[0]);
    u.push_back(s[1]);
    l.push_back(s[s.size() - 1]);
    l.push_back(s[s.size() - 2]);
    //printf("l[n-2]:%.2f %.2f\nl[n-1]:%.2f %.2f\n", l[l.size() - 2].x, l[l.size() - 2].y, l[l.size() - 1].x, l[l.size() - 1].y);
    for (int i = 2; i < s.size(); i++) {
        for (int n = u.size(); n >= 2 && isclock(u[n - (int) 2], u[n - (int) 1], s[i]) != true; n--) {
            //cout << u[n - 2].x << ' ' << u[n - 2].y << '\n' << u[n - 1].x << u[n - 1].y << endl;
            u.pop_back();
        }
        u.push_back(s[i]);
    }
    for (int i = s.size() - 3; i >= 0; i--) {
        //cout << i << endl;
        for (int n = l.size(); n >= 2 && isclock(l[n - (int) 2], l[n - (int) 1], s[i]) != true; n--) {
            //cout << i << endl;
            // printf("del:\nl[n-2]:%.2f %.2f\nl[n-1]:%.2f %.2f\n", l[n - 2].x, l[n - 2].y, l[n - 1].x, l[n - 1].y);

            l.pop_back();
        }

        l.push_back(s[i]);
    }
    //    for(auto &p : u) printf("%.2f %.2f\n",p.x,p.y);
    //    printf("yes\n");
    //    for(auto &p : l) printf("%.2f %.2f\n",p.x,p.y);

    for (int i = 1; i < u.size() - 1; i++) l.push_back(u[i]);
    return l;
}

//排序算法
bool zsortup(const Point &s1, const Point &s2) {
    return s1.z < s2.z;
}

bool zsortdown(const Point &s1, const Point &s2) {
    return s1.z > s2.z;
}

bool xsortup(const Point &s1, const Point &s2) {
    return s1.x < s2.x;
}

bool xsortdown(const Point &s1, const Point &s2) {
    return s1.x > s2.x;
}

bool ysortup(const Point &s1, const Point &s2) {
    return s1.y < s2.y;
}

bool ysortdown(const Point &s1, const Point &s2) {
    return s1.y > s2.y;
}

bool tsortup(const Point &s1, const Point &s2) {
    return s1.time < s2.time;
}

bool fsortup(const floatz &s1, const floatz &s2) {
    return s1.under < s2.under;
}

//清空少点面
void
clearempty(Polygo &fileroot, vector<double> &kongdown, int &time_b, int &time_r, int lim, Polygo &file2, double down,
           double dz) {
    Polygo filein;
    filein.assign(fileroot.begin() + time_b, fileroot.begin() + time_r);
    sort(filein.begin(), filein.end(), zsortup);
    int read_num = 0;//filein读取位置
    while (read_num < filein.size()) {
        Polygo zmid;
        while (filein[read_num].z + EPS < down + dz) {
            zmid.push_back(filein[read_num]);
            read_num++;
            if (read_num >= filein.size())
                break;
        }
        if (zmid.size() < lim) {
            kongdown.push_back(down);
            down += dz;
            zmid.clear();
            continue;
        }
        down += dz;
        file2.insert(file2.end(), zmid.begin(), zmid.end());
        zmid.clear();
    }
    filein.clear();
}

//合并计算层
void creatfull(Polygo &file2, vector<double> &kongdown, vector<floatz> &full, double &dz) {
    if (file2.front().z < kongdown.front() && fabs(file2.front().z + dz - kongdown.front()) >= EPS) {
        floatz mid;
        mid.under = file2.front().z;
        mid.up = kongdown.front() - dz;
        full.push_back(mid);
    }
    for (int i = 0; i < kongdown.size() - 1; i++) {
        if (fabs(kongdown[i] + dz - kongdown[i + 1]) >= EPS) {
            floatz mid;
            mid.under = kongdown[i] + dz;
            mid.up = kongdown[i + 1] - dz;
            full.push_back(mid);
        }
    }
    if (kongdown.back() < file2.back().z && fabs(kongdown.back() + dz - file2.back().z) >= EPS) {
        floatz mid;
        mid.under = kongdown.back() + dz;
        mid.up = file2.back().z;
        full.push_back(mid);
    }

    vector<int> Toosmall;
    for (int i = 0; i < full.size() - 1; i++) {
        if (fabs(full[i].up - full[i].under) <= dz + EPS) {
            if (fabs(full[i + 1].under - full[i].up) <= 2 * dz + EPS) {
                Toosmall.push_back(i);
            }
        }
    }
    if (!Toosmall.empty()) {
        for (int i = 0; i < Toosmall.size() - 1; i++) {
            int mid = i;
            while (Toosmall[mid] + 1 == Toosmall[mid + 1]) {
                mid++;
                if (mid == Toosmall.size() - 1)
                    break;
            }
            floatz midd;
            midd.under = full[Toosmall[i]].under;
            midd.up = full[Toosmall[mid] + 1].up;
            full.push_back(midd);
            i = mid;
        }
        if (Toosmall.size() == 1) {
            floatz midd;
            midd.under = full[Toosmall.back()].under;
            midd.up = full[Toosmall.back() + 1].up;
            full.push_back(midd);
        }
    }
    if (!Toosmall.empty()) {
        int del_n = 0;
        for (int i = 0; i < Toosmall.size() - 1; i++) {
            int lll = Toosmall[i] - del_n;
            full.erase(full.begin() + lll);
            del_n++;
            if (Toosmall[i] + 1 != Toosmall[i + 1]) {
                int kkk = Toosmall[i] + 1 - del_n;
                full.erase(full.begin() + kkk);
                del_n++;
            }
        }
        int lll = Toosmall.back() - del_n;
        full.erase(full.begin() + lll);
        del_n++;
        int kkk = Toosmall.back() + 1 - del_n;
        full.erase(full.begin() + kkk);
        del_n++;
    }
}

//求中间底面
double daymaxz(Polygo &file2, double &day_min_z, double &day_max_z, double &down) {
    double day_mid_max_z = file2.front().z;
    for (int i = 1; i < file2.size(); i++) {
        if (day_min_z > file2[i].z) {
            day_min_z = file2[i].z;
        }
        if (day_mid_max_z < file2[i].z) {
            day_mid_max_z = file2[i].z;
        }
    }
    if (day_min_z > day_max_z) {
        down = day_max_z;
    } else {
        down = day_min_z;
    }
    return day_mid_max_z;
}

// 写obj文件
void writeOBJ(Polygo &Out, ofstream &myout, int &num, double &dz, double &down) {
    string k = " ";
    string v = "v";
    string f = "f";
    string h = "\n";
    Out.pop_back();
    int num_vn = 1;
    Polygo Out2;
    Out2.assign(Out.begin(), Out.end());
    if (fabs(Out[0].z - down) > 10) {
        down = Out[0].z - dz;
    }
    for (int i = 0; i < Out.size(); i++) {
        myout << setiosflags(ios::fixed) << v + k << Out[i].x << k << Out[i].z << k << Out[i].y << h;
        Out[i].num = num;
        num++;
    }
    for (int i = 0; i < Out2.size(); i++) {
        Out2[i].z = down;
        myout << setiosflags(ios::fixed) << v + k << Out2[i].x << k << down << k << Out2[i].y << h;
        Out2[i].num = num;
        num++;
    }
    myout << setiosflags(ios::fixed) << "vn 0 1 0" << "\n";
    num_vn++;
    myout << setiosflags(ios::fixed) << "vn 0 -1 0" << "\n";
    num_vn++;
    for (int i = 1; i < Out.size() - 1; i++) {
        double c =
                (Out[i].x - Out[0].x) * (Out[i + 1].y - Out[0].y) - (Out[i + 1].x - Out[0].x) * (Out[i].y - Out[0].y);
        if (c >= 0) {
            myout << setiosflags(ios::fixed) << f << k << Out[0].num << "//" << num_vn - 2 << k << Out[i].num << "//"
                  << num_vn - 2 << k << Out[i + 1].num << "//" << num_vn - 2 << h;
            myout << setiosflags(ios::fixed) << f << k << Out[0].num << "//" << num_vn - 1 << k << Out[i + 1].num
                  << "//" << num_vn - 1 << k << Out[i].num << "//" << num_vn - 1 << h;
        } else {
            myout << setiosflags(ios::fixed) << f << k << Out[0].num << "//" << num_vn - 2 << k << Out[i + 1].num
                  << "//" << num_vn - 2 << k << Out[i].num << "//" << num_vn - 2 << h;
            myout << setiosflags(ios::fixed) << f << k << Out[0].num << "//" << num_vn - 1 << k << Out[i].num << "//"
                  << num_vn - 1 << k << Out[i + 1].num << "//" << num_vn - 1 << h;
        }
    }
    myout << f;
    for (int i = 1; i < Out2.size() - 1; i++) {
        double c = (Out2[i].x - Out2[0].x) * (Out2[i + 1].y - Out2[0].y) -
                   (Out2[i + 1].x - Out2[0].x) * (Out2[i].y - Out2[0].y);
        if (c >= 0) {
            myout << setiosflags(ios::fixed) << f << k << Out2[0].num << "//" << num_vn - 2 << k << Out2[i + 1].num
                  << "//" << num_vn - 2 << k << Out2[i].num << "//" << num_vn - 2 << h;
            myout << setiosflags(ios::fixed) << f << k << Out2[0].num << "//" << num_vn - 1 << k << Out2[i].num << "//"
                  << num_vn - 1 << k << Out2[i + 1].num << "//" << num_vn - 1 << h;
        } else {
            myout << setiosflags(ios::fixed) << f << k << Out2[0].num << "//" << num_vn - 2 << k << Out2[i].num << "//"
                  << num_vn - 2 << k << Out2[i + 1].num << "//" << num_vn - 2 << h;
            myout << setiosflags(ios::fixed) << f << k << Out2[0].num << "//" << num_vn - 1 << k << Out2[i + 1].num
                  << "//" << num_vn - 1 << k << Out2[i].num << "//" << num_vn - 1 << h;
        }
    }
    for (int i = 0; i < Out.size() - 1; i++) {
        double a = (Out2[i + 1].y - Out2[i].y) * (Out[i + 1].z - Out2[i].z) -
                   (Out[i + 1].y - Out2[i].y) * (Out2[i + 1].z - Out2[i].z);
        double b = (Out2[i + 1].z - Out2[i].z) * (Out[i + 1].x - Out2[i].x) -
                   (Out[i + 1].z - Out2[i].z) * (Out2[i + 1].x - Out2[i].x);
        double c = (Out2[i + 1].x - Out2[i].x) * (Out[i + 1].y - Out2[i].y) -
                   (Out[i + 1].x - Out2[i].x) * (Out2[i + 1].y - Out2[i].y);
        myout << setiosflags(ios::fixed) << "vn" << k << a << k << b << k << c << h;
        num_vn++;
        myout << f + k << Out2[i].num << "//" << num_vn - 1 << k << Out2[i + 1].num << "//" << num_vn - 1 << k
              << Out[i + 1].num << "//" << num_vn - 1 << k << Out[i].num << "//" << num_vn - 1 << h;
        myout << setiosflags(ios::fixed) << "vn" << k << -a << k << -b << k << -c << h;
        num_vn++;
        myout << f + k << Out[i].num << "//" << num_vn - 1 << k << Out[i + 1].num << "//" << num_vn - 1 << k
              << Out2[i + 1].num << "//" << num_vn - 1 << k << Out2[i].num << "//" << num_vn - 1 << h;
    }
    double a = (Out2.front().y - Out2.back().y) * (Out.front().z - Out2.back().z) -
               (Out.front().y - Out2.back().y) * (Out2.front().z - Out2.back().z);
    double b = (Out2.front().z - Out2.back().z) * (Out.front().x - Out2.back().x) -
               (Out.front().z - Out2.back().z) * (Out2.front().x - Out2.back().x);
    double c = (Out2.front().x - Out2.back().x) * (Out.front().y - Out2.back().y) -
               (Out.front().x - Out2.back().x) * (Out2.front().y - Out2.back().y);
    myout << setiosflags(ios::fixed) << "vn" << k << -a << k << -b << k << -c << h;
    num_vn++;
    myout << f + k << Out.back().num << "//" << num_vn - 1 << k << Out.front().num << "//" << num_vn - 1 << k
          << Out2.front().num << "//" << num_vn - 1 << k << Out2.back().num << "//" << num_vn - 1 << h << endl;
    myout << setiosflags(ios::fixed) << "vn" << k << a << k << b << k << c << h;
    num_vn++;
    myout << f + k << Out2.back().num << "//" << num_vn - 1 << k << Out2.front().num << "//" << num_vn - 1 << k
          << Out.front().num << "//" << num_vn - 1 << k << Out.back().num << "//" << num_vn - 1 << h << endl;
}

//转格式
void obj2gltf(string path, int num_time_name) {
    system("npm install -g obj2gltf");
    string sys1 = "obj2gltf -i ";
    string sys2 = " -o ";
    for (int i = 1; i < num_time_name; i++) {
        // obj2gltf -i
        string sys = sys1 + path + to_string(i) + ".obj" + sys2 + path + to_string(i) + ".gltf";
        system(sys.c_str());
    }
}

//时间规范字符串
string DayString(std::tm *day) {
    // 准备根据格式造字符串流
    stringstream fmt;   /* 或者使用 ostringstream */
    // 造字符串流
    fmt << day->tm_year + 1900 << "-" << setw(2) << setfill('0') << day->tm_mon + 1 << "-" << setw(2) << day->tm_mday
        << "T" << setw(2) << day->tm_hour << ":" << setw(2) << day->tm_min << ":" << setw(2) << day->tm_sec << ".0z";
    string targetString = fmt.str();
    return targetString;
}


//获取gltf文件名
//macOS、Ubuntu获取所有的文件名
void GetAllFiles(string path, vector<string> &filenames) {
    DIR *pDir; //指向根目录结构体的指针
    struct dirent *ptr; //dirent结构体指针，具体结构看开头的注释

    // 使用dirent.h下的 opendir() 打开根目录，并返回指针
    if (!(pDir = opendir(path.c_str())))
        return;

    // 使用dirent.h下的readdir逐个读取root下的文件
    while ((ptr = readdir(pDir)) != 0) {
        // 这里我理解他的指针应该是自动会指向到下一个文件，所以不用写指针的移动
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            filenames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
}


//macOS、Ubuntu获取特定格式的文件名
void GetAllFormatFiles(string path, vector<string> &files, string format) {
    DIR *pDir; //指向根目录结构体的指针
    struct dirent *ptr; //dirent结构体指针，具体结构看开头的注释
    // 使用dirent.h下的opendir()打开根目录，并返回指针
    if (!(pDir = opendir(path.c_str()))) {
        return;
    }
    // 使用dirent.h下的readdir逐个读取root下的文件
    while ((ptr = readdir(pDir)) != nullptr) {
        // 这里我理解他的指针应该是自动会指向到下一个文件，所以不用写指针的移动
        string sub_file = path + "/" + ptr->d_name; // 当前指针指向的文件名
        if (ptr->d_type != 8 && ptr->d_type != 4) { // 递归出口，当不是普通文件（8）和文件夹（4）时退出递归
            return;
        }
        // 普通文件直接加入到files
        if (ptr->d_type == 8) {
            // 相当于将命令下使用ls展示出来的文件中除了. 和 ..全部保存在files中
            // 当然下面可以写各种字符串的筛选逻辑，比如只要后缀有.jpg图片
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
                if (strstr(ptr->d_name, format.c_str())) {
                    files.push_back(sub_file);
                }
            } // 当前文件为文件夹(4)类型，那就以当前文件为root进行递归吧！
        } else if (ptr->d_type == 4) {
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
                GetAllFormatFiles(sub_file, files, format);
            }
        }
    }
    // 关闭根目录
    closedir(pDir);
}


// 该函数有两个参数，第一个为路径字符串(string类型，最好为绝对路径)；
// 第二个参数为文件夹与文件名称存储变量(vector类型,引用传递)。
// 在主函数中调用格式(并将结果保存在文件"AllFiles.txt"中，第一行为总数)：
//时间戳转时间
std::tm *gettm(long long timestamp) {
    auto milli = timestamp + (long long) 8 * 60 * 60 * 1000; //此处转化为东八区北京时间，如果是其它时区需要按需求修改
    auto mTime = std::chrono::milliseconds(milli);
    auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm *now = std::gmtime(&tt);
    printf("%4d年%02d月%02d日 %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
    return now;
}

//写czml文件
void writeCZML(string path, double min_t, double max_t, vector<num_day>& daylist, string longitude, string latitude, string heightDeo)
{
    string czmltxt = "/xt.czml";  //注意：Windows写"\\"，Ubuntu、macOS写"/"
    ofstream myout(path + czmltxt);
    //读取所有格式为gltf的文件
    vector<int> daytrue;
    vector<string> files;
    string format = ".gltf";
    GetAllFormatFiles(path, files, format);
    int size = files.size();
    for (int i = 0; i < size; i++)
    {
        size_t found = files[i].find(".");
        string strdou(files[i].c_str(), found);
        int a = atoi(strdou.c_str());
        daytrue.push_back(a);
    }
    sort(daytrue.begin(), daytrue.end());
    //开始写czml
    string k = "	";
    myout << "[" << endl;
    //首对象
    auto minday = min_t - (double)86400000;
    std::tm* begin_time = gettm(minday);
    string daystring1 = DayString(begin_time);

    auto maxday = max_t + (double)86400000;
    std::tm* end_time = gettm(maxday);
    string daystring2 = DayString(end_time);
    cout << daystring1 << endl;
    cout << daystring2 << endl;
    myout << k << "{" << endl;
    myout << k << k << "\"id\":\"document\"," << endl;
    myout << k << k << "\"name\":\"CZMLPolygon\"," << endl;
    myout << k << k << "\"version\":\"1.0\"," << endl;
    myout << k << k << "\"clock\":{" << endl;
    myout << k << k << "\"interval\":\"" + daystring1 + "/" + daystring2 + "\"," << endl;
    myout << k << k << "\"currentTime\":\"" + daystring1 + "\"," << endl;
    myout << k << k << "\"multiplier\":10000000" << endl;
    myout << k << k << "}" << endl;
    myout << k << "}," << endl;

    for (int i = 0; i < daytrue.size() - 1; i++)
    {
        int m = daytrue[i] - 1;
        std::tm* midtime = gettm(daylist[m].day);
        string daystring = DayString(midtime);
        myout << k << "{" << endl;
        myout << k << k << "\"id\":\"" + daystring + "\"," << endl;
        myout << k << k << "\"name\":\"" + daystring + "\"," << endl;
        myout << k << k << "\"availability\":\"" + daystring + "/" + daystring2 + "\"," << endl;
        myout << k << k << "\"position\":{" << endl;
        myout << k << k << k << "\"cartographicDegrees\":[" + longitude + "," + latitude + "," + heightDeo + "]" << endl;
        myout << k << k << "}," << endl;
        myout << k << k << "\"model\":{" << endl;
        myout << k << k << k << "\"gltf\":\"" + to_string(daytrue[i]) + ".gltf\"," << endl;
        myout << k << k << k << "\"scale\" : 1.0," << endl;
        myout << k << k << k << "\"minimumPixelSize\":1" << endl;
        myout << k << k << "}" << endl;
        myout << k << "}" << "," << endl;
    }
    int m = daytrue.back() - 1;
    std::tm* midtime = gettm(daylist[m].day);
    string daystring = DayString(midtime);
    myout << k << "{" << endl;
    myout << k << k << "\"id\":\"" + daystring + "\"," << endl;
    myout << k << k << "\"name\":\"" + daystring + "\"," << endl;
    myout << k << k << "\"availability\":\"" + daystring + "/" + daystring2 + "\"," << endl;
    myout << k << k << "\"position\":{" << endl;
    myout << k << k << k << "\"cartographicDegrees\":[" + longitude + "," + latitude + "," + heightDeo + "]" << endl;
    myout << k << k << "}," << endl;
    myout << k << k << "\"model\":{" << endl;
    myout << k << k << k << "\"gltf\":\"" + to_string(daytrue.back()) + ".gltf\"," << endl;
    myout << k << k << k << "\"scale\" : 1.0," << endl;
    myout << k << k << k << "\"minimumPixelSize\":1" << endl;
    myout << k << k << "}" << endl;
    myout << k << "}" << endl;
    myout << "]";
}

#endif //ROLLOFFMONOFILE_POINT_H
