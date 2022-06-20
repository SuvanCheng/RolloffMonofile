//
// Created by Suvan Cheng on 2022/6/20.
//

#ifndef ROLLOFFMONOFILE_SINGLEMODEL_H
#define ROLLOFFMONOFILE_SINGLEMODEL_H

#include "Point.h"

int SingleModel(string Pointpath, string path,
                double dz = 0.001, double max_face = 150, double min_face = 50,
                int lim = 10, int add_float = 2, int min_point_num = 10, int timegap = 1) {
    int num_time_name = 1;
    double day_min_z = 0;
    double day_max_z = 0;
    vector<num_day> daylist;//时间与编码
    const char *jcpath;;//高程内最少的点数
    double geodown = (double) 0;;
    string longitude = "116.6445206464275";
    string latitude = "33.24206160289476";
    string heightDeo = "19.5";
    Polygo fileroot; //原始点云
    double mov_x = 0;
    double mov_y = 0;

    double min_x = 0;
    double min_y = 0;
    double min_z = 0;
    double min_t = 0;

    double max_x = 0;
    double max_y = 0;
    double max_z = 0;
    double max_t = 0;
    //读取点云
    readFile(Pointpath);

    int num_jx = xx.size();
    int num_jy = yy.size();
    int num_jz = zz.size();
    int jmin = num_jx;
    if (jmin > num_jy) jmin = num_jy;
    if (jmin > num_jz) jmin = num_jz;
    for (int i = 0; i < jmin; i++) {
        Point mid;
        mid.x = xx[i];
        mid.y = yy[i];
        mid.z = zz[i];
        mid.time = 0;
        mid.num = 0;
        fileroot.push_back(mid);
    }
    min_x = fileroot[0].x;
    min_y = fileroot[0].y;
    min_z = fileroot[0].z;
    min_t = fileroot[0].time;

    max_x = fileroot[0].x;
    max_y = fileroot[0].y;
    max_z = fileroot[0].z;
    max_t = fileroot[0].time;
    //找最小值
    for (int i = 1; i < fileroot.size(); i++) {
        if (min_x > fileroot[i].x) min_x = fileroot[i].x;
        if (max_x < fileroot[i].x) max_x = fileroot[i].x;

        if (min_y > fileroot[i].y) min_y = fileroot[i].y;
        if (max_y < fileroot[i].y) max_y = fileroot[i].y;

        if (min_z > fileroot[i].z) min_z = fileroot[i].z;
        if (max_z < fileroot[i].z) max_z = fileroot[i].z;
    }
    double mid_x = (float) (max_x + min_x) / 2.0;
    double mid_y = (float) (max_y + min_y) / 2.0;
    double mid_xx = mov_x;
    double mid_yy = mov_y;
    if (mid_xx < EPS || mid_yy < EPS) {
    } else {
        mid_x = mid_xx;
        mid_y = mid_yy;
    }
    //归一化
    for (int i = 0; i < fileroot.size(); i++) {
        fileroot[i].x = fileroot[i].x - mid_x;
        fileroot[i].y = fileroot[i].y - mid_y;
    }
    geodown = min_z;
    day_max_z = max_z;
    path = path + "/";
//-------------------清空空面点云-------------------
    int time_r = 0;
    while (time_r < fileroot.size()) {
        int time_b = time_r;
        double limtime = fileroot[time_b].time + timegap * (double) 86400000;
        while (fileroot[time_r].time < limtime) {
            time_r++;
            if (time_r >= fileroot.size())
                break;
        }
        string obj = ".obj";
        string time_name = to_string(num_time_name);
        num_day timemid;
        timemid.num_dd = num_time_name;
        timemid.day = fileroot[time_b].time;
        daylist.push_back(timemid);
        ofstream myout(path + time_name + obj);
        myout << "o " << num_time_name << "\n";
        num_time_name++;
        Polygo file2;//清空少点面
        vector<double> kongdown;//实面高度
        vector<floatz> full;
        double down = min_z;
        //清空少点面
        clearempty(fileroot, kongdown, time_b, time_r, lim, file2, down, dz);
        if (file2.empty()) { continue; }
        creatfull(file2, kongdown, full, dz);
        sort(full.begin(), full.end(), fsortup);
        //输出准备
        int read_num = 0;
        int num = 1;//顺序标记，输入点顺序

        //切割z轴
        down = geodown;//设定底层高度
        day_min_z = file2.front().z;
        day_max_z = daymaxz(file2, day_min_z, day_max_z, down);
        sort(file2.begin(), file2.end(), zsortup);
        read_num = 0;
        for (int full_read = 0; full_read < full.size(); full_read++) {
            Polygo zfloat;
            while (full[full_read].under - dz + EPS < file2[read_num].z &&
                   file2[read_num].z + EPS < full[full_read].up + dz) {
                zfloat.push_back(file2[read_num]);
                read_num++;
                if (read_num >= file2.size())
                    break;
            }
            if (zfloat.empty()) {
                continue;
            }
            int z_num = zfloat.size();
            int beishu = (full[full_read].up - full[full_read].under) / dz;
            //查找边界
            sort(zfloat.begin(), zfloat.end(), zsortup);
            Polygo Out = andrewScan(zfloat);
            for (int i = 0; i < Out.size(); i++) {
                Out[i].z = zfloat.front().z;
            }
            double ans = 0.0;
            Point mid;
            mid.x = Out.front().x;
            mid.y = Out.front().y;
            mid.z = Out.front().z;
            Out.push_back(mid);
            for (int i = 0; i < Out.size() - 1; i++) {
                ans += (Out[i].x * Out[i + 1].y - Out[i + 1].x * Out[i].y);
            }
            ans = ans / 2;
            double face = fabs(ans);
            if (z_num < (beishu + add_float) * min_point_num && face > max_face) {
                zfloat.clear();
                continue;
            }
            if (face < min_face) {
                zfloat.clear();
                continue;
            }
            //按顺序输出点云
            writeOBJ(Out, myout, num, dz, down);
        }
        myout.close();
    }
    cout << "建模结束" << endl;
//==============转gltf文件======================
    cout << "转gltf文件" << endl;
    obj2gltf(path, num_time_name);
    return 0;
}

#endif //ROLLOFFMONOFILE_SINGLEMODEL_H
