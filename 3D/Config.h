//#ifndef CONFIG_H
//#define CONFIG_H
//
//#include <string>
//
//
//struct Config {
//    const std::string sep;
//    const std::string root_dir;
//    const std::string data_dir;
//    const std::string calib_dir;
//    const std::string calib_file;
//    const std::string project_dir;
//    const std::string simu_dir;
//    const std::string model_dir;
//    const std::string output_dir;
//    const std::string output_dir_L;
//    const std::string output_dir_R;
//    const std::string save_file_point3d;
//
//    bool write;
//    bool show;
//
//    const double A;
//    const double B;
//    const int N;
//    const int T;
//    const double T1;
//    const double T2;
//    const double T3;
//    const int W;
//    const int H;
//    const float B_min;
//
//    const int win;
//    const float pd;
//    const float min_z;
//    const float max_z;
//
//    // 构造函数，用于初始化所有的 const 成员变量
//    Config()
//        : sep("/"),
//        root_dir("E:/test3"),
//        data_dir(root_dir + sep + "data"),
//        calib_dir(data_dir + sep + "calib"),
//        calib_file(calib_dir + sep + "stereoCalib.txt"),
//        project_dir(data_dir + sep + "protect"),
//        simu_dir(data_dir + sep + "simulation"),
//        model_dir(data_dir + sep + "mouse"),
//        output_dir(root_dir + sep + "outputs"),
//        output_dir_L(output_dir + sep + "L"),
//        output_dir_R(output_dir + sep + "R"),
//        save_file_point3d(output_dir + sep + "xyz.txt"),
//        write(true),
//        show(true),
//        A(130),
//        B(90),
//        N(12),
//        T(3),
//        T1(28.0),
//        T2(26.0),
//        T3(24.0),
//        W(1280),
//        H(720),
//        B_min(5.0),
//        win(3),
//        pd(0.5),
//        min_z(260.0),
//        max_z(305.0)
//    {}
//};
//#endif 
//
//#ifndef CONFIG_H
//#define CONFIG_H
//
//#include <string>
//
//
//struct Config {
//    // 路径配置
//    std::string sep;
//    std::string root_dir;
//    std::string data_dir;
//    std::string calib_dir;
//    std::string calib_file;
//    std::string project_dir;
//    std::string simu_dir;
//    std::string model_dir;
//    std::string output_dir;
//    std::string output_dir_L;
//    std::string output_dir_R;
//    std::string save_file_point3d;
//
//    // 布尔配置
//    bool write;
//    bool show;
//
//    // 数值配置
//    double A;
//    double B;
//    int N;
//    int T;
//    double T1;
//    double T2;
//    double T3;
//    int W;
//    int H;
//    float B_min;
//
//    int win;
//    float pd;
//    float min_z;
//    float max_z;
//
//    // 构造函数设置默认值
//    Config();
//};
//
//#endif // CONFIG_H
// Config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <string>

struct Config {
    std::string sep;
    std::string root_dir;
    std::string data_dir;
    std::string calib_dir;
    std::string calib_file;
    std::string project_dir;
    std::string simu_dir;
    std::string model_dir;
    std::string output_dir;
    std::string output_dir_L;
    std::string output_dir_R;
    std::string save_file_point3d;

    bool write;
    bool show;

    double A;
    double B;
    int N;
    int T;
    double T1;
    double T2;
    double T3;
    int W;
    int H;
    float B_min;

    int win;
    float pd;
    float min_z;
    float max_z;

    // 构造函数，初始化参数为默认值或空
    Config()
        : sep("/"),
        root_dir(""),
        data_dir(""),
        calib_dir(""),
        calib_file(""),
        project_dir(""),
        simu_dir(""),
        model_dir(""),
        output_dir(""),
        output_dir_L(""),
        output_dir_R(""),
        save_file_point3d(""),
        write(true),
        show(false),
        A(130.0),
        B(90.0),
        N(12),
        T(3),
        T1(28.0),
        T2(26.0),
        T3(24.0),
        W(1280),
        H(720),
        B_min(5.0),
        win(3),
        pd(0.5f),
        min_z(260.0f),
        max_z(305.0f)
    {}
};

#endif // CONFIG_H
