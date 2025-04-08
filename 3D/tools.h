#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_set>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Dense>
#include <core/eigen.hpp>

//using namespace std;
//using namespace cv;

int ReadTxtMat(std::string& filename, std::vector<std::vector<double>>& data) 
{
    std::ifstream infile(filename);

    if (!infile) 
    {
        std::cerr << "Cannot open the file: " << filename << std::endl;
        return 1;
    }
    std::string line;

    while (std::getline(infile, line)) 
    {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) 
        {
            row.push_back(std::stod(value));
        }
        data.push_back(row);
    }
    infile.close();
    return 0;
}
float calculateDistance2D(const std::vector<float>& p1, const cv::Point2f& p2) 
{
    return std::sqrt((p1[1] - p2.x) * (p1[1] - p2.x) + (p1[0] - p2.y) * (p1[0] - p2.y));
}
float calculateDistance3D(const cv::Point3f& p1, const cv::Point3f& p2) 
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));// 求两个坐标点之间的欧式距离，差值平方以后相加再开方
}
double calculateDistance_ZP(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}
bool isCongruentTriangele(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2,double tolerance)
{
         //判断使用RANSAC算法随机提取的两个视角下的三个点是否构成全等三角形   
        
         // 保证两边都至少有3个点
        if (cloud1->points.size() < 3 || cloud2->points.size() < 3) 
        {
            return false;
        }

        // 计算第一个三角形的三边长度
        std::vector<double> sides1;
        float p1 = calculateDistance_ZP(cloud1->points[0], cloud1->points[1]);
        sides1.push_back(p1);
        float p2 = calculateDistance_ZP(cloud1->points[1], cloud1->points[2]);
        sides1.push_back(p2);
        float p3 = calculateDistance_ZP(cloud1->points[2], cloud1->points[0]);
        sides1.push_back(p3);

        // 计算第二个三角形的三边长度
        std::vector<double> sides2;
        float q1 = calculateDistance_ZP(cloud2->points[0], cloud2->points[1]);
        sides2.push_back(q1);
        float q2 = calculateDistance_ZP(cloud2->points[1], cloud2->points[2]);
        sides2.push_back(q2);
        float q3 = calculateDistance_ZP(cloud2->points[2], cloud2->points[0]);
        sides2.push_back(q3);

        // 对边长进行排序（从小到大），消除顶点顺序的影响
        std::sort(sides1.begin(), sides1.end());
        std::sort(sides2.begin(), sides2.end());

        // 逐边比较差值是否都在容差内
        for (size_t i = 0; i < 3; ++i) 
        {
            if (std::fabs(sides1[i] - sides2[i]) > tolerance) 
            return false;
        }
        // 如果全等，则输出两个三角形的边长
        /*std::cout << "第一个全等三角形边长: " << sides1[0] << endl << sides1[1] << sides1[2] << endl;
        std::cout << "第二个全等三角形边长: " << sides2[0] << endl << sides2[1] << sides2[2] << endl;*/

        return true;
}
void CircleThreshold(std::vector<std::vector<float>>& Points, std::vector<std::vector<float>>& circles, double threshold) {
    // 标记哪些点被保留
    std::vector<bool> isKept(Points.size(), true);

    for (int i = 0; i < Points.size(); i++) {
        if (!isKept[i]) continue; // 如果该点已被标记为去除，则跳过
        // 遍历所有后续点
        for (int j = i + 1; j < Points.size(); j++) {
            if (abs(Points[i][0] - Points[j][0]) < threshold && abs(Points[i][1] - Points[j][1]) < threshold) {
                // 如果两个点距离小于阈值，保留较大的第三个元素的点
                if (Points[i][2] > Points[j][2]) {
                    isKept[j] = false; // 标记 Points[j] 为不保留
                }
                else {
                    isKept[i] = false; // 标记 Points[i] 为不保留
                }
            }
        }
    }

    // 将保留下来的点添加到 circles 中
    for (int i = 0; i < Points.size(); i++) 
    {
        if (isKept[i]) {
            circles.push_back(Points[i]);
        }
    }
}
void PrintCirclePoints(std::vector<std::vector<float>>& circles) 
{
    //for (const auto& row1 : circles) {
    //    for (const auto& elem1 : row1) {
    //        std::cout << elem1 << " ";
    //    }
    //    std::cout << std::endl; // 换行
    //}
    for (int i = 0; i < circles.size(); i++)
    {
        cout << circles[i][0] << "--" << circles[i][1] << endl;
    }
}
void PrintDistence(std::vector<std::vector<double>>& circles) 
{
    for (const auto& row1 : circles) {
        for (const auto& elem1 : row1) {
            std::cout << elem1 << " ";
        }
        std::cout << std::endl; // 换行
    }
}
void DistenceMatrix(std::vector<std::vector<float>>& circles, std::vector<std::vector<double>>& diff_map, cv::Mat& Q,std::vector<cv::Point3f>& XYZ, std::vector<std::vector<double>>& distanceMatrix,int& i)
{
    cout << "-------------------------------------" << endl;
    cout << "Image" << i << "标志点的三维坐标：" << endl;
    for (int i = 0; i < circles.size(); i++)
    {
        cv::Point3f temp_point;
        cv::Mat temp = (cv::Mat_<double>(1, 4) <<circles[i][1],circles[i][0],diff_map[int(circles[i][0])][int(circles[i][1])], 1);// 创建一个包含圆心坐标和视差值的矩阵,diff_map[int(circles[i][0])][int(circles[i][1])] 获得圆心坐标的视差值
        cv::transpose(temp, temp);// 转置矩阵
        cv::Mat result = Q * temp;
        temp_point.x = result.at<double>(0) / result.at<double>(3);
        temp_point.y = result.at<double>(1) / result.at<double>(3);
        temp_point.z = result.at<double>(2) / result.at<double>(3);
        XYZ.push_back(temp_point);
        cout << temp_point << endl;
    }
    for (int i = 0; i < XYZ.size(); ++i)
    {
        std::vector<double> temp;
        for (int j = 0; j < XYZ.size(); ++j)
        {
            float distance = calculateDistance3D(XYZ[i], XYZ[j]);
            temp.push_back(distance);
        }
        distanceMatrix.push_back(temp);
    }
    //// 打印结果矩阵
    //cout << "-------------------------------------" << endl;
    //std::cout << "Image" << i << "距离矩阵:" << std::endl;
    //PrintDistence(distanceMatrix);
}
void DistenceMatrix1(vector<cv::Point3f>& XYZ1,vector<vector<double>>& distanceMatrix)
{
    for (int i = 0; i < XYZ1.size(); ++i)
    {
        vector<double> temp;
        for (int j = 0; j < XYZ1.size(); ++j)
        {
            float distance = calculateDistance3D(XYZ1[i], XYZ1[j]);
            temp.push_back(distance);
        }
        distanceMatrix.push_back(temp);
    }
    //// 打印结果矩阵
    //cout << "-------------------------------------" << endl;
    //std::cout << "潜在匹配点对的距离矩阵:" << std::endl;
    //PrintDistence(distanceMatrix);
}
void DistenceMatrix2(vector<cv::Point3f>& XYZ1, vector<cv::Point3f>& XYZ2, vector<vector<double>>& distanceMatrix)
{
    for (int i = 0; i < XYZ1.size(); ++i)
    {
        vector<double> temp;
        for (int j = 0; j < XYZ2.size(); ++j)
        {
            float distance = calculateDistance3D(XYZ1[i], XYZ2[j]);
            temp.push_back(distance);
        }
        distanceMatrix.push_back(temp);
    }
    //// 打印结果矩阵
    //cout << "-------------------------------------" << endl;
    //std::cout << "三点距离矩阵:" << std::endl;
    //PrintDistence(distanceMatrix);
}
bool compareColumn(const std::vector<double>& a, const std::vector<double>& b) {
    return a[0] < b[0];
}
int isPotentialMatch(const std::vector<double>& rowA, const std::vector<double>& rowB)
{
    int matchCount = 0;
    vector<std::pair<double, double>> matchedPairs;
    for (int i = 0; i < rowA.size(); ++i) {
        double valA = rowA[i];
        if (valA == 0) continue;
        for (int j = 0; j < rowB.size(); ++j) {
            double valB = rowB[j];
            if (valB == 0) continue;
            if (std::abs(valA - valB) <0.4)
            {
                matchCount++;
            }
        }
    }
    return matchCount;
}
vector<vector<int>> findMatchingPoints(const vector<std::vector<double>>& A, const vector<vector<double>>& B,vector<vector<int>>& matchingPoints,vector<cv::Point3f>& P1,vector<cv::Point3f>& Q1, vector<cv::Point3f>&P2, vector<cv::Point3f>& Q2)
{
    for (size_t i = 0; i < A.size(); ++i)
    {
        for (size_t j = 0; j < B.size(); ++j)
        {
            int count = isPotentialMatch(A[i], B[j]);
            if (count >=3)
            {
                matchingPoints.push_back({ static_cast<int>(i), static_cast<int>(j), count });
            }
        }
    }
    //cout << "潜在匹配点对" << endl;
    //for (const auto& match : matchingPoints)
    //{
    //    for (const auto& value : match)
    //    {
    //        cout << value << " ";
    //    }
    //    cout << endl;
    //}
    for (const auto& match : matchingPoints)
    {
        P2.push_back(P1[match[0]]);
        Q2.push_back(Q1[match[1]]);// 潜在匹配点对
    }
    return matchingPoints;
}
void DistenceMatrix_simulation(vector<cv::Point3f>& XYZ, vector<std::vector<double>>& distanceMatrix, int& i)
{
    for (int i = 0; i < XYZ.size(); ++i)
    {
        std::vector<double> temp;
        for (int j = 0; j < XYZ.size(); ++j)
        {
            float distance = calculateDistance3D(XYZ[i], XYZ[j]);
            temp.push_back(distance);
        }
        distanceMatrix.push_back(temp);
    }
    //// 打印结果矩阵
    //cout << "-------------------------------------" << endl;
    //std::cout << "Image" << i << "距离矩阵:" << std::endl;
    //PrintDistence(distanceMatrix);
}

