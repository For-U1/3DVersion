#ifndef POINTCLOUDPROCESSING_H
#define POINTCLOUDPROCESSING_H

#include <string>
#include <vector>
#include <QObject>
#include <QThread> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <random>
#include <filesystem>
#include <CCCoreLib/PointCloudTpl.h>
#include <CCCoreLib/RegistrationTools.h>
#include "calibrator_our.h"
#include <FLED.h>
//#include <tools.h>

// 定义点类型
using processingPointT = pcl::PointXYZRGBA;

using PointCloud = CCCoreLib::GenericIndexedCloudPersist;

using ccPointCloud = CCCoreLib::PointCloudTpl<PointCloud>;

//定义一个结构体来存储不同方法的路径参数设置，可根据需要增加其他参数，例如 RANSAC 参数等
struct Stitchingsettings
{
    std::string calibPath;     // 标定文件路径
    std::string imagePath;     // 标志点图像路径
    std::string cloudPointdataPath;  // 视差图和点云文件路径
    
};

struct ICPResult
{
    pcl::PointCloud<processingPointT>::Ptr alignedCloud;
    Eigen::Matrix4f transformation;
};

// 解析点云文件
pcl::PointCloud<processingPointT>::Ptr loadPointCloud(const std::string& file_path);

// 保存点云文件
void savePointCloud(const std::string& file_path, pcl::PointCloud<processingPointT>::Ptr cloud);

// 获取文件夹内所有支持的点云文件
std::vector<std::string> getPointCloudFiles(const std::string& folder_path);

// 点云处理线程类
class PointCloudProcessingThread : public QThread
{
    Q_OBJECT

public:
    PointCloudProcessingThread(const std::string& dataPath, const std::string& savePath, int method, const Stitchingsettings& set);

    ~PointCloudProcessingThread();

    void run();

    void stop();

    //使用cloudcompare中的ICP算法替代PCL库中的icp
    //pcl::PointCloud<processingPointT>::Ptr alignPointCloudsCC(pcl::PointCloud<processingPointT>::Ptr sourceCloud,pcl::PointCloud<processingPointT>::Ptr targetCloud);

    //cloudcompare中的ICP算法计算变换矩阵
    ICPResult alignPointCloudsCC(pcl::PointCloud<processingPointT>::Ptr sourceCloud,pcl::PointCloud<processingPointT>::Ptr targetCloud);

    //使用自定义函数计算rmse
    double computeRMSE(pcl::PointCloud<processingPointT>::Ptr transformedCloud,pcl::PointCloud<processingPointT>::Ptr targetCloud);


    //实现pcl::PointXYZRGBA向pcl::PointXYZ数据类型的转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(pcl::PointCloud<processingPointT>::Ptr inputCloud);

    //实现pcl::PointXYZ向pcl::PointXYZRGBA数据类型的转换
    pcl::PointCloud<processingPointT>::Ptr convertXYZRGBA(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    //使用RANSAC计算变换矩阵
    Eigen::Matrix4f WithRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudSrc, pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudTgt,
        double distance_threshold,
        int max_iterations,
        int min_inliers);

    //tools文件中的函数代码
    void ConvertToPointCloud_1(const std::vector<cv::Point3f>& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    float computeAverageDistanceErrorKDTree_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromTXT_1(const std::string& file_path);

    void CircleThreshold_1(std::vector<std::vector<float>>& Points, std::vector<std::vector<float>>& circles, double threshold);

    int ReadTxtMat_1(std::string& filename, std::vector<std::vector<double>>& data);
    
    float calculateDistance3D_1(const cv::Point3f& p1, const cv::Point3f& p2);

    void DistenceMatrix_1(std::vector<std::vector<float>>& circles, std::vector<std::vector<double>>& diff_map, cv::Mat& Q, std::vector<cv::Point3f>& XYZ, std::vector<std::vector<double>>& distanceMatrix, int& i);

    int isPotentialMatch_1(const std::vector<double>& rowA, const std::vector<double>& rowB);

    vector<vector<int>> findMatchingPoints_1(const vector<std::vector<double>>& A, const vector<vector<double>>& B, vector<vector<int>>& matchingPoints, vector<cv::Point3f>& P1, vector<cv::Point3f>& Q1, vector<cv::Point3f>& P2, vector<cv::Point3f>& Q2);

    //动态调整imagepath中的图片数量
    int countImagesInDirectory(const std::string& imagePath);

    std::string matrixToString(const Eigen::Matrix4f& matrix);

    //实现使用WithRANSAC进行标志点拼接方法
    void processPointCloudWithRANSAC();

signals:

    void processingFinished();

    void logMessage(const QString& message);

    void newPointCloud(pcl::PointCloud<processingPointT>::Ptr cloud);

    void updateProgress(int value);

private:

    std::string IcpdataPath;

    std::string IcpsavePath;

    bool isStopped;

    bool useICP;

    int stitchingMethod; // 0: ICP, 1: RANSAC

    Stitchingsettings set; // 保存高级设置参数
};

#endif // POINTCLOUDPROCESSING_H
