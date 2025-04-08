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

// ���������
using processingPointT = pcl::PointXYZRGBA;

using PointCloud = CCCoreLib::GenericIndexedCloudPersist;

using ccPointCloud = CCCoreLib::PointCloudTpl<PointCloud>;

//����һ���ṹ�����洢��ͬ������·���������ã��ɸ�����Ҫ������������������ RANSAC ������
struct Stitchingsettings
{
    std::string calibPath;     // �궨�ļ�·��
    std::string imagePath;     // ��־��ͼ��·��
    std::string cloudPointdataPath;  // �Ӳ�ͼ�͵����ļ�·��
    
};

struct ICPResult
{
    pcl::PointCloud<processingPointT>::Ptr alignedCloud;
    Eigen::Matrix4f transformation;
};

// ���������ļ�
pcl::PointCloud<processingPointT>::Ptr loadPointCloud(const std::string& file_path);

// ��������ļ�
void savePointCloud(const std::string& file_path, pcl::PointCloud<processingPointT>::Ptr cloud);

// ��ȡ�ļ���������֧�ֵĵ����ļ�
std::vector<std::string> getPointCloudFiles(const std::string& folder_path);

// ���ƴ����߳���
class PointCloudProcessingThread : public QThread
{
    Q_OBJECT

public:
    PointCloudProcessingThread(const std::string& dataPath, const std::string& savePath, int method, const Stitchingsettings& set);

    ~PointCloudProcessingThread();

    void run();

    void stop();

    //ʹ��cloudcompare�е�ICP�㷨���PCL���е�icp
    //pcl::PointCloud<processingPointT>::Ptr alignPointCloudsCC(pcl::PointCloud<processingPointT>::Ptr sourceCloud,pcl::PointCloud<processingPointT>::Ptr targetCloud);

    //cloudcompare�е�ICP�㷨����任����
    ICPResult alignPointCloudsCC(pcl::PointCloud<processingPointT>::Ptr sourceCloud,pcl::PointCloud<processingPointT>::Ptr targetCloud);

    //ʹ���Զ��庯������rmse
    double computeRMSE(pcl::PointCloud<processingPointT>::Ptr transformedCloud,pcl::PointCloud<processingPointT>::Ptr targetCloud);


    //ʵ��pcl::PointXYZRGBA��pcl::PointXYZ�������͵�ת��
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(pcl::PointCloud<processingPointT>::Ptr inputCloud);

    //ʵ��pcl::PointXYZ��pcl::PointXYZRGBA�������͵�ת��
    pcl::PointCloud<processingPointT>::Ptr convertXYZRGBA(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);

    //ʹ��RANSAC����任����
    Eigen::Matrix4f WithRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudSrc, pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudTgt,
        double distance_threshold,
        int max_iterations,
        int min_inliers);

    //tools�ļ��еĺ�������
    void ConvertToPointCloud_1(const std::vector<cv::Point3f>& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    float computeAverageDistanceErrorKDTree_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromTXT_1(const std::string& file_path);

    void CircleThreshold_1(std::vector<std::vector<float>>& Points, std::vector<std::vector<float>>& circles, double threshold);

    int ReadTxtMat_1(std::string& filename, std::vector<std::vector<double>>& data);
    
    float calculateDistance3D_1(const cv::Point3f& p1, const cv::Point3f& p2);

    void DistenceMatrix_1(std::vector<std::vector<float>>& circles, std::vector<std::vector<double>>& diff_map, cv::Mat& Q, std::vector<cv::Point3f>& XYZ, std::vector<std::vector<double>>& distanceMatrix, int& i);

    int isPotentialMatch_1(const std::vector<double>& rowA, const std::vector<double>& rowB);

    vector<vector<int>> findMatchingPoints_1(const vector<std::vector<double>>& A, const vector<vector<double>>& B, vector<vector<int>>& matchingPoints, vector<cv::Point3f>& P1, vector<cv::Point3f>& Q1, vector<cv::Point3f>& P2, vector<cv::Point3f>& Q2);

    //��̬����imagepath�е�ͼƬ����
    int countImagesInDirectory(const std::string& imagePath);

    std::string matrixToString(const Eigen::Matrix4f& matrix);

    //ʵ��ʹ��WithRANSAC���б�־��ƴ�ӷ���
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

    Stitchingsettings set; // ����߼����ò���
};

#endif // POINTCLOUDPROCESSING_H
