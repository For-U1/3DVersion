#include "PointCloudProcessing.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <chrono>

// 解析点云文件类型
pcl::PointCloud<processingPointT>::Ptr loadPointCloud(const std::string& file_path)
{
    pcl::PointCloud<processingPointT>::Ptr cloud(new pcl::PointCloud<processingPointT>);
    std::string extension = file_path.substr(file_path.find_last_of(".") + 1);

    if (extension == "pcd") 
    {
        if (pcl::io::loadPCDFile<processingPointT>(file_path, *cloud) == -1) 
        {
            std::cerr << "can~ t load PCD file: " << file_path << std::endl;
            return nullptr;
        }
    }
    else if (extension == "ply") 
    {
        if (pcl::io::loadPLYFile<processingPointT>(file_path, *cloud) == -1) 
        {
            std::cerr << "can~ t load PLY file: " << file_path << std::endl;
            return nullptr;
        }
    }
    else if (extension == "txt") 
    {
        std::ifstream infile(file_path);
        if (!infile.is_open()) 
        {
            std::cerr << "can~ t read TXT file: " << file_path << std::endl;
            return nullptr;
        }

        std::string line;
        while (std::getline(infile, line)) 
        {
            std::istringstream iss(line);
            processingPointT point;
            if (!(iss >> point.x >> point.y >> point.z)) 
            {
                break;
            }
            cloud->points.push_back(point);
        }
        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;
    }
    else 
    {
        std::cerr << "don~t support file: " << file_path << std::endl;
        return nullptr;
    }

    return cloud;
}

// 保存点云
void savePointCloud(const std::string& file_path, pcl::PointCloud<processingPointT>::Ptr cloud)
{
    std::string extension = file_path.substr(file_path.find_last_of(".") + 1);
    if (extension == "pcd") 
    {
        pcl::io::savePCDFileASCII(file_path, *cloud);
    }
    else if (extension == "ply") 
    {
        pcl::io::savePLYFileASCII(file_path, *cloud);
    }
    else if (extension == "txt") 
    {
        std::ofstream outfile(file_path);
        for (const auto& point : cloud->points) 
        {
            outfile << point.x << " " << point.y << " " << point.z << std::endl;
        }
    }
}

// 获取支持的点云文件列表
std::vector<std::string> getPointCloudFiles(const std::string& folder_path) 
{
    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(folder_path)) 
    {
        std::string ext = entry.path().extension().string();
        if (ext == ".txt" || ext == ".pcd" || ext == ".ply") 
        {
            files.push_back(entry.path().string());
        }
    }
    return files;
}

// 点云处理线程实现,使用构造函数传递参数
PointCloudProcessingThread::PointCloudProcessingThread(const std::string& dataPath, const std::string& savePath, int method, const Stitchingsettings& set)
:   IcpdataPath(dataPath),
    IcpsavePath(savePath),
    stitchingMethod(method),
    isStopped(false),
    useICP(method == 0),
    set(set) // 将传入的 set 复制到成员变量中
{
    // 可以在这里输出配置参数以调试
    std::cout << "Calib Path: " << set.calibPath << std::endl;
    std::cout << "Image Path: " << set.imagePath << std::endl;
    std::cout << "CloudPointdataPath: " << set.cloudPointdataPath << std::endl;
}


PointCloudProcessingThread::~PointCloudProcessingThread()//析构函数
{
}

void PointCloudProcessingThread::stop()
{
    isStopped = true;
}

double PointCloudProcessingThread::computeRMSE(pcl::PointCloud<processingPointT>::Ptr transformedCloud,
pcl::PointCloud<processingPointT>::Ptr targetCloud)
{
    if (transformedCloud->empty() || targetCloud->empty()) 
    {
        return -1.0;  // 避免空点云计算 RMSE
    }

    // **建立 KD-Tree 以加速最近邻搜索**
    pcl::KdTreeFLANN<processingPointT> kdtree;
    kdtree.setInputCloud(targetCloud);

    double rmse = 0.0;
    int valid_points = 0;

    // 遍历变换后的源点云
    for (const auto& point : transformedCloud->points) 
    {
        std::vector<int> pointIdx(1);
        std::vector<float> pointDist(1);

        // **搜索目标点云中与当前点最近的点**
        if (kdtree.nearestKSearch(point, 1, pointIdx, pointDist) > 0) 
        {
            rmse += pointDist[0] * pointDist[0];  // 累积平方误差
            valid_points++;
        }
    }

    // **计算最终 RMSE**
    if (valid_points > 0) 
    {
        rmse = std::sqrt(rmse / valid_points);
    }

    return rmse;
}

//CCICP
ICPResult PointCloudProcessingThread::alignPointCloudsCC(pcl::PointCloud<processingPointT>::Ptr sourceCloud,pcl::PointCloud<processingPointT>::Ptr targetCloud)
{
    ICPResult result;
    result.alignedCloud = nullptr;
    result.transformation = Eigen::Matrix4f::Identity(); // 默认单位矩阵

    if (!sourceCloud || !targetCloud || sourceCloud->empty() || targetCloud->empty()) {
        std::cout << "无效的输入点云数据！" << std::endl;
        return result;
    }

    // **1. 转换 PCL 点云 -> CloudCompare ccPointCloud**
    std::shared_ptr<ccPointCloud> ccSource(new ccPointCloud);
    std::shared_ptr<ccPointCloud> ccTarget(new ccPointCloud);

    for (const auto& pt : sourceCloud->points) {
        ccSource->addPoint(CCVector3(pt.x, pt.y, pt.z));
    }
    for (const auto& pt : targetCloud->points) {
        ccTarget->addPoint(CCVector3(pt.x, pt.y, pt.z));
    }

    std::cout << "源点云点数量：" << ccSource->size() << std::endl
        << "目标点云点数量：" << ccTarget->size() << std::endl;

    // **2. 设置 CloudCompare ICP 参数**
    CCCoreLib::ICPRegistrationTools::Parameters icpParams;
    icpParams.convType = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
    icpParams.minRMSDecrease = 0.000001;
    icpParams.nbMaxIterations = 50;
    icpParams.adjustScale = false;
    icpParams.filterOutFarthestPoints = true;
    icpParams.samplingLimit = 50000;
    icpParams.finalOverlapRatio = 1.0;
    icpParams.transformationFilters = 0;
    icpParams.maxThreadCount = 4;
    icpParams.useC2MSignedDistances = false;
    icpParams.normalsMatching = CCCoreLib::ICPRegistrationTools::NO_NORMAL;

    // **3. 执行 ICP**
    CCCoreLib::PointProjectionTools::Transformation transform;
    double finalRMS = 0.0;
    unsigned finalPointCount = 0;

    auto icpResult = CCCoreLib::ICPRegistrationTools::Register(ccTarget.get(), nullptr, ccSource.get(), icpParams, transform, finalRMS, finalPointCount);

    // **4. 处理 ICP 结果**
    if (icpResult >= CCCoreLib::ICPRegistrationTools::ICP_ERROR) {
        std::cout << "CloudCompare ICP 失败！" << std::endl;
        return result;
    }

    // **5. 将 CloudCompare 变换矩阵转换为 Eigen::Matrix4f**
    Eigen::Matrix4f transformMat = Eigen::Matrix4f::Identity();
    CCCoreLib::SquareMatrixd R = transform.R;

    if (R.isValid() && R.size() == 3) {
        for (unsigned j = 0; j < 3; ++j) {
            transformMat(0, j) = static_cast<float>(R.m_values[0][j] * transform.s);
            transformMat(1, j) = static_cast<float>(R.m_values[1][j] * transform.s);
            transformMat(2, j) = static_cast<float>(R.m_values[2][j] * transform.s);
        }
    }
    transformMat(0, 3) = transform.T.x;
    transformMat(1, 3) = transform.T.y;
    transformMat(2, 3) = transform.T.z;
    transformMat(3, 3) = transform.s;

    std::cout << "CloudCompare ICP 计算完成，转换矩阵：\n" << transformMat << std::endl;

    // **6. 应用变换矩阵到点云**
    pcl::PointCloud<processingPointT>::Ptr alignedCloud(new pcl::PointCloud<processingPointT>);
    pcl::transformPointCloud(*sourceCloud, *alignedCloud, transformMat);

    // **7. 返回结果**
    result.alignedCloud = alignedCloud;
    result.transformation = transformMat;
    return result;
}

//实现pcl::PointXYZRGBA向pcl::PointXYZ数据类型的转换
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessingThread::convertToXYZ(pcl::PointCloud<processingPointT>::Ptr inputCloud)
{
    // 空指针检查
    if (!inputCloud || inputCloud->empty()) 
    {
        //qWarning() << "convertToXYZ: inputCloud is null or empty!";
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : inputCloud->points)
    {
        outputCloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }
    outputCloud->width = outputCloud->points.size();
    outputCloud->height = 1;
    outputCloud->is_dense = inputCloud->is_dense;
    return outputCloud;
}

//实现pcl::PointXYZ向pcl::PointXYZRGBA数据类型的转换
pcl::PointCloud<processingPointT>::Ptr PointCloudProcessingThread::convertXYZRGBA(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    // 空指针检查
    if (!inputCloud || inputCloud->empty()) {
        //qWarning() << "convertToProcessingPointT: inputCloud is null or empty!";
        return pcl::PointCloud<processingPointT>::Ptr(new pcl::PointCloud<processingPointT>());
    }

    pcl::PointCloud<processingPointT>::Ptr outputCloud(new pcl::PointCloud<processingPointT>);

    for (const auto& pt : inputCloud->points)
    {
        processingPointT newPt;
        newPt.x = pt.x;
        newPt.y = pt.y;
        newPt.z = pt.z;
        newPt.r = 255;  // 设为白色
        newPt.g = 255;
        newPt.b = 255;
        newPt.a = 255;  // 透明度
        outputCloud->push_back(newPt);
    }

    outputCloud->width = outputCloud->points.size();
    outputCloud->height = 1;
    outputCloud->is_dense = inputCloud->is_dense;

    return outputCloud;
}

//使用RANSAC方法求解变换矩阵
Eigen::Matrix4f PointCloudProcessingThread::WithRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudSrc,pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudTgt,
    double distance_threshold,
    int max_iterations,
    int min_inliers)//使用RANSAC计算R、T
{
    auto start = std::chrono::high_resolution_clock::now();  // 计时开始
    int numPoints = CloudSrc->size();

    Eigen::Matrix4f BestTransformation = Eigen::Matrix4f::Identity();

    float best_rmse = std::numeric_limits<float>::max();

    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> best_inlier_pairs;

    std::random_device rd;
    std::mt19937 gen(rd());

    int max_inliers_count = 0;
    for (int iteration = 0; iteration < max_iterations; ++iteration)
    {
        // 1. **随机选择匹配点对**
        std::vector<int> indices(numPoints);
        for (int i = 0; i < numPoints; ++i) indices[i] = i;
        std::shuffle(indices.begin(), indices.end(), gen);

        pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointSrc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointTgt(new pcl::PointCloud<pcl::PointXYZ>());
        CloudPointSrc->resize(3);
        CloudPointTgt->resize(3);

        for (int i = 0; i < 3; ++i)
        {
            int idx = indices[i];
            CloudPointSrc->points[i] = CloudSrc->points[idx];
            CloudPointTgt->points[i] = CloudTgt->points[idx];
        }

        // 2. **直接计算变换矩阵（取消全等三角形判据）**
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
        Eigen::Matrix4f transformation;
        TESVD.estimateRigidTransformation(*CloudPointSrc, *CloudPointTgt, transformation);

        // 3. **应用变换并计算内点数量**
        pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrcTransformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*CloudSrc, *CloudSrcTransformed, transformation);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(CloudTgt);

        int inliers_count = 0;
        std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> inlier_pairs;
        std::unordered_map<int, float> closest_matches;

        for (size_t i = 0; i < CloudSrcTransformed->size(); ++i)
        {
            pcl::PointXYZ src_pt = CloudSrcTransformed->points[i];
            std::vector<int> nn_indices(1);
            std::vector<float> nn_dists(1);

            if (kdtree.nearestKSearch(src_pt, 1, nn_indices, nn_dists) > 0)
            {
                int tgt_idx = nn_indices[0];
                float dist = nn_dists[0];

                if (dist <= distance_threshold)
                {
                    if (closest_matches.find(tgt_idx) == closest_matches.end() || dist < closest_matches[tgt_idx])
                    {
                        closest_matches[tgt_idx] = dist;
                        inlier_pairs.push_back({ CloudSrc->points[i], CloudTgt->points[tgt_idx] });
                    }
                }
            }
        }

        inliers_count = inlier_pairs.size();
        float rmse = 0.0;
        for (const auto& pair : inlier_pairs)
        {
            Eigen::Vector3f src_pt(pair.first.x, pair.first.y, pair.first.z);
            Eigen::Vector3f tgt_pt(pair.second.x, pair.second.y, pair.second.z);
            Eigen::Vector3f transformed_src = transformation.topLeftCorner<3, 3>() * src_pt +
                transformation.topRightCorner<3, 1>();
            float error = (transformed_src - tgt_pt).norm();
            rmse += error * error;
        }
        rmse = sqrt(rmse / inliers_count);

        if (inliers_count > max_inliers_count || (inliers_count == max_inliers_count && rmse < best_rmse))
        {
            max_inliers_count = inliers_count;
            best_rmse = rmse;
            BestTransformation = transformation;
            best_inlier_pairs = inlier_pairs;
        }
    }

    if (max_inliers_count == 0)
    {
        std::cerr << "RANSAC 失败，未找到有效变换！返回单位矩阵。" << std::endl;
        return Eigen::Matrix4f::Identity();
    }

    // 4. **RANSAC 结束后，使用所有最终内点重新求解 R_final 和 T_final**
    pcl::PointCloud<pcl::PointXYZ>::Ptr FinalCloudSrc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr FinalCloudTgt(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& pair : best_inlier_pairs)
    {
        FinalCloudSrc->points.push_back(pair.first);
        FinalCloudTgt->points.push_back(pair.second);
    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> final_svd;
    Eigen::Matrix4f FinalTransformation;
    final_svd.estimateRigidTransformation(*FinalCloudSrc, *FinalCloudTgt, FinalTransformation);

    return FinalTransformation;
}

void PointCloudProcessingThread::ConvertToPointCloud_1(const std::vector<cv::Point3f>& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // 清空原有点云数据
    cloud->clear();
    // 遍历每个cv::Point3f，并转换为pcl::PointXYZ
    for (const auto& pt : input) {
        pcl::PointXYZ pcl_point(pt.x, pt.y, pt.z);  //将OpenCV点转为PCL点
        cloud->push_back(pcl_point);  // 将点添加到点云中
    }
}

float PointCloudProcessingThread::computeAverageDistanceErrorKDTree_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    if (cloud1->points.empty() || cloud2->points.empty()) 
    {
        std::cerr << "Point clouds are empty!" << std::endl;
        return -1.0f;
    }

    // 创建 KD-Tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);  // 将 cloud2 构建为 KD-Tree

    float totalError = 0.0f;
    int N = cloud1->points.size();

    // 遍历 cloud1 中的每个点
    for (int i = 0; i < N; ++i) 
    {
        pcl::PointXYZ searchPoint = cloud1->points[i];

        // 搜索最近邻
        std::vector<int> nearestNeighbors(1);  // 存储最近邻的索引
        std::vector<float> squaredDistances(1); // 存储最近邻的距离

        if (kdtree.nearestKSearch(searchPoint, 1, nearestNeighbors, squaredDistances) > 0) 
        {
            // 获取最近邻点与当前点的距离
            float distance = sqrt(squaredDistances[0]);  // 计算欧几里得距离
            totalError += distance;
        }
    }

    // 计算平均距离误差
    float averageError = totalError / N;
    return averageError;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessingThread::loadPointCloudFromTXT_1(const std::string& file_path) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream infile(file_path);
    if (!infile.is_open()) 
    {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return nullptr;
    }

    std::string line;
    while (std::getline(infile, line)) 
    {
        std::istringstream iss(line);
        pcl::PointXYZ point;
        if (!(iss >> point.x >> point.y >> point.z)) 
        {
            break; // Error handling
        }
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

void PointCloudProcessingThread::CircleThreshold_1(std::vector<std::vector<float>>& Points, std::vector<std::vector<float>>& circles, double threshold)
{
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
    for (int i = 0; i < Points.size(); i++) {
        if (isKept[i]) {
            circles.push_back(Points[i]);
        }
    }
}

int PointCloudProcessingThread::ReadTxtMat_1(std::string& filename, std::vector<std::vector<double>>& data)
{
    std::ifstream infile(filename);

    if (!infile) {
        std::cerr << "Cannot open the file: " << filename << std::endl;
        return 1;
    }
    std::string line;

    while (std::getline(infile, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }
        data.push_back(row);
    }
    infile.close();
    return 0;
}

float PointCloudProcessingThread::calculateDistance3D_1(const cv::Point3f& p1, const cv::Point3f& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));// 求两个坐标点之间的欧式距离，差值平方以后相加再开方
}

void PointCloudProcessingThread::DistenceMatrix_1(std::vector<std::vector<float>>& circles, std::vector<std::vector<double>>& diff_map, cv::Mat& Q, std::vector<cv::Point3f>& XYZ, std::vector<std::vector<double>>& distanceMatrix, int& i)
{
    cout << "-------------------------------------" << endl;
    cout << "Image" << i << "标志点的三维坐标：" << endl;
    for (int i = 0; i < circles.size(); i++)
    {
        cv::Point3f temp_point;
        cv::Mat temp = (cv::Mat_<double>(1, 4) << circles[i][1], circles[i][0], diff_map[int(circles[i][0])][int(circles[i][1])], 1);// 创建一个包含圆心坐标和视差值的矩阵,diff_map[int(circles[i][0])][int(circles[i][1])] 获得圆心坐标的视差值
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
            float distance = calculateDistance3D_1(XYZ[i], XYZ[j]);
            temp.push_back(distance);
        }
        distanceMatrix.push_back(temp);
    }
    //// 打印结果矩阵
    //cout << "-------------------------------------" << endl;
    //std::cout << "Image" << i << "距离矩阵:" << std::endl;
    //PrintDistence(distanceMatrix);
}

int PointCloudProcessingThread::isPotentialMatch_1(const std::vector<double>& rowA, const std::vector<double>& rowB)
{
    int matchCount = 0;
    vector<pair<double, double>> matchedPairs;
    for (int i = 0; i < rowA.size(); ++i) {
        double valA = rowA[i];
        if (valA == 0) continue;
        for (int j = 0; j < rowB.size(); ++j) {
            double valB = rowB[j];
            if (valB == 0) continue;
            if (std::abs(valA - valB) < 0.4)
            {
                matchCount++;
            }
        }
    }
    return matchCount;
}

vector<vector<int>> PointCloudProcessingThread::findMatchingPoints_1(const vector<std::vector<double>>& A, const vector<vector<double>>& B, vector<vector<int>>& matchingPoints, vector<cv::Point3f>& P1, vector<cv::Point3f>& Q1, vector<cv::Point3f>& P2, vector<cv::Point3f>& Q2)
{
    for (size_t i = 0; i < A.size(); ++i)
    {
        for (size_t j = 0; j < B.size(); ++j)
        {
            int count = isPotentialMatch_1(A[i], B[j]);
            if (count >= 3)
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

//计算每个imagepath的图片数量
int PointCloudProcessingThread::countImagesInDirectory(const std::string& imagePath)
{
    int count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(imagePath))
    {
        if (entry.path().extension() == ".bmp")  // 统计 .BMP 图片文件
        { 
            count++;
        }
    }
    return count;
}

std::string PointCloudProcessingThread::matrixToString(const Eigen::Matrix4f& matrix)
{
    std::stringstream ss;
    ss << matrix;
    return ss.str();
}

void PointCloudProcessingThread::processPointCloudWithRANSAC()
{
    /*std::string step = "\\";
    std::string dataPath = this->dataPath;
    std::string fileName = "Venus";
    std::string imagePath = dataPath + step + fileName + step + "Image";
    std::string saveImagePath = dataPath + step + fileName + step + "SaveImage";
    std::string calibPath = dataPath + step + fileName + step + "calib\\circleGridstereoCalib.txt";*/

    // 使用 set 参数
    std::string CalibPath = set.calibPath;
    std::string ImagePath = set.imagePath;
    std::string CloudPointdataPath = set.cloudPointdataPath;

    // 加载标定结果
    emit logMessage("loading CalibFile...");
    Calibrator_Registration calib;
    //calib.calib_registration(CalibPath, false);
    emit logMessage("CalibPath: " + QString::fromStdString(CalibPath));
    try 
    {
        calib.calib_registration(CalibPath, false);
    }
    catch (const std::exception& e) 
    {
        emit logMessage(QString("Error loading calibration file: %1").arg(e.what()));
        return;
    }

    cv::Mat Q = calib.Q;

    int totalImages = countImagesInDirectory(ImagePath);
    for (int k = 1; k <= totalImages; k++)
    {
        if (isStopped) return;  // 线程是否被请求停止
        int i = k;
        int j = k + 1;

        std::string imgPathAngle1 = ImagePath + "\\" + std::to_string(i) + "_L.bmp";
        std::string imgPathAngle2 = ImagePath + "\\" + std::to_string(j) + "_L.bmp";

        cv::Mat imgAngle1 = cv::imread(imgPathAngle1);
        cv::Mat imgAngle2 = cv::imread(imgPathAngle2);
        if (imgAngle1.empty() || imgAngle2.empty())
        {
            emit logMessage("无法加载图像：" + QString::fromStdString(imgPathAngle1) + " 或 " + QString::fromStdString(imgPathAngle2));
            continue;
        }

        // 立体矫正
        cv::Mat imgCAngle1, imgGAngle1, imgCAngle2, imgGAngle2;
        calib.rectifySingleL(imgAngle1, imgCAngle1);
        calib.rectifySingleL(imgAngle2, imgCAngle2);
        cv::cvtColor(imgCAngle1, imgGAngle1, cv::COLOR_RGB2GRAY);
        cv::cvtColor(imgCAngle2, imgGAngle2, cv::COLOR_RGB2GRAY);

        // 获取标志点圆心
        AAMED aamed(2500, 2500);
        aamed.SetParameters(CV_PI / 3, 3.4, 0.77);
        aamed.run_FLED(imgGAngle1);
        std::vector<std::vector<float>> pointsAngle1;
        aamed.getPointsInfo(pointsAngle1);
        aamed.run_FLED(imgGAngle2);
        std::vector<std::vector<float>> pointsAngle2;
        aamed.getPointsInfo(pointsAngle2);

        // 过滤圆心
        std::vector<std::vector<float>> circlesAngle1, circlesAngle2;
        double threshold = 3;
        CircleThreshold_1(pointsAngle1, circlesAngle1, threshold);
        CircleThreshold_1(pointsAngle2, circlesAngle2, threshold);

        // 计算三维坐标
        std::string difMapAngle1Path = CloudPointdataPath + "\\" + "dif_map_Venus_" + std::to_string(i) + ".txt";
        std::string difMapAngle2Path = CloudPointdataPath + "\\" + "dif_map_Venus_" + std::to_string(j) + ".txt";
        std::vector<std::vector<double>> difMapAngle1, difMapAngle2;
        ReadTxtMat_1(difMapAngle1Path, difMapAngle1);
        ReadTxtMat_1(difMapAngle2Path, difMapAngle2);

        std::vector<cv::Point3f> pAllPoint, qAllPoint;
        std::vector<std::vector<double>> pAllDistanceMatrix, qAllDistanceMatrix;
        DistenceMatrix_1(circlesAngle1, difMapAngle1, Q, pAllPoint, pAllDistanceMatrix, i);
        DistenceMatrix_1(circlesAngle2, difMapAngle2, Q, qAllPoint, qAllDistanceMatrix, j);

        // 查找匹配点对
        std::vector<cv::Point3f> pAllMatch, qAllMatch;
        std::vector<std::vector<int>> pqAllMatchingPoints;
        findMatchingPoints_1(pAllDistanceMatrix, qAllDistanceMatrix, pqAllMatchingPoints, pAllPoint, qAllPoint, pAllMatch, qAllMatch);

        // RANSAC计算变换矩阵
        pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr qCloud(new pcl::PointCloud<pcl::PointXYZ>);
        ConvertToPointCloud_1(pAllMatch, pCloud);
        ConvertToPointCloud_1(qAllMatch, qCloud);

        Eigen::Matrix4f transformation = WithRANSAC(pCloud, qCloud, 0.01, 5000, 3);
        //输出变换矩阵到日志
        emit logMessage("WithRANSAC Eigen::Matrix4f transformation：" + QString::fromStdString(matrixToString(transformation)));

        // 应用变换到点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAngle1 = loadPointCloudFromTXT_1(CloudPointdataPath + "\\" + "PointCloud_Venus_" + std::to_string(i) + ".txt");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAngle2 = loadPointCloudFromTXT_1(CloudPointdataPath + "\\" + "PointCloud_Venus_" + std::to_string(j) + ".txt");
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloudAngle1, *transformedCloud, transformation);

        float mse = computeAverageDistanceErrorKDTree_1(transformedCloud, cloudAngle2);
        //输出拼接误差到日志
        emit logMessage(" WithRANSAC MSE: " + QString::number(mse));

        pcl::PointCloud<processingPointT>::Ptr coloredCloud = convertXYZRGBA(transformedCloud);

        emit newPointCloud(coloredCloud);
    }

    emit processingFinished();
}

// ************run()函数，点云拼接流程函数*********************************************
void PointCloudProcessingThread::run()
{
    if (!useICP)
    {
        emit logMessage("USE WithRANSAC Registration Way...");
        processPointCloudWithRANSAC();
        return;  // 标志点拼接方法处理完毕后退出 run() 函数
    }
    else
    {
        std::vector<std::string> files = getPointCloudFiles(IcpdataPath);
        std::sort(files.begin(), files.end());
        emit logMessage(QString("Find %1 point cloud files.").arg(files.size()));

        if (files.size() < 2)
        {
            emit logMessage("The number of point cloud files is less than 2. Registration cannot proceed.");
            emit processingFinished();
            return;
        }

        pcl::PointCloud<processingPointT>::Ptr refCloud = loadPointCloud(files[0]);
        if (!refCloud)
        {
            emit logMessage("Failed to load the reference point cloud (file 1).");
            emit processingFinished();
            return;
        }

        pcl::PointCloud<processingPointT>::Ptr finalCloud(new pcl::PointCloud<processingPointT>);
        *finalCloud = *refCloud;
        Eigen::Matrix4f cumulativeMat = Eigen::Matrix4f::Identity();

        // 逐步拼接点云
        for (size_t i = 1; i < files.size(); ++i)
        {
            if (isStopped)
                break;

            pcl::PointCloud<processingPointT>::Ptr sourceCloud = loadPointCloud(files[i]);
            pcl::PointCloud<processingPointT>::Ptr targetCloud = loadPointCloud(files[i - 1]);
            if (!sourceCloud || !targetCloud)
            {
                emit logMessage(QString("Failed to load point cloud file %1. Skipping this file.").arg(i + 1));
                continue;
            }

            emit updateProgress((i * 100) / files.size());
            auto start_time = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4f T;  // 定义变换矩阵
            double rmse = -1.0;
            {
                std::stringstream ss;
                // 调用 CloudCompare 的 ICP 方法
                ICPResult icpResult = alignPointCloudsCC(sourceCloud, targetCloud);
               
                T = icpResult.transformation;

                if (icpResult.alignedCloud)
                {
                    rmse = computeRMSE(icpResult.alignedCloud, targetCloud);
                    ss << "Registration succeeded for file " << i + 1 << " to file " << i
                        << ".\nRMSE (CloudCompare ICP): " << rmse
                        << "\nTransformation matrix:\n" << T;
                }
                else
                {
                    T = Eigen::Matrix4f::Identity();
                    ss << "CloudCompare ICP registration failed for file " << i + 1 << " to file " << i
                        << ", using identity matrix.";
                }
                emit logMessage(QString::fromStdString(ss.str()));
            }

            cumulativeMat = T * cumulativeMat;  // 更新累计变换矩阵

            if (files.size() > 2)
            {
                std::stringstream final_ss;
                final_ss << "Cumulative transformation matrix (file " << i + 1 << " to file1):\n" << cumulativeMat;
                emit logMessage(QString::fromStdString(final_ss.str()));
            }

            pcl::PointCloud<processingPointT>::Ptr transformedCloud(new pcl::PointCloud<processingPointT>);
            pcl::transformPointCloud(*sourceCloud, *transformedCloud, cumulativeMat);

            // 计算重叠率
            pcl::KdTreeFLANN<processingPointT> kdtree;
            kdtree.setInputCloud(targetCloud);
            int overlapCount = 0;
            double distanceThreshold = 0.02;
            for (const auto& point : transformedCloud->points)
            {
                std::vector<int> pointIdx(1);
                std::vector<float> pointDist(1);
                if (kdtree.nearestKSearch(point, 1, pointIdx, pointDist) > 0)
                {
                    if (pointDist[0] < distanceThreshold)
                        overlapCount++;
                }
            }
            double overlapRatio = static_cast<double>(overlapCount) / transformedCloud->size();
            emit logMessage(QString("Overlap ratio (file %1 to file %2): %3%")
                .arg(i + 1).arg(i).arg(overlapRatio * 100, 0, 'f', 2));

            *finalCloud += *transformedCloud;

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            emit logMessage(QString("Time taken for registration (file %1 to file %2): %3 seconds")
                .arg(i + 1).arg(i).arg(elapsed.count(), 0, 'f', 6));

            std::string pairFilename = IcpsavePath + "/pair_cloud_" + std::to_string(i) + "_to_" + std::to_string(i - 1) + ".pcd";
            savePointCloud(pairFilename, transformedCloud);
            emit logMessage(QString("Saved merged point cloud: %1").arg(QString::fromStdString(pairFilename)));
        }  // for 循环结束

        // 循环结束后保存最终合并点云并发出信号
        std::string finalFilename = IcpsavePath + "/final_merged_cloud.pcd";
        savePointCloud(finalFilename, finalCloud);
        emit logMessage(QString("Final merged point cloud saved: %1").arg(QString::fromStdString(finalFilename)));
        emit newPointCloud(finalCloud);
        emit processingFinished();
    }
}



