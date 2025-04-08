//#include <algorithm>
//#include <boost/thread/thread.hpp>
//#include <Pcl/kdtree/kdtree_flann.h>
//#include <pcl/registration/icp.h>
//#include <pcl/common/concatenate.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <cmath>
//#include "FLED.h"
//#include "MeanShift.h"
//#include "tools.h"
//#include "MyTool.h"
//#include "Calibrator_our.h"
//#include <pcl/point_types.h>
//#include <pcl/common/common.h>
//#include <array>
//#include <chrono>
//#include <random>
//
//using namespace pcl;
//using namespace std;
//using namespace cv;
//// 计算法向量
//pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
//{
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
//	n.setInputCloud(target_cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(10);
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	n.compute(*normals);
//
//	return normals;
//}
//float computePointToPlaneRMSE(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, pcl::PointCloud<pcl::PointXYZ>::ConstPtr target)
//{
//	pcl::PointCloud<pcl::Normal>::Ptr normals_target = computeNormal(target);
//
//	float rmse = 0.0;
//
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//	tree->setInputCloud(target);
//	for (auto point_i : *source)
//	{
//		// 判断是否有无效点，如果有则跳过该点
//		if (!std::isfinite(point_i.x) || !std::isfinite(point_i.y) || !std::isfinite(point_i.z))
//			continue;
//		// K近邻搜索查找最近邻点
//		pcl::Indices nn_indices(1);
//		std::vector<float> nn_distances(1);
//		if (!tree->nearestKSearch(point_i, 1, nn_indices, nn_distances))
//			continue;
//		std::size_t point_nn_i = nn_indices.front(); // front返回当前vector容器中起始元素的引用
//		// 使用Map将点坐标和法向量转化为Eigen支持的数据结构
//		Eigen::Vector3f normal_target = (*normals_target)[point_nn_i].getNormalVector3fMap(),
//			point_source = point_i.getVector3fMap(),
//			point_target = (*target)[point_nn_i].getVector3fMap();
//		// 使用向量内积，计算点到面的距离
//		float dist = normal_target.dot(point_source - point_target);
//		rmse += dist * dist;
//
//	}
//	rmse = std::sqrt(rmse / static_cast<float> (source->size()));
//
//	return rmse;
//}
//Eigen::Matrix4f WithRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudSrc, pcl::PointCloud<pcl::PointXYZ>::Ptr& CloudTgt, double distance_threshold, int max_iterations, int min_inliers) {
//	auto start = std::chrono::high_resolution_clock::now();  // 开始计时
//
//	int nSrcPoint = CloudSrc->size();
//	int nTgtPoint = CloudTgt->size();
//	Eigen::Matrix4f BestTransformation = Eigen::Matrix4f::Identity();
//	int max_inliers_count = 0;
//
//	std::default_random_engine rng(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
//	std::uniform_int_distribution<int> dist_src(0, nSrcPoint - 1);
//	std::uniform_int_distribution<int> dist_tgt(0, nTgtPoint - 1);
//	// 创建随机数生成器
//	std::random_device rd;
//	std::mt19937 gen(rd());
//	std::uniform_int_distribution<> dis1(0, CloudSrc->points.size() - 1);
//	std::uniform_int_distribution<> dis2(0, CloudTgt->points.size() - 1);
//	for (int iteration = 0; iteration < max_iterations; ++iteration) 
//	{
//		// 随机选择三点对
//		pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointSrc(new pcl::PointCloud<pcl::PointXYZ>());
//		pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointTgt(new pcl::PointCloud<pcl::PointXYZ>());
//
//		CloudPointSrc->width = 3;
//		CloudPointSrc->height = 1;
//		CloudPointSrc->is_dense = false;
//		CloudPointSrc->resize(CloudPointSrc->width * CloudPointSrc->height);
//
//		CloudPointTgt->width = 3;
//		CloudPointTgt->height = 1;
//		CloudPointTgt->is_dense = false;
//		CloudPointTgt->resize(CloudPointTgt->width * CloudPointTgt->height);
//
//		for (int i = 0; i < 3; ++i) 
//		{
//			int src_index = dis1(gen);
//			int tgt_index = dis2(gen);
//			CloudPointSrc->points[i] = CloudSrc->points[src_index];
//			CloudPointTgt->points[i] = CloudTgt->points[tgt_index];
//		}
//
//			// 使用SVD计算变换矩阵
//			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
//			Eigen::Matrix4f transformation;
//			TESVD.estimateRigidTransformation(*CloudPointSrc, *CloudPointTgt, transformation);
//
//			// 应用变换并计算内点数量
//			pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSrcTransformed(new pcl::PointCloud<pcl::PointXYZ>());
//			pcl::transformPointCloud(*CloudSrc, *CloudSrcTransformed, transformation);
//
//			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//			kdtree.setInputCloud(CloudTgt);
//			int inliers_count = 0;
//			std::vector<int> nn_indices(1);
//			std::vector<float> nn_dists(1);
//
//			for (const auto& point : CloudSrcTransformed->points) 
//			{
//				kdtree.nearestKSearch(point, 1, nn_indices, nn_dists);
//				if (nn_dists[0] <= distance_threshold) 
//				{
//					++inliers_count;
//				}
//			}
//			// 更新最优变换矩阵
//			if (inliers_count > max_inliers_count && inliers_count >= min_inliers) 
//			{
//				max_inliers_count = inliers_count;
//				BestTransformation = transformation;
//			}
//	}
//	return BestTransformation;
//}
//int main()
//{
//	string Step = "\\";
//	string FileName = "David";
//	string DataPath = "E:\\Pointcloud" + Step + FileName + Step + "DataPath";
//	string CalibPath = "E:\\Pointcloud" + Step + FileName + Step + "calib\\stereoCalib.txt";
//	string ImagePath = "E:\\Pointcloud" + Step + FileName + Step + "Image";
//	string SavePointPath = "E:\\Pointcloud" + Step + FileName + Step + "SavePoint";
//	string SaveImagePath = "E:\\Pointcloud" + Step + FileName + Step + "SavaImage";
//	//****************************1、加载标定结果**************************//
//	Calibrator calibrator;
//	Calibrator imagenumber;
//	calibrator.calib(CalibPath, false);
//	cv::Mat Q = calibrator.Q;
//	cout << "投影矩阵：" << endl << Q << endl;
//	//****************************2、读取标志点图片**************************//
//	for (int k = 1; k <=14; k++)
//	{
//		int i = k;
//		int j = k+1;
//		string ImgPathAngle1 = ImagePath + Step + to_string(i) + "_l.bmp";
//		string ImgPathAngle2 = ImagePath + Step + to_string(j) + "_l.bmp";
//		Mat imgCAngle1, imgGAngle1;
//		Mat imgCAngle2, imgGAngle2;
//		cv::Mat imgAngle1 = cv::imread(ImgPathAngle1);
//		cv::Mat imgAngle2 = cv::imread(ImgPathAngle2);
//		calibrator.rectifySingleL(imgAngle1, imgCAngle1);
//		calibrator.rectifySingleL(imgAngle2, imgCAngle2);// 立体矫正
//		cv::cvtColor(imgCAngle1, imgGAngle1, cv::COLOR_RGB2GRAY);// 矫正后转为灰度图
//		cv::cvtColor(imgCAngle2, imgGAngle2, cv::COLOR_RGB2GRAY);
//		//****************************3、获取标志点圆心**************************//
//		AAMED aamed(2500, 2500);
//		aamed.SetParameters(CV_PI / 3, 3.4, 0.77); // config parameters of AAMED
//		aamed.run_FLED(imgGAngle1); // run AAMED
//		vector<vector<float>> PointsAngle1;
//		aamed.getPointsInfo(PointsAngle1);
//
//		//aamed.drawFLED(imgGAngle1, SaveImagePath + Step + to_string(i) + "_Result.bmp");
//		aamed.run_FLED(imgGAngle2); // run AAMED
//		vector<vector<float>> PointsAngle2;
//		aamed.getPointsInfo(PointsAngle2);
//
//		//aamed.drawFLED(imgGAngle2, SaveImagePath + Step + to_string(j) + "_Result.bmp");
//		std::vector<std::vector<float>> circlesAngle1;
//		std::vector<std::vector<float>> circlesAngle2;
//		double threshold = 3;
//		CircleThreshold(PointsAngle1, circlesAngle1, threshold);// 比较拟合的大小椭圆的长轴值，取拟合大圆的圆心作为标志点的圆心
//		CircleThreshold(PointsAngle2, circlesAngle2, threshold);
//		cout << "-------------------------------------" << endl;
//		cout << "拟合图像" + to_string(i) + "标志点圆心" << endl;
//		PrintCirclePoints(circlesAngle1);
//		cout << "-------------------------------------" << endl;
//		cout << "拟合图像" + to_string(j) + "标志点圆心" << endl;
//		PrintCirclePoints(circlesAngle2);
//		//****************************4、计算标志点三维坐标**************************//
//		string difMapAngle1Path = DataPath + Step + "dif_map_angle" + to_string(i) + ".txt";//读取两个角度的视差图
//		string difMapAngle2Path = DataPath + Step + "dif_map_angle" + to_string(j) + ".txt";
//		vector<std::vector<double>> difMapAngle1;
//		vector<std::vector<double>> difMapAngle2;
//		ReadTxtMat(difMapAngle1Path, difMapAngle1);
//		ReadTxtMat(difMapAngle2Path, difMapAngle2);
//		vector<std::vector<double>> P_All_distanceMatrix;
//		vector<std::vector<double>> Q_All_distanceMatrix;
//		vector<cv::Point3f> P_All_Point;
//		vector<cv::Point3f> Q_All_Point;
//		DistenceMatrix(circlesAngle1, difMapAngle1, Q, P_All_Point, P_All_distanceMatrix, i);// 根据拟合圆心的坐标和视差图矩阵，还有重投影矩阵Q，计算标志点的三维坐标，并且计算标志点之间的欧式距离(距离矩阵)
//		DistenceMatrix(circlesAngle2, difMapAngle2, Q, Q_All_Point, Q_All_distanceMatrix, j);
//		pcl::PointCloud<pcl::PointXYZ>::Ptr P_Temp(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PointCloud<pcl::PointXYZ>::Ptr Q_Temp(new pcl::PointCloud<pcl::PointXYZ>);
//		ConvertToPointCloud(P_All_Point, P_Temp);
//		ConvertToPointCloud(Q_All_Point, Q_Temp);
//		//****************************5、计算全部潜在匹配点对**************************//
//		cout << "开始计时" << endl;
//		auto start = std::chrono::high_resolution_clock::now();  // 开始计时
//		vector<vector<int>> PQ_All_matchingPoints;
//		vector<cv::Point3f> P_All_Match;
//		vector<cv::Point3f> Q_All_Match;//全部潜在匹配
//		findMatchingPoints(P_All_distanceMatrix, Q_All_distanceMatrix, PQ_All_matchingPoints, P_All_Point, Q_All_Point, P_All_Match, Q_All_Match);
//		//****************************6、基于随机抽样一致性寻找模型**************************//
//		pcl::PointCloud<pcl::PointXYZ>::Ptr P_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PointCloud<pcl::PointXYZ>::Ptr Q_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
//		ConvertToPointCloud(P_All_Match, P_Cloud);
//		ConvertToPointCloud(Q_All_Match, Q_Cloud);
//		double distance_threshold = 0.5;
//		int max_iterations = 80000;
//		int min_inliers = 3;
//		Eigen::Matrix4f transformation = WithRANSAC(P_Cloud, Q_Cloud, distance_threshold, max_iterations, min_inliers);
//		auto end = std::chrono::high_resolution_clock::now();  // 结束计时
//		std::chrono::duration<double> elapsed = end - start;   // 计算时间差
//		cout << "匹配时间: " << elapsed.count() << " 秒" << std::endl;
//		cout << "结束计时" << endl;
//		cout << "旋转平移矩阵：" << endl << transformation << endl;
//		//****************************10、应用变换结果**************************//
//		PointCloud<PointXYZ>::Ptr CloudAngle1 = loadPointCloudFromTXT(DataPath + Step + "pcl_angle" + to_string(i) + ".txt");
//		PointCloud<PointXYZ>::Ptr CloudAngle2 = loadPointCloudFromTXT(DataPath + Step + "pcl_angle" + to_string(j) + ".txt");
//		PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//		transformPointCloud(*CloudAngle1, *transformed_cloud, transformation);
//		float mse = computeAverageDistanceErrorKDTree(transformed_cloud, CloudAngle2);
//		cout << "平均距离误差:" << mse << endl;
//		cout << "----------------------------------------------------------" << endl;
//		visualizeCloud(transformed_cloud, CloudAngle2);
//	}
//	return 0;
//}