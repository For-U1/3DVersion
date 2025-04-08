#pragma once
#include <algorithm>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <boost/thread/thread.hpp>
#include <Pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h> // ICP with normals
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/common/concatenate.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cmath>
#include "FLED.h"
#include "MeanShift.h"
#include "PclTool.h"
#include "Calibrator_our.h"
#include <pcl/point_types.h>
#include <pcl/recognition/ransac_based/trimmed_icp.h>
#include <pcl/recognition/ransac_based/auxiliary.h>//tricp头文件
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/sample_consensus_prerejective.h>//　随机采样一致性配准
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <array>
#include <chrono>
#include <random>

using namespace std;
using namespace pcl;
// 使用K-d树找到离中心最近的k个点
void findNearestPointsUsingKdTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double cx, double cy, double cz, int k, pcl::PointCloud<pcl::PointXYZ>::Ptr& nearestPoints) {
	// 构建K-d树
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 创建一个目标点
	pcl::PointXYZ searchPoint(cx, cy, cz);

	// 存储最近邻点的索引和距离
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);

	// 使用K-d树进行k近邻搜索
	if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		nearestPoints->clear();
		for (int i = 0; i < k; ++i) {
			nearestPoints->points.push_back(cloud->points[pointIdxNKNSearch[i]]);
		}
	}
}
void ConvertToPointCloud(const std::vector<cv::Point3f>& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{
	// 清空原有点云数据
	cloud->clear();
	// 遍历每个cv::Point3f，并转换为pcl::PointXYZ
	for (const auto& pt : input) {
		pcl::PointXYZ pcl_point(pt.x, pt.y, pt.z);  //将OpenCV点转为PCL点
		cloud->push_back(pcl_point);  // 将点添加到点云中
	}
}
void ConvertToVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, vector<cv::Point3f>& input) {
	// 遍历每个pcl::PointXYZ，并转换为cv::Point3f
	input.clear();  // 清空目标 vector，以避免旧数据干扰
	for (const auto& pt : cloud->points) {
		cv::Point3f point(pt.x, pt.y, pt.z);  // 将PCL点转为OpenCV点
		input.push_back(point);  // 将点添加到目标vector中
	}
}
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromTXT(const std::string& file_path) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::ifstream infile(file_path);
	if (!infile.is_open()) {
		std::cerr << "无法打开文件: " << file_path << std::endl;
		return nullptr;
	}

	std::string line;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		pcl::PointXYZ point;
		if (!(iss >> point.x >> point.y >> point.z)) {
			break; // Error handling
		}
		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;

	return cloud;
}
void visualizeCloud(PointCloud<pcl::PointXYZ>::Ptr& trans_cloud, PointCloud<pcl::PointXYZ>::Ptr& target_cloud)
{
	// 初始化点云可视化对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("AfterRTEnd");
	//对转换后的源点云着色 (green)可视化.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trans_color(trans_cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(trans_cloud, trans_color, "output cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}
void visualizeCloud_S(PointCloud<pcl::PointXYZ>::Ptr& trans_cloud)
{
	// 初始化点云可视化对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("点云"));
	viewer->setBackgroundColor(30, 60, 120);
	viewer->setWindowName("显示点云");
	//对转换后的源点云着色 (green)可视化.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trans_color(trans_cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(trans_cloud, trans_color, "output cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}
void visualizeCloud_T(PointCloud<pcl::PointXYZ>::Ptr& target_cloud)
{
	// 初始化点云可视化对象
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("点云"));
	viewer->setBackgroundColor(255, 255, 255);
	viewer->setWindowName("显示点云");
	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}
double calculateDistance_test(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
	return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}
void removeDuplicates(pcl::PointCloud<pcl::PointXYZ>::Ptr& P_double_prime, pcl::PointCloud<pcl::PointXYZ>::Ptr& Q_double_prime) {
	std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> uniquePairs;

	for (size_t i = 0; i < P_double_prime->points.size(); ++i) {
		bool exists = false;
		// 检查当前点对是否已经存在
		for (const auto& pair : uniquePairs) {
			if (calculateDistance_ZP(pair.first, P_double_prime->points[i]) < 1e-3f &&
				calculateDistance_ZP(pair.second, Q_double_prime->points[i]) < 1e-3f) {
				exists = true;
				break;
			}
		}

		// 如果点对不存在，则添加到 uniquePairs
		if (!exists) {
			uniquePairs.emplace_back(P_double_prime->points[i], Q_double_prime->points[i]);
		}
	}

	// 清空原点云
	P_double_prime->points.clear();
	Q_double_prime->points.clear();

	// 重新填充去重后的点
	for (const auto& pair : uniquePairs) {
		P_double_prime->points.push_back(pair.first);
		Q_double_prime->points.push_back(pair.second);
	}

	// 更新点云的宽度（总点数）和高度
	P_double_prime->width = P_double_prime->points.size();
	P_double_prime->height = 1; // 点云是单行的
	Q_double_prime->width = Q_double_prime->points.size();
	Q_double_prime->height = 1;
}
bool arePointsDistinct(const PointXYZ& p1, const PointXYZ& p2, float epsilon = 0.0001f) {
	return calculateDistance_ZP(p1, p2) > epsilon;
}
bool areTrianglesCongruent(const PointXYZ& p1, const PointXYZ& p2, const PointXYZ& p3, const PointXYZ& q1, const PointXYZ& q2, const PointXYZ& q3) {
	// 计算第一个三角形的边长
	float a1 = calculateDistance_ZP(p1, p2);
	float b1 = calculateDistance_ZP(p2, p3);
	float c1 = calculateDistance_ZP(p3, p1);
	// 计算第二个三角形的边长
	float a2 = calculateDistance_ZP(q1, q2);
	float b2 = calculateDistance_ZP(q2, q3);
	float c2 = calculateDistance_ZP(q3, q1);
	// 将边长排序
	vector<float> sides1 = { a1, b1, c1 };
	vector<float> sides2 = { a2, b2, c2 };
	//sort(sides1.begin(), sides1.end());
	//sort(sides2.begin(), sides2.end());
	// 允许一定的误差
	const float epsilon = 0.8f;
	for (size_t i = 0; i < 3; ++i) {
		if (abs(sides1[i] - sides2[i]) > epsilon)
			return false;
	}
	return true;
}
bool areTrianglesIsosceles(const PointXYZ& p1, const PointXYZ& p2, const PointXYZ& p3, const PointXYZ& q1, const PointXYZ& q2, const PointXYZ& q3) {
	// 计算第一个三角形的边长
	float a1 = calculateDistance_ZP(p1, p2);
	float b1 = calculateDistance_ZP(p2, p3);
	float c1 = calculateDistance_ZP(p1, p3);

	// 计算第二个三角形的边长
	float a2 = calculateDistance_ZP(q1, q2);
	float b2 = calculateDistance_ZP(q2, q3);
	float c2 = calculateDistance_ZP(q1, q3);
	const float epsilon = 0.5f;

	//// 检查第一个三角形是否等腰
	//bool isIso1 = (abs(a1 - b1) < epsilon) || (abs(b1 - c1) < epsilon) || (abs(c1 - a1) < epsilon);
	//// 检查第二个三角形是否等腰
	//bool isIso2 = (abs(a2 - b2) < epsilon) || (abs(b2 - c2) < epsilon) || (abs(c2 - a2) < epsilon);
	// 检查第一个三角形是否等腰
	bool isIso1 = (abs(c1 - a1) < epsilon);
	// 检查第二个三角形是否等腰
	bool isIso2 = (abs(c2 - a2) < epsilon);

	return isIso1 && isIso2;
}
void calculateCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double& cx, double& cy, double& cz) {
	cx = cy = cz = 0.0;
	for (size_t i = 0; i < cloud->size(); i++) {
		cx += cloud->points[i].x;
		cy += cloud->points[i].y;
		cz += cloud->points[i].z;
	}
	cx /= cloud->size();
	cy /= cloud->size();
	cz /= cloud->size();
}
void FindBasePoint(vector<std::vector<double>>& P_distanceMatrix, vector<std::vector<double>>& Q_distanceMatrix, vector<cv::Point3f> P_Three_Match, vector<cv::Point3f> Q_Three_Match, vector<cv::Point3f>& P_Base, vector<cv::Point3f>& Q_Base)
{
	vector<vector<double>> distanceMatrix(P_distanceMatrix.size(), vector<double>(P_distanceMatrix[0].size()));
	int max = 0;
	for (size_t i = 0; i < distanceMatrix.size(); ++i)
	{
		int num = 0;
		for (size_t j = 0; j < distanceMatrix[i].size(); ++j)
		{
			distanceMatrix[i][j] = abs(Q_distanceMatrix[i][j] - P_distanceMatrix[i][j]);
			if (distanceMatrix[i][j] < 0.3)
			{
				num++;
			}
		}
		//cout << "最大匹配数量：" << num << endl;
		if (num > max)
		{
			max = num;
			P_Base.clear();
			Q_Base.clear();
			P_Base.push_back(P_Three_Match[i]);
			Q_Base.push_back(Q_Three_Match[i]);// 基准点对
		}
		//else if (num == max)
		//{
		//	P_Base.push_back(P_Three_Match[i]);
		//	Q_Base.push_back(Q_Three_Match[i]);
		//}
	}
	for (size_t i = 0; i < P_Base.size(); i++)
	{
		cout << "基准点对"<<i+1<<":" << P_Base[i] << "-" << Q_Base[i] << endl;
	}
}
void findPublicMatchedPointPairs(PointCloud<PointXYZ>::Ptr P_prime, PointCloud<PointXYZ>::Ptr Q_prime, PointCloud<PointXYZ>::Ptr p_base, PointCloud<PointXYZ>::Ptr q_base,PointCloud<PointXYZ>::Ptr &P_F_Match, PointCloud<PointXYZ>::Ptr& Q_F_Match)
{
	double distance_threshold = 0.4;

	size_t k = P_prime->points.size();
	size_t n = p_base->points.size();
	// 构建 k-d 树用于目标点云
	PointCloud<PointXYZ>::Ptr cloud_tgt(new PointCloud<PointXYZ>());
	for (const auto& q : Q_prime->points) {
		cloud_tgt->points.emplace_back(q);
	}
	KdTreeFLANN<PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_tgt);
	// Step 2: 遍历所有i和j，寻找全等三角形
	PointXYZ p_o1 = p_base->points[0];
	PointXYZ q_o1 = q_base->points[0];
	P_F_Match->push_back(p_o1);
	Q_F_Match->push_back(q_o1);
	for (size_t i = 0; i < k; ++i)
	{
		PointXYZ p_o2 = P_prime->points[i];
		PointXYZ q_o2 = Q_prime->points[i];
		for (size_t j = 0; j < k; ++j)
		{
			PointXYZ p_o3 = P_prime->points[j];
			PointXYZ q_o3 = Q_prime->points[j];
			// 确保 p_o1, p_o2, p_o3 和 q_o1, q_o2, q_o3 都是不同的
			if (!(arePointsDistinct(p_o1, p_o2) &&
				arePointsDistinct(p_o1, p_o3) &&
				arePointsDistinct(p_o2, p_o3) &&
				arePointsDistinct(q_o1, q_o2) &&
				arePointsDistinct(q_o1, q_o3) &&
				arePointsDistinct(q_o2, q_o3))) {
				continue; // 如果有重复点，跳过此次循环
			}
			if (areTrianglesCongruent(p_o1, p_o2, p_o3, q_o1, q_o2, q_o3))
			{
				//cout << "找到一对全等三角形" << endl;
				PointCloud<PointXYZ>::Ptr cloud_src(new PointCloud<PointXYZ>());
				PointCloud<PointXYZ>::Ptr cloud_tgt_triangle(new PointCloud<PointXYZ>());
				cloud_src->points.emplace_back(p_o1);
				cloud_src->points.emplace_back(p_o2);
				cloud_src->points.emplace_back(p_o3);

				cloud_tgt_triangle->points.emplace_back(q_o1);
				cloud_tgt_triangle->points.emplace_back(q_o2);
				cloud_tgt_triangle->points.emplace_back(q_o3);
				// 计算刚性变换矩阵
				registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
				Eigen::Matrix4f transformation;
				TESVD.estimateRigidTransformation(*cloud_src, *cloud_tgt_triangle, transformation);
				// 如果变换矩阵是单位矩阵，说明无法计算，跳过
				if (transformation.isIdentity()) {
					continue;
				}
				// 应用变换到 Q_prime，得到 Q_P
				PointCloud<PointXYZ>::Ptr cloud_P_Q(new PointCloud<PointXYZ>());
				transformPointCloud(*P_prime, *cloud_P_Q, transformation);
				// 计算重叠点数量
				int inliers = 0;
				vector<int> nn_indices(1);
				vector<float> nn_dists(1);
				for (size_t i = 0; i < cloud_P_Q->points.size(); i++)
				{
					kdtree.nearestKSearch(cloud_P_Q->points[i], 1, nn_indices, nn_dists);
					if (nn_dists[0] <= distance_threshold)
					{
						inliers++;
					}
				}
				//cout << "交换前重叠点数量：" << inliers << endl;
				if (areTrianglesIsosceles(p_o1, p_o2, p_o3, q_o1, q_o2, q_o3))// 等腰三角形
				{
					//cout << "该全等三角形是等腰三角形" << endl;
					// 交换 p_o2 和 p_o3
					PointXYZ p_o2_swapped = p_o3;
					PointXYZ p_o3_swapped = p_o2;

					// 重新计算变换矩阵
					PointCloud<PointXYZ>::Ptr cloud_src_swapped(new PointCloud<PointXYZ>());
					PointCloud<PointXYZ>::Ptr cloud_tgt_swapped(new PointCloud<PointXYZ>());
					cloud_src_swapped->points.emplace_back(p_o1);
					cloud_src_swapped->points.emplace_back(p_o2_swapped);
					cloud_src_swapped->points.emplace_back(p_o3_swapped);

					cloud_tgt_swapped->points.emplace_back(q_o1);
					cloud_tgt_swapped->points.emplace_back(q_o2);
					cloud_tgt_swapped->points.emplace_back(q_o3);
					Eigen::Matrix4f transformation_swapped;
					TESVD.estimateRigidTransformation(*cloud_src_swapped, *cloud_tgt_swapped, transformation_swapped);
					// 如果变换矩阵是单位矩阵，说明无法计算，跳过
					if (transformation_swapped.isIdentity()) {
						continue;
					}
					// 应用变换到 Q_prime，得到 Q_P1
					PointCloud<PointXYZ>::Ptr cloud_P1_Q(new PointCloud<PointXYZ>());
					transformPointCloud(*P_prime, *cloud_P1_Q, transformation_swapped);
					// 计算重叠点数量 number1
					int inliers_swapped = 0;
					vector<int> nn_indices_swapped(1);
					vector<float> nn_dists_swapped(1);
					for (size_t i = 0; i < cloud_P1_Q->points.size(); i++)
					{
						kdtree.nearestKSearch(cloud_P1_Q->points[i], 1, nn_indices_swapped, nn_dists_swapped);
						// 最近邻搜索
						if (nn_dists_swapped[0] <= distance_threshold)
						{
							inliers_swapped++;
						}
					}
					//cout << "交换之后重叠点数量：" << inliers_swapped << endl;
					if (inliers_swapped > inliers)
					{
						swap(p_o2, p_o3);
						P_F_Match->push_back(p_o2);
						P_F_Match->push_back(p_o3);
						Q_F_Match->push_back(q_o2);
						Q_F_Match->push_back(q_o3);
					}
				}
				if (inliers >= 3)
				{
					P_F_Match->push_back(p_o2);
					P_F_Match->push_back(p_o3);
					Q_F_Match->push_back(q_o2);
					Q_F_Match->push_back(q_o3);
				}
			}
		}
	}
	removeDuplicates(P_F_Match, Q_F_Match);
}
//float caculateRMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_source(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_target(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::copyPointCloud(*cloud_source, *xyz_source);
//	pcl::copyPointCloud(*cloud_target, *xyz_target);
//
//	float rmse = 0.0f;
//
//	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
//	tree->setInputCloud(xyz_target);
//
//	for (auto point_i : *xyz_source)
//	{
//		// 去除无效的点
//		if (!pcl_isfinite(point_i.x) || !pcl_isfinite(point_i.y) || !pcl_isfinite(point_i.z))
//			continue;
//		std::vector<int> nn_indices(1);
//		std::vector<float> nn_distances(1);
//		if (!tree->nearestKSearch(point_i, 1, nn_indices, nn_distances)) // K近邻搜索获取匹配点对
//			continue;
//		// // dist的计算方法之一
//		//size_t point_nn_i = nn_indices.front();
//		//float dist = squaredEuclideanDistance(point_i, xyz_target->points[point_nn_i]);
//		float dist = nn_distances[0]; // 获取最近邻对应点之间欧氏距离的平方
//		rmse += dist;                 // 计算平方距离之和
//	}
//	rmse = std::sqrt(rmse / static_cast<float> (xyz_source->points.size())); // 计算均方根误差
//
//	return rmse;
//}
void PrintPointCloud(const PointCloud<pcl::PointXYZ>::Ptr Pointcloud)
{
	cout << "点云坐标：" << endl;
	for (size_t i = 0; i < Pointcloud->size(); i++)
	{
		cout << Pointcloud->points[i].x << "-" << Pointcloud->points[i].y << "-" << Pointcloud->points[i].z << endl;
	}
	cout << "---------------------" << endl;
}
void PrintVector(const vector<cv::Point3f> P_Match, const vector<cv::Point3f> Q_Match)
{
	cout << "潜在匹配点对：" << endl;
	for (int i = 0; i < P_Match.size(); i++)
	{
		cout << P_Match[i] << "--" << Q_Match[i] << endl;
	}
	cout << "---------------------" << endl;
}
float computeAverageDistanceErrorKDTree(PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2) {
	if (cloud1->points.empty() || cloud2->points.empty()) {
		cerr << "Point clouds are empty!" << endl;
		return -1.0f;
	}

	// 创建 KD-Tree
	KdTreeFLANN<PointXYZ> kdtree;
	kdtree.setInputCloud(cloud2);  // 将 cloud2 构建为 KD-Tree

	float totalError = 0.0f;
	int N = cloud1->points.size();

	// 遍历 cloud1 中的每个点
	for (int i = 0; i < N; ++i) {
		PointXYZ searchPoint = cloud1->points[i];

		// 搜索最近邻
		std::vector<int> nearestNeighbors(1);  // 存储最近邻的索引
		std::vector<float> squaredDistances(1); // 存储最近邻的距离

		if (kdtree.nearestKSearch(searchPoint, 1, nearestNeighbors, squaredDistances) > 0) {
			// 获取最近邻点与当前点的距离
			float distance = sqrt(squaredDistances[0]);  // 计算欧几里得距离
			totalError += distance;
		}
	}

	// 计算平均距离误差
	float averageError = totalError / N;
	return averageError;
}
