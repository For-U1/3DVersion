#ifndef CH7_STRUCTURELIGHTSTEREOTRIANGLE_MODEL_H
#define CH7_STRUCTURELIGHTSTEREOTRIANGLE_MODEL_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>
#include <string>

using namespace std;

class Model {
    cv::Mat Q;
public:
    // Initialize the model with the given Q matrix
    bool init(cv::Mat &Q);

    // Calculate disparity and depth maps
    bool calcDistance(cv::Mat &dif_map, cv::Mat &depth_map, vector<pair<cv::Point2f, cv::Point2f>> &cps);

    // Save the 3D point cloud to a file
    bool saveXYZ(const string& filename, cv::Mat& depth_map, float min_z, float max_z);
};

#endif // CH7_STRUCTURELIGHTSTEREOTRIANGLE_MODEL_H
