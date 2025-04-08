//#include "Calibrate.h"
//
//bool CornerDetection::corner(const std::string& calib_file, bool showcornor, const std::string& savePath)
//{
//    clock_t star_time, end_time;
//    star_time = clock();
//    std::string saveFile = savePath + "/stereoCalib.txt";
//    double squareSize = 15.0;  // ���̸��ʵ�ʳߴ磬��λΪ����
//
//    vector<std::string> imageFileNamesL, imageFileNamesR;
//    for (int i = 1; i <= 20; ++i)
//    {
//        imageFileNamesL.push_back(calib_file + "/L/" + std::to_string(i) + ".bmp");//���ر궨ͼƬ���ļ���ʽ����Ƭ��������ʽ
//        imageFileNamesR.push_back(calib_file + "/R/" + std::to_string(i) + ".bmp");
//    }
//
//    vector<vector<Point2f>> imagePointsL, imagePointsR;
//    Size boardSize(11, 8); // ���̸�ߴ�
//
//    Size imageSize;
//
//    // �������������
//    vector<Point3f> worldPoints;
//    for (int i = 0; i < boardSize.height; i++)
//    {
//        for (int j = 0; j < boardSize.width; j++)
//        {
//            worldPoints.push_back(Point3f(j * squareSize, i * squareSize, 0));
//        }
//    }
//
//    // ����ͼ���ҵ����̽ǵ�
//    for (size_t i = 0; i < imageFileNamesL.size(); i++)
//    {
//        Mat imageL = imread(imageFileNamesL[i], IMREAD_GRAYSCALE);
//        if (imageL.empty()) {
//            cerr << "Failed to load left image: " << imageFileNamesL[i] << endl;
//            continue;  // ��������ͼƬ
//        }
//        Mat imageR = imread(imageFileNamesR[i], IMREAD_GRAYSCALE);
//        if (imageR.empty()) {
//            cerr << "Failed to load right image: " << imageFileNamesR[i] << endl;
//            continue;  // ��������ͼƬ
//        }
//        vector<Point2f> cornersL, cornersR;
//        bool foundL = findChessboardCorners(imageL, boardSize, cornersL);
//        cout << "Found corners in Left camera image " << std::to_string(i + 1) << endl;
//        bool foundR = findChessboardCorners(imageR, boardSize, cornersR);
//        cout << "Found corners in Right camera image " << std::to_string(i + 1) << endl;
//        if (foundL && foundR)
//        {
//            cornerSubPix(imageL, cornersL, Size(20, 20), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
//            cornerSubPix(imageR, cornersR, Size(20, 20), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
//            drawChessboardCorners(imageL, boardSize, cornersL, foundL);
//            drawChessboardCorners(imageR, boardSize, cornersR, foundR);
//            imagePointsL.push_back(cornersL);
//            imagePointsR.push_back(cornersR);
//
//            // ������нǵ��ͼ��ÿ��ͼ����Ψһ���ļ���
//            //std::string leftImagePath = savePath + "/left_image_with_corners_" + std::to_string(i + 1) + ".bmp";
//            //std::string rightImagePath = savePath + "/right_image_with_corners_" + std::to_string(i + 1) + ".bmp";
//            //bool leftSaved = cv::imwrite(leftImagePath, imageL);
//            //bool rightSaved = cv::imwrite(rightImagePath, imageR);
//
//            //if (!leftSaved || !rightSaved)
//            //{
//            //    std::cerr << "Failed to save images: "
//            //        << leftImagePath << " or " << rightImagePath << std::endl;
//            //    return false;  // �������ʧ�ܣ����� false
//            //}
//            //else
//            //{
//            //    std::cout << "Images saved successfully: "
//            //        << leftImagePath << " and " << rightImagePath << std::endl;
//            //}
//
//
//            if (imageSize == Size())
//            {
//                imageSize = imageL.size();
//            }
//            // ��ʾ���нǵ��ͼ��
//            // if (showcornor)
//            // {
//            //     namedWindow("Corners Left " + std::to_string(i + 1), WINDOW_NORMAL); // ����һ���ɵ�����С�Ĵ���
//            //     resizeWindow("Corners Left " + std::to_string(i + 1), 800, 600); // ���ô��ڴ�СΪ 800x600 ����
//            //     imshow("Corners Left " + std::to_string(i + 1), imageL);
//
//            //     namedWindow("Corners Right " + std::to_string(i + 1), WINDOW_NORMAL);
//            //     resizeWindow("Corners Right " + std::to_string(i + 1), 800, 600);
//            //     imshow("Corners Right " + std::to_string(i + 1), imageR);
//            //     waitKey(10);  // �ȴ�����������ʾ��һ��ͼ��
//            // }
//
//        }
//        else
//        {
//            cerr << "error" << i + 1 << endl;
//        }
//    }
//
//    if (imagePointsL.size() != imagePointsR.size() || imagePointsL.empty())
//    {
//        cerr << "Error: Number of corners detected in Left and Right images does not match or no corners detected." << endl;
//        return false;
//    }
//
//    // ׼�����������
//    vector<vector<Point3f>> objectPoints(imagePointsL.size(), worldPoints);
//    // ִ������궨
//    Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, R, T, E, F;
//    double rms = stereoCalibrate(
//        objectPoints, imagePointsL, imagePointsR,
//        cameraMatrixL, distCoeffsL,
//        cameraMatrixR, distCoeffsR,
//        imageSize, R, T, E, F,
//        CALIB_ZERO_DISPARITY,
//        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6)
//    );
//    end_time = clock();
//    cout << T << endl;
//    cout << "Camera calibration completed:  " << "total time:  " << (end_time - star_time) / 1000 << "s" << endl;
//    // ���뾶�������������
//    Mat radialDistCoeffsL = (Mat_<double>(1, 2) << distCoeffsL.at<double>(0, 0), distCoeffsL.at<double>(0, 1));
//    Mat tangentialDistCoeffsL = (Mat_<double>(1, 2) << distCoeffsL.at<double>(0, 2), distCoeffsL.at<double>(0, 3));
//    Mat radialDistCoeffsR = (Mat_<double>(1, 2) << distCoeffsR.at<double>(0, 0), distCoeffsR.at<double>(0, 1));
//    Mat tangentialDistCoeffsR = (Mat_<double>(1, 2) << distCoeffsR.at<double>(0, 2), distCoeffsR.at<double>(0, 3));
//    Mat T_new = (Mat_<double>(1, 3) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0));
//    // ����궨���
//    cout << T_new << endl;
//    ofstream file(saveFile);
//    saveSize(file, "imageSize", imageSize);
//    saveMat(file, "KK_L", cameraMatrixL);
//    saveMat(file, "KK_R", cameraMatrixR);
//    saveMat(file, "RadialDistortion_L", radialDistCoeffsL);
//    saveMat(file, "RadialDistortion_R", radialDistCoeffsR);
//    saveMat(file, "TangentialDistortion_L", tangentialDistCoeffsL);
//    saveMat(file, "TangentialDistortion_R", tangentialDistCoeffsR);
//    saveMat(file, "R", R);
//    saveMat(file, "T", T_new);
//    saveMat(file, "E", E);
//    saveMat(file, "F", F);
//    saveScalar(file, "error", rms);
//    file.close();
//    return true;
//}
//
//void CornerDetection::saveMat(ofstream& file, const std::string& name, const Mat& mat)
//{
//    file << name << endl;
//    for (int i = 0; i < mat.rows; ++i) {
//        for (int j = 0; j < mat.cols; ++j) {
//            file << fixed << setprecision(32) << setw(64) << mat.at<double>(i, j);
//            if (j != mat.cols - 1) file << "\t";
//        }
//        file << endl;
//    }
//    file << endl;
//}
//
//void CornerDetection::saveScalar(ofstream& file, const std::string& name, double value)
//{
//    file << name << endl;
//    file << fixed << setprecision(32) << setw(64) << value << endl << endl;
//}
//
//void CornerDetection::saveSize(ofstream& file, const std::string& name, const Size& size)
//{
//    file << name << endl;
//    file << fixed << setprecision(32) << setw(64) << static_cast<double>(size.height) << "\t"
//        << setw(64) << static_cast<double>(size.width) << "\t"
//        << setw(64) << 3.0 << endl << endl;
//}

// calibrate.cpp

#include "calibrate.h"

bool CornerDetection::corner(const std::string& calib_file, bool showcornor, const std::string& savePath,
    int boardWidth, int boardHeight, double squareSize,
    cv::Mat& R, cv::Mat& T, // ������ò���
    std::function<void(int)> progressCallback)
{
    clock_t star_time, end_time;
    star_time = clock();
    std::string saveFile = savePath + "/stereoCalib.txt";

    vector<std::string> imageFileNamesL, imageFileNamesR;
    for (int i = 1; i <= 20; ++i)
    {
        imageFileNamesL.push_back(calib_file + "/L/" + std::to_string(i) + ".bmp");
        imageFileNamesR.push_back(calib_file + "/R/" + std::to_string(i) + ".bmp");
    }

    vector<vector<cv::Point2f>> imagePointsL, imagePointsR;
    cv::Size boardSize(boardWidth, boardHeight);

    cv::Size imageSize;

    // �������������
    vector<cv::Point3f> worldPoints;
    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            worldPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    int totalImages = imageFileNamesL.size();

    // ����ͼ���ҵ����̽ǵ�
    for (size_t i = 0; i < imageFileNamesL.size(); i++)
    {
        cv::Mat imageL = cv::imread(imageFileNamesL[i], cv::IMREAD_GRAYSCALE);
        if (imageL.empty()) {
            cerr << "Failed to load left image: " << imageFileNamesL[i] << endl;
            continue;
        }
        cv::Mat imageR = cv::imread(imageFileNamesR[i], cv::IMREAD_GRAYSCALE);
        if (imageR.empty()) {
            cerr << "Failed to load right image: " << imageFileNamesR[i] << endl;
            continue;
        }
        vector<cv::Point2f> cornersL, cornersR;
        bool foundL = findChessboardCorners(imageL, boardSize, cornersL);
        cout << "Found corners in Left camera image " << std::to_string(i + 1) << endl;
        bool foundR = findChessboardCorners(imageR, boardSize, cornersR);
        cout << "Found corners in Right camera image " << std::to_string(i + 1) << endl;
        if (foundL && foundR)
        {
            cornerSubPix(imageL, cornersL, cv::Size(20, 20), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            cornerSubPix(imageR, cornersR, cv::Size(20, 20), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            drawChessboardCorners(imageL, boardSize, cornersL, foundL);
            drawChessboardCorners(imageR, boardSize, cornersR, foundR);
            imagePointsL.push_back(cornersL);
            imagePointsR.push_back(cornersR);

            if (imageSize == cv::Size())
            {
                imageSize = imageL.size();
            }
        }
        else
        {
            cerr << "Error finding corners in image " << i + 1 << endl;
        }

        // ���½���
        if (progressCallback)
        {
            int progressValue = static_cast<int>(((i + 1) * 100) / totalImages);
            progressCallback(progressValue);
        }
    }

    if (imagePointsL.size() != imagePointsR.size() || imagePointsL.empty())
    {
        cerr << "Error: Number of corners detected in Left and Right images does not match or no corners detected." << endl;
        return false;
    }

    // ׼�����������
    vector<vector<cv::Point3f>> objectPoints(imagePointsL.size(), worldPoints);

    // ִ������궨
    cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, E, F;
    double rms = stereoCalibrate(
        objectPoints, imagePointsL, imagePointsR,
        cameraMatrixL, distCoeffsL,
        cameraMatrixR, distCoeffsR,
        imageSize, R, T, E, F,
        cv::CALIB_ZERO_DISPARITY,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6)
    );
    // ��� R �� T,���Գ���ȷ�������Ϊ�վ���
    /*std::cout << "Rotation Matrix R:\n" << R << std::endl;
    std::cout << "Translation Vector T:\n" << T << std::endl;*/

    end_time = clock();
    cout << T << endl;
    cout << "Camera calibration completed: total time: " << (end_time - star_time) / 1000 << "s" << endl;

    // ���뾶�������������
    cv::Mat radialDistCoeffsL = (cv::Mat_<double>(1, 2) << distCoeffsL.at<double>(0, 0), distCoeffsL.at<double>(0, 1));
    cv::Mat tangentialDistCoeffsL = (cv::Mat_<double>(1, 2) << distCoeffsL.at<double>(0, 2), distCoeffsL.at<double>(0, 3));
    cv::Mat radialDistCoeffsR = (cv::Mat_<double>(1, 2) << distCoeffsR.at<double>(0, 0), distCoeffsR.at<double>(0, 1));
    cv::Mat tangentialDistCoeffsR = (cv::Mat_<double>(1, 2) << distCoeffsR.at<double>(0, 2), distCoeffsR.at<double>(0, 3));
    cv::Mat T_new = T.t(); // ת��Ϊ������

    // ����궨���
    cout << T_new << endl;
    ofstream file(saveFile);
    saveSize(file, "imageSize", imageSize);
    saveMat(file, "KK_L", cameraMatrixL);
    saveMat(file, "KK_R", cameraMatrixR);
    saveMat(file, "RadialDistortion_L", radialDistCoeffsL);
    saveMat(file, "RadialDistortion_R", radialDistCoeffsR);
    saveMat(file, "TangentialDistortion_L", tangentialDistCoeffsL);
    saveMat(file, "TangentialDistortion_R", tangentialDistCoeffsR);
    saveMat(file, "R", R);
    saveMat(file, "T", T_new);
    saveMat(file, "E", E);
    saveMat(file, "F", F);
    saveScalar(file, "error", rms);
    file.close();
    return true;
}

void CornerDetection::saveMat(std::ofstream& file, const std::string& name, const cv::Mat& mat)
{
    file << name << endl;
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            file << fixed << setprecision(32) << setw(64) << mat.at<double>(i, j);
            if (j != mat.cols - 1) file << "\t";
        }
        file << endl;
    }
    file << endl;
}

void CornerDetection::saveScalar(std::ofstream& file, const std::string& name, double value)
{
    file << name << endl;
    file << fixed << setprecision(32) << setw(64) << value << endl << endl;
}

void CornerDetection::saveSize(std::ofstream& file, const std::string& name, const cv::Size& size)
{
    file << name << endl;
    file << fixed << setprecision(32) << setw(64) << static_cast<double>(size.height) << "\t"
        << setw(64) << static_cast<double>(size.width) << "\t"
        << setw(64) << 3.0 << endl << endl;
}
