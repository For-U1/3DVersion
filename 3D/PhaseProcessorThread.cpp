//#include "PhaseProcessorThread.h"
//#include <iostream>
//
//// 构造函数
////PhaseProcessorThread::PhaseProcessorThread(const std::shared_ptr<Config>& config, QObject* parent)
////    : QThread(parent), stopFlag(false), config(config)
//PhaseProcessorThread::PhaseProcessorThread(const std::shared_ptr<Config>& config)
//    : stopFlag(false), config(config)
//{
//    // 确保 config 不为空
//    if (!config) 
//    {
//        std::cerr << "Config is null!" << std::endl;
//    }
//}
//
//// 析构函数
//PhaseProcessorThread::~PhaseProcessorThread()
//{
//    stopProcessing(); // 确保线程在析构时停止
//    wait();            // 等待线程完成
//}
//
//// 启动处理
//void PhaseProcessorThread::startProcessing()
//{
//    std::cout << "PhaseProcessorThread::startProcessing() called." << std::endl;
//    setStopFlag(false);
//    if (!isRunning())
//    {
//        std::cout << "Thread is not running. Starting thread." << std::endl;
//        start();
//    }
//    else
//    {
//        std::cout << "Thread is already running." << std::endl;
//    }
//}
//
//// 停止处理
//void PhaseProcessorThread::stopProcessing()
//{
//    setStopFlag(true);  // 使用线程安全的方式设置 stopFlag
//}
//
//// 线程运行函数
//void PhaseProcessorThread::run()
//{
//    std::cout << "Phase processing started." << std::endl;
//    
//    emit processingStarted();
//
//    start_time = clock();
//
//    // ############# 相机标定 #############
//    std::cout << "Starting camera calibration." << std::endl;
//    calibrator.calib(config->calib_file); // 使用 config 中的 calib_file 参数,标定生成的文件
//    std::cout << "Camera calibration finished." << std::endl;
//
//    // ############# 生成相位图 #############
//    if (config->write)
//    {
//        std::cout << "Generating phase patterns." << std::endl;
//        double T123 = phaser.calcT123(config->T1, config->T2, config->T3);
//        phaser.makePatterns(config->A, config->B, config->N, config->W, config->H, config->T1, config->T2, config->T3, config->project_dir);
//        phaser.makePatterns(config->A, config->B, config->N, static_cast<int>(T123), 1, config->T1, config->T2, config->T3, config->simu_dir);
//        std::cout << "Phase patterns generation finished." << std::endl;
//        if (getStopFlag()) return;  // 检查线程停止标志
//    }
//
//    // ############# 解相位 #############
//    cv::Mat pha_L, pha_R;
//    cv::Mat B_L, B_R;
//    std::vector<std::string> files_L, files_R;
//    std::vector<std::string> filesSimu;
//
//    std::cout << "Starting phase unwrapping." << std::endl;
//    globLR(config->T, config->N, config->model_dir, files_L, files_R);
//    glob(config->T, config->N, config->simu_dir, filesSimu);
//
//    phaser.calcPhase(files_L, filesSimu, config->N, config->T1, config->T2, config->T3, pha_L, B_L, config->output_dir_L, config->write);
//    phaser.calcPhase(files_R, filesSimu, config->N, config->T1, config->T2, config->T3, pha_R, B_R, config->output_dir_R, config->write);
//
//    if (getStopFlag()) return;  // 检查线程停止标志
//
//    // ############# 相位滤波 #############
//    std::cout << "Applying modulation filter." << std::endl;
//    phaser.filterB(pha_L, B_L, config->B_min);
//    phaser.filterB(pha_R, B_R, config->B_min);
//
//    // ############# 立体校正 #############
//    std::cout << "Rectifying phase maps." << std::endl;
//    cv::Mat phaC_L, phaC_R;
//    calibrator.rectify(pha_L, pha_R, phaC_L, phaC_R);
//    phaser.filterGrad(phaC_L);
//    phaser.filterGrad(phaC_R);
//
//    if (getStopFlag()) return;  // 检查线程停止标志
//
//    // ############# 相位匹配 #############
//    std::cout << "Starting phase matching." << std::endl;
//    std::vector<std::pair<cv::Point2f, cv::Point2f>> cps;
//    matcher.init(config->win, config->pd);
//    matcher.phaseMatch(phaC_L, phaC_R, cps);
//    std::cout << "Phase matching finished." << std::endl;
//
//    if (getStopFlag()) return;  // 检查线程停止标志
//
//    // ############# 计算点云 #############
//    std::cout << "Calculating point cloud." << std::endl;
//    cv::Mat dif_map = cv::Mat::zeros(phaC_L.size(), CV_32FC1);
//    cv::Mat depth_map = cv::Mat::zeros(phaC_R.size(), CV_32FC1);
//    model.init(calibrator.Q);
//    model.calcDistance(dif_map, depth_map, cps);
//    model.saveXYZ(config->save_file_point3d, depth_map, config->min_z, config->max_z);
//
//    std::cout << "Point cloud calculation finished." << std::endl;
//
//    // ############# 查看结果 #############
//    end_time = clock();
//    std::cout << "Cost time:\t" << (end_time - start_time) / 1000. << "S" << std::endl;
//    if (config->write)
//    {
//        saveMat(config->output_dir_L + "/pha.txt", pha_L);
//        saveMat(config->output_dir_R + "/pha.txt", pha_R);
//        saveMat(config->output_dir_L + "/phaC_L.txt", phaC_L);
//        saveMat(config->output_dir_R + "/phaC_R.txt", phaC_R);
//
//        cv::Mat depth_map_64;
//        depth_map.convertTo(depth_map_64, CV_64FC1);
//        saveMat(config->output_dir + "/depth.txt", depth_map_64);
//    }
//    if (config->show)
//    {
//        showImagePair(pha_L, pha_R, "pha", 0.5, false, true);
//        showImagePair(phaC_L, phaC_R, "phaC", 0.5, true, true);
//    }
//    cv::waitKey(1000);
//    cv::destroyAllWindows();
//    // //return 0;
//
//    std::cout << "Phase processing completed. Sending signal to main thread." << std::endl;
//    emit processingStopped();
//}
//
//// 安全获取 stopFlag 的值
//bool PhaseProcessorThread::getStopFlag()
//{
//    QMutexLocker locker(&mutex);  // 加锁
//    return stopFlag;
//}
//
//// 安全设置 stopFlag 的值
//void PhaseProcessorThread::setStopFlag(bool value)
//{
//    QMutexLocker locker(&mutex);  // 加锁
//    stopFlag = value;
//}

//#include "PhaseProcessorThread.h"
////#include <QDebug>//会导致重载类型不匹配的报错
//#include <chrono>
//
//PhaseProcessorThread::PhaseProcessorThread(std::shared_ptr<Config> config, QObject* parent)
//    : QThread(parent),
//    m_stopFlag(false),
//    m_config(config)
//{
//    if (!m_config)
//    {
//        std::cout << "Config is null!";
//        throw std::invalid_argument("Config is null!");
//    }
//}
//
//PhaseProcessorThread::~PhaseProcessorThread()
//{
//    stopProcessing(); // 请求停止
//    if (isRunning()) {
//        wait(3000); // 等待最多3秒
//        if (isRunning()) {
//            terminate(); // 强制终止（不推荐，尽量避免使用）
//        }
//    }
//}
//
//void PhaseProcessorThread::startProcessing()
//{
//    std::cout << "PhaseProcessorThread::startProcessing() called.";
//    setStopFlag(false);
//    if (!isRunning())
//    {
//        std::cout << "Thread is not running. Starting thread.";
//        start();
//    }
//    else
//    {
//        std::cout << "Thread is already running.";
//    }
//}
//
//void PhaseProcessorThread::stopProcessing()
//{
//    setStopFlag(true);  // 使用线程安全的方式设置 stopFlag
//}
//
//bool PhaseProcessorThread::getStopFlag()
//{
//    return m_stopFlag.load();
//}
//
//void PhaseProcessorThread::setStopFlag(bool value)
//{
//    m_stopFlag.store(value);
//}
//
//void PhaseProcessorThread::run()
//{
//    std::cout << "Phase processing started.";
//    emit processingStarted();
//
//    start_time = std::chrono::high_resolution_clock::now();
//
//    
//        // 假设有总共 10 个步骤，每完成一个步骤更新 10%
//        int total_steps = 10;
//        int current_step = 0;
//
//        // ############# 相机标定 #############
//        std::cout << "Starting camera calibration.";
//        calibrator.calib(m_config->calib_file); // 使用 config 中的 calib_file 参数,标定生成的文件
//        std::cout << "Camera calibration finished.";
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//        if (getStopFlag()) {
//            emit processingStopped();
//            return;  // 检查线程停止标志
//        }
//
//        // ############# 生成相位图 #############
//        if (m_config->write)
//        {
//            std::cout << "Generating phase patterns.";
//            double T123 = phaser.calcT123(m_config->T1, m_config->T2, m_config->T3);
//            phaser.makePatterns(m_config->A, m_config->B, m_config->N, m_config->W, m_config->H, m_config->T1, m_config->T2, m_config->T3, m_config->project_dir);
//            phaser.makePatterns(m_config->A, m_config->B, m_config->N, static_cast<int>(T123), 1, m_config->T1, m_config->T2, m_config->T3, m_config->simu_dir);
//            std::cout << "Phase patterns generation finished.";
//            current_step++;
//            emit progressUpdated(current_step * 100 / total_steps);
//            if (getStopFlag()) {
//                emit processingStopped();
//                return;  // 检查线程停止标志
//            }
//        }
//
//        // ############# 解相位 #############
//        std::cout << "Starting phase unwrapping.";
//        cv::Mat pha_L, pha_R;
//        cv::Mat B_L, B_R;
//        std::vector<std::string> files_L, files_R;
//        std::vector<std::string> filesSimu;
//        globLR(m_config->N, m_config->T, m_config->simu_dir, files_L, files_R);
//        glob(m_config->N, m_config->T, m_config->simu_dir, filesSimu);
//
//        phaser.calcPhase(files_L, filesSimu, m_config->N, m_config->T1, m_config->T2, m_config->T3, pha_L, B_L, m_config->output_dir_L, m_config->write);
//        phaser.calcPhase(files_R, filesSimu, m_config->N, m_config->T1, m_config->T2, m_config->T3, pha_R, B_R, m_config->output_dir_R, m_config->write);
//
//        std::cout << "Phase unwrapping finished.";
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//        if (getStopFlag()) {
//            emit processingStopped();
//            return;  // 检查线程停止标志
//        }
//
//        // ############# 相位滤波 #############
//        std::cout << "Applying modulation filter.";
//        phaser.filterB(pha_L, B_L, m_config->B_min);
//        phaser.filterB(pha_R, B_R, m_config->B_min);
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//        if (getStopFlag()) {
//            emit processingStopped();
//            return;
//        }
//
//        // ############# 立体校正 #############
//        std::cout << "Rectifying phase maps.";
//        cv::Mat phaC_L, phaC_R;
//        calibrator.rectify(pha_L, pha_R, phaC_L, phaC_R);
//        phaser.filterGrad(phaC_L);
//        phaser.filterGrad(phaC_R);
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//        if (getStopFlag()) {
//            emit processingStopped();
//            return;
//        }
//
//        // ############# 相位匹配 #############
//        std::cout << "Starting phase matching.";
//        std::vector<std::pair<cv::Point2f, cv::Point2f>> cps;
//        matcher.init(m_config->win, m_config->pd);
//        matcher.phaseMatch(phaC_L, phaC_R, cps);
//        std::cout << "Phase matching finished.";
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//        if (getStopFlag()) {
//            emit processingStopped();
//            return;
//        }
//
//        // ############# 计算点云 #############
//        std::cout << "Calculating point cloud.";
//        cv::Mat dif_map = cv::Mat::zeros(phaC_L.size(), CV_32FC1);
//        cv::Mat depth_map = cv::Mat::zeros(phaC_R.size(), CV_32FC1);
//        model.init(calibrator.Q);
//        model.calcDistance(dif_map, depth_map, cps);
//        model.saveXYZ(m_config->save_file_point3d, depth_map, m_config->min_z, m_config->max_z);
//        std::cout << "Point cloud calculation finished.";
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//        if (getStopFlag()) {
//            emit processingStopped();
//            return;
//        }
//
//        // ############# 查看结果 #############
//        std::cout << "Displaying results.";
//        if (m_config->show)
//        {
//            showImagePair(pha_L, pha_R, "pha", 0.5, false, true);
//            showImagePair(phaC_L, phaC_R, "phaC", 0.5, true, true);
//        }
//        cv::waitKey(1000);
//        cv::destroyAllWindows();
//        std::cout << "Displaying results finished.";
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//
//        // ############# 完成处理 #############
//        end_time = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double> elapsed = end_time - start_time;
//        std::cout << "Cost time:\t" << elapsed.count() << "S";
//
//        std::cout << "Phase processing completed. Sending signal to main thread.";
//        emit processingStopped();
//    
//}
#include "PhaseProcessorThread.h"
#include <iostream>

// 构造函数
PhaseProcessorThread::PhaseProcessorThread(Config* config)
    : stopFlag(false), config(config)
{
    // 确保 config 不为空
    if (!config)
    {
        std::cerr << "Config is null!" << std::endl;
    }
}

// 析构函数
PhaseProcessorThread::~PhaseProcessorThread()
{
    stopProcessing(); // 确保线程在析构时停止
    wait();            // 等待线程完成
}

// 启动处理
void PhaseProcessorThread::startProcessing()
{
    std::cout << "PhaseProcessorThread::startProcessing() called." << std::endl;
    setStopFlag(false);
    if (!isRunning())
    {
        std::cout << "Thread is not running. Starting thread." << std::endl;
        start();
    }
    else
    {
        std::cout << "Thread is already running." << std::endl;
    }
}

// 停止处理
void PhaseProcessorThread::stopProcessing()
{
    setStopFlag(true);  // 使用线程安全的方式设置 stopFlag
}

// 线程运行函数
void PhaseProcessorThread::run()
{
    std::cout << "Phase processing started." << std::endl;

    emit processingStarted();

    start_time = clock();

    // ############# 相机标定 #############
    std::cout << "Starting camera calibration." << std::endl;
    calibrator.calib(config->calib_file); // 使用 config 中的 calib_file 参数,标定生成的文件
    std::cout << "Camera calibration finished." << std::endl;
    emit processingProgress(10);

    if (getStopFlag()) return;

    // ############# 生成相位图 #############
    if (config->write)
    {
        std::cout << "Generating phase patterns." << std::endl;
        double T123 = phaser.calcT123(config->T1, config->T2, config->T3);
        phaser.makePatterns(config->A, config->B, config->N, config->W, config->H, config->T1, config->T2, config->T3, config->project_dir);
        phaser.makePatterns(config->A, config->B, config->N, static_cast<int>(T123), 1, config->T1, config->T2, config->T3, config->simu_dir);
        std::cout << "Phase patterns generation finished." << std::endl;
        emit processingProgress(20);
        if (getStopFlag()) return;  // 检查线程停止标志
    }

    // ############# 解相位 #############
    cv::Mat pha_L, pha_R;
    cv::Mat B_L, B_R;
    std::vector<std::string> files_L, files_R;
    std::vector<std::string> filesSimu;

    std::cout << "Starting phase unwrapping." << std::endl;
    globLR(config->T, config->N, config->model_dir, files_L, files_R);
    glob(config->T, config->N, config->simu_dir, filesSimu);

    phaser.calcPhase(files_L, filesSimu, config->N, config->T1, config->T2, config->T3, pha_L, B_L, config->output_dir_L, config->write);
    phaser.calcPhase(files_R, filesSimu, config->N, config->T1, config->T2, config->T3, pha_R, B_R, config->output_dir_R, config->write);

    if (getStopFlag()) return;  // 检查线程停止标志
    emit processingProgress(40);

    // ############# 相位滤波 #############
    std::cout << "Applying modulation filter." << std::endl;
    phaser.filterB(pha_L, B_L, config->B_min);
    phaser.filterB(pha_R, B_R, config->B_min);
    emit processingProgress(50);

    // ############# 立体校正 #############
    std::cout << "Rectifying phase maps." << std::endl;
    cv::Mat phaC_L, phaC_R;
    calibrator.rectify(pha_L, pha_R, phaC_L, phaC_R);
    phaser.filterGrad(phaC_L);
    phaser.filterGrad(phaC_R);

    if (getStopFlag()) return;  // 检查线程停止标志
    emit processingProgress(60);

    // ############# 相位匹配 #############
    std::cout << "Starting phase matching." << std::endl;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> cps;
    matcher.init(config->win, config->pd);
    matcher.phaseMatch(phaC_L, phaC_R, cps);
    std::cout << "Phase matching finished." << std::endl;

    if (getStopFlag()) return;  // 检查线程停止标志
    emit processingProgress(80);

    // ############# 计算点云 #############
    std::cout << "Calculating point cloud." << std::endl;
    cv::Mat dif_map = cv::Mat::zeros(phaC_L.size(), CV_32FC1);
    cv::Mat depth_map = cv::Mat::zeros(phaC_R.size(), CV_32FC1);
    model.init(calibrator.Q);
    model.calcDistance(dif_map, depth_map, cps);
    model.saveXYZ(config->save_file_point3d, depth_map, config->min_z, config->max_z);

    std::cout << "Point cloud calculation finished." << std::endl;
    emit processingProgress(90);

    // ############# 查看结果 #############
    end_time = clock();
    std::cout << "Cost time:\t" << (end_time - start_time) / 1000. << "S" << std::endl;
    if (config->write)
    {
        saveMat(config->output_dir_L + "/pha.txt", pha_L);
        saveMat(config->output_dir_R + "/pha.txt", pha_R);
        saveMat(config->output_dir_L + "/phaC_L.txt", phaC_L);
        saveMat(config->output_dir_R + "/phaC_R.txt", phaC_R);

        cv::Mat depth_map_64;
        depth_map.convertTo(depth_map_64, CV_64FC1);
        saveMat(config->output_dir + "/depth.txt", depth_map_64);
    }
    if (config->show)
    {
        // 如果需要使用 Qt 来显示图像，需要将 OpenCV 的 Mat 转换为 QImage，然后在主线程中显示
        // 这里暂时省略，或确保使用线程安全的方式显示图像
    }

    emit processingProgress(100);

    std::cout << "Phase processing completed. Sending signal to main thread." << std::endl;
    emit processingStopped();
}

// 安全获取 stopFlag 的值
bool PhaseProcessorThread::getStopFlag()
{
    QMutexLocker locker(&mutex);  // 加锁
    return stopFlag;
}

// 安全设置 stopFlag 的值
void PhaseProcessorThread::setStopFlag(bool value)
{
    QMutexLocker locker(&mutex);  // 加锁
    stopFlag = value;
}
