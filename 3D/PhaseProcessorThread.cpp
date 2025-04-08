//#include "PhaseProcessorThread.h"
//#include <iostream>
//
//// ���캯��
////PhaseProcessorThread::PhaseProcessorThread(const std::shared_ptr<Config>& config, QObject* parent)
////    : QThread(parent), stopFlag(false), config(config)
//PhaseProcessorThread::PhaseProcessorThread(const std::shared_ptr<Config>& config)
//    : stopFlag(false), config(config)
//{
//    // ȷ�� config ��Ϊ��
//    if (!config) 
//    {
//        std::cerr << "Config is null!" << std::endl;
//    }
//}
//
//// ��������
//PhaseProcessorThread::~PhaseProcessorThread()
//{
//    stopProcessing(); // ȷ���߳�������ʱֹͣ
//    wait();            // �ȴ��߳����
//}
//
//// ��������
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
//// ֹͣ����
//void PhaseProcessorThread::stopProcessing()
//{
//    setStopFlag(true);  // ʹ���̰߳�ȫ�ķ�ʽ���� stopFlag
//}
//
//// �߳����к���
//void PhaseProcessorThread::run()
//{
//    std::cout << "Phase processing started." << std::endl;
//    
//    emit processingStarted();
//
//    start_time = clock();
//
//    // ############# ����궨 #############
//    std::cout << "Starting camera calibration." << std::endl;
//    calibrator.calib(config->calib_file); // ʹ�� config �е� calib_file ����,�궨���ɵ��ļ�
//    std::cout << "Camera calibration finished." << std::endl;
//
//    // ############# ������λͼ #############
//    if (config->write)
//    {
//        std::cout << "Generating phase patterns." << std::endl;
//        double T123 = phaser.calcT123(config->T1, config->T2, config->T3);
//        phaser.makePatterns(config->A, config->B, config->N, config->W, config->H, config->T1, config->T2, config->T3, config->project_dir);
//        phaser.makePatterns(config->A, config->B, config->N, static_cast<int>(T123), 1, config->T1, config->T2, config->T3, config->simu_dir);
//        std::cout << "Phase patterns generation finished." << std::endl;
//        if (getStopFlag()) return;  // ����߳�ֹͣ��־
//    }
//
//    // ############# ����λ #############
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
//    if (getStopFlag()) return;  // ����߳�ֹͣ��־
//
//    // ############# ��λ�˲� #############
//    std::cout << "Applying modulation filter." << std::endl;
//    phaser.filterB(pha_L, B_L, config->B_min);
//    phaser.filterB(pha_R, B_R, config->B_min);
//
//    // ############# ����У�� #############
//    std::cout << "Rectifying phase maps." << std::endl;
//    cv::Mat phaC_L, phaC_R;
//    calibrator.rectify(pha_L, pha_R, phaC_L, phaC_R);
//    phaser.filterGrad(phaC_L);
//    phaser.filterGrad(phaC_R);
//
//    if (getStopFlag()) return;  // ����߳�ֹͣ��־
//
//    // ############# ��λƥ�� #############
//    std::cout << "Starting phase matching." << std::endl;
//    std::vector<std::pair<cv::Point2f, cv::Point2f>> cps;
//    matcher.init(config->win, config->pd);
//    matcher.phaseMatch(phaC_L, phaC_R, cps);
//    std::cout << "Phase matching finished." << std::endl;
//
//    if (getStopFlag()) return;  // ����߳�ֹͣ��־
//
//    // ############# ������� #############
//    std::cout << "Calculating point cloud." << std::endl;
//    cv::Mat dif_map = cv::Mat::zeros(phaC_L.size(), CV_32FC1);
//    cv::Mat depth_map = cv::Mat::zeros(phaC_R.size(), CV_32FC1);
//    model.init(calibrator.Q);
//    model.calcDistance(dif_map, depth_map, cps);
//    model.saveXYZ(config->save_file_point3d, depth_map, config->min_z, config->max_z);
//
//    std::cout << "Point cloud calculation finished." << std::endl;
//
//    // ############# �鿴��� #############
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
//// ��ȫ��ȡ stopFlag ��ֵ
//bool PhaseProcessorThread::getStopFlag()
//{
//    QMutexLocker locker(&mutex);  // ����
//    return stopFlag;
//}
//
//// ��ȫ���� stopFlag ��ֵ
//void PhaseProcessorThread::setStopFlag(bool value)
//{
//    QMutexLocker locker(&mutex);  // ����
//    stopFlag = value;
//}

//#include "PhaseProcessorThread.h"
////#include <QDebug>//�ᵼ���������Ͳ�ƥ��ı���
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
//    stopProcessing(); // ����ֹͣ
//    if (isRunning()) {
//        wait(3000); // �ȴ����3��
//        if (isRunning()) {
//            terminate(); // ǿ����ֹ�����Ƽ�����������ʹ�ã�
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
//    setStopFlag(true);  // ʹ���̰߳�ȫ�ķ�ʽ���� stopFlag
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
//        // �������ܹ� 10 �����裬ÿ���һ��������� 10%
//        int total_steps = 10;
//        int current_step = 0;
//
//        // ############# ����궨 #############
//        std::cout << "Starting camera calibration.";
//        calibrator.calib(m_config->calib_file); // ʹ�� config �е� calib_file ����,�궨���ɵ��ļ�
//        std::cout << "Camera calibration finished.";
//        current_step++;
//        emit progressUpdated(current_step * 100 / total_steps);
//        if (getStopFlag()) {
//            emit processingStopped();
//            return;  // ����߳�ֹͣ��־
//        }
//
//        // ############# ������λͼ #############
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
//                return;  // ����߳�ֹͣ��־
//            }
//        }
//
//        // ############# ����λ #############
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
//            return;  // ����߳�ֹͣ��־
//        }
//
//        // ############# ��λ�˲� #############
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
//        // ############# ����У�� #############
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
//        // ############# ��λƥ�� #############
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
//        // ############# ������� #############
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
//        // ############# �鿴��� #############
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
//        // ############# ��ɴ��� #############
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

// ���캯��
PhaseProcessorThread::PhaseProcessorThread(Config* config)
    : stopFlag(false), config(config)
{
    // ȷ�� config ��Ϊ��
    if (!config)
    {
        std::cerr << "Config is null!" << std::endl;
    }
}

// ��������
PhaseProcessorThread::~PhaseProcessorThread()
{
    stopProcessing(); // ȷ���߳�������ʱֹͣ
    wait();            // �ȴ��߳����
}

// ��������
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

// ֹͣ����
void PhaseProcessorThread::stopProcessing()
{
    setStopFlag(true);  // ʹ���̰߳�ȫ�ķ�ʽ���� stopFlag
}

// �߳����к���
void PhaseProcessorThread::run()
{
    std::cout << "Phase processing started." << std::endl;

    emit processingStarted();

    start_time = clock();

    // ############# ����궨 #############
    std::cout << "Starting camera calibration." << std::endl;
    calibrator.calib(config->calib_file); // ʹ�� config �е� calib_file ����,�궨���ɵ��ļ�
    std::cout << "Camera calibration finished." << std::endl;
    emit processingProgress(10);

    if (getStopFlag()) return;

    // ############# ������λͼ #############
    if (config->write)
    {
        std::cout << "Generating phase patterns." << std::endl;
        double T123 = phaser.calcT123(config->T1, config->T2, config->T3);
        phaser.makePatterns(config->A, config->B, config->N, config->W, config->H, config->T1, config->T2, config->T3, config->project_dir);
        phaser.makePatterns(config->A, config->B, config->N, static_cast<int>(T123), 1, config->T1, config->T2, config->T3, config->simu_dir);
        std::cout << "Phase patterns generation finished." << std::endl;
        emit processingProgress(20);
        if (getStopFlag()) return;  // ����߳�ֹͣ��־
    }

    // ############# ����λ #############
    cv::Mat pha_L, pha_R;
    cv::Mat B_L, B_R;
    std::vector<std::string> files_L, files_R;
    std::vector<std::string> filesSimu;

    std::cout << "Starting phase unwrapping." << std::endl;
    globLR(config->T, config->N, config->model_dir, files_L, files_R);
    glob(config->T, config->N, config->simu_dir, filesSimu);

    phaser.calcPhase(files_L, filesSimu, config->N, config->T1, config->T2, config->T3, pha_L, B_L, config->output_dir_L, config->write);
    phaser.calcPhase(files_R, filesSimu, config->N, config->T1, config->T2, config->T3, pha_R, B_R, config->output_dir_R, config->write);

    if (getStopFlag()) return;  // ����߳�ֹͣ��־
    emit processingProgress(40);

    // ############# ��λ�˲� #############
    std::cout << "Applying modulation filter." << std::endl;
    phaser.filterB(pha_L, B_L, config->B_min);
    phaser.filterB(pha_R, B_R, config->B_min);
    emit processingProgress(50);

    // ############# ����У�� #############
    std::cout << "Rectifying phase maps." << std::endl;
    cv::Mat phaC_L, phaC_R;
    calibrator.rectify(pha_L, pha_R, phaC_L, phaC_R);
    phaser.filterGrad(phaC_L);
    phaser.filterGrad(phaC_R);

    if (getStopFlag()) return;  // ����߳�ֹͣ��־
    emit processingProgress(60);

    // ############# ��λƥ�� #############
    std::cout << "Starting phase matching." << std::endl;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> cps;
    matcher.init(config->win, config->pd);
    matcher.phaseMatch(phaC_L, phaC_R, cps);
    std::cout << "Phase matching finished." << std::endl;

    if (getStopFlag()) return;  // ����߳�ֹͣ��־
    emit processingProgress(80);

    // ############# ������� #############
    std::cout << "Calculating point cloud." << std::endl;
    cv::Mat dif_map = cv::Mat::zeros(phaC_L.size(), CV_32FC1);
    cv::Mat depth_map = cv::Mat::zeros(phaC_R.size(), CV_32FC1);
    model.init(calibrator.Q);
    model.calcDistance(dif_map, depth_map, cps);
    model.saveXYZ(config->save_file_point3d, depth_map, config->min_z, config->max_z);

    std::cout << "Point cloud calculation finished." << std::endl;
    emit processingProgress(90);

    // ############# �鿴��� #############
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
        // �����Ҫʹ�� Qt ����ʾͼ����Ҫ�� OpenCV �� Mat ת��Ϊ QImage��Ȼ�������߳�����ʾ
        // ������ʱʡ�ԣ���ȷ��ʹ���̰߳�ȫ�ķ�ʽ��ʾͼ��
    }

    emit processingProgress(100);

    std::cout << "Phase processing completed. Sending signal to main thread." << std::endl;
    emit processingStopped();
}

// ��ȫ��ȡ stopFlag ��ֵ
bool PhaseProcessorThread::getStopFlag()
{
    QMutexLocker locker(&mutex);  // ����
    return stopFlag;
}

// ��ȫ���� stopFlag ��ֵ
void PhaseProcessorThread::setStopFlag(bool value)
{
    QMutexLocker locker(&mutex);  // ����
    stopFlag = value;
}
