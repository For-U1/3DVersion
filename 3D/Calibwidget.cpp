//#include "Calibwidget.h"
//#include <QDebug>
//#include <QFileDialog>
//#include <QPixmap>
//#include <QImage>
//#include <QApplication>
//#include <QMessageBox>
//
//
//CalibWidget::CalibWidget(QWidget* parent)
//    : QWidget(parent),
//    //leftImageLabel(new QLabel(this)),
//    //rightImageLabel(new QLabel(this)),
//    rotationTranslationLabel(new QLabel("calibtration output", this)),//在构造函数中初始化显示立体标定的结果
//    startButton(new QPushButton("Start Calibration", this)),
//    stopButton(new QPushButton("Stop Calibration", this)),
//    progressBar(new QProgressBar(this)),  // 初始化进度条
//    calibrateThread(nullptr)
//
//{
//    // Layout setup
//    QVBoxLayout* mainLayout = new QVBoxLayout;
//
//    QHBoxLayout* showLayout = new QHBoxLayout;
//
//    QHBoxLayout* buttonLayout = new QHBoxLayout;
//
//    //leftImageLabel = new QLabel("left cornerdetection image show", this);
//    //rightImageLabel = new QLabel("right cornerdetection image show", this);
//
//    //imageLayout->addWidget(leftImageLabel);
//    //imageLayout->addWidget(rightImageLabel);
//
//    showLayout->addWidget(rotationTranslationLabel);
//
//    buttonLayout->addWidget(startButton);
//    buttonLayout->addWidget(stopButton);
//
//    //mainLayout->addLayout(imageLayout);
//    mainLayout->addLayout(buttonLayout);
//    mainLayout->addWidget(progressBar);  // 将进度条添加到布局中
//    mainLayout->addLayout(showLayout);  // 添加旋转平移矩阵显示控件到布局中
//    
//    setLayout(mainLayout);
//
//
//    // // Initialize paths
//    // calibDirPath = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
//    // savePath = QFileDialog::getExistingDirectory(this, "Select Save Path");
//
//    // Button connections
//    connect(startButton, &QPushButton::clicked, this, &CalibWidget::startCalibration);
//    connect(stopButton, &QPushButton::clicked, this, &CalibWidget::stopCalibration);
//}
//
//CalibWidget::~CalibWidget()
//{
//    stopCalibration();
//}
//
//// void CalibWidget::startCalibration()
//// {
//
////     // Initialize paths
////     calibDirPath = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
////     savePath = QFileDialog::getExistingDirectory(this, "Select Save Path");
//
////     if (calibrateThread) {
////         delete calibrateThread;
////     }
//
////     calibrateThread = new CalibrateThread(calibDirPath, savePath, this);
////     connect(calibrateThread, &CalibrateThread::calibrationFinished, this, &CalibWidget::updateImages);
//
////     calibrateThread->start();
//// }
//
//
//
//void CalibWidget::startCalibration()
//{
//    // 确保在标定过程中 UI 仍然保持响应
//    QApplication::processEvents();
//
//    // 禁用按钮以防止重复点击
//    startButton->setEnabled(false);
//    startButton->setEnabled(true);
//
//
//    // 初始化路径，选择标定目录
//    qDebug() << "Selecting calibration directory...";  // 调试信息
//    QString calibDirPath = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
//    if (calibDirPath.isEmpty()) {
//        QMessageBox::warning(this, "Warning", "Please select a valid calibration directory.");
//        startButton->setEnabled(true); // 重新启用按钮
//        return;
//    }
//    qDebug() << "Calibration directory selected successfully:" << calibDirPath;  // 调试信息
//
//
//    // 选择标定结果保存路径
//    qDebug() << "Selecting save path...";  // 调试信息
//    QString savePath = QFileDialog::getExistingDirectory(this, "Select Save Path");
//    if (savePath.isEmpty()) {
//        QMessageBox::warning(this, "Warning", "Please select a valid save path.");
//        startButton->setEnabled(true); // 重新启用按钮
//        return;
//    }
//    qDebug() << "Save path selected successfully:" << savePath;  // 调试信息
//
//    // 安全删除之前的线程（如果存在）
//    if (calibrateThread) {
//        calibrateThread->stop();
//        calibrateThread->wait(); // 等待线程结束
//        delete calibrateThread;
//        calibrateThread = nullptr;
//    }
//
//    // 创建新的标定线程
//    calibrateThread = new CalibrateThread(calibDirPath, savePath, this);
//    // 连接标定完成信号到显示结果的槽函数
//    connect(calibrateThread, &CalibrateThread::calibrationResults, this, &CalibWidget::displayCalibrationResults);
//    //connect(calibrateThread, &CalibrateThread::calibrationFinished, this, [=](const Mat& R, const Mat& T) {
//    //    // 显示旋转平移矩阵
//    //    QString rotationTranslationText = QString("Rotation Matrix R:\n%1\n\nTranslation Vector T:\n%2")
//    //        .arg(QString::fromStdString(matToString(R)))
//    //        .arg(QString::fromStdString(matToString(T)));
//    //    rotationTranslationLabel->setText(rotationTranslationText);
//    //    });
//    //connect(calibrateThread, &CalibrateThread::calibrationFinished, this, &CalibWidget::updateImages);
//    //信号与槽函数用于连接标定完成信号与显示的槽函数，后面再调试。
//
//    // 启动标定线程
//    calibrateThread->start();
//}
//
//
//
//
//void CalibWidget::stopCalibration()
//{
//    if (calibrateThread) {
//        calibrateThread->stop();
//        calibrateThread->wait();
//        delete calibrateThread;
//        calibrateThread = nullptr;
//    }
//}
//
//void CalibWidget::displayCalibrationResults(const cv::Mat& R, const cv::Mat& T)
//{
//    QString rotationMatrixText = "Rotation Matrix (R):\n";
//
//    // 将旋转矩阵 R 转换为文本格式
//    for (int i = 0; i < R.rows; ++i) {
//        for (int j = 0; j < R.cols; ++j) {
//            rotationMatrixText += QString::number(R.at<double>(i, j), 'f', 6) + " ";
//        }
//        rotationMatrixText += "\n";  // 换行
//    }
//
//    QString translationVectorText = "Translation Vector (T):\n";
//
//    // 将平移向量 T 转换为文本格式
//    for (int i = 0; i < T.rows; ++i) {
//        translationVectorText += QString::number(T.at<double>(i, 0), 'f', 6) + " ";
//    }
//
//    // 在 QLabel 上显示旋转矩阵和平移向量
//    rotationTranslationLabel->setText(rotationMatrixText + "\n" + translationVectorText);
//
//    // 输出调试信息
//    qDebug() << "Calibration Results Displayed:\n" << rotationMatrixText << translationVectorText;
//}
//
//// void CalibWidget::updateImages(bool success)
//// {
////     if (success) {
////         // Load and display the corner detection results
////         // Replace these with the actual paths where your images are saved
////         QImage leftImage("path_to_left_image_with_corners.bmp");  // Update with actual image path
////         QImage rightImage("path_to_right_image_with_corners.bmp");  // Update with actual image path
//
////         leftImageLabel->setPixmap(QPixmap::fromImage(leftImage));
////         rightImageLabel->setPixmap(QPixmap::fromImage(rightImage));
////     } else {
////         qDebug() << "Calibration failed";
////     }
//// }
//
//
//// void CalibWidget::updateImages(bool success)//用于显示标定结果，后面在调试。
//// {
//
////     if (success) {
////         // 假设标定线程在给定的路径下保存了角点检测结果图片
////         QString leftImagePath = savePath + "/left_image_with_corners_.bmp";  // 修改为实际路径
////         QString rightImagePath = savePath + "/right_image_with_corners_.bmp";  // 修改为实际路径
//
////         QImage leftImage(leftImagePath);
////         QImage rightImage(rightImagePath);
//
////         if (!leftImage.isNull() && !rightImage.isNull()) {
////             leftImageLabel->setPixmap(QPixmap::fromImage(leftImage).scaled(leftImageLabel->size(), Qt::KeepAspectRatio));
////             rightImageLabel->setPixmap(QPixmap::fromImage(rightImage).scaled(rightImageLabel->size(), Qt::KeepAspectRatio));
////         } else {
////             qDebug() << "Failed to load images: " << leftImagePath << " or " << rightImagePath;
////             QMessageBox::warning(this, "Warning", "Failed to load calibration images.");
////         }
////     } else {
////         qDebug() << "Calibration failed";
////         QMessageBox::warning(this, "Warning", "Calibration failed.");
////     }
//// }
//

// calibwidget.cpp

//#include "calibwidget.h"
//#include <QDebug>
//#include <QFileDialog>
//#include <QVBoxLayout>
//#include <QHBoxLayout>
//#include <QMessageBox>
//#include <QApplication>
//
//CalibWidget::CalibWidget(QWidget* parent)
//    : QWidget(parent),
//    startButton(new QPushButton("Start Calibration", this)),
//    stopButton(new QPushButton("Stop Calibration", this)),
//    progressBar(new QProgressBar(this)),
//    rotationTranslationLabel(new QLabel("Calibration Output", this)),
//    calibrateThread(nullptr)
//{
//    // Layout setup
//    QVBoxLayout* mainLayout = new QVBoxLayout;
//
//    // 参数输入控件
//    rowsSpinBox = new QSpinBox(this);
//    rowsSpinBox->setRange(2, 50);
//    rowsSpinBox->setValue(8);
//
//    colsSpinBox = new QSpinBox(this);
//    colsSpinBox->setRange(2, 50);
//    colsSpinBox->setValue(11);
//
//    squareSizeSpinBox = new QDoubleSpinBox(this);
//    squareSizeSpinBox->setRange(1.0, 100.0);
//    squareSizeSpinBox->setValue(15.0);
//    squareSizeSpinBox->setDecimals(2);
//
//    QLabel* rowsLabel = new QLabel("Rows:", this);
//    QLabel* colsLabel = new QLabel("Columns:", this);
//    QLabel* squareSizeLabel = new QLabel("Square Size (mm):", this);
//
//    QHBoxLayout* paramsLayout = new QHBoxLayout;
//    paramsLayout->addWidget(rowsLabel);
//    paramsLayout->addWidget(rowsSpinBox);
//    paramsLayout->addWidget(colsLabel);
//    paramsLayout->addWidget(colsSpinBox);
//    paramsLayout->addWidget(squareSizeLabel);
//    paramsLayout->addWidget(squareSizeSpinBox);
//
//    QHBoxLayout* buttonLayout = new QHBoxLayout;
//    buttonLayout->addWidget(startButton);
//    buttonLayout->addWidget(stopButton);
//
//    // 设置 rotationTranslationLabel 的对齐方式和字体
//    rotationTranslationLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
//    rotationTranslationLabel->setWordWrap(true);  // 允许自动换行
//    QFont font = rotationTranslationLabel->font();
//    font.setPointSize(10);
//    rotationTranslationLabel->setFont(font);
//    rotationTranslationLabel->setMinimumSize(400, 200); //根据需要调整大小
//
//    mainLayout->addLayout(paramsLayout);
//    mainLayout->addLayout(buttonLayout);
//    mainLayout->addWidget(progressBar);
//    mainLayout->addWidget(rotationTranslationLabel);
//
//    setLayout(mainLayout);
//
//    // Button connections
//    connect(startButton, &QPushButton::clicked, this, &CalibWidget::startCalibration);
//    connect(stopButton, &QPushButton::clicked, this, &CalibWidget::stopCalibration);
//}
//
//CalibWidget::~CalibWidget()
//{
//    stopCalibration();
//}
//
//void CalibWidget::startCalibration()
//{
//    QApplication::processEvents();
//
//    startButton->setEnabled(false);
//    stopButton->setEnabled(true);
//
//    // Select calibration directory
//    QString calibDirPath = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
//    if (calibDirPath.isEmpty()) {
//        QMessageBox::warning(this, "Warning", "Please select a valid calibration directory.");
//        startButton->setEnabled(true);
//        return;
//    }
//
//    // Select save path
//    QString savePath = QFileDialog::getExistingDirectory(this, "Select Save Path");
//    if (savePath.isEmpty()) {
//        QMessageBox::warning(this, "Warning", "Please select a valid save path.");
//        startButton->setEnabled(true);
//        return;
//    }
//
//    // Get chessboard parameters
//    int boardWidth = colsSpinBox->value();
//    int boardHeight = rowsSpinBox->value();
//    double squareSize = squareSizeSpinBox->value();
//
//    // Stop existing thread if any
//    if (calibrateThread) {
//        calibrateThread->stop();
//        calibrateThread->wait();
//        delete calibrateThread;
//        calibrateThread = nullptr;
//    }
//
//    // Create new calibration thread
//    calibrateThread = new CalibrateThread(calibDirPath, savePath, boardWidth, boardHeight, squareSize, this);
//
//    // Connect signals and slots
//    connect(calibrateThread, &CalibrateThread::progressUpdated, this, &CalibWidget::updateProgressBar);
//    connect(calibrateThread, &CalibrateThread::calibrationResults, this, &CalibWidget::displayCalibrationResults, Qt::QueuedConnection);
//    connect(calibrateThread, &CalibrateThread::calibrationFinished, this, &CalibWidget::calibrationFinished);
//
//    // Reset progress bar
//    progressBar->setValue(0);
//    progressBar->setMaximum(100);
//
//    // Start calibration thread
//    calibrateThread->start();
//}
//
//void CalibWidget::stopCalibration()
//{
//    startButton->setEnabled(true);
//    stopButton->setEnabled(false);
//
//    if (calibrateThread) {
//        calibrateThread->stop();
//        calibrateThread->wait();
//        delete calibrateThread;
//        calibrateThread = nullptr;
//    }
//}
//
//void CalibWidget::updateProgressBar(int value)
//{
//    progressBar->setValue(value);
//}
//
//void CalibWidget::displayCalibrationResults(const cv::Mat& R, const cv::Mat& T)
//{
//    qDebug() << "displayCalibrationResults() called.";//添加调试信息
//
//    if (R.empty() || T.empty())
//    {
//        qDebug() << "Received empty R or T matrix.";
//        rotationTranslationLabel->setText("Calibration failed: Received empty R or T matrix.");
//        return;
//    }
//      
//    QString rotationMatrixText = "Rotation Matrix (R):\n";
//
//    // Convert R to text
//    for (int i = 0; i < R.rows; ++i) 
//    {
//        for (int j = 0; j < R.cols; ++j) 
//        {
//            rotationMatrixText += QString::number(R.at<double>(i, j), 'f', 6) + " ";
//        }
//        rotationMatrixText += "\n";
//    }
//
//    QString translationVectorText = "Translation Vector (T):\n";
//
//    // Convert T to text
//    for (int i = 0; i < T.rows; ++i) 
//    {
//        translationVectorText += QString::number(T.at<double>(i, 0), 'f', 6) + " ";
//    }
//
//    rotationTranslationLabel->setText(rotationMatrixText + "\n" + translationVectorText);
//
//    qDebug() << "Calibration Results Displayed:\n" << rotationMatrixText << translationVectorText;
//}
//
//void CalibWidget::calibrationFinished(bool success)
//{
//    startButton->setEnabled(true);
//    stopButton->setEnabled(false);
//
//    if (success) {
//        QMessageBox::information(this, "Success", "Calibration completed successfully.");
//    }
//    else {
//        QMessageBox::warning(this, "Failure", "Calibration failed.");
//    }
//}
// calibwidget.cpp

// calibwidget.cpp

#include "calibwidget.h"
#include <QDebug>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QApplication>
#include <QScrollArea>

CalibWidget::CalibWidget(QWidget* parent)
    : QWidget(parent),
    startButton(new QPushButton("Start Calibration", this)),
    stopButton(new QPushButton("Stop Calibration", this)),
    progressBar(new QProgressBar(this)),
    rotationTranslationLabel(new QLabel("Calibration Output", this)),
    calibrateThread(nullptr)
{
    // 设置窗口标题
    setWindowTitle("Stereo Calibration Tool");

    // 布局设置
    QVBoxLayout* mainLayout = new QVBoxLayout;

    // 参数输入控件
    rowsSpinBox = new QSpinBox(this);
    rowsSpinBox->setRange(2, 50);
    rowsSpinBox->setValue(8);

    colsSpinBox = new QSpinBox(this);
    colsSpinBox->setRange(2, 50);
    colsSpinBox->setValue(11);

    squareSizeSpinBox = new QDoubleSpinBox(this);
    squareSizeSpinBox->setRange(1.0, 100.0);
    squareSizeSpinBox->setValue(15.0);
    squareSizeSpinBox->setDecimals(2);

    QLabel* rowsLabel = new QLabel("Rows:", this);
    QLabel* colsLabel = new QLabel("Columns:", this);
    QLabel* squareSizeLabel = new QLabel("Square Size (mm):", this);

    QHBoxLayout* paramsLayout = new QHBoxLayout;
    paramsLayout->addWidget(rowsLabel);
    paramsLayout->addWidget(rowsSpinBox);
    paramsLayout->addWidget(colsLabel);
    paramsLayout->addWidget(colsSpinBox);
    paramsLayout->addWidget(squareSizeLabel);
    paramsLayout->addWidget(squareSizeSpinBox);

    QHBoxLayout* buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);

    // 设置 rotationTranslationLabel 的属性
    rotationTranslationLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    rotationTranslationLabel->setWordWrap(true);
    rotationTranslationLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    rotationTranslationLabel->setMinimumSize(400, 200);

    // 使用滚动区域包裹标签
    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setWidget(rotationTranslationLabel);
    scrollArea->setWidgetResizable(true);

    mainLayout->addLayout(paramsLayout);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(progressBar);
    mainLayout->addWidget(scrollArea);

    setLayout(mainLayout);

    // 按钮连接
    connect(startButton, &QPushButton::clicked, this, &CalibWidget::startCalibration);
    connect(stopButton, &QPushButton::clicked, this, &CalibWidget::stopCalibration);
}

CalibWidget::~CalibWidget()
{
    stopCalibration();
}

void CalibWidget::startCalibration()
{
    QApplication::processEvents();

    startButton->setEnabled(false);
    stopButton->setEnabled(true);

    // 选择标定目录
    QString calibDirPath = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
    if (calibDirPath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please select a valid calibration directory.");
        startButton->setEnabled(true);
        return;
    }

    // 选择保存路径
    QString savePath = QFileDialog::getExistingDirectory(this, "Select Save Path");
    if (savePath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please select a valid save path.");
        startButton->setEnabled(true);
        return;
    }

    // 获取棋盘格参数
    int boardWidth = colsSpinBox->value();
    int boardHeight = rowsSpinBox->value();
    double squareSize = squareSizeSpinBox->value();

    // 停止已有的线程（如果有）
    if (calibrateThread) {
        calibrateThread->stop();
        calibrateThread->wait();
        delete calibrateThread;
        calibrateThread = nullptr;
    }

    // 创建新的标定线程
    calibrateThread = new CalibrateThread(calibDirPath, savePath, boardWidth, boardHeight, squareSize, this);

    // 连接信号和槽
    connect(calibrateThread, &CalibrateThread::progressUpdated, this, &CalibWidget::updateProgressBar);
    connect(calibrateThread, &CalibrateThread::calibrationResults,
        this, &CalibWidget::displayCalibrationResults, Qt::QueuedConnection);
    connect(calibrateThread, &CalibrateThread::calibrationFinished, this, &CalibWidget::calibrationFinished);

    // 重置进度条
    progressBar->setValue(0);
    progressBar->setMaximum(100);

    // 清空之前的标定结果显示
    rotationTranslationLabel->setText("Calibration Output");

    // 启动标定线程
    calibrateThread->start();
}

void CalibWidget::stopCalibration()
{
    startButton->setEnabled(true);
    stopButton->setEnabled(false);

    if (calibrateThread) {
        calibrateThread->stop();
        calibrateThread->wait();
        delete calibrateThread;
        calibrateThread = nullptr;
    }
}

void CalibWidget::updateProgressBar(int value)
{
    progressBar->setValue(value);
}

void CalibWidget::displayCalibrationResults(const QVector<double>& R_data, int R_rows, int R_cols,
    const QVector<double>& T_data, int T_rows, int T_cols)
{
    qDebug() << "displayCalibrationResults() called.";

    if (R_data.isEmpty() || T_data.isEmpty())
    {
        qDebug() << "Received empty R or T data.";
        rotationTranslationLabel->setText("Calibration failed: Received empty R or T data.");
        return;
    }

    // 重新构建 R 矩阵
    cv::Mat R(R_rows, R_cols, CV_64F);
    int idx = 0;
    for (int i = 0; i < R_rows; ++i)
    {
        for (int j = 0; j < R_cols; ++j)
        {
            R.at<double>(i, j) = R_data[idx++];
        }
    }

    // 重新构建 T 矩阵
    cv::Mat T(T_rows, T_cols, CV_64F);
    idx = 0;
    for (int i = 0; i < T_rows; ++i)
    {
        for (int j = 0; j < T_cols; ++j)
        {
            T.at<double>(i, j) = T_data[idx++];
        }
    }

    // 显示矩阵
    QString rotationMatrixText = "Rotation Matrix (R):\n";
    for (int i = 0; i < R.rows; ++i)
    {
        for (int j = 0; j < R.cols; ++j)
        {
            rotationMatrixText += QString::number(R.at<double>(i, j), 'f', 6) + " ";
        }
        rotationMatrixText += "\n";
    }

    QString translationVectorText = "Translation Vector (T):\n";
    for (int i = 0; i < T.rows; ++i)
    {
        for (int j = 0; j < T.cols; ++j)
        {
            translationVectorText += QString::number(T.at<double>(i, j), 'f', 6) + " ";
        }
        translationVectorText += "\n";
    }

    rotationTranslationLabel->setText(rotationMatrixText + "\n" + translationVectorText);
    rotationTranslationLabel->adjustSize();
}

void CalibWidget::calibrationFinished(bool success)
{
    startButton->setEnabled(true);
    stopButton->setEnabled(false);

    if (success) {
        QMessageBox::information(this, "Success", "Calibration completed successfully.");
    }
    else {
        QMessageBox::warning(this, "Failure", "Calibration failed.");
    }
}

