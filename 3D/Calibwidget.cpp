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
//    rotationTranslationLabel(new QLabel("calibtration output", this)),//�ڹ��캯���г�ʼ����ʾ����궨�Ľ��
//    startButton(new QPushButton("Start Calibration", this)),
//    stopButton(new QPushButton("Stop Calibration", this)),
//    progressBar(new QProgressBar(this)),  // ��ʼ��������
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
//    mainLayout->addWidget(progressBar);  // ����������ӵ�������
//    mainLayout->addLayout(showLayout);  // �����תƽ�ƾ�����ʾ�ؼ���������
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
//    // ȷ���ڱ궨������ UI ��Ȼ������Ӧ
//    QApplication::processEvents();
//
//    // ���ð�ť�Է�ֹ�ظ����
//    startButton->setEnabled(false);
//    startButton->setEnabled(true);
//
//
//    // ��ʼ��·����ѡ��궨Ŀ¼
//    qDebug() << "Selecting calibration directory...";  // ������Ϣ
//    QString calibDirPath = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
//    if (calibDirPath.isEmpty()) {
//        QMessageBox::warning(this, "Warning", "Please select a valid calibration directory.");
//        startButton->setEnabled(true); // �������ð�ť
//        return;
//    }
//    qDebug() << "Calibration directory selected successfully:" << calibDirPath;  // ������Ϣ
//
//
//    // ѡ��궨�������·��
//    qDebug() << "Selecting save path...";  // ������Ϣ
//    QString savePath = QFileDialog::getExistingDirectory(this, "Select Save Path");
//    if (savePath.isEmpty()) {
//        QMessageBox::warning(this, "Warning", "Please select a valid save path.");
//        startButton->setEnabled(true); // �������ð�ť
//        return;
//    }
//    qDebug() << "Save path selected successfully:" << savePath;  // ������Ϣ
//
//    // ��ȫɾ��֮ǰ���̣߳�������ڣ�
//    if (calibrateThread) {
//        calibrateThread->stop();
//        calibrateThread->wait(); // �ȴ��߳̽���
//        delete calibrateThread;
//        calibrateThread = nullptr;
//    }
//
//    // �����µı궨�߳�
//    calibrateThread = new CalibrateThread(calibDirPath, savePath, this);
//    // ���ӱ궨����źŵ���ʾ����Ĳۺ���
//    connect(calibrateThread, &CalibrateThread::calibrationResults, this, &CalibWidget::displayCalibrationResults);
//    //connect(calibrateThread, &CalibrateThread::calibrationFinished, this, [=](const Mat& R, const Mat& T) {
//    //    // ��ʾ��תƽ�ƾ���
//    //    QString rotationTranslationText = QString("Rotation Matrix R:\n%1\n\nTranslation Vector T:\n%2")
//    //        .arg(QString::fromStdString(matToString(R)))
//    //        .arg(QString::fromStdString(matToString(T)));
//    //    rotationTranslationLabel->setText(rotationTranslationText);
//    //    });
//    //connect(calibrateThread, &CalibrateThread::calibrationFinished, this, &CalibWidget::updateImages);
//    //�ź���ۺ����������ӱ궨����ź�����ʾ�Ĳۺ����������ٵ��ԡ�
//
//    // �����궨�߳�
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
//    // ����ת���� R ת��Ϊ�ı���ʽ
//    for (int i = 0; i < R.rows; ++i) {
//        for (int j = 0; j < R.cols; ++j) {
//            rotationMatrixText += QString::number(R.at<double>(i, j), 'f', 6) + " ";
//        }
//        rotationMatrixText += "\n";  // ����
//    }
//
//    QString translationVectorText = "Translation Vector (T):\n";
//
//    // ��ƽ������ T ת��Ϊ�ı���ʽ
//    for (int i = 0; i < T.rows; ++i) {
//        translationVectorText += QString::number(T.at<double>(i, 0), 'f', 6) + " ";
//    }
//
//    // �� QLabel ����ʾ��ת�����ƽ������
//    rotationTranslationLabel->setText(rotationMatrixText + "\n" + translationVectorText);
//
//    // ���������Ϣ
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
//// void CalibWidget::updateImages(bool success)//������ʾ�궨����������ڵ��ԡ�
//// {
//
////     if (success) {
////         // ����궨�߳��ڸ�����·���±����˽ǵ�����ͼƬ
////         QString leftImagePath = savePath + "/left_image_with_corners_.bmp";  // �޸�Ϊʵ��·��
////         QString rightImagePath = savePath + "/right_image_with_corners_.bmp";  // �޸�Ϊʵ��·��
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
//    // ��������ؼ�
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
//    // ���� rotationTranslationLabel �Ķ��뷽ʽ������
//    rotationTranslationLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
//    rotationTranslationLabel->setWordWrap(true);  // �����Զ�����
//    QFont font = rotationTranslationLabel->font();
//    font.setPointSize(10);
//    rotationTranslationLabel->setFont(font);
//    rotationTranslationLabel->setMinimumSize(400, 200); //������Ҫ������С
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
//    qDebug() << "displayCalibrationResults() called.";//��ӵ�����Ϣ
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
    // ���ô��ڱ���
    setWindowTitle("Stereo Calibration Tool");

    // ��������
    QVBoxLayout* mainLayout = new QVBoxLayout;

    // ��������ؼ�
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

    // ���� rotationTranslationLabel ������
    rotationTranslationLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    rotationTranslationLabel->setWordWrap(true);
    rotationTranslationLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    rotationTranslationLabel->setMinimumSize(400, 200);

    // ʹ�ù������������ǩ
    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setWidget(rotationTranslationLabel);
    scrollArea->setWidgetResizable(true);

    mainLayout->addLayout(paramsLayout);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(progressBar);
    mainLayout->addWidget(scrollArea);

    setLayout(mainLayout);

    // ��ť����
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

    // ѡ��궨Ŀ¼
    QString calibDirPath = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
    if (calibDirPath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please select a valid calibration directory.");
        startButton->setEnabled(true);
        return;
    }

    // ѡ�񱣴�·��
    QString savePath = QFileDialog::getExistingDirectory(this, "Select Save Path");
    if (savePath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please select a valid save path.");
        startButton->setEnabled(true);
        return;
    }

    // ��ȡ���̸����
    int boardWidth = colsSpinBox->value();
    int boardHeight = rowsSpinBox->value();
    double squareSize = squareSizeSpinBox->value();

    // ֹͣ���е��̣߳�����У�
    if (calibrateThread) {
        calibrateThread->stop();
        calibrateThread->wait();
        delete calibrateThread;
        calibrateThread = nullptr;
    }

    // �����µı궨�߳�
    calibrateThread = new CalibrateThread(calibDirPath, savePath, boardWidth, boardHeight, squareSize, this);

    // �����źźͲ�
    connect(calibrateThread, &CalibrateThread::progressUpdated, this, &CalibWidget::updateProgressBar);
    connect(calibrateThread, &CalibrateThread::calibrationResults,
        this, &CalibWidget::displayCalibrationResults, Qt::QueuedConnection);
    connect(calibrateThread, &CalibrateThread::calibrationFinished, this, &CalibWidget::calibrationFinished);

    // ���ý�����
    progressBar->setValue(0);
    progressBar->setMaximum(100);

    // ���֮ǰ�ı궨�����ʾ
    rotationTranslationLabel->setText("Calibration Output");

    // �����궨�߳�
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

    // ���¹��� R ����
    cv::Mat R(R_rows, R_cols, CV_64F);
    int idx = 0;
    for (int i = 0; i < R_rows; ++i)
    {
        for (int j = 0; j < R_cols; ++j)
        {
            R.at<double>(i, j) = R_data[idx++];
        }
    }

    // ���¹��� T ����
    cv::Mat T(T_rows, T_cols, CV_64F);
    idx = 0;
    for (int i = 0; i < T_rows; ++i)
    {
        for (int j = 0; j < T_cols; ++j)
        {
            T.at<double>(i, j) = T_data[idx++];
        }
    }

    // ��ʾ����
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

