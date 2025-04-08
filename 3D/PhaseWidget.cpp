//#include "PhaseWidget.h"
//#include <QMessageBox>
//#include <iostream>
//
//PhaseWidget::PhaseWidget(QWidget* parent)
//    : QWidget(parent),
//    config(std::make_shared<Config>())
//{
//    // 不再从文件加载配置，因为配置已在构造函数中初始化
//    std::cout << "Config parameters initialized directly in code." << std::endl;
//
//    // 创建 PhaseProcessorThread 对象，父对象为 this
//    phaseProcessorThread = new PhaseProcessorThread(config);
//
//    // 设置布局
//    QHBoxLayout* layout = new QHBoxLayout(this);
//
//    startButton = new QPushButton("Phase Start", this);
//    stopButton = new QPushButton("Phase Stop", this);
//    // settingsButton = new QPushButton("Settings", this); // 已移除
//
//    layout->addWidget(startButton);
//    layout->addWidget(stopButton);
//    // layout->addWidget(settingsButton); // 已移除
//
//    setLayout(layout);
//
//    // 连接信号和槽
//    connect(startButton, &QPushButton::clicked, this, &PhaseWidget::onStart);
//    connect(stopButton, &QPushButton::clicked, this, &PhaseWidget::onStop);
//    // connect(settingsButton, &QPushButton::clicked, this, &PhaseWidget::onSettings); // 已移除
//
//    connect(phaseProcessorThread, &PhaseProcessorThread::processingStarted, this, &PhaseWidget::onProcessingStarted);
//    connect(phaseProcessorThread, &PhaseProcessorThread::processingStopped, this, &PhaseWidget::onProcessingStopped);
//}
//
//PhaseWidget::~PhaseWidget()
//{
//    if (phaseProcessorThread && phaseProcessorThread->isRunning()) 
//    {
//        phaseProcessorThread->stopProcessing();
//        phaseProcessorThread->wait();
//    }
//    delete phaseProcessorThread;
//}
//
//void PhaseWidget::onStart()
//{
//    if (phaseProcessorThread && !phaseProcessorThread->isRunning())
//    {
//        phaseProcessorThread->startProcessing();
//    }
//}
//
//void PhaseWidget::onStop()
//{
//    std::cout << "PhaseWidget::onStart() called." << std::endl;
//    if (phaseProcessorThread && phaseProcessorThread->isRunning())
//    {
//        phaseProcessorThread->stopProcessing();
//        std::cout << "PhaseProcessorThread started." << std::endl;
//    }
//    else
//    {
//        std::cout << "PhaseProcessorThread is null or already running." << std::endl;
//    }
//}
//
//void PhaseWidget::onProcessingStarted()
//{
//    std::cout << "Processing started." << std::endl;
//}
//
//void PhaseWidget::onProcessingStopped()
//{
//    std::cout << "Processing stopped." << std::endl;
//}

//#include "PhaseWidget.h"
//#include <QMessageBox>
//#include <QHBoxLayout>
//#include <QFormLayout>
//#include <QFileDialog>
//#include <QDebug>
//
//PhaseWidget::PhaseWidget(QWidget* parent)
//    : QWidget(parent),
//    m_config(std::make_shared<Config>()),
//    m_phaseProcessorThread(std::make_unique<PhaseProcessorThread>(m_config, this))
//{
//    // 初始化 UI
//    setupUI();
//
//    // 连接信号和槽
//    connectSignals();
//}
//
//PhaseWidget::~PhaseWidget()
//{
//    if (m_phaseProcessorThread && m_phaseProcessorThread->isRunning())
//    {
//        m_phaseProcessorThread->stopProcessing();
//        m_phaseProcessorThread->wait();
//    }
//    // std::unique_ptr 会自动删除对象
//}
//
//void PhaseWidget::setupUI()
//{
//    // 创建主布局
//    m_mainLayout = new QVBoxLayout(this);
//
//    // 创建表单布局用于配置参数
//    m_formLayout = new QFormLayout();
//
//    // ############# 路径配置控件 #############
//
//    // sep
//    m_sepLineEdit = new QLineEdit(QString::fromStdString(m_config->sep), this);
//    m_formLayout->addRow(tr("Separator:"), m_sepLineEdit);
//
//    // root_dir
//    m_rootDirLineEdit = new QLineEdit(QString::fromStdString(m_config->root_dir), this);
//    QWidget* rootDirWidget = new QWidget(this);
//    QHBoxLayout* rootDirLayout = new QHBoxLayout(rootDirWidget);
//    rootDirLayout->addWidget(m_rootDirLineEdit);
//    rootDirLayout->setContentsMargins(0, 0, 0, 0);
//    rootDirWidget->setLayout(rootDirLayout);
//    m_formLayout->addRow(tr("Root Directory:"), rootDirWidget);
//
//    // data_dir
//    m_dataDirLineEdit = new QLineEdit(QString::fromStdString(m_config->data_dir), this);
//    QWidget* dataDirWidget = new QWidget(this);
//    QHBoxLayout* dataDirLayout = new QHBoxLayout(dataDirWidget);
//    dataDirLayout->addWidget(m_dataDirLineEdit);
//    dataDirLayout->setContentsMargins(0, 0, 0, 0);
//    dataDirWidget->setLayout(dataDirLayout);
//    m_formLayout->addRow(tr("Data Directory:"), dataDirWidget);
//
//    // calib_dir
//    m_calibDirLineEdit = new QLineEdit(QString::fromStdString(m_config->calib_dir), this);
//    QWidget* calibDirWidget = new QWidget(this);
//    QHBoxLayout* calibDirLayout = new QHBoxLayout(calibDirWidget);
//    calibDirLayout->addWidget(m_calibDirLineEdit);
//    calibDirLayout->setContentsMargins(0, 0, 0, 0);
//    calibDirWidget->setLayout(calibDirLayout);
//    m_formLayout->addRow(tr("Calibration Directory:"), calibDirWidget);
//
//    // calib_file
//    m_calibFileLineEdit = new QLineEdit(QString::fromStdString(m_config->calib_file), this);
//    m_calibBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* calibFileWidget = new QWidget(this);
//    QHBoxLayout* calibFileLayout = new QHBoxLayout(calibFileWidget);
//    calibFileLayout->addWidget(m_calibFileLineEdit);
//    calibFileLayout->addWidget(m_calibBrowseButton);
//    calibFileLayout->setContentsMargins(0, 0, 0, 0);
//    calibFileWidget->setLayout(calibFileLayout);
//    m_formLayout->addRow(tr("Calibration File:"), calibFileWidget);
//
//    // project_dir
//    m_projectDirLineEdit = new QLineEdit(QString::fromStdString(m_config->project_dir), this);
//    m_projectDirBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* projectDirWidget = new QWidget(this);
//    QHBoxLayout* projectDirLayout = new QHBoxLayout(projectDirWidget);
//    projectDirLayout->addWidget(m_projectDirLineEdit);
//    projectDirLayout->addWidget(m_projectDirBrowseButton);
//    projectDirLayout->setContentsMargins(0, 0, 0, 0);
//    projectDirWidget->setLayout(projectDirLayout);
//    m_formLayout->addRow(tr("Project Directory:"), projectDirWidget);
//
//    // simu_dir
//    m_simuDirLineEdit = new QLineEdit(QString::fromStdString(m_config->simu_dir), this);
//    m_simuDirBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* simuDirWidget = new QWidget(this);
//    QHBoxLayout* simuDirLayout = new QHBoxLayout(simuDirWidget);
//    simuDirLayout->addWidget(m_simuDirLineEdit);
//    simuDirLayout->addWidget(m_simuDirBrowseButton);
//    simuDirLayout->setContentsMargins(0, 0, 0, 0);
//    simuDirWidget->setLayout(simuDirLayout);
//    m_formLayout->addRow(tr("Simulation Directory:"), simuDirWidget);
//
//    // model_dir
//    m_modelDirLineEdit = new QLineEdit(QString::fromStdString(m_config->model_dir), this);
//    m_modelDirBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* modelDirWidget = new QWidget(this);
//    QHBoxLayout* modelDirLayout = new QHBoxLayout(modelDirWidget);
//    modelDirLayout->addWidget(m_modelDirLineEdit);
//    modelDirLayout->addWidget(m_modelDirBrowseButton);
//    modelDirLayout->setContentsMargins(0, 0, 0, 0);
//    modelDirWidget->setLayout(modelDirLayout);
//    m_formLayout->addRow(tr("Model Directory:"), modelDirWidget);
//
//    // output_dir
//    m_outputDirLineEdit = new QLineEdit(QString::fromStdString(m_config->output_dir), this);
//    m_outputDirBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* outputDirWidget = new QWidget(this);
//    QHBoxLayout* outputDirLayout = new QHBoxLayout(outputDirWidget);
//    outputDirLayout->addWidget(m_outputDirLineEdit);
//    outputDirLayout->addWidget(m_outputDirBrowseButton);
//    outputDirLayout->setContentsMargins(0, 0, 0, 0);
//    outputDirWidget->setLayout(outputDirLayout);
//    m_formLayout->addRow(tr("Output Directory:"), outputDirWidget);
//
//    // output_dir_L
//    m_outputDirLLineEdit = new QLineEdit(QString::fromStdString(m_config->output_dir_L), this);
//    m_outputDirLBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* outputDirLWidget = new QWidget(this);
//    QHBoxLayout* outputDirLLayout = new QHBoxLayout(outputDirLWidget);
//    outputDirLLayout->addWidget(m_outputDirLLineEdit);
//    outputDirLLayout->addWidget(m_outputDirLBrowseButton);
//    outputDirLLayout->setContentsMargins(0, 0, 0, 0);
//    outputDirLWidget->setLayout(outputDirLLayout);
//    m_formLayout->addRow(tr("Output Directory L:"), outputDirLWidget);
//
//    // output_dir_R
//    m_outputDirRLineEdit = new QLineEdit(QString::fromStdString(m_config->output_dir_R), this);
//    m_outputDirRBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* outputDirRWidget = new QWidget(this);
//    QHBoxLayout* outputDirRLayout = new QHBoxLayout(outputDirRWidget);
//    outputDirRLayout->addWidget(m_outputDirRLineEdit);
//    outputDirRLayout->addWidget(m_outputDirRBrowseButton);
//    outputDirRLayout->setContentsMargins(0, 0, 0, 0);
//    outputDirRWidget->setLayout(outputDirRLayout);
//    m_formLayout->addRow(tr("Output Directory R:"), outputDirRWidget);
//
//    // save_file_point3d
//    m_saveFilePoint3DLineEdit = new QLineEdit(QString::fromStdString(m_config->save_file_point3d), this);
//    m_saveFilePoint3DBrowseButton = new QPushButton(tr("Browse"), this);
//    QWidget* saveFilePoint3DWidget = new QWidget(this);
//    QHBoxLayout* saveFilePoint3DLayout = new QHBoxLayout(saveFilePoint3DWidget);
//    saveFilePoint3DLayout->addWidget(m_saveFilePoint3DLineEdit);
//    saveFilePoint3DLayout->addWidget(m_saveFilePoint3DBrowseButton);
//    saveFilePoint3DLayout->setContentsMargins(0, 0, 0, 0);
//    saveFilePoint3DWidget->setLayout(saveFilePoint3DLayout);
//    m_formLayout->addRow(tr("Save Point3D File:"), saveFilePoint3DWidget);
//
//    // ############# 数值配置控件 #############
//
//    // A
//    m_ASpinBox = new QDoubleSpinBox(this);
//    m_ASpinBox->setRange(-10000.0, 10000.0);
//    m_ASpinBox->setValue(m_config->A);
//    m_ASpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("A:"), m_ASpinBox);
//
//    // B
//    m_BSpinBox = new QDoubleSpinBox(this);
//    m_BSpinBox->setRange(-10000.0, 10000.0);
//    m_BSpinBox->setValue(m_config->B);
//    m_BSpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("B:"), m_BSpinBox);
//
//    // N
//    m_NSpinBox = new QSpinBox(this);
//    m_NSpinBox->setRange(1, 1000);
//    m_NSpinBox->setValue(m_config->N);
//    m_formLayout->addRow(tr("N:"), m_NSpinBox);
//
//    // T
//    m_TSpinBox = new QSpinBox(this);
//    m_TSpinBox->setRange(1, 1000);
//    m_TSpinBox->setValue(m_config->T);
//    m_formLayout->addRow(tr("T:"), m_TSpinBox);
//
//    // T1
//    m_T1SpinBox = new QDoubleSpinBox(this);
//    m_T1SpinBox->setRange(0.0, 10000.0);
//    m_T1SpinBox->setValue(m_config->T1);
//    m_T1SpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("T1:"), m_T1SpinBox);
//
//    // T2
//    m_T2SpinBox = new QDoubleSpinBox(this);
//    m_T2SpinBox->setRange(0.0, 10000.0);
//    m_T2SpinBox->setValue(m_config->T2);
//    m_T2SpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("T2:"), m_T2SpinBox);
//
//    // T3
//    m_T3SpinBox = new QDoubleSpinBox(this);
//    m_T3SpinBox->setRange(0.0, 10000.0);
//    m_T3SpinBox->setValue(m_config->T3);
//    m_T3SpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("T3:"), m_T3SpinBox);
//
//    // W
//    m_WSpinBox = new QSpinBox(this);
//    m_WSpinBox->setRange(1, 10000);
//    m_WSpinBox->setValue(m_config->W);
//    m_formLayout->addRow(tr("W:"), m_WSpinBox);
//
//    // H
//    m_HSpinBox = new QSpinBox(this);
//    m_HSpinBox->setRange(1, 10000);
//    m_HSpinBox->setValue(m_config->H);
//    m_formLayout->addRow(tr("H:"), m_HSpinBox);
//
//    // B_min
//    m_BMinSpinBox = new QDoubleSpinBox(this);
//    m_BMinSpinBox->setRange(0.0, 10000.0);
//    m_BMinSpinBox->setValue(m_config->B_min);
//    m_BMinSpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("B_min:"), m_BMinSpinBox);
//
//    // win
//    m_winSpinBox = new QSpinBox(this);
//    m_winSpinBox->setRange(1, 1000);
//    m_winSpinBox->setValue(m_config->win);
//    m_formLayout->addRow(tr("Window Size:"), m_winSpinBox);
//
//    // pd
//    m_pdSpinBox = new QDoubleSpinBox(this);
//    m_pdSpinBox->setRange(0.0, 1.0);
//    m_pdSpinBox->setValue(m_config->pd);
//    m_pdSpinBox->setDecimals(2);
//    m_formLayout->addRow(tr("PD:"), m_pdSpinBox);
//
//    // min_z
//    m_minZSpinBox = new QDoubleSpinBox(this);
//    m_minZSpinBox->setRange(-10000.0, 10000.0);
//    m_minZSpinBox->setValue(m_config->min_z);
//    m_minZSpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("min_z:"), m_minZSpinBox);
//
//    // max_z
//    m_maxZSpinBox = new QDoubleSpinBox(this);
//    m_maxZSpinBox->setRange(-10000.0, 10000.0);
//    m_maxZSpinBox->setValue(m_config->max_z);
//    m_maxZSpinBox->setDecimals(4);
//    m_formLayout->addRow(tr("max_z:"), m_maxZSpinBox);
//
//    // ############# 布尔配置控件 #############
//
//    // write
//    m_writeCheckBox = new QCheckBox(tr("Write Output"), this);
//    m_writeCheckBox->setChecked(m_config->write);
//    m_formLayout->addRow(tr("Write Output:"), m_writeCheckBox);
//
//    // show
//    m_showCheckBox = new QCheckBox(tr("Show Results"), this);
//    m_showCheckBox->setChecked(m_config->show);
//    m_formLayout->addRow(tr("Show Results:"), m_showCheckBox);
//
//    // 添加表单布局到主布局
//    m_mainLayout->addLayout(m_formLayout);
//
//    // ############# 进度条 #############
//    m_progressBar = new QProgressBar(this);
//    m_progressBar->setRange(0, 100);
//    m_progressBar->setValue(0);
//    m_mainLayout->addWidget(m_progressBar);
//
//    // ############# 按钮布局 #############
//    m_buttonLayout = new QHBoxLayout();
//
//    m_startButton = new QPushButton(tr("Phase Start"), this);
//    m_stopButton = new QPushButton(tr("Phase Stop"), this);
//    m_resetButton = new QPushButton(tr("Reset to Default"), this);
//
//    m_buttonLayout->addWidget(m_startButton);
//    m_buttonLayout->addWidget(m_stopButton);
//    m_buttonLayout->addWidget(m_resetButton);
//
//    // 添加按钮布局到主布局
//    m_mainLayout->addLayout(m_buttonLayout);
//
//    setLayout(m_mainLayout);
//}
//
//void PhaseWidget::connectSignals()
//{
//    // 连接浏览按钮
//    connect(m_calibBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString fileName = QFileDialog::getOpenFileName(this, tr("Select Calibration File"), "", tr("Calibration Files (*.json *.xml *.yaml)"));
//        if (!fileName.isEmpty()) {
//            m_calibFileLineEdit->setText(fileName);
//        }
//        });
//
//    connect(m_projectDirBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString dir = QFileDialog::getExistingDirectory(this, tr("Select Project Directory"));
//        if (!dir.isEmpty()) {
//            m_projectDirLineEdit->setText(dir);
//        }
//        });
//
//    connect(m_simuDirBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString dir = QFileDialog::getExistingDirectory(this, tr("Select Simulation Directory"));
//        if (!dir.isEmpty()) {
//            m_simuDirLineEdit->setText(dir);
//        }
//        });
//
//    connect(m_modelDirBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString dir = QFileDialog::getExistingDirectory(this, tr("Select Model Directory"));
//        if (!dir.isEmpty()) {
//            m_modelDirLineEdit->setText(dir);
//        }
//        });
//
//    connect(m_outputDirBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory"));
//        if (!dir.isEmpty()) {
//            m_outputDirLineEdit->setText(dir);
//        }
//        });
//
//    connect(m_outputDirLBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory L"));
//        if (!dir.isEmpty()) {
//            m_outputDirLLineEdit->setText(dir);
//        }
//        });
//
//    connect(m_outputDirRBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString dir = QFileDialog::getExistingDirectory(this, tr("Select Output Directory R"));
//        if (!dir.isEmpty()) {
//            m_outputDirRLineEdit->setText(dir);
//        }
//        });
//
//    connect(m_saveFilePoint3DBrowseButton, &QPushButton::clicked, this, [this]() {
//        QString fileName = QFileDialog::getSaveFileName(this, tr("Select Save File for Point3D"), "", tr("Point3D Files (*.txt *.xyz)"));
//        if (!fileName.isEmpty()) {
//            m_saveFilePoint3DLineEdit->setText(fileName);
//        }
//        });
//
//    // 连接开始和停止按钮
//    connect(m_startButton, &QPushButton::clicked, this, &PhaseWidget::onStart);
//    connect(m_stopButton, &QPushButton::clicked, this, &PhaseWidget::onStop);
//
//    // 连接重置按钮
//    connect(m_resetButton, &QPushButton::clicked, this, [this]() {
//        // 恢复默认值
//        m_config = std::make_shared<Config>();
//
//        // 更新界面控件
//        m_sepLineEdit->setText(QString::fromStdString(m_config->sep));
//        m_rootDirLineEdit->setText(QString::fromStdString(m_config->root_dir));
//        m_dataDirLineEdit->setText(QString::fromStdString(m_config->data_dir));
//        m_calibDirLineEdit->setText(QString::fromStdString(m_config->calib_dir));
//        m_calibFileLineEdit->setText(QString::fromStdString(m_config->calib_file));
//        m_projectDirLineEdit->setText(QString::fromStdString(m_config->project_dir));
//        m_simuDirLineEdit->setText(QString::fromStdString(m_config->simu_dir));
//        m_modelDirLineEdit->setText(QString::fromStdString(m_config->model_dir));
//        m_outputDirLineEdit->setText(QString::fromStdString(m_config->output_dir));
//        m_outputDirLLineEdit->setText(QString::fromStdString(m_config->output_dir_L));
//        m_outputDirRLineEdit->setText(QString::fromStdString(m_config->output_dir_R));
//        m_saveFilePoint3DLineEdit->setText(QString::fromStdString(m_config->save_file_point3d));
//
//        m_ASpinBox->setValue(m_config->A);
//        m_BSpinBox->setValue(m_config->B);
//        m_NSpinBox->setValue(m_config->N);
//        m_TSpinBox->setValue(m_config->T);
//        m_T1SpinBox->setValue(m_config->T1);
//        m_T2SpinBox->setValue(m_config->T2);
//        m_T3SpinBox->setValue(m_config->T3);
//        m_WSpinBox->setValue(m_config->W);
//        m_HSpinBox->setValue(m_config->H);
//        m_BMinSpinBox->setValue(m_config->B_min);
//
//        m_winSpinBox->setValue(m_config->win);
//        m_pdSpinBox->setValue(m_config->pd);
//        m_minZSpinBox->setValue(m_config->min_z);
//        m_maxZSpinBox->setValue(m_config->max_z);
//
//        m_writeCheckBox->setChecked(m_config->write);
//        m_showCheckBox->setChecked(m_config->show);
//
//        m_progressBar->setValue(0); // 重置进度条
//
//        QMessageBox::information(this, tr("Reset"), tr("Configuration reset to default values."));
//        });
//
//    // 连接处理线程信号
//    connect(m_phaseProcessorThread.get(), &PhaseProcessorThread::processingStarted, this, &PhaseWidget::onProcessingStarted);
//    connect(m_phaseProcessorThread.get(), &PhaseProcessorThread::processingStopped, this, &PhaseWidget::onProcessingStopped);
//    connect(m_phaseProcessorThread.get(), &PhaseProcessorThread::progressUpdated, this, [this](int percentage) {
//        m_progressBar->setValue(percentage);
//        });
//}
//
//void PhaseWidget::onStart()
//{
//    if (m_phaseProcessorThread && !m_phaseProcessorThread->isRunning())
//    {
//        // 更新 Config 对象的值
//        m_config->sep = m_sepLineEdit->text().toStdString();
//        m_config->root_dir = m_rootDirLineEdit->text().toStdString();
//        m_config->data_dir = m_dataDirLineEdit->text().toStdString();
//        m_config->calib_dir = m_calibDirLineEdit->text().toStdString();
//        m_config->calib_file = m_calibFileLineEdit->text().toStdString();
//        m_config->project_dir = m_projectDirLineEdit->text().toStdString();
//        m_config->simu_dir = m_simuDirLineEdit->text().toStdString();
//        m_config->model_dir = m_modelDirLineEdit->text().toStdString();
//        m_config->output_dir = m_outputDirLineEdit->text().toStdString();
//        m_config->output_dir_L = m_outputDirLLineEdit->text().toStdString();
//        m_config->output_dir_R = m_outputDirRLineEdit->text().toStdString();
//        m_config->save_file_point3d = m_saveFilePoint3DLineEdit->text().toStdString();
//
//        m_config->A = m_ASpinBox->value();
//        m_config->B = m_BSpinBox->value();
//        m_config->N = m_NSpinBox->value();
//        m_config->T = m_TSpinBox->value();
//        m_config->T1 = m_T1SpinBox->value();
//        m_config->T2 = m_T2SpinBox->value();
//        m_config->T3 = m_T3SpinBox->value();
//        m_config->W = m_WSpinBox->value();
//        m_config->H = m_HSpinBox->value();
//        m_config->B_min = m_BMinSpinBox->value();
//
//        m_config->win = m_winSpinBox->value();
//        m_config->pd = m_pdSpinBox->value();
//        m_config->min_z = m_minZSpinBox->value();
//        m_config->max_z = m_maxZSpinBox->value();
//
//        m_config->write = m_writeCheckBox->isChecked();
//        m_config->show = m_showCheckBox->isChecked();
//
//        m_phaseProcessorThread->startProcessing();
//    }
//}
//
//void PhaseWidget::onStop()
//{
//    qDebug() << "PhaseWidget::onStop() called.";
//    if (m_phaseProcessorThread && m_phaseProcessorThread->isRunning())
//    {
//        m_phaseProcessorThread->stopProcessing();
//        qDebug() << "PhaseProcessorThread stopping.";
//    }
//    else
//    {
//        qDebug() << "PhaseProcessorThread is null or not running.";
//    }
//}
//
//void PhaseWidget::onProcessingStarted()
//{
//    qDebug() << "Processing started.";
//    m_startButton->setEnabled(false);
//    m_stopButton->setEnabled(true);
//    m_resetButton->setEnabled(false);
//
//    // 禁用配置控件以防止在处理过程中更改
//    m_sepLineEdit->setEnabled(false);
//    m_rootDirLineEdit->setEnabled(false);
//    m_dataDirLineEdit->setEnabled(false);
//    m_calibDirLineEdit->setEnabled(false);
//    m_calibFileLineEdit->setEnabled(false);
//    m_calibBrowseButton->setEnabled(false);
//    m_projectDirLineEdit->setEnabled(false);
//    m_projectDirBrowseButton->setEnabled(false);
//    m_simuDirLineEdit->setEnabled(false);
//    m_simuDirBrowseButton->setEnabled(false);
//    m_modelDirLineEdit->setEnabled(false);
//    m_modelDirBrowseButton->setEnabled(false);
//    m_outputDirLineEdit->setEnabled(false);
//    m_outputDirBrowseButton->setEnabled(false);
//    m_outputDirLLineEdit->setEnabled(false);
//    m_outputDirLBrowseButton->setEnabled(false);
//    m_outputDirRLineEdit->setEnabled(false);
//    m_outputDirRBrowseButton->setEnabled(false);
//    m_saveFilePoint3DLineEdit->setEnabled(false);
//    m_saveFilePoint3DBrowseButton->setEnabled(false);
//
//    m_ASpinBox->setEnabled(false);
//    m_BSpinBox->setEnabled(false);
//    m_NSpinBox->setEnabled(false);
//    m_TSpinBox->setEnabled(false);
//    m_T1SpinBox->setEnabled(false);
//    m_T2SpinBox->setEnabled(false);
//    m_T3SpinBox->setEnabled(false);
//    m_WSpinBox->setEnabled(false);
//    m_HSpinBox->setEnabled(false);
//    m_BMinSpinBox->setEnabled(false);
//
//    m_winSpinBox->setEnabled(false);
//    m_pdSpinBox->setEnabled(false);
//    m_minZSpinBox->setEnabled(false);
//    m_maxZSpinBox->setEnabled(false);
//
//    m_writeCheckBox->setEnabled(false);
//    m_showCheckBox->setEnabled(false);
//}
//
//void PhaseWidget::onProcessingStopped()
//{
//    qDebug() << "Processing stopped.";
//    m_startButton->setEnabled(true);
//    m_stopButton->setEnabled(false);
//    m_resetButton->setEnabled(true);
//
//    // 重新启用配置控件
//    m_sepLineEdit->setEnabled(true);
//    m_rootDirLineEdit->setEnabled(true);
//    m_dataDirLineEdit->setEnabled(true);
//    m_calibDirLineEdit->setEnabled(true);
//    m_calibFileLineEdit->setEnabled(true);
//    m_calibBrowseButton->setEnabled(true);
//    m_projectDirLineEdit->setEnabled(true);
//    m_projectDirBrowseButton->setEnabled(true);
//    m_simuDirLineEdit->setEnabled(true);
//    m_simuDirBrowseButton->setEnabled(true);
//    m_modelDirLineEdit->setEnabled(true);
//    m_modelDirBrowseButton->setEnabled(true);
//    m_outputDirLineEdit->setEnabled(true);
//    m_outputDirBrowseButton->setEnabled(true);
//    m_outputDirLLineEdit->setEnabled(true);
//    m_outputDirLBrowseButton->setEnabled(true);
//    m_outputDirRLineEdit->setEnabled(true);
//    m_outputDirRBrowseButton->setEnabled(true);
//    m_saveFilePoint3DLineEdit->setEnabled(true);
//    m_saveFilePoint3DBrowseButton->setEnabled(true);
//
//    m_ASpinBox->setEnabled(true);
//    m_BSpinBox->setEnabled(true);
//    m_NSpinBox->setEnabled(true);
//    m_TSpinBox->setEnabled(true);
//    m_T1SpinBox->setEnabled(true);
//    m_T2SpinBox->setEnabled(true);
//    m_T3SpinBox->setEnabled(true);
//    m_WSpinBox->setEnabled(true);
//    m_HSpinBox->setEnabled(true);
//    m_BMinSpinBox->setEnabled(true);
//
//    m_winSpinBox->setEnabled(true);
//    m_pdSpinBox->setEnabled(true);
//    m_minZSpinBox->setEnabled(true);
//    m_maxZSpinBox->setEnabled(true);
//
//    m_writeCheckBox->setEnabled(true);
//    m_showCheckBox->setEnabled(true);
//
//    m_progressBar->setValue(0); // 重置进度条
//
//    QMessageBox::information(this, tr("Processing Completed"), tr("Phase processing has been completed."));
//}
#include "PhaseWidget.h"
#include <QMessageBox>
#include <iostream>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QScrollArea>
#include <QFileDialog>

PhaseWidget::PhaseWidget(QWidget* parent)
    : QWidget(parent),
    config(std::make_shared<Config>())
{
    // 创建 PhaseProcessorThread 对象，父对象为 this
    phaseProcessorThread = new PhaseProcessorThread(config.get());

    // 主布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 创建设置控件的布局
    createSettingsLayout();

    // 创建启动和停止按钮，以及进度条
    startButton = new QPushButton("Phase Start", this);
    stopButton = new QPushButton("Phase Stop", this);
    progressBar = new QProgressBar(this);
    progressBar->setRange(0, 100);
    progressBar->setValue(0);

    // 按钮布局
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);

    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(progressBar);

    setLayout(mainLayout);

    // 连接信号和槽
    connect(startButton, &QPushButton::clicked, this, &PhaseWidget::onStart);
    connect(stopButton, &QPushButton::clicked, this, &PhaseWidget::onStop);

    connect(phaseProcessorThread, &PhaseProcessorThread::processingStarted, this, &PhaseWidget::onProcessingStarted);
    connect(phaseProcessorThread, &PhaseProcessorThread::processingStopped, this, &PhaseWidget::onProcessingStopped);
    connect(phaseProcessorThread, &PhaseProcessorThread::processingProgress, this, &PhaseWidget::onProcessingProgress);
}

PhaseWidget::~PhaseWidget()
{
    if (phaseProcessorThread && phaseProcessorThread->isRunning())
    {
        phaseProcessorThread->stopProcessing();
        phaseProcessorThread->wait();
    }
    delete phaseProcessorThread;
}

void PhaseWidget::createSettingsLayout()
{
    // 使用滚动区域来包含大量的设置控件
    QScrollArea* scrollArea = new QScrollArea(this);
    QWidget* settingsWidget = new QWidget(scrollArea);
    QFormLayout* formLayout = new QFormLayout(settingsWidget);

    // 创建控件
    rootDirLineEdit = new QLineEdit(this);
    dataDirLineEdit = new QLineEdit(this);
    calibDirLineEdit = new QLineEdit(this);
    calibFileLineEdit = new QLineEdit(this);
    projectDirLineEdit = new QLineEdit(this);
    simuDirLineEdit = new QLineEdit(this);
    modelDirLineEdit = new QLineEdit(this);
    outputDirLineEdit = new QLineEdit(this);
    outputDirLLineEdit = new QLineEdit(this); 
    outputDirRLineEdit = new QLineEdit(this);
    saveFilePoint3dLineEdit = new QLineEdit(this);

    browseRootDirButton = new QPushButton("Browse", this);
    browseDataDirButton = new QPushButton("Browse", this);
    browseCalibDirButton = new QPushButton("Browse", this);
    browseCalibFileButton = new QPushButton("Browse", this);
    browseProjectDirButton = new QPushButton("Browse", this);
    browseSimuDirButton = new QPushButton("Browse", this);
    browseModelDirButton = new QPushButton("Browse", this);
    browseOutputDirButton = new QPushButton("Browse", this);
    browseOutputDirLButton = new QPushButton("Browse", this); 
    browseOutputDirRButton = new QPushButton("Browse", this);
    browseSaveFilePoint3dButton = new QPushButton("Browse", this);

    writeCheckBox = new QCheckBox(this);
    showCheckBox = new QCheckBox(this);

    ALineEdit = new QLineEdit(this);
    BLineEdit = new QLineEdit(this);
    NLineEdit = new QLineEdit(this);
    TLineEdit = new QLineEdit(this);
    T1LineEdit = new QLineEdit(this);
    T2LineEdit = new QLineEdit(this);
    T3LineEdit = new QLineEdit(this);
    WLineEdit = new QLineEdit(this);
    HLineEdit = new QLineEdit(this);
    BMinLineEdit = new QLineEdit(this);

    winLineEdit = new QLineEdit(this);
    pdLineEdit = new QLineEdit(this);
    minZLineEdit = new QLineEdit(this);
    maxZLineEdit = new QLineEdit(this);

    // 为路径添加“浏览”按钮
    // Root Directory
    QHBoxLayout* rootDirLayout = new QHBoxLayout();
    rootDirLayout->addWidget(rootDirLineEdit);
    rootDirLayout->addWidget(browseRootDirButton);
    formLayout->addRow("Root Directory:", rootDirLayout);

    // Data Directory
    QHBoxLayout* dataDirLayout = new QHBoxLayout();
    dataDirLayout->addWidget(dataDirLineEdit);
    dataDirLayout->addWidget(browseDataDirButton);
    formLayout->addRow("Data Directory:", dataDirLayout);

    // Calibration Directory
        QHBoxLayout * calibDirLayout = new QHBoxLayout();
    calibDirLayout->addWidget(calibDirLineEdit);
    calibDirLayout->addWidget(browseCalibDirButton);
    formLayout->addRow("Calibration Directory:", calibDirLayout);

    // Calibration File
    QHBoxLayout* calibFileLayout = new QHBoxLayout();
    calibFileLayout->addWidget(calibFileLineEdit);
    calibFileLayout->addWidget(browseCalibFileButton);
    formLayout->addRow("Calibration File:", calibFileLayout);

    // Project Directory
    QHBoxLayout* projectDirLayout = new QHBoxLayout();
    projectDirLayout->addWidget(projectDirLineEdit);
    projectDirLayout->addWidget(browseProjectDirButton);
    formLayout->addRow("Project Directory:", projectDirLayout);

    // Simulation Directory
    QHBoxLayout* simuDirLayout = new QHBoxLayout();
    simuDirLayout->addWidget(simuDirLineEdit);
    simuDirLayout->addWidget(browseSimuDirButton);
    formLayout->addRow("Simulation Directory:", simuDirLayout);

    // Model Directory
    QHBoxLayout* modelDirLayout = new QHBoxLayout();
    modelDirLayout->addWidget(modelDirLineEdit);
    modelDirLayout->addWidget(browseModelDirButton);
    formLayout->addRow("Model Directory:", modelDirLayout);

    // Output Directory
    QHBoxLayout* outputDirLayout = new QHBoxLayout();
    outputDirLayout->addWidget(outputDirLineEdit);
    outputDirLayout->addWidget(browseOutputDirButton);
    formLayout->addRow("Output Directory:", outputDirLayout);

    // Output Directory L
    QHBoxLayout* outputDirLLayout = new QHBoxLayout();
    outputDirLLayout->addWidget(outputDirLLineEdit);
    outputDirLLayout->addWidget(browseOutputDirLButton);
    formLayout->addRow("Output Directory L:", outputDirLLayout);

    // Output Directory R
    QHBoxLayout* outputDirRLayout = new QHBoxLayout();
    outputDirRLayout->addWidget(outputDirRLineEdit);
    outputDirRLayout->addWidget(browseOutputDirRButton);
    formLayout->addRow("Output Directory R:", outputDirRLayout);

    // Save Point3D File
    QHBoxLayout* saveFilePoint3dLayout = new QHBoxLayout();
    saveFilePoint3dLayout->addWidget(saveFilePoint3dLineEdit);
    saveFilePoint3dLayout->addWidget(browseSaveFilePoint3dButton);
    formLayout->addRow("Save Point3D File:", saveFilePoint3dLayout);

    // 其他参数
    formLayout->addRow("Write Files:", writeCheckBox);
    formLayout->addRow("Show Results:", showCheckBox);

    formLayout->addRow("A:", ALineEdit);
    formLayout->addRow("B:", BLineEdit);
    formLayout->addRow("N:", NLineEdit);
    formLayout->addRow("T:", TLineEdit);
    formLayout->addRow("T1:", T1LineEdit);
    formLayout->addRow("T2:", T2LineEdit);
    formLayout->addRow("T3:", T3LineEdit);
    formLayout->addRow("Width (W):", WLineEdit);
    formLayout->addRow("Height (H):", HLineEdit);
    formLayout->addRow("B_min:", BMinLineEdit);

    formLayout->addRow("Window Size (win):", winLineEdit);
    formLayout->addRow("Phase Difference (pd):", pdLineEdit);
    formLayout->addRow("Min Z:", minZLineEdit);
    formLayout->addRow("Max Z:", maxZLineEdit);

    settingsWidget->setLayout(formLayout);
    scrollArea->setWidget(settingsWidget);
    scrollArea->setWidgetResizable(true);

    // 将滚动区域添加到主布局
    layout()->addWidget(scrollArea);

    // 加载配置参数到界面
    loadConfigToUI();

    // 连接“浏览”按钮的信号槽
    connect(browseRootDirButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Root Directory");
        if (!dir.isEmpty()) {
            rootDirLineEdit->setText(dir);
        }
        });

    connect(browseDataDirButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Data Directory");
        if (!dir.isEmpty()) {
            dataDirLineEdit->setText(dir);
        }
        });

    connect(browseCalibDirButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Calibration Directory");
        if (!dir.isEmpty()) {
            calibDirLineEdit->setText(dir);
        }
        });

    connect(browseCalibFileButton, &QPushButton::clicked, this, [=]() {
        QString file = QFileDialog::getOpenFileName(this, "Select Calibration File");
        if (!file.isEmpty()) {
            calibFileLineEdit->setText(file);
        }
        });

    connect(browseProjectDirButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Project Directory");
        if (!dir.isEmpty()) {
            projectDirLineEdit->setText(dir);
        }
        });

    connect(browseSimuDirButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Simulation Directory");
        if (!dir.isEmpty()) {
            simuDirLineEdit->setText(dir);
        }
        });

    connect(browseModelDirButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Model Directory");
        if (!dir.isEmpty()) {
            modelDirLineEdit->setText(dir);
        }
        });

    connect(browseOutputDirButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Output Directory");
        if (!dir.isEmpty()) {
            outputDirLineEdit->setText(dir);
        }
        });

    connect(browseOutputDirLButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Output Directory L");
        if (!dir.isEmpty()) {
            outputDirLLineEdit->setText(dir);
        }
        });

    connect(browseOutputDirRButton, &QPushButton::clicked, this, [=]() {
        QString dir = QFileDialog::getExistingDirectory(this, "Select Output Directory R");
        if (!dir.isEmpty()) {
            outputDirRLineEdit->setText(dir);
        }
        });

    connect(browseSaveFilePoint3dButton, &QPushButton::clicked, this, [=]() {
        QString file = QFileDialog::getSaveFileName(this, "Select Save Point3D File");
        if (!file.isEmpty()) {
            saveFilePoint3dLineEdit->setText(file);
        }
        });
}

void PhaseWidget::loadConfigToUI()
{
    rootDirLineEdit->setText(QString::fromStdString(config->root_dir));
    dataDirLineEdit->setText(QString::fromStdString(config->data_dir));
    calibDirLineEdit->setText(QString::fromStdString(config->calib_dir));
    calibFileLineEdit->setText(QString::fromStdString(config->calib_file));
    projectDirLineEdit->setText(QString::fromStdString(config->project_dir));
    simuDirLineEdit->setText(QString::fromStdString(config->simu_dir));
    modelDirLineEdit->setText(QString::fromStdString(config->model_dir));
    outputDirLineEdit->setText(QString::fromStdString(config->output_dir));
    outputDirLLineEdit->setText(QString::fromStdString(config->output_dir_L)); 
    outputDirRLineEdit->setText(QString::fromStdString(config->output_dir_R));
    saveFilePoint3dLineEdit->setText(QString::fromStdString(config->save_file_point3d));

    writeCheckBox->setChecked(config->write);
    showCheckBox->setChecked(config->show);

    ALineEdit->setText(QString::number(config->A));
    BLineEdit->setText(QString::number(config->B));
    NLineEdit->setText(QString::number(config->N));
    TLineEdit->setText(QString::number(config->T));
    T1LineEdit->setText(QString::number(config->T1));
    T2LineEdit->setText(QString::number(config->T2));
    T3LineEdit->setText(QString::number(config->T3));
    WLineEdit->setText(QString::number(config->W));
    HLineEdit->setText(QString::number(config->H));
    BMinLineEdit->setText(QString::number(config->B_min));

    winLineEdit->setText(QString::number(config->win));
    pdLineEdit->setText(QString::number(config->pd));
    minZLineEdit->setText(QString::number(config->min_z));
    maxZLineEdit->setText(QString::number(config->max_z));
}

void PhaseWidget::saveUIToConfig()
{
    config->root_dir = rootDirLineEdit->text().toStdString();
    config->data_dir = dataDirLineEdit->text().toStdString();
    config->calib_dir = calibDirLineEdit->text().toStdString();
    config->calib_file = calibFileLineEdit->text().toStdString();
    config->project_dir = projectDirLineEdit->text().toStdString();
    config->simu_dir = simuDirLineEdit->text().toStdString();
    config->model_dir = modelDirLineEdit->text().toStdString();
    config->output_dir = outputDirLineEdit->text().toStdString();
    config->output_dir_L = outputDirLLineEdit->text().toStdString(); 
    config->output_dir_R = outputDirRLineEdit->text().toStdString();
    config->save_file_point3d = saveFilePoint3dLineEdit->text().toStdString();

    config->write = writeCheckBox->isChecked();
    config->show = showCheckBox->isChecked();

    config->A = ALineEdit->text().toDouble();
    config->B = BLineEdit->text().toDouble();
    config->N = NLineEdit->text().toInt();
    config->T = TLineEdit->text().toInt();
    config->T1 = T1LineEdit->text().toDouble();
    config->T2 = T2LineEdit->text().toDouble();
    config->T3 = T3LineEdit->text().toDouble();
    config->W = WLineEdit->text().toInt();
    config->H = HLineEdit->text().toInt();
    config->B_min = BMinLineEdit->text().toFloat();

    config->win = winLineEdit->text().toInt();
    config->pd = pdLineEdit->text().toFloat();
    config->min_z = minZLineEdit->text().toFloat();
    config->max_z = maxZLineEdit->text().toFloat();

    // 根据 root_dir 和 sep 更新其他路径
    /*config->sep = "/";
    config->output_dir_L = config->output_dir + config->sep + "L";
    config->output_dir_R = config->output_dir + config->sep + "R";*/
}

void PhaseWidget::onStart()
{
    // 在启动处理之前，保存界面上的参数到配置
    saveUIToConfig();

    if (phaseProcessorThread && !phaseProcessorThread->isRunning())
    {
        phaseProcessorThread->startProcessing();
    }
}

void PhaseWidget::onStop()
{
    if (phaseProcessorThread && phaseProcessorThread->isRunning())
    {
        phaseProcessorThread->stopProcessing();
    }
}

void PhaseWidget::onProcessingStarted()
{
    std::cout << "Processing started." << std::endl;
    progressBar->setValue(0);
}

void PhaseWidget::onProcessingStopped()
{
    std::cout << "Processing stopped." << std::endl;
    progressBar->setValue(100);
}

void PhaseWidget::onProcessingProgress(int value)
{
    progressBar->setValue(value);
}
