#include "AdvancedSettingsDialog.h"
#include <QFileDialog>

AdvancedSettingsDialog::AdvancedSettingsDialog(QWidget* parent): QDialog(parent)
{
    setWindowTitle("RegisWays");

    tabWidget = new QTabWidget(this);

    // ----- ICP 页面 -----
    icpPage = new QWidget(this);
    QFormLayout* icpLayout = new QFormLayout(icpPage);
    IcpDataPathLineEdit = new QLineEdit(icpPage);
    QPushButton* browseIcpButton = new QPushButton("Browse", icpPage);
    QHBoxLayout* icpHLayout = new QHBoxLayout();
    icpHLayout->addWidget(IcpDataPathLineEdit);
    icpHLayout->addWidget(browseIcpButton);
    icpLayout->addRow(new QLabel("IcpDataPath:"), icpHLayout);
    icpPage->setLayout(icpLayout);
    tabWidget->addTab(icpPage, "ICP Settings");

    // 连接 ICP 浏览按钮
    connect(browseIcpButton, &QPushButton::clicked, this, [this]() {
        QString fileName = QFileDialog::getOpenFileName(this, "Select ICP Data File", "", "Point Cloud Files (*.pcd *.ply *.txt)");
        if (!fileName.isEmpty())
            IcpDataPathLineEdit->setText(fileName);
        });

    // ----- RANSAC 页面 -----
    ransacPage = new QWidget(this);
    QFormLayout* ransacLayout = new QFormLayout(ransacPage);
    CalibPathLineEdit = new QLineEdit(ransacPage);
    QPushButton* browseCalibButton = new QPushButton("Browse", ransacPage);
    QHBoxLayout* calibHLayout = new QHBoxLayout();
    calibHLayout->addWidget(CalibPathLineEdit);
    calibHLayout->addWidget(browseCalibButton);

    ImagePathLineEdit = new QLineEdit(ransacPage);
    QPushButton* browseImageButton = new QPushButton("Browse", ransacPage);
    QHBoxLayout* imageHLayout = new QHBoxLayout();
    imageHLayout->addWidget(ImagePathLineEdit);
    imageHLayout->addWidget(browseImageButton);

    CloudPointdataPathLineEdit = new QLineEdit(ransacPage);
    QPushButton* browseCloudDataButton = new QPushButton("Browse", ransacPage);
    QHBoxLayout* cloudHLayout = new QHBoxLayout();
    cloudHLayout->addWidget(CloudPointdataPathLineEdit);
    cloudHLayout->addWidget(browseCloudDataButton);

    ransacLayout->addRow(new QLabel("CalibPath:"), calibHLayout);
    ransacLayout->addRow(new QLabel("ImagePath:"), imageHLayout);
    ransacLayout->addRow(new QLabel("CloudPointDataPath:"), cloudHLayout);
    ransacPage->setLayout(ransacLayout);
    tabWidget->addTab(ransacPage, "RANSAC Settings");

    // 连接 RANSAC 浏览按钮
    connect(browseCalibButton, &QPushButton::clicked, this, [this]() {
        // 使用 getOpenFileName 获取校准文件路径
        QString fileName = QFileDialog::getOpenFileName(this, "Select Calibration File", "", "Calibration Files (*.txt *.xml *.yaml)");
        if (!fileName.isEmpty())
            CalibPathLineEdit->setText(fileName);
        });
    connect(browseImageButton, &QPushButton::clicked, this, [this]() {
        // 如果 ImagePath 指向文件夹，可以使用 getExistingDirectory
        QString folder = QFileDialog::getExistingDirectory(this, "Select Image Folder");
        if (!folder.isEmpty())
            ImagePathLineEdit->setText(folder);
        });
    connect(browseCloudDataButton, &QPushButton::clicked, this, [this]() {
        QString folder = QFileDialog::getExistingDirectory(this, "Select Cloud Data Folder");
        if (!folder.isEmpty())
            CloudPointdataPathLineEdit->setText(folder);
        });

    // ----- 按钮区域 -----
    okButton = new QPushButton("EnSure", this);
    cancelButton = new QPushButton("CanCel", this);
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch();
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);

    // 总布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(tabWidget);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    // 连接按钮信号
    connect(okButton, &QPushButton::clicked, this, &AdvancedSettingsDialog::accept);
    connect(cancelButton, &QPushButton::clicked, this, &AdvancedSettingsDialog::reject);
}

AdvancedSettingsDialog::~AdvancedSettingsDialog()
{
}

QString AdvancedSettingsDialog::getIcpDataPath() const
{
    return IcpDataPathLineEdit->text();
}

QString AdvancedSettingsDialog::getCalibPath() const
{
    return CalibPathLineEdit->text();
}

QString AdvancedSettingsDialog::getImagePath() const
{
    return ImagePathLineEdit->text();
}

QString AdvancedSettingsDialog::getCloudPointdataPath() const
{
    return CloudPointdataPathLineEdit->text();
}
