#include "Mycode.h"
#include "ui_Mycode.h"

#include "RegistrationWidget.h"//点云显示与拼接模块

//#include "ProjectWidget.h"

#include "CameraWidget.h"

#include "Calibwidget.h"

#include "PhaseWidget.h"

#include <QPushButton>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QDebug>


Mycode::Mycode(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    //创建不同的模块页面
    Camerawidget* cameracontrol = new Camerawidget(1,this);  // 相机控制模块
    //ProjectWidget* projectcontrol = new ProjectWidget(this);  // 投影仪控制模块
    CalibWidget* calibration = new CalibWidget(this);  // 相机标定模块
    PhaseWidget* phaseWidget = new PhaseWidget(this);  // 相位计算模块
    RegistrationWidget* registration = new RegistrationWidget( this);  // 点云显示和点云拼接模块
    

    //创建按钮
    QPushButton* pbtn1 = new QPushButton("Camera Control",this);
    //QPushButton* pbtn2 = new QPushButton("Project Control", this);
    QPushButton* pbtn3 = new QPushButton("Calibration", this);
    QPushButton* pbtn4 = new QPushButton("Phase", this);
    QPushButton* pbtn5 = new QPushButton("PointCloudProcess", this);

    // 创建 QStackedWidget 用于页面切换
    QStackedWidget* stack_widget = new QStackedWidget(this);
    stack_widget->addWidget(cameracontrol);      // 把 camerawidget 添加到堆栈
    //stack_widget->addWidget(projectcontrol);     // 把 projectcontrol 添加到堆栈
    stack_widget->addWidget(calibration);        // 将 CalibWidget 添加到堆栈
    stack_widget->addWidget(phaseWidget);        // 将 PhaseWidget 添加到堆栈
    stack_widget->addWidget(registration);       // 把Registration 添加到堆栈

     // 设置默认页面
    stack_widget->setCurrentIndex(0);

    // 布局管理
    QHBoxLayout* h_layout = new QHBoxLayout();
    h_layout->setMargin(0);
    h_layout->addWidget(pbtn1);
    //h_layout->addWidget(pbtn2);
    h_layout->addWidget(pbtn3);
    h_layout->addWidget(pbtn4);
    h_layout->addWidget(pbtn5);

    QVBoxLayout* v_layout = new QVBoxLayout();
    v_layout->addLayout(h_layout);
    v_layout->addWidget(stack_widget);

    ui.centralWidget->setLayout(v_layout); // 使用对象访问成员

    // 连接按钮点击事件与页面切换
    connect(pbtn1, &QPushButton::clicked, this, [=]() {
        stack_widget->setCurrentIndex(0);
        qDebug() << "Switched to page 1: Camera Control";
        });

    /*connect(pbtn2, &QPushButton::clicked, this, [=]() {
        stack_widget->setCurrentIndex(1);
        qDebug() << "Switched to page 2: Project Control";
        });*/

    connect(pbtn3, &QPushButton::clicked, this, [=]() {
        stack_widget->setCurrentIndex(1);
        qDebug() << "Switched to page 3: Calibration";
        });

    connect(pbtn4, &QPushButton::clicked, this, [=]() {
        stack_widget->setCurrentIndex(2);
        qDebug() << "Switched to page 4: Phase";
        });

    connect(pbtn5, &QPushButton::clicked, this, [=]() {
        stack_widget->setCurrentIndex(3);
        qDebug() << "Switched to page 5: PointCloudProcess";
        });

    /*connect(pbtn6, &QPushButton::clicked, this, [=]() {
        stack_widget->setCurrentIndex(3);
        qDebug() << "Switched to page 6: Point Cloud Processing";
        });*/

}

Mycode::~Mycode()
{

}
