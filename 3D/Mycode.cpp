#include "Mycode.h"
#include "ui_Mycode.h"

#include "RegistrationWidget.h"//������ʾ��ƴ��ģ��

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

    //������ͬ��ģ��ҳ��
    Camerawidget* cameracontrol = new Camerawidget(1,this);  // �������ģ��
    //ProjectWidget* projectcontrol = new ProjectWidget(this);  // ͶӰ�ǿ���ģ��
    CalibWidget* calibration = new CalibWidget(this);  // ����궨ģ��
    PhaseWidget* phaseWidget = new PhaseWidget(this);  // ��λ����ģ��
    RegistrationWidget* registration = new RegistrationWidget( this);  // ������ʾ�͵���ƴ��ģ��
    

    //������ť
    QPushButton* pbtn1 = new QPushButton("Camera Control",this);
    //QPushButton* pbtn2 = new QPushButton("Project Control", this);
    QPushButton* pbtn3 = new QPushButton("Calibration", this);
    QPushButton* pbtn4 = new QPushButton("Phase", this);
    QPushButton* pbtn5 = new QPushButton("PointCloudProcess", this);

    // ���� QStackedWidget ����ҳ���л�
    QStackedWidget* stack_widget = new QStackedWidget(this);
    stack_widget->addWidget(cameracontrol);      // �� camerawidget ��ӵ���ջ
    //stack_widget->addWidget(projectcontrol);     // �� projectcontrol ��ӵ���ջ
    stack_widget->addWidget(calibration);        // �� CalibWidget ��ӵ���ջ
    stack_widget->addWidget(phaseWidget);        // �� PhaseWidget ��ӵ���ջ
    stack_widget->addWidget(registration);       // ��Registration ��ӵ���ջ

     // ����Ĭ��ҳ��
    stack_widget->setCurrentIndex(0);

    // ���ֹ���
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

    ui.centralWidget->setLayout(v_layout); // ʹ�ö�����ʳ�Ա

    // ���Ӱ�ť����¼���ҳ���л�
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
