#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QFileDialog>
#include "PCLViewer.h"

class PointCloudWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);

private slots:
    void onLoadPointCloud();  // 加载点云文件的槽函数

private:
    PCLViewer *viewer;  // PCLViewer 对象用于显示点云
    QPushButton *loadButton;  // 按钮用于加载点云
};

#endif // POINTCLOUDWIDGET_H


