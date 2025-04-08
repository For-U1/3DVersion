#pragma once
#ifndef ADVANCEDSETTINGSDIALOG_H
#define ADVANCEDSETTINGSDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QTabWidget>
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QString>
#include <QHBoxLayout>
#include <QLabel>

class AdvancedSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AdvancedSettingsDialog(QWidget* parent = nullptr);
    ~AdvancedSettingsDialog();

    // 获取 ICP 页面参数
    QString getIcpDataPath() const;
    // 获取 RANSAC 页面参数
    QString getCalibPath() const;
    QString getImagePath() const;
    QString getCloudPointdataPath() const;

private:
    QTabWidget* tabWidget;

    // ICP 配置页面控件
    QWidget* icpPage;
    QLineEdit* IcpDataPathLineEdit; // ICP 数据路径

    // RANSAC 配置页面控件
    QWidget* ransacPage;
    QLineEdit* CalibPathLineEdit;             // 校准文件路径
    QLineEdit* ImagePathLineEdit;             // 物体标志点图像路径
    QLineEdit* CloudPointdataPathLineEdit;    // 视差图文件夹路径

    QPushButton* okButton;
    QPushButton* cancelButton;
};

#endif // ADVANCEDSETTINGSDIALOG_H
