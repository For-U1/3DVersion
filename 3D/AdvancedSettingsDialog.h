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

    // ��ȡ ICP ҳ�����
    QString getIcpDataPath() const;
    // ��ȡ RANSAC ҳ�����
    QString getCalibPath() const;
    QString getImagePath() const;
    QString getCloudPointdataPath() const;

private:
    QTabWidget* tabWidget;

    // ICP ����ҳ��ؼ�
    QWidget* icpPage;
    QLineEdit* IcpDataPathLineEdit; // ICP ����·��

    // RANSAC ����ҳ��ؼ�
    QWidget* ransacPage;
    QLineEdit* CalibPathLineEdit;             // У׼�ļ�·��
    QLineEdit* ImagePathLineEdit;             // �����־��ͼ��·��
    QLineEdit* CloudPointdataPathLineEdit;    // �Ӳ�ͼ�ļ���·��

    QPushButton* okButton;
    QPushButton* cancelButton;
};

#endif // ADVANCEDSETTINGSDIALOG_H
