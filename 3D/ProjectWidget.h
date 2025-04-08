//// ProjectWidget.h
//#pragma once 
//#include <QWidget>
//#include <QPushButton>
//#include <QLineEdit>
//#include <QTextEdit>
//#include <QLabel>
//#include <QVBoxLayout>
//#include <QHBoxLayout>
//#include <memory>
//#include <thread>
//#include <atomic>
//
//#include "ProjectionControlThread.h"
//
//class ProjectWidget : public QWidget
//{
//    Q_OBJECT
//
//public:
//    explicit ProjectWidget(QWidget* parent = nullptr);
//    ~ProjectWidget();
//
//private slots:
//    void browseLeftPath();
//    void browseRightPath();
//    void startProjection();
//    void stopProjection();
//    void appendLog(const QString& message);
//
//private:
//    // UI Components
//    QLabel* leftPathLabel;
//    QLineEdit* leftPathLineEdit;
//    QPushButton* browseLeftPathButton;
//
//    QLabel* rightPathLabel;
//    QLineEdit* rightPathLineEdit;
//    QPushButton* browseRightPathButton;
//
//    QPushButton* startProjectionButton;
//    QPushButton* stopProjectionButton;
//
//    QTextEdit* logTextEdit;
//
//    // Control Thread
//    std::unique_ptr<ProjectionControlThread> projectionThread;
//    std::thread controlThread;
//    std::atomic<bool> threadRunning;
//};
//
//
