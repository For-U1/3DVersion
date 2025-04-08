//Point Cloud Library (PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>  // 添加此行
//#include <memory>

// 将 PointT 替换为 ViewerPointT，避免与其他文件冲突
using ViewerPointT = pcl::PointXYZRGBA;
using ViewerPointCloudT = pcl::PointCloud<ViewerPointT>;

class PCLViewer : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    explicit PCLViewer(int win_size, QWidget* parent = nullptr);

    ~PCLViewer();

    bool loadPointCloudFromFile();  // 从文件加载点云数据

    bool loadTXTFile(const std::string& filename, pcl::PointCloud<ViewerPointT>::Ptr cloud); //增加对TXT文档的支持

    void setFilePath(const std::string& path);  // 设置点云文件路径

    void clearPointClouds();//清空显示窗口当前的点云文件

    void addPointCloud(pcl::PointCloud<ViewerPointT>::Ptr cloud, const std::string& id); //添加点云文件到当前的显示窗口

    void setPointCloud(pcl::PointCloud<ViewerPointT>::Ptr cloud);//设置点云数据

    void addOrientationMarkerWidgetAxesToview(vtkRenderWindowInteractor* interactor, double x, double y, double x_wide, double y_wide);

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;  // PCL 可视化对象

    ViewerPointCloudT::Ptr cloud;  // 点云对象

    std::string filePath;  // 点云文件路径

    vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_member_;  // 坐标轴部件
};
