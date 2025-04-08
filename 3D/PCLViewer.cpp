// #include "PCLViewer.h"   //原始生成随机点云文件
// #include <qpainter.h>
// #include <qdebug.h>
// #include <pcl/common/io.h>
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/obj_io.h>
// #include <pcl/PolygonMesh.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/vtk_lib_io.h>          //loadPolygonFileOBJ所属头文件；
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/common/common_headers.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>
// #include <pcl/io/ply_io.h>
// #include "vtkGenericOpenGLRenderWindow.h"


// typedef pcl::PointXYZRGBA PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;

// PCLViewer::PCLViewer(int win_size, QWidget *parent) : QVTKOpenGLNativeWidget(parent)
// {
// #if VTK_MAJOR_VERSION > 8
//     auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
//     auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
//     renderWindow2->AddRenderer(renderer2);
//     viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
//     this->setRenderWindow(viewer->getRenderWindow());
//     viewer->setupInteractor(this->interactor(), this->renderWindow());
// #else
//     viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
//     this->SetRenderWindow(viewer->getRenderWindow());
//     viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
// #endif

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//     // Fill in the cloud data
//     cloud->width  = 200;
//     cloud->height = 1;
//     cloud->points.resize (cloud->width * cloud->height);

//     for (std::size_t i = 0; i < cloud->size (); ++i)
//     {
//         (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//         (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//         (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//     }

//     viewer->addPointCloud (cloud, "cloud");

//     // 如果point 有变动 使用 _viewer->updatePointCloud(_cloud, _poindCloudID);

//     // 显示结果图
//     viewer->setBackgroundColor (0, 0, 0); //设置背景

//     /*添加坐标轴到视图的左下角*/
//     axes_widget_member_ = NULL;
//     addOrientationMarkerWidgetAxesToview(viewer->getRenderWindow()->GetInteractor(), 0.0, 0.0, 0.20, 0.2);

//     viewer->resetCamera ();
//     update ();
// }

// PCLViewer::~PCLViewer()
// {
// }

// void PCLViewer::addOrientationMarkerWidgetAxesToview(vtkRenderWindowInteractor* interactor, double x, double y, double x_wide, double y_wide)
// {
//     if (axes_widget_member_ == NULL)
//     {
//         vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

//         axes_widget_member_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
//         axes->SetPosition(0, 0, 0);
//         axes->SetTotalLength(2, 2, 2);
//         axes->SetShaftType(0);
//         axes->GetZAxisShaftProperty()->SetColor(0,0,200);

//         axes->SetCylinderRadius(0.02);
//         axes_widget_member_->SetOrientationMarker(axes);
//         axes_widget_member_->SetInteractor(interactor);
//         axes_widget_member_->SetViewport(x,y, x_wide, y_wide);
//         axes_widget_member_->SetEnabled(true);
//         axes_widget_member_->InteractiveOn();
//         // axes_widget_member_->InteractiveOff();
//     }
//     else
//     {
//         axes_widget_member_->SetEnabled(true);
//         pcl::console::print_warn(stderr, "Orientation Widget Axes already exists, just enabling it");
//     }
// }

#include "PCLViewer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <QDebug>
PCLViewer::PCLViewer(int win_size, QWidget *parent)
    : QVTKOpenGLNativeWidget(parent), filePath("")
{
    cloud.reset(new PointCloudT);  // 初始化点云对象

#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    this->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->interactor(), this->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    this->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
#endif

    viewer->setBackgroundColor(0, 0, 0);  // 设置背景色为黑色
    //viewer->setBackgroundColor(.5, .5, .5);//设置背景色为白色

    //loadPointCloudFromFile();  // 加载点云数据，如果在PCLViewer类的构造函数中初始化这个函数，会在编译运行时输出调试信息

    // 添加坐标轴
    addOrientationMarkerWidgetAxesToview(this->renderWindow()->GetInteractor(), 0.0, 0.0, 0.2, 0.2);
}

PCLViewer::~PCLViewer()//析构函数
{
}

// 设置点云文件路径
void PCLViewer::setFilePath(const std::string &path)
{
    filePath = path;
}

// 从文件加载点云
bool PCLViewer::loadPointCloudFromFile()
{
    if (filePath.empty())
    {
        qDebug() << "File path is empty!";
        return false;
    }

    std::string extension = filePath.substr(filePath.find_last_of(".") + 1);

    cloud.reset(new PointCloudT);  // 初始化点云

    if (extension == "pcd")
    {
        if (pcl::io::loadPCDFile<PointT>(filePath, *cloud) == -1)
        {
            qDebug() << "Failed to load PCD file:" << QString::fromStdString(filePath);
            return false;
        }
    }
    else if (extension == "ply")
    {
        if (pcl::io::loadPLYFile<PointT>(filePath, *cloud) == -1)
        {
            qDebug() << "Failed to load PLY file:" << QString::fromStdString(filePath);
            return false;
        }
    }
    else
    {
        qDebug() << "Unsupported file format:" << QString::fromStdString(filePath);
        return false;
    }

    bool hasColor = false;
    for (const auto& point : cloud->points)
    {
        if (point.r != 0 || point.g != 0 || point.b != 0)
        {
            hasColor = true;
            break;
        }
    }

    viewer->removeAllPointClouds();  // 移除已有的点云

    if (hasColor)
    {
        viewer->addPointCloud<PointT>(cloud, "colored_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud");
        qDebug() << "Loaded colored point cloud";
    }
    else
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> whiteColor(cloud, 255, 255, 255);
        viewer->addPointCloud<PointT>(cloud, whiteColor, "white_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "white_cloud");
        qDebug() << "Loaded point cloud without color (displaying in white)";
    }

    viewer->resetCamera();  // 重置摄像头视角
    return true;
}

// 添加坐标轴标记到视图
void PCLViewer::addOrientationMarkerWidgetAxesToview(vtkRenderWindowInteractor* interactor, double x, double y, double x_wide, double y_wide)
{
    if (!axes_widget_member_)
    {
        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        axes_widget_member_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        axes_widget_member_->SetOrientationMarker(axes);
        axes_widget_member_->SetInteractor(interactor);
        axes_widget_member_->SetViewport(x, y, x_wide, y_wide);
        axes_widget_member_->SetEnabled(true);
        axes_widget_member_->InteractiveOn();
    }
}



