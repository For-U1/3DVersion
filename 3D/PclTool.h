#ifndef PCL_TOOL_H
#define PCL_TOOL_H
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/poisson.hpp>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>        // 用于PCL与VTK之间的转换                    
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkFillHolesFilter.h>                         // VTK的补洞滤波器
#include <vtkPolyData.h>                                // VTK的PolyData
class PclTool
{
public:
    pcl::PolygonMesh projectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PolygonMesh poissonReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PolygonMesh fillHoles(pcl::PolygonMesh& triangles);
};
#endif // PCL_TOOL_H

