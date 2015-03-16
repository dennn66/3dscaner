// STL
#include <iostream>
#include <limits>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

bool keypressed = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "v" && event.keyDown ())
  {
    std::cout << "v was pressed" << std::endl;
    keypressed = true;
  } else {
    keypressed = false;
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
//  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}

int main (int argc, char** argv)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

if(argc >1 ) pcl::io::loadPCDFile(argv[1],*mls_cloud);
else pcl::io::loadPCDFile("scaned_cloud.pcd",*mls_cloud);

std::cout << "size: " << mls_cloud->points.size () << std::endl;

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (mls_cloud);
  n.setInputCloud (mls_cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

//       pcl::visualization::PCLVisualizer::Ptr viewer ("3D Viewer");
viewer = interactionCustomizationVis ();
        int v1 (0);
        viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText ("Filtered Cloud", 10, 10, "v1 text", v1);
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "original cloud", v1);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1 (mls_cloud, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ> (mls_cloud, single_color1, "filtered cloud", v1);
        viewer->addCoordinateSystem (0.1, "coord", v1);

        int v2 (1);
        viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText ("Point Cloud with normals", 10, 10, "v2 text", v2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (mls_cloud, 255 , 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (mls_cloud, "cloud with normals", v2);
        viewer->addCoordinateSystem (0.1, "coord", v2);

  //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (mls_cloud, normals, 10, 0.05, "normals", v1);


        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud with normals");
        viewer->resetCamera ();

        while (!keypressed) {
                viewer->spinOnce ();
        }
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*mls_cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  //pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.03);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100); //100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (*triangles);

  // Additional vertex information
//  std::vector<int> parts = gp3.getPartIDs();
//  std::vector<int> states = gp3.getPointStates();
  
  pcl::io::saveVTKFile ("mesh.vtk", *triangles);
viewer->addPolygonMesh(*triangles,"meshes",0);
keypressed = false;
        while (!keypressed) {
                viewer->spinOnce ();
        }

  return(0);

}
