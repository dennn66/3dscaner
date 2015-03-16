#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

int FilterScanedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud)

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  //pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  //reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
  //reader.read<pcl::PointXYZ> ("out.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1);
  pcl::PCDWriter writer;

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("outliers.pcd", *cloud_filtered, false);

  sor.setNegative (false);
  sor.filter (*cloud_filtered);
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  writer.write<pcl::PointXYZ> ("inliers.pcd", *cloud_filtered, false);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud_filtered);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (mls_tree);
  mls.setSearchRadius (0.002);

  // Reconstruct
  mls.process (mls_points);

//pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);
copyPointCloud(mls_points, *mls_cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr mlscloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::savePCDFileASCII ("test_cloud.pcd", *mls_cloud);

pcl::io::loadPCDFile("test_cloud.pcd",*mlscloud);

std::cout << "size: " << mlscloud->points.size () << std::endl;
std::vector<int> indices2;
pcl::removeNaNFromPointCloud(*mlscloud, *mls_cloud, indices2);
std::cout << "size: " << mls_cloud->points.size () << std::endl;
pcl::io::savePCDFileASCII ("test_cloud_after.pcd", *mls_cloud);



       pcl::visualization::PCLVisualizer viewer ("3D Viewer");

        int v1 (0);
        viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
        viewer.setBackgroundColor (0, 0, 0, v1);
        viewer.addText ("First Point Cloud", 10, 10, "v1 text", v1);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "original cloud", v1);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1 (cloud_filtered, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ> (cloud_filtered, single_color1, "filtered cloud", v1);
        viewer.addCoordinateSystem (0.1, "coord", v1);

        int v2 (1);
        viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer.addText ("Second Point Cloud", 10, 10, "v2 text", v2);
       pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (cloud, 255 , 0, 0);
        viewer.addPointCloud<pcl::PointXYZ> (cloud, single_color2, "MLS cloud", v2);
        viewer.addCoordinateSystem (0.1, "coord", v2);

        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "MLS cloud");
        viewer.resetCamera ();

        while (!viewer.wasStopped ()) {
                viewer.spinOnce ();
        }

  return (0);
}

int RenderScanedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud,  pcl::PolygonMesh::Ptr triangles)
{


       pcl::visualization::PCLVisualizer viewer ("3D Viewer");

        int v1 (0);
        viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
        viewer.setBackgroundColor (0, 0, 0, v1);
        viewer.addText ("Filtered Cloud", 10, 10, "v1 text", v1);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "original cloud", v1);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1 (mls_cloud, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ> (mls_cloud, single_color1, "filtered cloud", v1);
        viewer.addCoordinateSystem (0.1, "coord", v1);
/*
        int v2 (1);
        viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer.addText ("Point Cloud with normals", 10, 10, "v2 text", v2);
      // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (cloud_with_normals, 255 , 0, 0);
        viewer.addPointCloud<pcl::PointXYZ> (cloud_with_normals, "cloud with normals", v2);
        viewer.addCoordinateSystem (0.1, "coord", v2);
*/
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered cloud");
//        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud with normals");
        viewer.resetCamera ();

        while (!viewer.wasStopped ()) {
                viewer.spinOnce ();
        }
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (mls_cloud);
  n.setInputCloud (mls_cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
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
  gp3.setSearchRadius (0.003);

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


  return(0);
}
