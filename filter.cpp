#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>


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


       pcl::visualization::PCLVisualizer viewer ("3D Viewer");

        int v1 (0);
        viewer.createViewPort (0.0, 0.0, 0.5, 0.5, v1);
        viewer.setBackgroundColor (0, 0, 0, v1);
        viewer.addText ("origin cloud", 10, 10, "v1 text", v1);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "original cloud", v1);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1 (cloud, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ> (cloud, single_color1, "origin cloud", v1);
        viewer.addCoordinateSystem (0.1, "coord", v1);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "origin cloud");
      viewer.resetCamera ();

         viewer.spinOnce ();

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (2.5);
  pcl::PCDWriter writer;

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("outliers.pcd", *cloud_filtered, false);

  sor.setNegative (false);
  sor.filter (*cloud_filtered);
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  writer.write<pcl::PointXYZ> ("inliers.pcd", *cloud_filtered, false);


        int v2 (1);
        viewer.createViewPort (0.5, 0.0, 1.0, 0.5, v2);
        viewer.setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer.addText ("inliers cloud", 10, 10, "v2 text", v2);
       pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (cloud_filtered, 255 , 0, 0);
        viewer.addPointCloud<pcl::PointXYZ> (cloud_filtered, single_color2, "inliers cloud", v2);
        viewer.addCoordinateSystem (0.1, "coord", v2);

        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "inliers cloud");
        viewer.spinOnce ();

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
  mls.setSearchRadius (0.01);

  // Reconstruct
  std::cerr << "Start MLS pocessing... " << std::endl;
  mls.process (mls_points);
  std::cerr << "Done." << std::endl;

pcl::PointCloud<pcl::PointXYZ>::Ptr mlscloud (new pcl::PointCloud<pcl::PointXYZ>);
copyPointCloud(mls_points, *mlscloud);

        int v3 (2);
        viewer.createViewPort (0.0, 0.5, 0.5, 1.0, v3);
        viewer.setBackgroundColor (0, 0, 0, v3);
        viewer.addText ("MLS cloud", 10, 10, "v3 text", v3);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "original cloud", v1);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3 (mlscloud, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ> (mlscloud, single_color3, "MLS cloud", v3);
        viewer.addCoordinateSystem (0.1, "coord", v3);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "MLS cloud");
        viewer.spinOnce ();


std::cout << "size: " << mlscloud->points.size () << std::endl;
std::vector<int> indices2;
pcl::removeNaNFromPointCloud(*mlscloud, *mls_cloud, indices2);
std::cout << "size: " << mls_cloud->points.size () << std::endl;

        int v4 (3);
        viewer.createViewPort (0.5, 0.5, 1.0, 1.0, v4);
        viewer.setBackgroundColor (0.3, 0.3, 0.3, v4);
        viewer.addText ("MLS cloud wo nan", 10, 10, "v4 text", v4);
       pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color4 (mls_cloud, 255 , 0, 0);
        viewer.addPointCloud<pcl::PointXYZ> (mls_cloud, single_color4, "MLS cloud wo nan", v4);
        viewer.addCoordinateSystem (0.1, "coord", v4);

        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "MLS cloud wo nan");

        while (!viewer.wasStopped ()) {
                viewer.spinOnce ();
        }

  return (0);
}


/**
 *  Main function.
 */
int main(int argc, char *argv[])
{
  int debug_mode = 0;
  int result;
   
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);


if(argc >1 ) pcl::io::loadPCDFile(argv[1],*cloud);
else pcl::io::loadPCDFile("scaned_cloud.pcd",*cloud);

  FilterScanedCloud(cloud, mls_cloud);

  pcl::io::savePCDFileASCII ("filtered_cloud.pcd", *mls_cloud);
  std::cerr << "Saved " << mls_cloud->points.size () << " data points to filtered_cloud.pcd" << std::endl;
   return EXIT_SUCCESS;
}

