#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "scanlib.hpp"

/**
 *  Main function.
 */
int main(int argc, char *argv[])
{

  int result;
   
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (argc == 2) {
  	result = Scan(argv[1], cloud);
  } else {
  	result = Scan("./images/new/", cloud);
  }
  if(result !=0) return EXIT_SUCCESS;

  pcl::io::savePCDFileASCII ("scaned_cloud.pcd", *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to scaned_cloud.pcd" << std::endl;
   return EXIT_SUCCESS;
}

