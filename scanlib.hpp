#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

/**
 *  Scan 
 */
int Scan(const char* fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int ScanPreview(const char* fileName);
int findLaserLine(IplImage* frame, IplImage* laserline, unsigned int laserThreshold);
