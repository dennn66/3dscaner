#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <inifile++.hpp>
#include <iostream>
#include <fstream>
#include <cstring>

#include "scanlib.hpp"

#define PI 3.14159265
#define STEPS 400
#define MIDLE_PIX 240
#define M_PER_PIX_X 0.000723
#define M_PER_PIX_Y 0.000523
#define LASER_THRESHOLD 150

int laserThreshold = LASER_THRESHOLD;
double zoom_x = M_PER_PIX_X;
double zoom_y = M_PER_PIX_Y;
int center_point_x = MIDLE_PIX;
int center_point_y = 480;
int left_point_x = 0;

// точки
CvPoint2D32f srcQuad[4], dstQuad[4];
// матрица преобразования
CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);


/**
 *  Scan 
 */
int Scan(const char* fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	char output_filename[512];
	CvCapture* capture;
	IplImage* frame=0;
	IplImage* laserline=0;


	sprintf(output_filename, "%sLaser0.jpg", fileName);
	printf("[i] load... %s\n",  output_filename);
	frame = cvLoadImage(output_filename,1);


        laserline = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	try {
		inifilepp::parser p("scaner.ini");
		const inifilepp::parser::entry *ent;

		while((ent = p.next())) {
			int v;
			if(strcmp(ent->sect, "srcQuad")==0){ 
				v = atoi(ent->val);
				if(strcmp(ent->key, "point0.x")==0) srcQuad[0].x = v;
				if(strcmp(ent->key, "point0.y")==0) srcQuad[0].y = v;
				if(strcmp(ent->key, "point1.x")==0) srcQuad[1].x = v;
				if(strcmp(ent->key, "point1.y")==0) srcQuad[1].y = v;
				if(strcmp(ent->key, "point2.x")==0) srcQuad[2].x = v;
				if(strcmp(ent->key, "point2.y")==0) srcQuad[2].y = v;
				if(strcmp(ent->key, "point3.x")==0) srcQuad[3].x = v;
				if(strcmp(ent->key, "point3.y")==0) srcQuad[3].y = v;
			} else
			if(strcmp(ent->sect, "dstQuad")==0){ 
				v = atoi(ent->val);
				if(strcmp(ent->key, "point0.x")==0) dstQuad[0].x = v;
				if(strcmp(ent->key, "point0.y")==0) dstQuad[0].y = v;
				if(strcmp(ent->key, "point1.x")==0) dstQuad[1].x = v;
				if(strcmp(ent->key, "point1.y")==0) dstQuad[1].y = v;
				if(strcmp(ent->key, "point2.x")==0) dstQuad[2].x = v;
				if(strcmp(ent->key, "point2.y")==0) dstQuad[2].y = v;
				if(strcmp(ent->key, "point3.x")==0) dstQuad[3].x = v;
				if(strcmp(ent->key, "point3.y")==0) dstQuad[3].y = v;
			} else
			if(strcmp(ent->sect, "Zoom")==0){
				std::cout << ent->val << " " << atof(ent->val) << "\n";
				if(strcmp(ent->key, "zoom.x")==0) zoom_x = atof(ent->val);
				if(strcmp(ent->key, "zoom.y")==0) zoom_y = atof(ent->val);
			} else
			if(strcmp(ent->sect, "Center")==0){
				v = atoi(ent->val);
				if(strcmp(ent->key, "center.point.x")==0) center_point_x = v;
				if(strcmp(ent->key, "center.point.y")==0) center_point_y = v;
			}
		}
	} catch(inifilepp::parser::exception &e) {
		std::cerr << "parse error at (" << e.line << "," << e.cpos << ")\n";
		srcQuad[0].x = 172;           //src Top left
		srcQuad[0].y = 95;
		srcQuad[1].x = 279;  //src Top right
		srcQuad[1].y = 95;
		srcQuad[2].x = 172;           //src Bottom left
		srcQuad[2].y = 118;
		srcQuad[3].x = 279;  //src Bot right
		srcQuad[3].y = 118;

		dstQuad[0].x = 172;  //dst Top left
		dstQuad[0].y = 95;
		dstQuad[1].x = 279;  //dst Top right
		dstQuad[1].y = 118;
		dstQuad[2].x = 172;  //dst Bottom left
		dstQuad[2].y = 118;      
		dstQuad[3].x = 279;  //dst Bot right
		dstQuad[3].y = 141;
	}
	// получаем матрицу преобразования
	cvGetPerspectiveTransform(srcQuad,dstQuad,warp_matrix);

	laserThreshold = ScanPreview(fileName);
	if(laserThreshold == 0){
		// освобождаем ресурсы
		cvReleaseImage(&frame);
		cvReleaseImage(&laserline);
		return(-1);
	}

        cvNamedWindow("capture",CV_WINDOW_AUTOSIZE);
        cvNamedWindow("laser line",CV_WINDOW_AUTOSIZE);
	cvMoveWindow("capture", 0, 0);
	cvMoveWindow("laser line", laserline->width+10, 0);



	for(int step=0; step < STEPS; step++){
		// получаем кадр
			sprintf(output_filename, "%sLaser%d.jpg", fileName, step);
			printf("[i] load... %s\n", output_filename);
			frame = cvLoadImage(output_filename,1);

		// показываем
		cvShowImage("capture", frame);
		findLaserLine(frame, laserline, laserThreshold);
		// пробегаемся по пикселям - ищем центральные точки пятен для каждой строки
	        int x=0, y=0;
		for(y=0; y<laserline->height; y++){
			uchar* ptr_laserline = (uchar*) (laserline->imageData + y * laserline->widthStep);

			for(x=0;x<laserline->width; x++){
				if(ptr_laserline[x]>0){
					double angle = (double)step*2.0*PI/(double)STEPS;
					double radius = (double)(x-center_point_x)*zoom_x;
					pcl::PointXYZ basic_point;
					basic_point.x = radius*sin(angle);
					basic_point.y = radius*cos(angle);
					basic_point.z = (double)y*zoom_y;
/*
std::cout << "zoom_x: " << zoom_x << std::endl;
std::cout << "zoom_y: " << zoom_y << std::endl;

std::cout << "pixels: " << center_point_x << " " << x << " "<< y << std::endl;
std::cout << "radius: " << radius << std::endl;
std::cout << "angle: " << angle << std::endl;
std::cout << "basic_point.x " << basic_point.x << std::endl;
std::cout << "basic_point.y " << basic_point.y << std::endl;
std::cout << "basic_point.z " << basic_point.z << std::endl;
cvWaitKey(3000);
*/
					cloud->points.push_back(basic_point);
				}
			}
		}
		cloud->width = (int) cloud->points.size ();
		cloud->height = 1;

		cvShowImage("laser line",laserline);
		char c = cvWaitKey(33);
		if (c == 27) { // нажата ESC
			// освобождаем ресурсы
			cvReleaseImage(&frame);
			cvReleaseImage(&laserline);
			return(-1);
		}
	}

        // освобождаем ресурсы
	cvReleaseImage(&frame);
        cvReleaseImage(&laserline);
        cvDestroyAllWindows();
	return(0);
}

void myTrackbarThreshold(int pos) {
     //   laserThreshold = pos;
}

int ScanPreview(const char* fileName)
{

	char input_filename[512];
	IplImage* frame=0;
	IplImage* laserline=0;


        // получаем кадр
	sprintf(input_filename, "%sLaser0.jpg", fileName);
	frame = cvLoadImage(input_filename,1);
        laserline = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	cvNamedWindow("capture", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("laser line",CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar("Brightness threshold", "laser line", &laserThreshold, 255, myTrackbarThreshold);
	cvCreateTrackbar("Left x threshold", "laser line", &left_point_x, frame->width, myTrackbarThreshold);
	cvCreateTrackbar("Bottom y threshold", "laser line", &center_point_y, frame->height, myTrackbarThreshold);
	cvMoveWindow("capture", 0, 0);
	cvMoveWindow("laser line", laserline->width+10, 0);
        cvShowImage("capture", frame);     

        while(1){
		findLaserLine(frame, laserline, laserThreshold);
                // показываем
 		cvShowImage("laser line",laserline);
		char c = cvWaitKey(33);
                if (c == 27) { // нажата ESC
			laserThreshold = 0;
                        break;
                }
                else if(c == 13) { // Enter
                        break;
                }
        }
        // освобождаем ресурсы
        cvReleaseImage(&frame);
        cvReleaseImage(&laserline);
        cvDestroyAllWindows();
   return(laserThreshold);

}


/**
 *  ScanStep 
 */
int findLaserLine(IplImage* frame, IplImage* laserline, unsigned int laserThreshold)
{


        // готовим матрицу для фильтра

        float kernel[10];
        kernel[0]=0.1;
        kernel[1]=0.1;
        kernel[2]=0.1;

        kernel[3]=0.1;
        kernel[4]=0.1;
        kernel[5]=0.1;

        kernel[6]=0.1;
        kernel[7]=0.1;
        kernel[8]=0.1;
        kernel[9]=0.1;
        // матрица
        CvMat kernel_matrix=cvMat(1,10,CV_32FC1,kernel);


//	char output_filename[512];
	double width = 640;
	double height = 480;



	width = frame->width;
	height = frame->height;
	// для хранения каналов HSV
	IplImage* hsv = 0;
	IplImage* h_plane = 0;
	IplImage* s_plane = 0;
	IplImage* v_plane = 0;

	// для хранения фильтрованной картинки
	IplImage* v_filtred = 0;
	IplImage* v_laserline = 0;

        // создаём картинки
        hsv = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 3 );
        h_plane = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
        s_plane = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
        v_plane = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
        v_filtred = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
        v_laserline = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );

        //  конвертируем в HSV 
        cvSaveImage("Frame0.jpg", frame,0);

        cvCvtColor( frame, hsv, CV_BGR2HSV ); 
//        cvColor( frame, hsv, CV_BGR2HSV ); 
        cvSaveImage("Hsv0.jpg", hsv,0);

        // разбиваем на отельные каналы
        cvSplit( hsv, h_plane, s_plane, v_plane, 0 );
        cvSaveImage("v_plane0.jpg", v_plane,0);
        // накладываем фильтр
        cvFilter2D(v_plane, v_filtred, &kernel_matrix, cvPoint(-1,-1));

        int x=0, y=0;
	cvZero(laserline);

        int spot_state=0, spot_begin=laserline->widthStep;

        // пробегаемся по пикселям - ищем максимум яркости в каждой строке 
        for(y=0; y<center_point_y; y++){
		int max=0;
                uchar* ptr_v_filtred = (uchar*) (v_filtred->imageData + y * v_filtred->widthStep);
		uchar* ptr_laserline = (uchar*) (v_laserline->imageData + y * v_laserline->widthStep);

		for(x=left_point_x; x<v_filtred->width; x++){
			if(ptr_v_filtred[x] > max) max = ptr_v_filtred[x];
		}
		if(max > laserThreshold){
			spot_begin=laserline->widthStep;
		        for(x=laserline->width-1; x>=left_point_x; x--){
				if(ptr_v_filtred[x]==max){
					if(spot_state==0){
						 spot_state=1;
						 spot_begin=x;
					}
				} else {
					if(spot_state==1){ 
						spot_state=0;
						if(spot_begin>=x) ptr_laserline[(x+spot_begin)/2]=255;
					}	
				}
			}
		}
	}
	// преобразование перспективы
	cvWarpPerspective(v_laserline,laserline,warp_matrix, CV_WARP_INVERSE_MAP);

	cvSaveImage("laserline0.jpg", laserline,0);

        // освобождаем ресурсы

        cvReleaseImage(&hsv);
        cvReleaseImage(&h_plane);
        cvReleaseImage(&s_plane);
        cvReleaseImage(&v_plane);
        cvReleaseImage(&v_filtred);
        cvReleaseImage(&v_laserline);

   return(0);

}

