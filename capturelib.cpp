

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <cstring>

#include "arduino_driver.h"
#include "capturelib.hpp"

#define STEPS 400

int fd;

int initHardware(const char* fileName, unsigned int baud){
	const char fileMode[] = "r+";
	printf("Init Hardware %s:%d\n", fileName, baud);
	fd = openDeviceFile(fileName, fileMode);
	setAttr(fd, baud);
	displayResult(fd);
	printf("Baud: ");
	sendCommand(fd, "b\n");
	displayResult(fd);
	return(0);
}

int laserOn(){
	printf("Turning on step laser... ");
	sendCommand(fd, "w 7 1\n");
	displayResult(fd);
}
int laserOff(){
	printf("Turning off laser... ");
	sendCommand(fd, "w 7 0\n");
	displayResult(fd);
}
int hardwareOff(){
	printf("Turning off hardware... ");
	laserOff();
}
int stepForward(){
	printf("Step forward... ");
	sendCommand(fd, "f 1\n");
}
int waitHardware(){
	displayResult(fd);
}
//using namespace cv;

/**
 *  Scan 
 */
int Scan(const char* fileName, unsigned int baud, const char* outputFolder)
{
	char output_filename[512];
	CvCapture* capture;
	IplImage* frame=0;

	initHardware(fileName, baud);
	// получаем любую подключённую камеру
	capture = cvCreateCameraCapture(CV_CAP_ANY); //cvCaptureFromCAM( 0 );
	assert( capture );
	printf("[i] Found camera %.0f x %.0f\n", 
		cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH), 
		cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT) );
        frame = cvQueryFrame( capture );

        cvNamedWindow("capture",CV_WINDOW_AUTOSIZE);
	cvMoveWindow("capture", 0, 0);
	for(int lasermode=1; lasermode >=0; lasermode--){
		if(lasermode==1) laserOn(); else laserOff();
		bool preview =true;
		while(preview){
			// получаем кадр
			frame = cvQueryFrame( capture );
			// показываем
			cvShowImage("capture", frame);
			char c = cvWaitKey(33);
			if (c == 27) { //  ESC
				// 
				hardwareOff(); 
				cvReleaseCapture( &capture );
				return(-1);
			}
			if (c == 13) { //  Enter
				// 
				 std::cerr << "Capture..." << std::endl;
				preview = false;
				break;
			}
		}
		for(int step=0; step < STEPS; step++){
			// получаем кадр
			frame = cvQueryFrame( capture );
			sprintf(output_filename, "%s%s%d.jpg", outputFolder, lasermode==1?"Laser":"Color", step);
			printf("[i] capture... %s\n", output_filename);
			cvSaveImage(output_filename, frame,0);

			// показываем
			cvShowImage("capture", frame);
			stepForward();
			char c = cvWaitKey(33);
			if (c == 27) { // нажата ESC
				// освобождаем ресурсы
				hardwareOff(); 
				cvReleaseCapture( &capture );
				return(-1);
			}
			waitHardware();
		}
		laserOff();
	}
        // освобождаем ресурсы
	hardwareOff();
	cvReleaseCapture( &capture );
        cvDestroyAllWindows();
	return(0);
}

