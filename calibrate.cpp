#include <cv.h>
#include <highgui.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <inifile++.hpp>
#include <iostream>
#include <fstream>
#include <cstring>

#define PI 3.14159265
#define MIDLE_PIX 240
#define M_PER_PIX_X 0.000723
#define M_PER_PIX_Y 0.000523

int selected_point = -1;
// точки
CvPoint2D32f srcQuad[4], dstQuad[4];

double zoom_x = M_PER_PIX_X;
double zoom_y = M_PER_PIX_Y;
int middle_point_x = MIDLE_PIX;
int bottom_point_y = 480;

void myTrackbar1(int pos){
}
// рисуем целеуказатель
void drawTarget(IplImage* img, int x, int y, int radius)
{
        cvCircle(img,cvPoint(x, y),radius,CV_RGB(0,255,0),2,8);
        cvLine(img, cvPoint(x-radius/2, y-radius/2), cvPoint(x+radius/2, y+radius/2),CV_RGB(250,0,0),1,8);
        cvLine(img, cvPoint(x-radius/2, y+radius/2), cvPoint(x+radius/2, y-radius/2),CV_RGB(250,0,0),1,8);
}

// обработчик событий от мышки
void myMouseCallback( int event, int x, int y, int flags, void* param )
{
        IplImage* img = (IplImage*) param;

        switch( event ){
                case CV_EVENT_MOUSEMOVE: 
			if(selected_point >=0) {
				dstQuad[selected_point].x = x;
				dstQuad[selected_point].y = y;
			}
			break;
                case CV_EVENT_LBUTTONDOWN:
                        for(int i = 0; i<4; i++){
				if( (x - dstQuad[i].x)*(x - dstQuad[i].x) + (y - dstQuad[i].y)*(y - dstQuad[i].y) < 100) {
					selected_point = i;
					break;
				}
			}
                        break;

                case CV_EVENT_LBUTTONUP:
			if(selected_point >=0) {
				dstQuad[selected_point].x = x;
				dstQuad[selected_point].y = y;
				selected_point = -1;
			}
                        break;
        }
}

int main(int argc, char* argv[])
{
        IplImage *src=0, *dst=0, *points_src=0, *points_dst=0, *mix_src=0, *mix_dst=0;
	CvCapture* capture;
	IplImage* frame=0;

	if(argc >1){
	       src = cvLoadImage(argv[1],1);
	} 
	else {

		capture = cvCreateCameraCapture(CV_CAP_ANY); //cvCaptureFromCAM( 0 );
		assert( capture );
		printf("[i] Found camera %.0f x %.0f\n", 
			cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH), 
			cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT) );
		src = cvQueryFrame( capture );

	}


	try {
		inifilepp::parser p("scaner.ini");
		const inifilepp::parser::entry *ent;

		while((ent = p.next())) {
			int v;
			v = atoi(ent->val);
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point0.x")==0)
				srcQuad[0].x = v;
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point0.y")==0)
				srcQuad[0].y = v;
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point1.x")==0)
				srcQuad[1].x = v;
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point1.y")==0)
				srcQuad[1].y = v;
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point2.x")==0)
				srcQuad[2].x = v;
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point2.y")==0)
				srcQuad[2].y = v;
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point3.x")==0)
				srcQuad[3].x = v;
			if(strcmp(ent->sect, "srcQuad")==0 && strcmp(ent->key, "point3.y")==0)
				srcQuad[3].y = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point0.x")==0)
				dstQuad[0].x = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point0.y")==0)
				dstQuad[0].y = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point1.x")==0)
				dstQuad[1].x = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point1.y")==0)
				dstQuad[1].y = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point2.x")==0)
				dstQuad[2].x = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point2.y")==0)
				dstQuad[2].y = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point3.x")==0)
				dstQuad[3].x = v;
			if(strcmp(ent->sect, "dstQuad")==0 && strcmp(ent->key, "point3.y")==0)
				dstQuad[3].y = v;
			if(strcmp(ent->sect, "Zoom")==0 && strcmp(ent->key, "zoom.x")==0)
				zoom_x = v;
			if(strcmp(ent->sect, "Zoom")==0 && strcmp(ent->key, "zoom.y")==0)
				zoom_y = v;
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
        assert( src != 0 );
        // клонируем картинку
        dst = cvCloneImage(src);
        // клонируем картинку
        points_src = cvCloneImage(src);
	cvZero(points_src);
        points_dst = cvCloneImage(points_src);
        mix_src = cvCloneImage(points_src);
        mix_dst = cvCloneImage(points_src);


        cvNamedWindow( "image", 1 );
        cvNamedWindow( "cvWarpPerspective", 1 );
        // вешаем обработчик мышки
        cvSetMouseCallback( "image", myMouseCallback, (void*) points_dst);

	cvMoveWindow("image", 0, 0);
	cvMoveWindow("cvWarpPerspective", src->width+10, 0);

	
        // матрица преобразования
        CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);

		// задаём точки

		std::cout <<		
		srcQuad[0].x << " " << dstQuad[0].x << "\n" <<
		srcQuad[0].y << " " << dstQuad[0].y << "\n" <<
		srcQuad[1].x << " " << dstQuad[1].x << "\n" <<
		srcQuad[1].y << " " << dstQuad[1].y << "\n" <<
		srcQuad[2].x << " " << dstQuad[2].x << "\n" <<
		srcQuad[2].y << " " << dstQuad[2].y << "\n" <<     
		srcQuad[3].x << " " << dstQuad[3].x << "\n" <<
		srcQuad[3].y << " " << dstQuad[3].y << "\n";

        while(1){

		srcQuad[0].x = dstQuad[0].x;  //dst Top left
		srcQuad[0].y = dstQuad[0].y;
		srcQuad[1].x = dstQuad[1].x;  //dst Top right
		srcQuad[1].y = dstQuad[0].y;
		srcQuad[2].x = dstQuad[0].x;  //dst Bottom left
		srcQuad[2].y = dstQuad[0].y+(dstQuad[3].y-dstQuad[1].y);      
		srcQuad[3].x = dstQuad[1].x;  //dst Bot right
		srcQuad[3].y = dstQuad[0].y+(dstQuad[3].y-dstQuad[1].y);

		cvZero(points_src);
		cvZero(points_dst);
                drawTarget(points_src, srcQuad[0].x, srcQuad[0].y, 10);
                drawTarget(points_src, srcQuad[1].x, srcQuad[1].y, 10);
                drawTarget(points_src, srcQuad[2].x, srcQuad[2].y, 10);
                drawTarget(points_src, srcQuad[3].x, srcQuad[3].y, 10);
                drawTarget(points_dst, dstQuad[0].x, dstQuad[0].y, 10);
                drawTarget(points_dst, dstQuad[1].x, dstQuad[1].y, 10);
                drawTarget(points_dst, dstQuad[2].x, dstQuad[2].y, 10);
                drawTarget(points_dst, dstQuad[3].x, dstQuad[3].y, 10);

		cvOr(src, points_dst, mix_src);
		cvShowImage( "image", mix_src );

		// получаем матрицу преобразования
		cvGetPerspectiveTransform(srcQuad,dstQuad,warp_matrix);
		// преобразование перспективы
		cvWarpPerspective(src,dst,warp_matrix, CV_WARP_INVERSE_MAP);

		// показываем

		cvOr(dst, points_src, mix_dst);
		cvShowImage( "cvWarpPerspective", mix_dst );

		char c = cvWaitKey(33);
                if (c == 27) { // нажата ESC

                        break;
                }                
		else if(c == 13) { // Enter
			int ini;
			ini = fileno(fopen("scaner.ini", "w"));
			char coord[255];
			sprintf(coord, "[srcQuad]\n"); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point0.x = %.0f\n", srcQuad[0].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point0.y = %.0f\n", srcQuad[0].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point1.x = %.0f\n", srcQuad[1].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point1.y = %.0f\n", srcQuad[1].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point2.x = %.0f\n", srcQuad[2].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point2.y = %.0f\n", srcQuad[2].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point3.x = %.0f\n", srcQuad[3].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point3.y = %.0f\n", srcQuad[3].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, "[dstQuad]\n"); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point0.x = %.0f\n", dstQuad[0].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point0.y = %.0f\n", dstQuad[0].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point1.x = %.0f\n", dstQuad[1].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point1.y = %.0f\n", dstQuad[1].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point2.x = %.0f\n", dstQuad[2].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point2.y = %.0f\n", dstQuad[2].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point3.x = %.0f\n", dstQuad[3].x); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " point3.y = %.0f\n", dstQuad[3].y); 
			write(ini, coord, strlen(coord));
			sprintf(coord, "[Zoom]\n"); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " zoom.x = %f\n", 0.06/(srcQuad[0].x-srcQuad[1].x)); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " zoom.y = %f\n", 0.08/(srcQuad[2].y-srcQuad[0].y)); 
			write(ini, coord, strlen(coord));
			sprintf(coord, "[Center]\n"); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " center.point.x = %.0f\n", (srcQuad[0].x+srcQuad[1].x)/2); 
			write(ini, coord, strlen(coord));
			sprintf(coord, " center.point.y = %.0f\n", srcQuad[2].y); 
			write(ini, coord, strlen(coord));

			close(ini);
                        break;
                }

	}
        // освобождаем ресурсы
        cvReleaseMat(&warp_matrix);
        cvReleaseImage(&src);
        cvReleaseImage(&dst);

        // удаляем окна
        cvDestroyAllWindows();
        return 0;
}
