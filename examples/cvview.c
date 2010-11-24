/*
 *  cvview.c
 *  libfreenect
 *  Taken from https://gist.github.com/708654
 */

#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include "libfreenect.h"
#include <pthread.h>

freenect_context *f_ctx;
freenect_device *f_dev;

IplImage* rgbBack;
IplImage* rgbFront;

IplImage* depthBack;
IplImage* depthFront;
IplImage* depthCalibrated;

pthread_t kinect_thread;
pthread_mutex_t backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t framesReady_cond = PTHREAD_COND_INITIALIZER;
int got_frames = 0;

uint16_t t_gamma[2048];

void depth_cb(freenect_device *dev, freenect_depth *depth, uint32_t timestamp) {
	pthread_mutex_lock(&backbuf_mutex);
	{
		int i;
		uchar* gl_depth_back = (uchar*) depthBack->imageData;
		
		for (i=0; i<FREENECT_FRAME_PIX; i++) {
			int pval = t_gamma[depth[i]]; 
			
			// Grayscale version
			//			gl_depth_back[i] = (float) ((2048 * 256) / (depth[i] - 2048));
			
			int lb = pval & 0xff;
			switch (pval>>8) {
				case 0:
					gl_depth_back[3*i+0] = 255;
					gl_depth_back[3*i+1] = 255-lb;
					gl_depth_back[3*i+2] = 255-lb;
					break;
				case 1:
					gl_depth_back[3*i+0] = 255;
					gl_depth_back[3*i+1] = lb;
					gl_depth_back[3*i+2] = 0;
					break;
				case 2:
					gl_depth_back[3*i+0] = 255-lb;
					gl_depth_back[3*i+1] = 255;
					gl_depth_back[3*i+2] = 0;
					break;
				case 3:
					gl_depth_back[3*i+0] = 0;
					gl_depth_back[3*i+1] = 255;
					gl_depth_back[3*i+2] = lb;
					break;
				case 4:
					gl_depth_back[3*i+0] = 0;
					gl_depth_back[3*i+1] = 255-lb;
					gl_depth_back[3*i+2] = 255;
					break;
				case 5:
					gl_depth_back[3*i+0] = 0;
					gl_depth_back[3*i+1] = 0;
					gl_depth_back[3*i+2] = 255-lb;
					break;
					
				default:
					gl_depth_back[3*i+0] = 0;
					gl_depth_back[3*i+1] = 0;
					gl_depth_back[3*i+2] = 0;
					break;
			}
		}
		
		
		got_frames++;
		
		pthread_cond_signal(&framesReady_cond);
	}
	pthread_mutex_unlock(&backbuf_mutex);
}

void rgb_cb(freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp) 
{
	pthread_mutex_lock(&backbuf_mutex);
	got_frames++;
	cvSetData(rgbBack, rgb, 3*FREENECT_FRAME_W);
	pthread_cond_signal(&framesReady_cond);
	pthread_mutex_unlock(&backbuf_mutex);
}


void *kinect_threadFunc(void *arg) {
	while(freenect_process_events(f_ctx) >= 0) {  
		
	}
	pthread_exit(NULL);
	return NULL;
}

int main( int argc, char** argv ) { 
	
	int res;
	int i;
	
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
	
	printf("Kinect camera test\n");
	
	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}
	
	if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
		printf("Could not open device\n");
		return 1;
	}
	
	cvNamedWindow( "RGB", CV_WINDOW_AUTOSIZE );
	cvMoveWindow( "RGB", 0, 0);
	rgbBack = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 3);
	rgbFront = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 3);
	
	cvNamedWindow( "Depth", CV_WINDOW_AUTOSIZE );
	cvMoveWindow("Depth", FREENECT_FRAME_W, 0);
	depthBack = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 3);
	depthFront = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 3);
	
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_rgb_callback(f_dev, rgb_cb);
	freenect_set_rgb_format(f_dev, FREENECT_FORMAT_RGB);
	freenect_set_depth_format(f_dev, FREENECT_FORMAT_11_BIT);
	
	res = pthread_create(&kinect_thread, NULL, kinect_threadFunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		return 1;
	}
	
	freenect_start_depth(f_dev);
	freenect_start_rgb(f_dev);
	
	while(1) {  
		pthread_mutex_lock(&backbuf_mutex);
		{	
			while (got_frames < 2) {
				pthread_cond_wait(&framesReady_cond, &backbuf_mutex);
			}
			
			cvConvertImage(rgbBack, rgbFront, CV_BGR2GRAY);
			cvConvertImage(depthBack, depthFront, CV_BGR2GRAY);
			
			got_frames = 0;
		}
		pthread_mutex_unlock(&backbuf_mutex);
		
		
		cvShowImage("RGB", rgbFront);
		cvShowImage("Depth", depthFront);
		
		char c = cvWaitKey(10);
		if( c == 27 ) break;
	}
	
	
	cvDestroyWindow( "RGB" );
	cvDestroyWindow( "Depth" );
}
