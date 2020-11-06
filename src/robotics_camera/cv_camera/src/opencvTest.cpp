#include <opencv2/opencv.hpp>
#include <ctime>
#include <iostream>
#include <sys/time.h>
#include <stdio.h>
using namespace cv;


int main()
{
	VideoCapture capture(0);
	if (!capture.isOpened())
		return -1;
	Mat edges;
        int xx = capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
    	int yy = capture.set(cv::CAP_PROP_EXPOSURE, 1.0);


        printf("Seting function return value: auto exposure:%d, exposure %d. \n",xx,yy);
        int num=0;

	struct   timeval   start,stop,diff;
  	gettimeofday(&start,0);
	while (num < 300)
	{
		num++;
                Mat frame;
		capture >> frame;
		if (frame.empty())
		{
			break;
		}
		else
		{
			cvtColor(frame, edges, COLOR_BGR2GRAY);
			blur(edges, edges, Size(7, 7));
			//Canny(edges, edges, 0, 30, 3);
			//imshow("Video", frame);
		}
		waitKey(3);
	}


	gettimeofday(&stop,0);
	printf("Total time:%d seconds\n",stop.tv_sec - start.tv_sec);

	capture.release();
	destroyAllWindows();
	return 0;
}
