#include <iostream>
#include <time.h>
#include "markerchessboard.hpp"

using namespace cv;
using namespace std;

int main( int argc, char **argv )
{
	char *fileName = "example.png";

	Mat image = cv::imread(fileName, 1);
	assert(!image.empty());

	CvMarkerChessboard board = CvMarkerChessboard(cv::Size(14, 10));
	vector<Point2f> corners;

	//no tracking at first
	double tt = clock();
	bool isFound = board.findCorners(image, false, corners, 5);
	cout<<"find corner cost time"<<clock() - tt<<endl;
	if(isFound)
	{
		Mat showImage = image.clone();
		cv::drawChessboardCorners( showImage, cv::Size(14, 10), corners, true );
		cv::imshow("find corners", showImage);

		//demo tracking 
		tt = clock();
		isFound = board.findCorners(image, true, corners, 5);
		cout<<"tracking corners cost time "<<clock() - tt<<endl;
		if(isFound)
		{
			showImage = image.clone();
			cv::drawChessboardCorners( showImage, cv::Size(14, 10), corners, true );
			cv::imshow("find corners by tracking", showImage);
		}
	}
	cvWaitKey(0);
	return 0;
}
