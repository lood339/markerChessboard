#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <time.h>
#include "marker.hpp"

using namespace cv;
using namespace std;



int main( int argc, char **argv )
{    
	char *fileName = "example.png";
	int markerNumber = 4;

	Mat image = cv::imread(fileName, 1);
	assert(!image.empty());

	vector<CvxMarker> detectedMarkers;
	MarkerParameters mParameters;

	//1 detect marker position from an image
	double tt = clock();
	bool isOk = CvxMarker::detect(image, markerNumber, detectedMarkers, mParameters);
	cout<<"detect cost time "<<clock() - tt<<endl;
	if(isOk)
	{
		Mat detectResult = image.clone();
		for(int i = 0; i<detectedMarkers.size(); i++)
		{
			for(int j = 0; j<4; j++)
			{
				cv::line(detectResult, cv::Point(detectedMarkers[i].corners[j]),  
					cv::Point(detectedMarkers[i].corners[(j+1)%4]), cv::Scalar(0, 0, 255));
			}
			cout<<detectedMarkers[i].id<<endl;
		}
		cv::imshow("detect result", detectResult);
		
		Mat trackResult = image.clone();

		//2 tracking marker position from previous postion
		//  it coudl be done in a video in which the marker chessboard is moving continuously
		tt = clock();
		bool isTrackOk = CvxMarker::track(image, markerNumber, detectedMarkers, mParameters);
		cout<<"track cost time "<<clock() - tt<<endl;
		if(isTrackOk)
		{
			cout<<"tracking successful"<<endl;
			for(unsigned int i = 0; i<detectedMarkers.size(); i++)
			{
				for(int j = 0; j<4; j++)
				{
					cv::line(trackResult, cv::Point(detectedMarkers[i].corners[j]),  
						cv::Point(detectedMarkers[i].corners[(j+1)%4]), cv::Scalar(0, 0, 255));
				}
				cout<<detectedMarkers[i].id<<endl;
			}
			cv::imshow("track result", trackResult);
		}
		else
		{
			cout<<"tracking failed"<<endl;
		}

	}
	else
	{
		cout<<"not find marker chessboard in the image"<<endl;
		cv::imshow("input image", image);
	}

	cvWaitKey(0);
    return 0;
}

