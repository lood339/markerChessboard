#ifndef _OPENCV_MARKER_HPP_
#define _OPENCV_MARKER_HPP_

#include <vector>
#include "cvuImage.h"

/************************************************************************/
/* 
	marker detection
	the algorithm is from
	ArUco: a minimal library for Augmented Reality applications based on OpenCv
	http://www.uco.es/investiga/grupos/ava/node/26
*/
/************************************************************************/
struct MarkerParameters
{
	MarkerParameters()
	{
		thresholdParameter = 7.0;
		minPixels = 100;
		maxPixels = 560;
	}
	float thresholdParameter;
	int   minPixels;    //minumum pixel numbers of perimeter
	int   maxPixels;    //minumum pixel numbers of perimeter
};


class CvxMarker
{
public:
	CvxMarker();
	~CvxMarker();
	void getCenterPoints(void);
	bool getSubArea(int dilateLength, int w, int h);
	//addOffset is used in marker tracking, in sub images
	void addOffset(int x, int y);
	bool operator<(const CvxMarker &other)const;

private:
	void getPerimeter(void);

public:
	int id;   //0-1023
	Point2f center;	
	vector<Point2f> corners;	
	CvRect subArea;
	float perimeter;
public:
	//detect marker in an image
	static bool detect(const Mat &image, const int markerNumber, vector<CvxMarker> &outMarkers, 
					   const MarkerParameters &criteria);

	
	//tracking marker position in consequence frames
	static bool track(const Mat &image, const int markerNumber, vector<CvxMarker> &inoutMarkers, 
		              const MarkerParameters &perimeter);

	//create marker image, id >=0 && id < 1024
	static cv::Mat createMarkerImage(int id, int size);

};


#endif