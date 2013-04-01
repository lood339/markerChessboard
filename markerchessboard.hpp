#ifndef _OPENCV_MARKER_CHESSBOARD_HPP_
#define _OPENCV_MARKER_CHESSBOARD_HPP_

#include <vector>
#include "cvuImage.h"
#include "marker.hpp"

using std::vector;
/************************************************************************/
/* 
	implementation corner detection in paper
	"Calibration for High-Definition Cameras Rigs with Marker Chessboard"
	in CVPR2012 workshop in 3DCINE
*/
/************************************************************************/

class CvMarkerChessboard
{
public:
	CvMarkerChessboard(const CvSize &size);
	~CvMarkerChessboard();

	
	/************************************************************************/
	/* 
		corner detection function
		input:
		isTracking:    tracking marker position in continues frames
		DistThreshold: maxi distortion value in pixel, 
			   		   for HD camera, is in [2, 5], for web camera it may larger than 5
		output:
		pts2d:         chessboard corner position		
	*/
	/************************************************************************/

	bool findCorners(const Mat &image, bool isTracking, vector<Point2f> &outCorners, 
					 int distortionThreshold = 5);

private:
	CvSize patternSize;				  //chessboard size
	vector<Point2f> boardCorners;     //physical position of corners in 2d
	vector<Point2f> markerCenters;    //physical position of marker centers
	vector<CvxMarker> preMarkers;     //previous frame marker position
	vector<CvxMarker> curMarkers;     //current frame marker position
	
	/************************************************************************/
	/* 
		imageSize: such as 1920 * 1080
		boradSize: such as 14 * 10
		thMax: threshold of maximum perimeter of candidate marker 
		thMin: threshold of minimum perimeter of candidate marker 
	*/
	/************************************************************************/
	static void GetThreshold(const CvSize &imageSize, const CvSize &boardSize, int &thMax, int &thMin);

	//find homography of 4 correspondings without scaling
	static Mat findHomography4Pts(const vector<Point2f> &src, const vector<Point2f> &dst);
	
};









#endif