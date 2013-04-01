#include "markerchessboard.hpp"
#include <time.h>


/************************************************************************/
/*   CvMarkerChessboard                                                */
/************************************************************************/
CvMarkerChessboard::CvMarkerChessboard(const CvSize &size)
{
	CV_Assert(size.width >=2 && size.height >= 2);

	const float unitLength = 16.5; //arbitrary postion number
	patternSize = size;
	markerCenters.push_back(Point2f(-unitLength, -unitLength));
	markerCenters.push_back(Point2f(unitLength*size.width, -unitLength));
	markerCenters.push_back(Point2f(unitLength*size.width, unitLength*size.height));
	markerCenters.push_back(Point2f(-unitLength, unitLength*size.height));

	boardCorners.resize(size.height*size.width);
	for (int y = 0; y<size.height; ++y)
	{
		for (int x = 0; x<size.width; ++x)
		{
			float f_x = x * unitLength;
			float f_y = y * unitLength;
			int idx = y * size.width + x;
			boardCorners[idx] = Point2f(f_x, f_y);
		}
	}

}
CvMarkerChessboard::~CvMarkerChessboard()
{
	markerCenters.clear();    //physical position of marker centers
	preMarkers.clear();     //previous frame marker position
	curMarkers.clear(); 
	boardCorners.clear();
}


bool CvMarkerChessboard::findCorners(const Mat &image, bool isTracking, vector<Point2f> &outCorners, 
									  int distortionThreshold)
{
	//check parameters
	CV_Assert(!image.empty());
	CV_Assert(image.depth() == CV_8U);
	
	Mat grayImage;
	if (image.channels() == 1)
	{
		grayImage = image;
	}
	else 
	{
		cv::cvtColor(image, grayImage, CV_BGR2GRAY);
	}

	int h = image.rows;
	int w = image.cols;

	//get marker parameters
	MarkerParameters mParameters;		
	CvMarkerChessboard::GetThreshold(cvSize(w, h), patternSize, mParameters.maxPixels, mParameters.minPixels);

	//tracking marker position in four sub area of the image
	curMarkers.clear();
	if (isTracking && preMarkers.size() == 4)
	{
		for (unsigned int i = 0; i<4; ++i)
		{
			CvRect r = preMarkers[i].subArea;
			vector<CvxMarker> subMarker;
			bool rst = CvxMarker::detect(grayImage(r), 1, subMarker,  mParameters);			
			if (rst)
			{
				subMarker[0].addOffset(r.x, r.y);
				curMarkers.push_back(subMarker[0]);
			}
			else
			{
				curMarkers.clear();
				break;
			}
		}		
	}
	else
	{
		//detecting in whole image
		bool ret = CvxMarker::detect(grayImage, 4, curMarkers, mParameters);
		if (!ret)
		{ 
			curMarkers.clear();
		}
	}		
	if (curMarkers.size() == 4)
	{
		for (unsigned int i = 0; i<4; ++i)
		{
			curMarkers[i].getCenterPoints();
			curMarkers[i].getSubArea(10, w, h);
		}
	}
	else
	{
	//	fprintf(stderr, "can not found markers!\n");
	//	fprintf(stderr, "reason: \n 1. don't have 4 markers in the image!\n 2. image is blur.\n");			
		return false;
	}

	//get H
	CV_Assert(markerCenters.size() == 4);
	CV_Assert(curMarkers.size() == 4);
	vector<Point2f> markerCentersInImage;
	for (unsigned int i = 0; i<curMarkers.size(); ++i)
	{
		markerCentersInImage.push_back(curMarkers[i].center);
	}
	Mat homo = CvMarkerChessboard::findHomography4Pts(markerCenters, markerCentersInImage);

	if(homo.empty())
	{
		return false;
	}

	//get initial chessboard corner position
	outCorners.resize(patternSize.width * patternSize.height);
	double *pData = (double*)(homo.data);
	for (unsigned int i = 0; i<boardCorners.size(); ++i)
	{
		Point2f p1 = boardCorners[i];
		Point2f p2;
		float scale = 1.0;
		p2.x  = (float)(pData[0] * p1.x  + pData[1] * p1.y + pData[2]);
		p2.y  = (float)(pData[3] * p1.x  + pData[4] * p1.y + pData[5]);
		scale = (float)(pData[6] * p1.x  + pData[7] * p1.y + pData[8]);
		p2.x /= scale;
		p2.y /= scale;
		CV_Assert(p2.x >=0 && p2.x < w);
		CV_Assert(p2.y >=0 && p2.y < h);
		outCorners[i] = p2;
	}

	//get refined corner position
	vector<Point2f> unRef = outCorners;
	int winLength = 11;
	if (w < 800)
	{
		winLength = 5;
	}	
	cv::cornerSubPix(grayImage, outCorners, Size(winLength, winLength), Size(-1, -1), 
						TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


	//check final points with maxiDistortion
	for (unsigned int i = 0; i<outCorners.size(); ++i)
	{
		if (fabs(unRef[i].x - outCorners[i].x) > distortionThreshold 
			|| fabs(unRef[i].y - outCorners[i].y) > distortionThreshold) 
		{			
			outCorners.clear();
		//	fprintf(stderr, "distortion threshold check failed\n");
			return false;
		}
	}

	//keep tracking
	preMarkers = curMarkers;
	return true;
}





void CvMarkerChessboard::GetThreshold(const CvSize &imageSize, const CvSize &boardSize, int &thMax, int &thMin)
{
	float avg_w = 1.0f * imageSize.width/boardSize.width;
	float avg_h = 1.0f * imageSize.height/boardSize.height;
	float avg_max = std::max(avg_w, avg_h);
	thMax = (int)(avg_w * 4 * 1.2);
	thMin = thMax/6;
}

Mat CvMarkerChessboard::findHomography4Pts(const vector<Point2f> &src, const vector<Point2f> &dst)
{
	Mat h;
	if (src.size() != 4 || dst.size() != 4)
	{
	//	printf("Error: CvuBoard::findHomography4Pts input size must be 4\n");
		return h;
	}	
	h = cv::findHomography(src, dst);

	//re-scale the homography matrix since opencv keep h[8] = 1.0;
	double *pData = (double*)(h.data);
	double scale = 1.0/(pData[6] * src[0].x + pData[7] * src[0].y + pData[8]);
	h *= scale;
	return h;
}







