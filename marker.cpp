#include "marker.hpp"
#include <utility>

using std::vector;


CvxMarker::CvxMarker()
{
	id  = -1;
	perimeter = -1;
	corners.resize(4);
}
CvxMarker::~CvxMarker()
{
	corners.clear();
}

void CvxMarker::getPerimeter()
{
	perimeter = 0.0;
	for (unsigned int i=0; i<4; i++)
	{
		int j = (i+1)%4;
		float d_x = corners[i].x - corners[j].x;
		float d_y = corners[i].y - corners[j].y;
		float d = sqrt(d_x * d_x + d_y * d_y);
		perimeter += d;
	}
}
/*
float CvxMarker::GetPerimeter(void)
{
	return perimeter;
}
*/

static void warp(const Mat &in, Mat &out, const Size size, const vector<Point2f> & points)
{
	CV_Assert(points.size() == 4);

	//obtain the perspective transform
	Point2f  pointsIn[4], pointsRes[4];
	for (unsigned int i=0; i<4; i++)
	{
		pointsIn[i] = points[i];
	}
	pointsRes[0] =  Point2f(0.0f, 0.0f);
	pointsRes[1] =  Point2f((float)(size.width-1), 0.0f);
	pointsRes[2] =  Point2f((float)(size.width-1), (float)(size.height-1));
	pointsRes[3] =  Point2f(0, (float)(size.height-1));
	Mat M = cv::getPerspectiveTransform(pointsIn, pointsRes);
	cv::warpPerspective(in, out,  M, size);
}

static int hammDistMarker(Mat  bits)
{
	int ids[4][5]=
	{
		{
			1,0,0,0,0
		}
		,
		{
			1,0,1,1,1
		}
		,
		{
			0,1,0,0,1
		}
		,
		{
			0,1,1,1,0
		}
	};
	int dist=0;

	for (int y=0;y<5;y++)
	{
		int minSum = INT_MAX;
		//hamming distance to each possible word
		for (int p=0; p<4; p++)
		{
			int sum=0;
			//now, count
			for (int x=0;x<5;x++)
			{
				sum +=  (bits.at<uchar>(y,x) == ids[p][x])?0:1;
			}
			if (minSum > sum)
			{
				minSum = sum;
			}
		}
		//do the and
		dist+=minSum;
	}
	return dist;
}

static void rotate(const Mat &in, Mat &out)
{
	assert(!in.empty());
	in.copyTo(out);
	for (int i=0; i<in.rows; i++)
	{
		for (int j=0; j<in.cols; j++)
		{
			out.at<uchar>(i,j)=in.at<uchar>(in.cols-j-1,i);
		}
	}	
}
static int mat2id(Mat &bits)
{
	int val=0;
	for (int y=0;y<5;y++)
	{
		val<<=1;
		if ( bits.at<uchar>(y,1)) val|=1;
		val<<=1;
		if ( bits.at<uchar>(y,3)) val|=1;
	}
	return val;
}
static int getMarkerId(Mat &in, int &nRotations)
{
	CV_Assert(in.rows == in.cols);
	CV_Assert(in.type() == CV_8UC1);
	CV_Assert(in.rows == 70);

	Mat grey;
	grey = in;
	//threshold image
	cv::threshold(grey, grey,125, 255, THRESH_BINARY|THRESH_OTSU);

	//Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
	//the external border shoould be entirely black
	int swidth = 10;
	for (int y = 0; y < 7; y++)
	{
		int inc = 6;
		//for first and last row, check the whole border
		if (y == 0 || y == 6)
		{
			inc = 1;
		}
		for (int x=0; x<7; x+=inc)
		{
			int Xstart = swidth * x;
			int Ystart = swidth * y;
			Mat square = grey(Rect(Xstart, Ystart ,swidth, swidth));
			int nZ = cv::countNonZero(square);
			if (nZ> (swidth*swidth) /2) {
				return -1;//can not be a marker because the border element is not black!
			}
		}
	}

	//get information(for each inner square, determine if it is  black or white)
	Mat bits = Mat::zeros(5,5,CV_8UC1);	

	for (int y=0; y<5; y++)
	{
		for (int x=0; x<5; x++)
		{
			int Xstart=(x+1)*(swidth);
			int Ystart=(y+1)*(swidth);
			Mat square = grey(Rect(Xstart,Ystart,swidth,swidth));
			int nZ = countNonZero(square);
			if (nZ> (swidth*swidth) /2)
			{
				bits.at<uchar>(y,x) = 1;
			}
		}
	}

	//checkl all possible rotations
	Mat rotations[4];
	int dist = -1;
	rotations[0] = bits;	
	dist = hammDistMarker(rotations[0]);

	//first distance, second Id
	std::pair<int,int> minDist(dist, 0);
	for (unsigned int i = 1; i<4; i++)
	{
		if (minDist.first == 0)
		{
			break;
		}
		//rotate
		rotate(rotations[i-1], rotations[i]);
		//get the hamming distance to the nearest possible word
		dist = hammDistMarker( rotations[i]) ;
		if (dist < minDist.first)
		{
			minDist.first =  dist;
			minDist.second = i;
		}
	}

	//not match with marker
	if (minDist.first != 0)
	{
		//@todo: correct if any error
		return -1;
	}
	else 
	{
		nRotations = minDist.second;
		return mat2id(rotations[ minDist.second]);
	}	
}

bool CvxMarker::track(const Mat &image, const int markerNumber, vector<CvxMarker> &inoutMarkers, 
					  const MarkerParameters &perimeter)

{
	//check input parameters	
	CV_Assert(!image.empty());
	CV_Assert(image.type() == CV_8UC1 || image.type() == CV_8UC3);
	CV_Assert(markerNumber == inoutMarkers.size());

	//convert color image to a gray image
	Mat grayMat;
	if(image.type() == CV_8UC3)
	{
		cv::cvtColor(image, grayMat, CV_BGR2GRAY);
	}
	else
	{
		grayMat = image;
	}

	// detect marker position in a sub area of the image
	for (unsigned int i = 0; i<inoutMarkers.size(); ++i)
	{
		CvRect r = inoutMarkers[i].subArea;
		vector<CvxMarker> subMarker;
		bool rst = CvxMarker::detect(grayMat(r), 1, subMarker, perimeter);	
		if (rst)
		{
			subMarker[0].addOffset(r.x, r.y);
			inoutMarkers[i] = subMarker[0];
		}
		else
		{
			inoutMarkers.clear();
			return false;
		}
	}

	//make sure maker inside the image
	for (unsigned int i = 0; i<inoutMarkers.size(); ++i)
	{
		bool isInside = inoutMarkers[i].getSubArea(10, grayMat.cols, grayMat.rows);
		if (!isInside)
		{
			inoutMarkers.clear();
			return false;
		}
	}
	return inoutMarkers.size() == markerNumber;
}

static void cvxFindControus(IplImage *biImage, vector<vector<cv::Point> > &contoursVec, 
							int mode, int method, int minNum, int maxNum)
{
	CV_Assert(biImage);
	CV_Assert(biImage->nChannels == 1);

	CvMemStorage *mmst = cvCreateMemStorage(0);
	CV_Assert(mmst);
	CvSeq * contoure = 0;
	cvFindContours(biImage, mmst, &contoure, sizeof(CvContour), CV_RETR_LIST, method);	
	CvSeq * headContour = contoure;
	while (contoure)
	{
		if (contoure->total >= minNum && contoure->total <= maxNum)
		{
 			vector<cv::Point> cc;
 			cc.resize(contoure->total);
 			for (int i = 0; i<contoure->total; ++i) {
 				CvPoint *p = (CvPoint*)cvGetSeqElem(contoure, i);
 				cc[i] = cvPoint(p->x, p->y);				
 			}
 			contoursVec.push_back(cc);
		}		
		contoure = contoure->h_next;
	}
	if (headContour)
	{
		cvClearSeq(headContour);
	}
	cvReleaseMemStorage(&mmst);
}

static void cvxApproxPolyDP(const vector<Point> &curve, vector<Point> & approxCurve,
					 double epsilon, bool closed)
{
	//
	CvMat ccurve = Mat(curve);
	CvMemStorage *mmst = cvCreateMemStorage(0);

	CvSeq* contoure = cvApproxPoly(&ccurve, sizeof(CvContour), mmst, CV_POLY_APPROX_DP, epsilon, closed);
	if( contoure && contoure->total > 0 )
	{
		approxCurve.resize(contoure->total);
		for (int i = 0; i<contoure->total; ++i) {
			CvPoint *p = (CvPoint*)cvGetSeqElem(contoure, i);
			approxCurve[i] = cvPoint(p->x, p->y);				
		}		
	}
	if (contoure)
	{
		cvClearSeq(contoure);
	}	
	cvReleaseMemStorage(&mmst);
}

bool CvxMarker::operator<(const CvxMarker &other)const
{
	if (id == other.id)
	{
		return perimeter > other.perimeter;
	}
	else
	{
		return id < other.id;
	}		
}

bool CvxMarker::detect(const Mat &image, const int markerNumber, vector<CvxMarker> &outMarkers, 
					   const MarkerParameters &perimeter)
{
	CV_Assert(!image.empty());
	CV_Assert(image.type() == CV_8UC1 || image.type() == CV_8UC3);
	
	//convert color image to a gray image
	Mat grayMat;
	if(image.type() == CV_8UC3)
	{
		cv::cvtColor(image, grayMat, CV_BGR2GRAY);
	}
	else
	{
		grayMat = image;
	}
	IplImage grayImage = grayMat;
	IplImage *thresholdImage = cvCloneImage(&grayImage);	
	vector<vector<cv::Point> > contours;

	//step 1: get 4 corner polygon	
	vector<CvxMarker > candidateMarkers;
	cvAdaptiveThreshold(&grayImage, thresholdImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 7, 7);	
	cvxFindControus(thresholdImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, 
					perimeter.minPixels, perimeter.maxPixels);
	cvReleaseImage(&thresholdImage);

	for (unsigned int i=0; i<contours.size();i++)
	{
		if (contours[i].size() >= perimeter.minPixels && contours[i].size() <= perimeter.maxPixels)
		{
			vector<Point> approxCurve;			
			int minDist = INT_MAX;    //dis ^ dis
			cvxApproxPolyDP(contours[i], approxCurve, 10, true);			
			
			//make sure 4 corners and convex
			if (approxCurve.size() != 4 || !cv::isContourConvex(Mat(approxCurve)))
			{
				continue;
			}

			//make sure that the distance between consecutive points is large enough			
			for (int j=0; j<4; j++)
			{
				int d_x = approxCurve[j].x - approxCurve[(j+1)%4].x;
				int d_y = approxCurve[j].y - approxCurve[(j+1)%4].y;
				int d = d_x * d_x + d_y * d_y;
				if (d < minDist)
				{
					minDist = d;
				}
			}
			
			//check that distance is not very small
			if (minDist > perimeter.minPixels)
			{
				CvxMarker marker;
				candidateMarkers.push_back(marker);
				for (int j = 0; j<4; ++j)
				{
					candidateMarkers.back().corners[j] = Point2f(approxCurve[j]);
				}			
			}			
		}
	}

	contours.clear();
	// not enough markers 
	if(candidateMarkers.size() < markerNumber)
	{
		candidateMarkers.clear();
		return false;
	}

	//step 2: get marker ids
	//sort the points in anti-clockwise order
	// 0  1
	// 3  2 in image plane
	for (unsigned int i = 0; i < candidateMarkers.size(); i++)
	{

		//trace a line between the first and second point.
		//if the third point is at the right side, then the points are anti-clockwise
		double dx1 = candidateMarkers[i].corners[1].x - candidateMarkers[i].corners[0].x;
		double dy1 = candidateMarkers[i].corners[1].y - candidateMarkers[i].corners[0].y;
		double dx2 = candidateMarkers[i].corners[2].x - candidateMarkers[i].corners[0].x;
		double dy2 = candidateMarkers[i].corners[2].y - candidateMarkers[i].corners[0].y;
		double dir = (dx1*dy2)-(dy1*dx2);

		if (dir  < 0.0)		 //if the third point is in the left side, then sort in anti-clockwise order
		{
			std::swap(candidateMarkers[i].corners[1], candidateMarkers[i].corners[3]);
		}
	}

	//remove these elements whose corners are too close to each other
	vector<std::pair<int,int>  > tooNearCandidates;
	for (unsigned int i=0; i<candidateMarkers.size(); i++)
	{
		//calculate the average distance of each corner to the nearest corner of the other marker candidate
		for (unsigned int j=i+1; j<candidateMarkers.size(); j++)
		{
			double dist = 0.0;
			for (int k=0; k<4; k++)
			{
				float d_x = candidateMarkers[i].corners[k].x - candidateMarkers[j].corners[k].x;
				float d_y = candidateMarkers[i].corners[k].y - candidateMarkers[j].corners[k].y;
				dist += double(d_x * d_x + d_y * d_y);
			}
			dist /= 4.0;
			//if distance is too small
			if (dist < 100.0) {
				tooNearCandidates.push_back(std::pair<int,int>(i,j));
			}
		}
	}
	
	//computer marker perimeter
	for (unsigned int i = 0; i<candidateMarkers.size(); ++i)
	{
		candidateMarkers[i].getPerimeter();
	}

	//mark for removal the element of  the pair with smaller perimeter
	vector<bool> toRemove (candidateMarkers.size(), false);
	for (unsigned int i=0;i<tooNearCandidates.size();i++)
	{
		if ( candidateMarkers[tooNearCandidates[i].first ].perimeter > (candidateMarkers[tooNearCandidates[i].second].perimeter ))
		{
			toRemove[tooNearCandidates[i].second] = true;
		}
		else
		{
			toRemove[tooNearCandidates[i].first] = true;
		}
	}

	outMarkers.clear();    //clear input
	//identify the markers
	for (unsigned int i=0;i<candidateMarkers.size();i++)
	{
		if (!toRemove[i])
		{

			//Find projective homography
			Mat canonicalMarker;
			int nRotations;
			warp(Mat(&grayImage), canonicalMarker, Size(70, 70), candidateMarkers[i].corners);			
 			
 			int id = getMarkerId(canonicalMarker,nRotations);
			if (id != -1)
			{
				//sort the points so that they are always in the same order as in standard position
				candidateMarkers[i].id = id;
				std::rotate(candidateMarkers[i].corners.begin(), 
							candidateMarkers[i].corners.begin() + 4 - nRotations, 
							candidateMarkers[i].corners.end());
				outMarkers.push_back(candidateMarkers[i]);				
			}
		}
	}
	candidateMarkers.clear();

	if (outMarkers.size() < markerNumber)
	{		
		outMarkers.clear();
		return false;
	}
	
	//small id , larger perimeter put in the front
	std::sort(outMarkers.begin(), outMarkers.end());

	for (vector<CvxMarker>::iterator it = outMarkers.begin(); it != outMarkers.end();)
	{
		//remove markers with same Id but small perimeter
		vector<CvxMarker>::iterator next = it + 1;
		while(it != outMarkers.end() &&
			  next != outMarkers.end() &&
			  next->id == it->id)
		{
			next = outMarkers.erase(next);			
		}
		it = next;
	}

	if (outMarkers.size() != markerNumber)
	{
		outMarkers.clear();		
		return false;
	}

	//step 3: refine marker corners
	vector<Point2f> corners;
	for (unsigned int i=0; i<outMarkers.size(); i++)
	{
		for (int c=0; c<4; c++)
		{
			corners.push_back(outMarkers[i].corners[c]);
		}
	}
	cv::cornerSubPix(Mat(&grayImage), corners, cvSize(5,5), cvSize(-1,-1), 
		         cvTermCriteria (CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,30, 0.01));
	//copy back
	for (unsigned int i=0; i<outMarkers.size(); i++)
	{
		for (int c=0; c<4; c++)     
		{
			outMarkers[i].corners[c] = corners[i*4+c];
		}
	}

	for (unsigned int i = 0; i<outMarkers.size(); ++i)
	{
		//make sure all corners inside of image
		bool isInside = outMarkers[i].getSubArea(10, grayImage.width, grayImage.height);
		if (!isInside)
		{
			outMarkers.clear();
			return false;
		}
		outMarkers[i].getCenterPoints();
	}
	return outMarkers.size() == markerNumber;
}

Mat CvxMarker::createMarkerImage(int id, int size)
{
	CV_Assert(id >= 0 && id<1024);
	
	Mat marker(size,size, CV_8UC1);
	marker.setTo(Scalar(0));
	//for each line, create
	int swidth=size/7;
	int ids[4]={0x10,0x17,0x09,0x0e};
	for (int y=0; y<5; y++)
	{
		int index=(id>>2*(4-y)) & 0x0003;
		int val=ids[index];
		for (int x=0; x<5; x++)
		{
			Mat roi=marker(Rect((x+1)* swidth,(y+1)* swidth,swidth,swidth));
			if ( ( val>>(4-x) ) & 0x0001 )
			{
				roi.setTo(Scalar(255));
			}
			else
			{
				roi.setTo(Scalar(0));
			}
		}
	}
	return marker;
}
void CvxMarker::getCenterPoints(void)
{
	center = Point2f(0, 0);
	for (unsigned int i = 0; i<4; ++i)
	{
		center.x += corners[i].x;
		center.y += corners[i].y;
	}
	center.x /= 4.0;
	center.y /= 4.0;
}

bool CvxMarker::getSubArea(int dilateLength, int w, int h)
{
	Point2f p1 = Point2f(INT_MAX, INT_MAX);
	Point2f p2 = Point2f(INT_MIN, INT_MIN);
	for (unsigned int i = 0; i<4; ++i)
	{
		p1.x = std::min(p1.x, corners[i].x);
		p1.y = std::min(p1.y, corners[i].y);
		p2.x = std::max(p2.x, corners[i].x);
		p2.y = std::max(p2.y, corners[i].y);
	}
	CvPoint p3 = cvPoint(cvRound(p1.x), cvRound(p1.y));
	CvPoint p4 = cvPoint(cvRound(p2.x), cvRound(p2.y));

	//too close to image margin
	if (p3.x < 3 || p3.y < 3 || p4.x >= w - 3 || p4.y >=h-3)
	{
		return false;
	}
	p3.x -= dilateLength;
	p3.y -= dilateLength;
	p4.x += dilateLength;
	p4.y += dilateLength;
	
	p3.x = std::max(0, p3.x);
	p3.y = std::max(0, p3.y);
	p4.x = std::min(p4.x, w-1);
	p4.y = std::min(p4.y, h-1);
	subArea = cvRect(p3.x, p3.y, p4.x - p3.x, p4.y - p3.y);
	return true;
}

void CvxMarker::addOffset(int x, int y)
{
	for (unsigned int i = 0; i<corners.size(); ++i)
	{
		corners[i].x +=x;
		corners[i].y +=y;
	}
	getCenterPoints();
}