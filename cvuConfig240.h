#ifndef CVU_IMAGE_CONFIG_240_H
#define CVU_IMAGE_CONFIG_240_H 1

#ifdef _DEBUG   //debug

#ifndef CV_CORE_240_LIB
#define CV_CORE_240_LIB 1
#pragma comment(lib, "opencv_core240d.lib")
#endif

#ifndef CV_IMGPROC_240_LIB
#define CV_IMGPROC_240_LIB 1
#pragma comment(lib, "opencv_imgproc240d.lib")
#endif

#ifndef CV_HIGHGUI_240_LIB
#define CV_HIGHGUI_240_LIB 1
#pragma comment(lib, "opencv_highgui240d.lib")
#endif

#ifndef CV_CALIB3D_240_LIB
#define CV_CALIB3D_240_LIB 1
#pragma comment(lib, "opencv_calib3d240d.lib")
#endif

#ifndef CV_FEATURE2D_240_LIB
#define CV_FEATURE2D_240_LIB 1
#pragma comment(lib, "opencv_features2d240d.lib")
#endif

#ifndef CV_NONFREE_240_LIB
#define CV_NONFREE_240_LIB 1
#pragma comment(lib, "opencv_nonfree240d.lib")
#endif

#ifndef CV_FLANN_240_LIB
#define CV_FLANN_240_LIB 1
#pragma comment(lib, "opencv_flann240d.lib")
#endif

#ifndef CV_CONTRIB_240_LIB
#define CV_CONTRIB_240_LIB 1
#pragma comment(lib, "opencv_contrib240d.lib")
#endif

#else   //release

#ifndef CV_CORE_240_LIB
#define CV_CORE_240_LIB 1
#pragma comment(lib, "opencv_core240.lib")
#endif

#ifndef CV_IMGPROC_240_LIB
#define CV_IMGPROC_240_LIB 1
#pragma comment(lib, "opencv_imgproc240.lib")
#endif

#ifndef CV_HIGHGUI_240_LIB
#define CV_HIGHGUI_240_LIB 1
#pragma comment(lib, "opencv_highgui240.lib")
#endif

#ifndef CV_CALIB3D_240_LIB
#define CV_CALIB3D_240_LIB 1
#pragma comment(lib, "opencv_calib3d240.lib")
#endif

#ifndef CV_FEATURE2D_240_LIB
#define CV_FEATURE2D_240_LIB 1
#pragma comment(lib, "opencv_features2d240.lib")
#endif

#ifndef CV_NONFREE_240_LIB
#define CV_NONFREE_240_LIB 1
#pragma comment(lib, "opencv_nonfree240.lib")
#endif

#ifndef CV_FLANN_240_LIB
#define CV_FLANN_240_LIB 1
#pragma comment(lib, "opencv_flann240.lib")
#endif

#ifndef CV_CONTRIB_240_LIB
#define CV_CONTRIB_240_LIB 1
#pragma comment(lib, "opencv_contrib240.lib")
#endif

#endif  //else

#endif