#pragma once
//
//#include <stdio.h>
//#include <tchar.h>
//#include <iostream>
//#include <fstream>

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define CV_VERSION_ID       CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef _DEBUG
#define cvLIB(name) "opencv_" name CV_VERSION_ID "d"
#else
#define cvLIB(name) "opencv_" name CV_VERSION_ID
#endif

#pragma comment( lib, cvLIB("core") )
#pragma comment( lib, cvLIB("imgproc") )
#pragma comment( lib, cvLIB("highgui") )
#pragma comment( lib, cvLIB("flann") )
#pragma comment( lib, cvLIB("features2d") )
#pragma comment( lib, cvLIB("calib3d") )
#pragma comment( lib, cvLIB("gpu") )
#pragma comment( lib, cvLIB("legacy") )
#pragma comment( lib, cvLIB("ml") )
#pragma comment( lib, cvLIB("objdetect") )
#pragma comment( lib, cvLIB("ts") )
#pragma comment( lib, cvLIB("video") )
#pragma comment( lib, cvLIB("contrib") )
#pragma comment( lib, cvLIB("nonfree") )

#ifdef _DEBUG
#define _3partyLIB(name) name "d"
#else
#define _3partyLIB(name) name
#endif

#pragma comment(lib, _3partyLIB("libpng"))
#pragma comment(lib, _3partyLIB("libtiff"))
#pragma comment(lib, _3partyLIB("IlmImf"))
#pragma comment(lib, _3partyLIB("libjasper"))
#pragma comment(lib, _3partyLIB("libjpeg"))
#pragma comment(lib, _3partyLIB("zlib"))
#pragma comment(lib,"comctl32")