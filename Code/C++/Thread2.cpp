#include <ctime>
#include <iostream>
#include <raspicam/raspicam_cv.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "GPIOClass.h"
using namespace std;

int frame = 2;
int cam = 1;
int count = 0;

int change(){
    if(cam ==1)
    {
        epin->setval_gpio("0");
        f1pin->setval_gpio("1");
        f2pin->setval_gpio("0");
    }
    else if(cam ==2)
    {
        epin->setval_gpio("1");
        f1pin->setval_gpio("0");
        f2pin->setval_gpio("1");
    }
    else if(cam ==3)
    {
        epin->setval_gpio("0");
        f1pin->setval_gpio("0");
        f2pin->setval_gpio("1");
    }
    else if(cam ==4)
    {
        epin->setval_gpio("1");
        f1pin->setval_gpio("1");
        f2pin->setval_gpio("1");
    }
    
    cam - cam + 2;
    if(cam >4)
    {
        cam = 1;
    }
    
    return 0;
}
int main ( int argc,char **argv ) {
    GPIOClass* f1pin = new GPIOClass("15");
    GPIOClass* f2pin = new GPIOClass("16");
    GPIOClass* epin = new GPIOClass("7");
    
    f1pin->export_gpio(); //export GPIO15
    f2pin->export_gpio(); //export GPIO16
    epin->export_gpio(); //export GPIO7
    
    f1pin->setdir_gpio("out");
    f2pin->setdir_gpio("out");
    epin->setdir_gpio("out");
    
    f1pin->setval_gpio("0");
    f2pin->setval_gpio("0");
    epin->setval_gpio("0");
    
    raspicam::RaspiCam_Cv Camera;
    cv::Mat img1, img2;
    //set camera params
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //Open camera
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    //Start capture
    Camera.grab();
    Camera.retrieve ( img1);
    Camera.release();
    
    change();
    
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //Open camera
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    //Start capture
    Camera.grab();
    Camera.retrieve ( img2);
    Camera.release();
    
    int alg = STEREO_SGBM;
    int SADWindowSize=17;
    int numberOfDisparities= 32;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    
    Size img_size = img1.size();
    
    cv::Rect roi1, roi2;
    cv::Mat Q;
    
    cv::Mat M1, D1, M2, D2;//extrinsic parameters
    cv::Mat R, T, R1, P1, R2, P2;
    cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
    
    cv::Mat img1r, img2r;
    cv::remap(img1, img1r, map11, map12, INTER_LINEAR);
    cv::remap(img2, img2r, map21, map22, INTER_LINEAR);
    
    img1 = img1r;
    img2 = img2r;
    
    int cn = img1.channels();
    
    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->compute(img1, img2, disp);
    
    cv::Mat xyz;
    cv::reprojectImageTo3D(disp, xyz, Q, true);
    
    
}
