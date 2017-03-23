#include <iostream>
#include "slamBase.h"
using namespace std;
using namespace cv;
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

int main( int argc, char** argv )
{
	//read imgs
	cv::Mat img1 = cv::imread("/home/limy/myVO/data/imgL0.png");
	cv::Mat img2 = cv::imread("/home/limy/myVO/data/imgL1.png");
	cv::Mat disimg1 = cv::imread("/home/limy/myVO/data/imgL0.png",0);
	cv::Mat disimg2 = cv::imread("/home/limy/myVO/data/imgR0.png",0);
	//cv::Mat depth1 = cv::imread("./data/depth1.png");
	//cv::Mat depth2 = cv::imread("./data/depth2.png");
/*
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> descriptor;
	detector = cv::FeatureDetector::create("ORB");
	descriptor = cv::DescriptorExtractor::create("ORB");

	vector<cv::KeyPoint> kp1,kp2;
	detector->detect(img1,kp1);
	detector->detect(img2,kp2);
	cout<<"keypoints number of two imgs:"<<kp1.size()<<","<<kp2.size()<<endl;

	cv::Mat imgShow;
	cv::drawKeypoints(img1,kp1,imgShow,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::imshow("keypoints",imgShow);
	cv::imwrite("/home/limy/myVO/data/keypoints.png",imgShow);
	cv::waitKey(0);

	cv::Mat desp1,desp2;
	descriptor->compute(img1,kp1,desp1);
	descriptor->compute(img2,kp2,desp2);

	vector<cv::DMatch> matches;
	cv::BFMatcher matcher;
	matcher.match(desp1,desp2,matches);
	cout<<"Find "<<matches.size()<<" matches."<<endl;

	cv::Mat imgMatches;
	cv::drawMatches(img1,kp1,img2,kp2,matches,imgMatches);
	cv::imshow("matches", imgMatches);
	cv::imwrite("/home/limy/myVO/data/matches.png", imgMatches);
	cv::waitKey(0);

	vector<cv::DMatch> goodMatches;
	double minDis = 9999;
	for(size_t i=0;i < matches.size();i++)
	{
		if(matches[i].distance < minDis)
			minDis = matches[i].distance;
	}
	cout<<"min dis = "<<minDis<<endl;

	for(size_t i=0; i<matches.size();i++)
	{
		if(matches[i].distance < 10*minDis)
			goodMatches.push_back(matches[i]);
	}
	
	cout<<"good matches = "<<goodMatches.size()<<endl;
	cv::drawMatches(img1,kp1,img2,kp2,goodMatches,imgMatches);
	cv::imshow("good matches", imgMatches);
	cv::imwrite("/home/limy/myVO/data/good_matches.png", imgMatches);
	cv::waitKey(0);
*/
	Mat disp,disp1;
 	
 	int nDisparities=16,SADWindowSize=21;

   	StereoBM bm;
   	bm.state->minDisparity = 0;
   	bm.state->numberOfDisparities = nDisparities;
   	bm.state->SADWindowSize = SADWindowSize;
   	bm(disimg1,disimg2,disp,CV_32F);
   	cout<<disp<<endl;
   	cv::imwrite("/home/limy/myVO/data/disparity.png", disp);
   
	imshow("disparity",disp);
    waitKey(0);
    disp.convertTo(disp1, CV_8U,255/16);  
    imshow("disparity1",disp1);
    waitKey(0);

   	Size img_size = disimg1.size();
   	cout<<"origin img size:"<<img_size.height<<"x"<<img_size.width<<endl;
   	Size disp_size = disp.size();
   	cout<<"disp img size:"<<img_size.height<<"x"<<img_size.width<<endl;
    //if(disp.empty())cout<<"disp error!"<<endl;
    //else cout<<"disp no error!"<<endl;  
    //StereoSGBM sgbm;  
/* 	
 	Size img_size = disimg1.size();  
    Rect roi1, roi2;  
    Mat Q;  
    int numberOfDisparities=16,SADWindowSize = 9;
  
    bm.state->roi1 = roi1;  
    bm.state->roi2 = roi2;  
    bm.state->preFilterSize = 9;
    bm.state->preFilterCap = 31;  
    bm.state->SADWindowSize = 9;  
    bm.state->minDisparity = 0;  
    bm.state->numberOfDisparities = 16;  
    bm.state->textureThreshold = 10;  
    bm.state->uniquenessRatio = 5;  
    bm.state->speckleWindowSize = 100;  
    bm.state->speckleRange = 32;  
    bm.state->disp12MaxDiff = 1;  

    sgbm.preFilterCap = 63;  
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;  
  
    int cn = img1.channels();  
  
    sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;  
    sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;  
    sgbm.minDisparity = 0;  
    sgbm.numberOfDisparities = numberOfDisparities;  
    sgbm.uniquenessRatio = 10;  
    sgbm.speckleWindowSize = bm.state->speckleWindowSize;  
    sgbm.speckleRange = bm.state->speckleRange;  
    sgbm.disp12MaxDiff = 1;  
    sgbm.fullDP = 0;  
 
    var.levels = 3;                                 // ignored with USE_AUTO_PARAMS  
    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS  
    var.nIt = 25;  
    var.minDisp = -numberOfDisparities;  
    var.maxDisp = 0;  
    var.poly_n = 3;  
    var.poly_sigma = 0.0;  
    var.fi = 15.0f;  
    var.lambda = 0.03f;  
    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS  
    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS  
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING;
*/





/*
	vector<cv::Point3f> pts_obj;
	vector<cv::Point2f> pts_img;

	CAMERA_INTRINSIC_PARAMETERS C;
	C.cx = 325.5;
	C.cy = 253.5;
	C.fx = 518.0;
	C.fy = 519.0;
	C.scale = 1000.0;

	for(size_t i=0; i<goodMatches.size(); i++)
	{
		cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
		ushort d = depth1.ptr<ushort>(int (p.y))[int (p.x)];
		if(d==0)
			continue;
		pts_img.push_back(cv::Point2f(kp2[goodMatches[i].trainIdx].pt));
		cv::Point3f pt(p.x,p.y,d);
		cv::Point3f pd = point2dTo3d(pt,C);
		pts_obj.push_back(pd);
	}

	double camera_matrix_data[3][3] ={
		{C.fx,0,C.cx},
		{0,C.fy,C.cy},
		{0,0,1}
	};

	cv::Mat cameraMatrix(3,3,CV_64F,camera_matrix_data);
	cv::Mat rvec, tvec, inliers;
	cv::solvePnPRansac(pts_obj,pts_img,cameraMatrix,cv::Mat(),rvec,tvec,false,100,1.0,100,inliers);

	cout<<"inliers:"<<inliers.rows<<endl;
	cout<<"R="<<rvec<<endl;
	cout<<"T="<<tvec<<endl;

	vector<cv::DMatch> matchesShow;
	for(size_t i=0; i<inliers.rows;i++)
	{
		matchesShow.push_back(goodMatches[inliers.ptr<int>(i)[0]]);		
	}
	cv::drawMatches(img1,kp1,img2,kp2,matchesShow,imgMatches);
    cv::imshow( "inlier matches", imgMatches );
    cv::imwrite( "./data/inliers.png", imgMatches );
    cv::waitKey( 0 );
*/
    return 0;

}
