#include <bits/stdc++.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

Mat img1(1000, 1000, CV_8UC3, Scalar(255, 255, 255));

int main(int argc, char **argv)
{

	

	Point pt1,pt2;

	pt1.x = 500;
	pt1.y = 500;

	cout<<sin(1);

	/*line( img1, Point( 200, 200 ),Point(500,500), Scalar( 0, 0, 255 ), 1, 8 );
	//circle(img1,pt1,100,Scalar(255,0,0),1,8);

	imshow("test",img1);
	waitKey(0);*/
	return(0);


}
