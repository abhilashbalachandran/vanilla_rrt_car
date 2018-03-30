#include <bits/stdc++.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace cv;

//variables
const double pi = 3.1415926535897;
const double L = 0.4;
const int sc = 400;


Mat img1(1000, 1000, CV_8UC3, Scalar(255, 255, 255));
Mat img2;

//data structure to store the rrt tree
class node
{
public:
	double x[3];
	double u[2];
	int index;
	int parent;
	node()
	{
		
		 u[0] = 0.0;
		 u[1] = 0.0; //control inputs
		index = 0;   //index of current node
	}
	
};

//stores the whole path
vector <node> path;


//used to generate a random node in a path
node gen_random(node xf);

//used to find the closest point in the tree to the randomly generated point
node find_closest(vector <node> x,node xrand);

//extend function in rrt
int extend(node xclosest, node xrand, node *xnew);

//checks for collision
//TODO general way of accepting collision from a .yaml file or .jpeg file from a user using opencv
int collision(vector <node> xa);

//for backtracking the tree from goal to start
void back_track(vector <node> x,node xnew, node xf);

//used to draw the tree
void draw(vector <node> xa);