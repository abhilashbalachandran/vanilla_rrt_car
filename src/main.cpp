#include "vanilla_rrt.hpp"



node gen_random(node xf)
{
	int bias = rand() % 10 +1;
	node x;

	if(bias<8)
	{
		double r1 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
		x.x[0] = 0+(2.2-0)*r1;
		double r2 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
		x.x[1] = 0+(2-0)*r2;
		double lb = -pi;
		double ub = pi;
		double r3 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
		x.x[2] = lb+(ub-lb)*r3;
	}
	else
	{
		x.x[0] = xf.x[0];
		x.x[1] = xf.x[1];
		x.x[2] = xf.x[2];
	}

	return x;
}


node find_closest(vector <node> x,node xrand)
{
	int m = x.size();
	vector <double> dist;

	for(int i=0;i<m;i++)
	{
		double temp = sqrt(pow(x[i].x[0]-xrand.x[0],2)+pow(x[i].x[1]-xrand.x[1],2)+pow(x[i].x[2]-xrand.x[2],2));
		dist.push_back(temp);
	}
	int index = distance(dist.begin(),min_element(dist.begin(),dist.end()));
	node xnear = x[index];
	return (xnear);
}


int extend(node xclosest, node xrand, node *xnew)
{
	double v1 = 0.2; double v2 = -0.2;
	double lb = -pi/6; double ub = pi/6;
	double r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
	double phi = lb+(ub-lb)*r;
	double t = 0.5, dt =0.1;
	double th0 = xclosest.x[2], x0 = xclosest.x[0], y0 = xclosest.x[1];
	double v,a,x1,y1,th1,x2,th2,y2;


	double r1 = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);

	v = v2+(v1-v2)*r1;
	a = double(v/L*tan(phi));
	int j=1;
	vector <node> xa;
	node xtemp;
	//generate path by steering 
	for (float i=0;i<=t/dt;i=i+dt)
	{
		
		th1 = a*i+th0;
		x1 = double(v*dt/a*sin(th1)+x0-v*dt/a*sin(th0));
		y1 = double(-v*dt/a*cos(th1)+y0+v*dt/a*cos(th0));
		xtemp.x[0] = x1;
		xtemp.x[1] = y1;
		xtemp.x[2] = th1;
		xtemp.u[0] = v;
		xtemp.u[1] = phi;
		xtemp.index = xclosest.index;
		xa.push_back(xtemp);
		j++;

	}

	


	int flag = collision(xa);

	if(flag==0)
		{draw(xa);
			}

	

	xnew->x[0] = xtemp.x[0];
	xnew->x[1] = xtemp.x[1];
	xnew->x[2] = xtemp.x[2];
	xnew->u[0] = v;
	xnew->u[1] = phi;




	cout<<"flag"<<flag<<endl;
	cout<<"y value"<<xtemp.x[1];

	return(collision(xa));




}


int collision(vector <node> xa)
{	int flag=0;
	for (int i=0;i<xa.size();i++)
	{	
		double w = 0.2;
		double l = 0.4;
		double r = double(sqrt(0.05));
		double xc = xa[i].x[0];
		double yc = xa[i].x[1];


		if(yc>1.5-r)		//not colliding for y
		{

				flag = 1;
				break;
		}
		else if(yc<0.5+r&&xc<1+r)
		{

				flag = 1;
				break;
		}

		else if(yc<0.5+r&&xc>1.7-r)
		{

				flag = 1;
				break;
		}
			
	}
	return(flag);
}



	//implement collision here

void back_track(vector <node> x,node xnew, node xf)
{	vector <node> xt;
	xt.push_back(xf);
	double t = 0.5, dt =0.1;

	int index_new = xnew.index;
	while(1)
	{
		xt.push_back(x[index_new]);
		index_new = x[index_new].parent;
		if(index_new==-1)
			break;
	}

	Point pt1,pt2;
	int m = xt.size();
	for(int j=m-1;j>0;j--)
	{
		double th0 = xt[j].x[2], x0 = xt[j].x[0], y0 = xt[j].x[1];
		double v = xt[j-1].u[0]; double phi = xt[j-1].u[1];
		double a,x1,y1,th1;


	vector <node> xa;
	node xtemp;
	a = double(v/L*tan(phi));
	//generate path by steering 
	for (float i=0;i<=t/dt;i=i+dt)
	{
		
		th1 = a*i+th0;
		x1 = double(v*dt/a*sin(th1)+x0-v*dt/a*sin(th0));
		y1 = double(-v*dt/a*cos(th1)+y0+v*dt/a*cos(th0));
		xtemp.x[0] = x1;
		xtemp.x[1] = y1;
		xtemp.x[2] = th1;
		xtemp.u[0] = v;
		xtemp.u[1] = phi;
		xa.push_back(xtemp);
		path.push_back(xtemp);
	    cout<<"updated values = "<<x1<<" "<<y1<<" "<<th1<<endl;

	}

		for(int i=1;i<xa.size();i++)
	{	
		pt1.x = sc*(xa[i-1].x[0]);
		pt1.y = sc*(xa[i-1].x[1]);
		pt2.x = sc*(xa[i].x[0]);
		pt2.y = sc*(xa[i].x[1]);
		line( img1, pt1,pt2, Scalar( 0, 0, 0 ), 2, 8 );
		
	}
		Mat img2;
	flip(img1,img2,0);
	imshow("test",img2);
		waitKey(400);
	}
	
}
void draw(const vector <node> xa)
{	
	Point pt1,pt2;
	int m = xa.size();

	for(int i=1;i<m;i++)
	{	
		pt1.x = sc*(xa[i-1].x[0]);
		pt1.y = sc*(xa[i-1].x[1]);
		pt2.x = sc*(xa[i].x[0]);
		pt2.y = sc*(xa[i].x[1]);
		//cout<<"points = "<<pt1.x<<pt1.y<<pt2.x<<pt2.y;
		line( img1, pt1,pt2, Scalar( 0, 0, 255 ), 1, 8 );
		 //imshow("test",img1);

/*Uncomment the following line to see how the tree grows*/

		//waitKey(10);  
		
		
	}
	
}

int main(int argc, char **argv)
{	
	srand ( time(NULL) );


	//initialize ros
 	ros::init(argc,argv, "amin");
 	ros::NodeHandle n;

 	ros::Publisher joint_pub;
	tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;


 	//initial and final nodes
 	node xi;
 	node xf;
 	node xnew;

 	xi.x[0] = 0.3;
 	xi.x[1] = 1;
 	xi.x[2]  = 0;
 	xi.u[0] = 0.001;
 	xi.u[1] = 0.001;
 	xi.index = 0;
 	xi.parent = -1;

 	xf.x[0] = 1.3;
 	xf.x[1] = 0.25;
 	xf.x[2]  = double(pi/2);
 	xf.u[0] = 0.001;
 	xf.u[1] = 0.001;
 	xf.index = 1;
 	xf.parent = 0;

 	//draw the xi and xf points
 	circle(img1,Point(xi.x[0]*sc,xi.x[1]*sc),1,Scalar(255,0,0),10,8);
 	circle(img1,Point(xf.x[0]*sc,xf.x[1]*sc),1,Scalar(0,0,0),10,8);

 	//draw obstacles
 	rectangle(img1,Point(0*sc,1.5*sc),Point(980,2*sc),Scalar(125,100,255),2,8);
 	rectangle(img1,Point(0*sc,0),Point(1*sc,0.5*sc),Scalar(125,100,255),2,8);
 	rectangle(img1,Point(1.7*sc,0*sc),Point(980,0.5*sc),Scalar(125,100,255),2,8);
 	waitKey(0);

 	cout<<"drew initial and final points"<<endl;

 	//our tree
 	vector <node> x;
 	x.push_back(xi);
 	int i=0;
 	double error = 0.1;
 	double dist = 100;

 	//inside ros::ok()
 	//int j=1;

 		//tree growing
 		while(dist>error)
 		{	
 			//generate random node
	 		node xrand = gen_random(xf);


	 		node xclosest = find_closest(x,xrand);
	 		//cout<<"closest = "<<xclosest.x[0]<<" "<<xclosest.x[1]<<endl;

	 		//writing the extend function
	 		int flag = extend(xclosest,xrand,&xnew);
	 		//cout<<"x new added = "<<xnew.x[0]<<" "<<xnew.x[1]<<endl;

	 		if(flag ==0)
	 		{
	 			dist = sqrt(pow(xnew.x[0]-xf.x[0],2)+pow(xnew.x[1]-xf.x[1],2)+pow(xnew.x[2]-xf.x[2],2));
	 			xnew.index = i+1;
	 			xnew.parent = xclosest.index;
	 		
	 			//cout<<"index and parent= "<<xnew.index<<" "<<xnew.parent<<endl;
	 			x.push_back(xnew);
	 			if(xnew.x[1]<0.23)
	 				cout<<"fail"<<endl;
	 			i++;
	 			//cout<<dist<<endl;
	 			if(dist<error)
	 			{
	 			    back_track(x,xnew,xf);
	 			}

	 		}

 		}

 		//adjust image display for displaying through opencv
 		flip(img1,img2,0);
	    imshow("test",img2);
 		waitKey(0);


 		//publish tf data to display with rviz

 		for(int i=0;i<path.size();i++)
 		{
 			//cout<<"values="<<path[i].x[0]<<" "<<path[i].x[1]<<" "<<path[i].x[2]<<endl;
 			odom_trans.header.frame_id="world";
			odom_trans.child_frame_id="car_body";
			odom_trans.header.stamp = ros::Time::now();
			odom_trans.transform.translation.y = 10*path[i].x[1];
			odom_trans.transform.translation.x = 10*path[i].x[0];
			odom_trans.transform.translation.z = 0.1;
			odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,path[i].x[2]);
			//ROS_INFO("odom ran");
			ros::Duration(0.01).sleep();
			broadcaster.sendTransform(odom_trans);
			cout << "\033[2J\033[1;1H";
			if(i<path.size()-1)
			{
			cout<<"VELOCTIY = "<<path[i+1].u[0]<<endl;
			cout<<"STEERING = "<<path[i+1].u[1]<<endl;
			}
			else
			{
				cout<<"VELOCTIY = 0"<<endl;
				cout<<"STEERING = 0"<<endl;
			}

 		}


 		            
	 			    waitKey(0);
	 			    return(0);


}
