#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

void GoalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);			// call back goal position
void CurrentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);			// call back current position

// a class for polygon obstacles
class polygon{
	float x1,y1,x2,y2,x3,y3,x4,y4;
public:
	polygon(float a, float b, float c, float d, float e, float f, float g, float h) {x1=a;y1=b;x2=c;y2=d;x3=e;y3=f;x4=g;y4=h;}
	void get_vertex1(float &i, float &j) {i=x1;j=y1;}
	void get_vertex2(float &i, float &j) {i=x2;j=y2;}
	void get_vertex3(float &i, float &j) {i=x3;j=y3;}
	void get_vertex4(float &i, float &j) {i=x4;j=y4;}
	void get_edge1(float &i, float &j,float &k,float &l) {i = x1;j=y1;k=x2;l=y2;}
	void get_edge2(float &i, float &j,float &k,float &l) {i = x2;j=y2;k=x3;l=y3;}
	void get_edge3(float &i, float &j,float &k,float &l) {i = x3;j=y3;k=x4;l=y4;}
	void get_edge4(float &i, float &j,float &k,float &l) {i = x4;j=y4;k=x1;l=y1;}
};

// a class used when try to find the closest point on the edge of an obstacle to a control point
class closest_point{
public:
	float x,y,dist;	// x,y coordinates and its distance from the control point
	void set_point(float i,float j,float k) {x = i;y = j;k = dist;}

};

// a class for workspace potential
class potential{	
public:	
	float p_x,p_y;	// x,y components

};

// a class for control points
class control_point{
	float x,y,zeta,eta;	// x,y coordinates in the world frame; attractive/repulsive scaling factors
public:
	control_point(float a, float b) {zeta = a;eta = b;}
	void set_control_point(float a, float b) {x = a;y=b;}
	potential attractive(geometry_msgs::Pose2D Goal_position);	// calculate attractive potential of this control point
	potential repulsive(polygon p1,polygon p2,polygon p3,polygon p4); // calculate repulsive potential of this control point
	closest_point point_to_line_distance(float v1_x,float v1_y,float v2_x,float v2_y); // calculate the shortest distance from this control point to each obstacle

	geometry_msgs::Twist controller(nav_msgs::Odometry Current_position,float ax, float ay, potential p_a,potential p_r); // depending on the potential, calculate the velocity input
};


closest_point control_point::point_to_line_distance(float v1_x,float v1_y,float v2_x,float v2_y)
{
	int i;
	closest_point point;
	float dx = (fabs(v2_x-v1_x))/1000;	// check along the line defined by v1_x,v1_y and v2_x,v2_y for 1000 intervals
	float dy = (fabs(v2_y-v1_y))/1000;	// check along the line defined by v1_x,v1_y and v2_x,v2_y for 1000 intervals
	float dist;
	float dist_min=10; // give an large initial value for minimum distance, since the grid is 5x5

	for (i=0;i<=1000;i++){
		if (v1_x<=v2_x&&v1_y<=v2_y){	// if v1_x and v1_y are small, iteration starting from them
			dist = sqrt(pow((v1_x+i*dx)-x,2)+pow((v1_y+i*dy)-y,2));
			// if find a shorter distance, replace dist_min and store the distance and x,y coordiante of that point 	
			if (dist <=dist_min)	
			{
				dist_min = dist;
				point.set_point(v1_x+i*dx,v1_y+i*dy,dist_min);
			}
		}

		if (v1_x<=v2_x&&v1_y>=v2_y){	// if v1_x and v2_y are small, iteration starting from them
			dist = sqrt(pow((v1_x+i*dx)-x,2)+pow((v2_y+i*dy)-y,2));
			// if find a shorter distance, replace dist_min and store the distance and x,y coordiante of that point 
			if (dist <=dist_min)
			{
				dist_min = dist;
				point.set_point(v1_x+i*dx,v2_y+i*dy,dist_min);
			}
		}		

		if (v1_x>=v2_x&&v1_y>=v2_y){	// if v2_x and v2_y are small, iteration starting from them
			dist = sqrt(pow((v2_x+i*dx)-x,2)+pow((v2_y+i*dy)-y,2));
			// if find a shorter distance, replace dist_min and store the distance and x,y coordiante of that point 
			if (dist <=dist_min)
			{
				dist_min = dist;
				point.set_point(v2_x+i*dx,v2_y+i*dy,dist_min);
			}
		}

		if (v1_x>=v2_x&&v1_y<=v2_y){	// if v2_x and v1_y are small, iteration starting from them
			dist = sqrt(pow((v2_x+i*dx)-x,2)+pow((v1_y+i*dy)-y,2));
			// if find a shorter distance, replace dist_min and store the distance and x,y coordiante of that point 
			if (dist <=dist_min)
			{
				dist_min = dist;
				point.set_point(v2_x+i*dx,v1_y+i*dy,dist_min);
			}
		}
	}
	return point;	// return the closest point on each obstacle from this control point
}


potential control_point::attractive(geometry_msgs::Pose2D Goal_position)
{
	float x_error,y_error;
	potential p_a;
	x_error = x-Goal_position.x;
	y_error = y-Goal_position.y;
	
	// calculate the potential in x and y directions
	p_a.p_x = zeta*x_error;
	p_a.p_y = zeta*y_error;

	return p_a;
}

potential control_point::repulsive(polygon p1,polygon p2,polygon p3,polygon p4)
{
	// get shortest distances to all four obstacles from this control point
	float i,j,k,l;
	closest_point point1,point2,point3,point4;	
	float dist_min1=10,dist_min2=10,dist_min3=10,dist_min4=10;
	float x_min1,y_min1,x_min2,y_min2,x_min3,y_min3,x_min4,y_min4,potential_x,potential_y;
	potential p_r;
	
	// first obstacle: get edges one by one, then get the shortest distance to this edge, 
	//	           then determine which one is the shorest among four and then store
	//		   this shortest distance as well as x,y coordiantes of that point

	p1.get_edge1(i,j,k,l);	
	point1 = point_to_line_distance(i,j,k,l);
	if (point1.dist<=dist_min1){
		dist_min1 = point1.dist;
		x_min1 = point1.x;
		y_min1 = point1.y;
	}
	p1.get_edge2(i,j,k,l);
	point1 = point_to_line_distance(i,j,k,l);
	if (point1.dist<=dist_min1){
		dist_min1 = point1.dist;
		x_min1 = point1.x;
		y_min1 = point1.y;
	}
	p1.get_edge3(i,j,k,l);
	point1 = point_to_line_distance(i,j,k,l);
	if (point1.dist<=dist_min1){
		dist_min1 = point1.dist;
		x_min1 = point1.x;
		y_min1 = point1.y;
	}
	p1.get_edge4(i,j,k,l);
	point1 = point_to_line_distance(i,j,k,l);
	if (point1.dist<=dist_min1){
		dist_min1 = point1.dist;
		x_min1 = point1.x;
		y_min1 = point1.y;
	}

	// second obstacle: get edges one by one, then get the shortest distance to this edge, 
	//	           then determine which one is the shorest among four and then store
	//		   this shortest distance as well as x,y coordiantes of that point
	p2.get_edge1(i,j,k,l);
	point2 = point_to_line_distance(i,j,k,l);
	if (point2.dist<=dist_min2){
		dist_min2 = point2.dist;
		x_min2 = point2.x;
		y_min2 = point2.y;
	}
	p2.get_edge2(i,j,k,l);
	point2 = point_to_line_distance(i,j,k,l);
	if (point2.dist<=dist_min2){
		dist_min2 = point2.dist;
		x_min2 = point2.x;
		y_min2 = point2.y;
	}
	p2.get_edge3(i,j,k,l);
	point2 = point_to_line_distance(i,j,k,l);
	if (point2.dist<=dist_min2){
		dist_min2 = point2.dist;
		x_min2 = point2.x;
		y_min2 = point2.y;
	}
	p2.get_edge4(i,j,k,l);
	point2 = point_to_line_distance(i,j,k,l);
	if (point2.dist<=dist_min2){
		dist_min2 = point2.dist;
		x_min2 = point2.x;
		y_min2 = point2.y;
	}

	// third obstacle: get edges one by one, then get the shortest distance to this edge, 
	//	           then determine which one is the shorest among four and then store
	//		   this shortest distance as well as x,y coordiantes of that point
	p3.get_edge1(i,j,k,l);
	point3 = point_to_line_distance(i,j,k,l);
	if (point3.dist<=dist_min3){
		dist_min3 = point3.dist;
		x_min3 = point3.x;
		y_min3 = point3.y;
	}
	p3.get_edge2(i,j,k,l);
	point3 = point_to_line_distance(i,j,k,l);
	if (point3.dist<=dist_min3){
		dist_min3 = point3.dist;
		x_min3 = point3.x;
		y_min3 = point3.y;
	}
	p3.get_edge3(i,j,k,l);
	point3 = point_to_line_distance(i,j,k,l);
	if (point3.dist<=dist_min3){
		dist_min3 = point3.dist;
		x_min3 = point3.x;
		y_min3 = point3.y;
	}
	p3.get_edge4(i,j,k,l);
	point3 = point_to_line_distance(i,j,k,l);
	if (point3.dist<=dist_min3){
		dist_min3 = point3.dist;
		x_min3 = point3.x;
		y_min3 = point3.y;
	}


	// fourth obstacle: get edges one by one, then get the shortest distance to this edge, 
	//	           then determine which one is the shorest among four and then store
	//		   this shortest distance as well as x,y coordiantes of that point
	p4.get_edge1(i,j,k,l);
	point4 = point_to_line_distance(i,j,k,l);
	if (point4.dist<=dist_min4){
		dist_min4 = point4.dist;
		x_min4 = point4.x;
		y_min4 = point4.y;
	}
	p4.get_edge2(i,j,k,l);
	point4 = point_to_line_distance(i,j,k,l);
	if (point4.dist<=dist_min4){
		dist_min4 = point4.dist;
		x_min4 = point4.x;
		y_min4 = point4.y;
	}
	p4.get_edge3(i,j,k,l);
	point4 = point_to_line_distance(i,j,k,l);
	if (point4.dist<=dist_min4){
		dist_min4 = point4.dist;
		x_min4 = point4.x;
		y_min4 = point4.y;
	}
	p4.get_edge4(i,j,k,l);
	point4 = point_to_line_distance(i,j,k,l);
	if (point4.dist<=dist_min4){
		dist_min4 = point4.dist;
		x_min4 = point4.x;
		y_min4 = point4.y;
	}

	// eventually, as we get the shortest distance and x,y coordiantes of the point for each obstacle for this control point,
	// we sum all four obstacles' repulsive effects on this control point
	
	float Q=2;
	if (dist_min1>Q) dist_min2=Q;if (dist_min2>Q) dist_min3=Q;if (dist_min3>Q) dist_min3=Q;if (dist_min3>Q) dist_min3=Q;
	// however, if a obstalce is further away than 2, just igonre it by setting dist_min to Q
	p_r.p_x = eta*(1/Q-1/dist_min1)*1/(pow(dist_min1,2))*(x-x_min1)/sqrt(pow(x-x_min1,2)+pow(y-y_min1,2))
		+eta*(1/Q-1/dist_min2)*1/(pow(dist_min2,2))*(x-x_min2)/sqrt(pow(x-x_min2,2)+pow(y-y_min2,2))
		+eta*(1/Q-1/dist_min3)*1/(pow(dist_min3,2))*(x-x_min3)/sqrt(pow(x-x_min3,2)+pow(y-y_min3,2))
		+eta*(1/Q-1/dist_min4)*1/(pow(dist_min4,2))*(x-x_min4)/sqrt(pow(x-x_min4,2)+pow(y-y_min4,2));

	p_r.p_y = eta*(1/Q-1/dist_min1)*1/(pow(dist_min1,2))*(y-y_min1)/sqrt(pow(x-x_min1,2)+pow(y-y_min1,2))
		+eta*(1/Q-1/dist_min2)*1/(pow(dist_min2,2))*(y-y_min2)/sqrt(pow(x-x_min2,2)+pow(y-y_min2,2))
		+eta*(1/Q-1/dist_min3)*1/(pow(dist_min3,2))*(y-y_min3)/sqrt(pow(x-x_min3,2)+pow(y-y_min3,2))
		+eta*(1/Q-1/dist_min4)*1/(pow(dist_min4,2))*(y-y_min4)/sqrt(pow(x-x_min4,2)+pow(y-y_min4,2));
	
	return p_r; 

}

geometry_msgs::Twist control_point::controller(nav_msgs::Odometry Current_position, float ax,float ay,potential p_a,potential p_r)
{
	geometry_msgs::Twist controller;
	// from the derivation on book, equation (4.23), we can directly use this expression and skip jacobian calculation
	controller.linear.x = p_a.p_x+p_r.p_x;
	controller.linear.y = p_a.p_y+p_r.p_y;

	//controller.angular.z = -controller.linear.x*(ax*sin(Current_position.pose.pose.orientation.z)+ay*cos(Current_position.pose.pose.orientation.z))+controller.linear.y*(ax*cos(Current_position.pose.pose.orientation.z)-ay*sin(Current_position.pose.pose.orientation.z));
	
	return controller;
}



bool STOP = true;			// a flag to start/stop the motion of youbot
nav_msgs::Odometry Current_position;
geometry_msgs::Pose2D Goal_position;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "youbot_plane");
	ros::NodeHandle n;

	ros::Subscriber Goal_sub = n.subscribe("GoalPosition",100,&GoalPoseCallback);	// subscribe to rostopic GoalPosition
	ros::Subscriber Current_sub = n.subscribe("odom",100,&CurrentPoseCallback);	// subscribe to rostopic odom
	ros::Publisher Twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);	// publish to /cmd_vel
	ros::Rate loop_rate(10);

	control_point c1(0.37,0.1),c2(0.37,0.1),c3(0.37,0.1),c4(0.37,0.1);	// use 4 control points

	// hard code the vertices locations of all 4 obstacles
	polygon p1(2.0,3.0,2.0,2.0,3.0,2.0,3.0,3.0),p2(4.625,1.375,4.625,0.625,5.375,0.625,5.375,1.375),p3(2.75,5.25,2.75,4.75,3.25,4.75,3.25,5.25),p4(0,3.125,0,2.875,0.125,2.875,0.125,3.125);

	geometry_msgs::Twist input,control1,control2,control3,control4;	// control input for each control point
	potential pa1,pa2,pa3,pa4,pr1,pr2,pr3,pr4;	// attractive/repulsive potential for each control point
	
	while (ros::ok() && n.ok())
	{
		ros::spinOnce();
		if (STOP == false)
		{
			// original names are too long, use shorthand notation here
			float x,y,theta;
			x = Current_position.pose.pose.position.x;
			y = Current_position.pose.pose.position.y;
			theta = Current_position.pose.pose.orientation.z;
 
			// update control point positions in the world frame
			c1.set_control_point(x+0.3*cos(theta)-0.2*sin(theta),y+0.3*sin(theta)+0.2*cos(theta));
			c2.set_control_point(x+0.3*cos(theta)+0.2*sin(theta),y+0.3*sin(theta)-0.2*cos(theta));
			c3.set_control_point(x-0.3*cos(theta)+0.2*sin(theta),y-0.3*sin(theta)-0.2*cos(theta));
			c4.set_control_point(x-0.3*cos(theta)-0.2*sin(theta),y-0.3*sin(theta)+0.2*cos(theta));
			
			// calculate the attractive potential for each control point
			pa1 = c1.attractive(Goal_position);
			pa2 = c2.attractive(Goal_position);
			pa3 = c3.attractive(Goal_position);
			pa4 = c4.attractive(Goal_position);
			
			// calculate the attractive potential for each control point
			pr1 = c1.repulsive(p1,p2,p3,p4);
			pr2 = c2.repulsive(p1,p2,p3,p4);
			pr3 = c3.repulsive(p1,p2,p3,p4);
			pr4 = c4.repulsive(p1,p2,p3,p4);

			// convert the potential to velocity input
			control1 = c1.controller(Current_position,0.3,0.2,pa1,pr1);
			control2 = c2.controller(Current_position,0.3,-0.2,pa2,pr2);
			control3 = c3.controller(Current_position,-0.3,-0.2,pa3,pr3);
			control4 = c4.controller(Current_position,-0.3,0.2,pa4,pr4);

			// youbot overall velocity will be the sum of velocity inputs of all four control points
			input.linear.x = control1.linear.x+ control2.linear.x+control3.linear.x+control4.linear.x;
			input.linear.y = control1.linear.y+ control2.linear.y+control3.linear.y+control4.linear.y;

			//input.angular.z = control1.angular.z+control2.angular.z+control3.angular.z+control4.angular.z;
			
			//if(input.linear.x>1) input.linear.x = 1; if(input.linear.x<-1) input.linear.x = -1; 
			//if(input.linear.y>1) input.linear.y = 1; if(input.linear.y<-1) input.linear.y = -1; 
			//if(input.angular.z>1) input.angular.z = 1; if(input.angular.z<-1) input.angular.z = -1;
 
			Twist_pub.publish(input);	// publish the control input to youbot
			printf("Position x: %f,Position y: %f \n",Current_position.pose.pose.position.x,Current_position.pose.pose.position.y);
		}	
		else
		{
			printf("Waiting...\n");
		} 
		loop_rate.sleep();
	}

	ros::spin();
	return 0;

}

void GoalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)	
{
	STOP = false;
	Goal_position.x = msg->x;
	Goal_position.y = msg->y;
	
}
void CurrentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	Current_position.pose.pose.position.x = msg->pose.pose.position.x;
	Current_position.pose.pose.position.y = msg->pose.pose.position.y;
	Current_position.pose.pose.orientation.z = msg->pose.pose.orientation.z;
}
