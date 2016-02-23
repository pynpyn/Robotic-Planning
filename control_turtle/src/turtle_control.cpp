/* This C++ program will control our turtle to track a goal position. After user published a topic of goal position and orientation, this program will subscribe to that topic and also subscribe to turtle's current position and orientation. Then, PID control will track the goal position and drive the angle and distance errors to a sufficiently small amount. 

Usage:
1. Run roscore
2. Run turtlesim_node
3. Run control_turtle
4. publish a goal position topic 

*/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>


void GoalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);				// call back goal position
void CurrentPoseCallback(const turtlesim::Pose::ConstPtr& msg);					// call back current position
float *AngError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position);		// track angle error
float *DisError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position);		// track distance error

bool STOP = true;			// a flag to start/stop the motion of turtle
turtlesim::Pose Current_position;
geometry_msgs::Pose2D Goal_position;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_turtle");
	ros::NodeHandle n;

	ros::Subscriber Goal_sub = n.subscribe("turtle1/GoalPosition",5,&GoalPoseCallback);	// subscribe to rostopic turtle1/GoalPosition
	ros::Subscriber Current_sub = n.subscribe("turtle1/pose",5,&CurrentPoseCallback);	// subscribe to rostopic turtle1/pose
	ros::Publisher Twist_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);	// publish to turtle1/cmd_vel
	ros::Rate loop_rate(10);

	ROS_INFO_STREAM("Ready to send position command");
	float *Error_Angle;	// angle error pointer
	float *Error_Distance;  // distance error pointer
	// angle/distance error array is [P error, I error , D error]
	geometry_msgs::Twist controller;
	

	while (ros::ok() && n.ok())
	{
		ros::spinOnce();
		if (STOP == false)
		{
			Error_Angle = AngError(Current_position,Goal_position);		// return a pointer that points to angle error array
			Error_Distance = DisError(Current_position,Goal_position);	// return a pointer that points to distance error array
			printf("Angle Error: %f, Distnace Error: %f \n",*Error_Angle,*Error_Distance);

			controller.linear.x = 2*(*(Error_Distance))+0.005*(*(Error_Distance+1))+0.005*(*(Error_Distance+2)); 
			controller.angular.z = 10*(*(Error_Angle))+0.001*(*(Error_Angle+1))+0.001*(*(Error_Angle+2));

			Twist_pub.publish(controller);	// publish the control input to the turtle
			if (sqrt(pow(*Error_Angle,2))<=0.01 && sqrt(pow(*Error_Distance,2)) <= 0.01)	STOP = true;	// if the error is 												sufficiently small, then stops the turtle
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
void CurrentPoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
	Current_position.x = msg->x;
	Current_position.y = msg->y;
	Current_position.theta = msg->theta;
}

float *AngError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position)
// angle error array is [P error, I error , D error]
{
	float angle_d = atan2f(Goal_position.y-Current_position.y,Goal_position.x-Current_position.x);
	static float error_angle[3] = {0,0,0}; 
	static float error_old = 0;

	error_angle[0] = angle_d - Current_position.theta;	// Proportional error
	error_angle[0] = atan2f(sin(error_angle[0]),cos(error_angle[0])); 
	error_angle[2] = (error_angle[0]-error_old)/(0.1);	// derivative error
	error_angle[1] = error_angle[1]+error_angle[0]*(1.0/10); // integral error
	error_old = error_angle[0];
	
	return &error_angle[0];

}

float *DisError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position)
{
// distance error array is [P error, I error , D error]
	static float error_distance[3] = {0,0,0}; 
	static float error1_old = 0;
	error_distance[0] = sqrt(pow(Goal_position.y-Current_position.y,2)+pow(Goal_position.x-Current_position.x,2)); // proportional error
	error_distance[2] = (error_distance[0]-error1_old)/(0.1);		// derivative error
	error_distance[1] = error_distance[1]+error_distance[0]*(1.0/10); // integral error
	error1_old = error_distance[0];

	return &error_distance[0];
}
