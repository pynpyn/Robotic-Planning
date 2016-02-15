#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>


void GoalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
void CurrentPoseCallback(const turtlesim::Pose::ConstPtr& msg);
float AngError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position);
float DisError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position);

bool STOP = true;
turtlesim::Pose Current_position;
geometry_msgs::Pose2D Goal_position;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_turtle");
	ros::NodeHandle n;

	ros::Subscriber Goal_sub = n.subscribe("turtle1/GoalPosition",5,&GoalPoseCallback);
	ros::Subscriber Current_sub = n.subscribe("turtle1/pose",5,&CurrentPoseCallback);
	ros::Publisher Twist_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);
	ros::Rate loop_rate(10);

	ROS_INFO_STREAM("Ready to send position command");
	float *Error_Angle;
	float Error_Distance = 0;
	geometry_msgs::Twist controller;
	

	while (ros::ok() && n.ok())
	{
		ros::spinOnce();
		if (STOP == false)
		{
			Error_Angle = (float *) AngError(Current_position,Goal_position);
			Error_Distance = DisError(Current_position,Goal_position);
			printf("Angle Error: %f, Distnace Error: %f \n",*Error_Angle,Error_Distance);

			controller.linear.x = 0.5*Error_Distance;
			controller.angular.z = 0.5*(*(Error_Angle))+0.01*(*(Error_Angle+1))+0.01*(*(Error_Angle+2));

			Twist_pub.publish(controller);
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

float AngError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position)
{
	float angle_d = atan2f(Goal_position.y-Current_position.y,Goal_position.x-Current_position.x);
	static float error_angle[3] = {0,0,0}; 
	static float error_old = 0;

	error_angle[0] = angle_d - Current_position.theta;	// Proportional error
	error_angle[0] = atan2f(sin(error_angle),cos(error_angle)); 
	error_angle[2] = (error_angle[0]-error_old)/(0.1);	// derivative error
	error_angle[1] = error_angle[1]+error_angle[0]*(1.0/10); // integral error
	error_old = error_angle[0];
	
	return error_angle;

}

float DisError(turtlesim::Pose Current_position, geometry_msgs::Pose2D Goal_position)
{
	float error_distance = sqrt(pow(Goal_position.y-Current_position.y,2)+pow(Goal_position.x-Current_position.x,2));
	return error_distance;
}
