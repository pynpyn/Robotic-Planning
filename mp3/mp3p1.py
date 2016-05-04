#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from rrt import *

INIT = (0.,0.)
GOAL = (5., 5.)
EPSILON = .5

fullRRT = mergeRRT(buildRRT(INIT, 30), buildRRT(GOAL, 30), 25)
path = shortest_path_djk(fullRRT, INIT, GOAL)
# Yinan, uncomment line below to see planned trajectory
# if you uncomment, robot will not move until you close the window
#plot_RRT(fullRRT, path)

def _vector_from_to(p1, p2):
    return (EPSILON*(p2[0]-p1[0]),EPSILON*(p2[1]-p1[1]))

class YouBot(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        self.pos = None
        rospy.init_node('mp3')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pos = (x, y)
    
    def run(self):
        i = 1
        curr_goal = path[i]
        loop_rate = rospy.Rate(5)
        print "starting"
        while not rospy.is_shutdown():
            if self.pos is not None:
                if Point(self.pos).distance(Point(curr_goal)) >= .05:
                    vel = _vector_from_to(self.pos, curr_goal)
                    vel_msg = Twist()
                    vel_msg.linear.x = vel[0]
                    vel_msg.linear.y = vel[1]
                    vel_msg.linear.z = 0
                    self.vel_pub.publish(vel_msg)
                elif curr_goal == GOAL:
                    print "done"
                    return
                else:
                    i += 1
                    curr_goal = path[i]
            loop_rate.sleep()
        print "stopping"

if __name__ == '__main__':
    y = YouBot()
    y.run()
