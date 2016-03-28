#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from math import sin, cos

# shapely is python library for geometry
from shapely.geometry import box, LineString, Point, LinearRing

# hard coded to make life easier, can be dynamically set easily
GOAL_POS = Point((5,5))

# format - tuple of obstacle and threshold distance (Q*_i)
obs_1 = (box(2,2,3,3), .75)
obs_2 = (box(5.0-(.75/2),1.0-(.75/2),5.0+(.75/2), 1.0+(.75/2)), .2)
obs_3 = (box(2.75,4.75, 3.25, 5.25), .2)
obs_4 = (box(0.0-(.25/2), 3.0-(.25/2), 0.0+(.25/2), 3.0+(.25/2)), .2)
wall_1 = (LineString([(-1,6),(6,6)]), .2)
wall_2 = (LineString([(-1,6),(-1,-1)]), .2)
wall_3 = (LineString([(6,6), (6,-1)]), .2)
wall_4 = (LineString([(6,-1), (-1,-1)]), .2)

obstacles = [obs_1, obs_2, obs_3, obs_4, wall_1, wall_2, wall_3, wall_4]

class YouBot(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        self.pos = None
        rospy.init_node('mp2')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z 
        # control point updating
        # 3 control points: 1 in center and 2 corners, but 2 is enough
        self.pos = Point((x, y))
        # this is taken from Yinan's code
        c1 = Point(x+.3*cos(theta)-.2*sin(theta),
                y+.3*sin(theta)+.2*cos(theta))

        c2 = Point(x+.3*cos(theta)+.2*sin(theta),
                y+.3*sin(theta)-.2*cos(theta))

        # format - tuple of each control point's position, zeta, and eta
        self.control_points = [
            (self.pos, .2, 1),
            (c1, .2, 1),
            (c2, .2, 1)
        ]

    def compute_attractive_ws(self, control_point):
        point, zeta, _ = control_point
        switch_thresh = 2 #if <2 units away from goal, switch to quad potential
        d2goal = point.distance(GOAL_POS)
        
        # this taken straight from book
        # have to multiply by -1 because vector points away from goal

        # quadratic potential if clos to goal
        if d2goal <= switch_thresh:
            return Point((-1*zeta*(point.x-GOAL_POS.x),
                          -1*zeta*(point.y-GOAL_POS.y)))
        # conic potential otherwise
        else:
            return Point(((-1*switch_thresh*zeta*(point.x-GOAL_POS.x))/d2goal,
                           -1*(switch_thresh*zeta*(point.y-GOAL_POS.y))/d2goal))

    def compute_repulsive_ws(self, control_point, obstacle):
        point, _, eta = control_point
        # need to unpack obstacle tuple into polygon and threshold
        obstacle_poly = obstacle[0]
        obstacle_thresh = obstacle[1]

        d2obstacle = point.distance(obstacle_poly)
        # this is straight from book
        if d2obstacle > obstacle_thresh:
            return Point(0,0)
        else:
            # scalar is length of vector that points away from closest obstacle
            # point
            scalar = eta * ((obstacle_thresh**-1)-(d2obstacle**-1)) * (d2obstacle**-2)
            # construct gradient vector

            # find closest point, you can ignore the details Yinan
            pol_ext = obstacle_poly
            if obstacle_poly.geom_type != 'LineString':
                pol_ext = LinearRing(obstacle_poly.exterior.coords)
            d = pol_ext.project(point)
            p = pol_ext.interpolate(d)
            # closest point
            c = Point(list(p.coords)[0])
            dqc = c.distance(point)
            # from book, formula for delta vector
            delta_d_i = Point(((point.x-c.x)/dqc, (point.y-c.y)/dqc))

            return Point((-1*delta_d_i.x*scalar, -1*delta_d_i.y*scalar))

    # helper
    def sum_forces(self, forces_list):
        fx = 0
        fy = 0
        for force in forces_list:
            fx += force.x
            fy += force.y
        return Point((fx, fy))

    def run(self):
        loop_rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.pos is not None:
                # computes attractive potential for each control point, stores
                # in list in order of control points
                attractive_forces = map(self.compute_attractive_ws,
                                        self.control_points)
                attractive_force = self.sum_forces(attractive_forces)

                # repulsive forces is more complicated, requires nested sum
                repulsive_forces = []
                for cp in self.control_points:
                    # for each cp, find repulsive forces from each obstacle
                    forces = [self.compute_repulsive_ws(cp, obstacle) for obstacle
                            in obstacles]
                    repulsive_forces.append(forces)
                # second sum, sum repulsion for each control point and then sum
                # all those sums
                repulsive_force = self.sum_forces(map(self.sum_forces,
                    repulsive_forces))
#                print 'attractive - %s' % attractive_force
#                print 'repulsive - %s' % repulsive_force
                # now we have attractive and repulsive forces for each cp
                vel = Twist()
                vel.linear.x = attractive_force.x + repulsive_force.x
                vel.linear.y = attractive_force.y + repulsive_force.y
                vel.linear.z = 0
                # and done!
                self.vel_pub.publish(vel)

            loop_rate.sleep()

if __name__ == '__main__':
    y = YouBot()
    y.run()
