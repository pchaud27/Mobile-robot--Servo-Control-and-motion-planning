#!/usr/bin/env python

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2
import math
 
# global constants
angle_eps = 0.2
dis_eps = 0.01

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        
        #print "range[90]: ", msg.ranges[90]


# divide robot motion in 3 scenario
state_dict = {
    0: 'go to goal',
    1: 'Follow the wall',
}
# define initial scenario
state = 0


def main():
    global pub

    # initialize ROS node
    rospy.init_node("bug_1")
    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(20)

    # Set the goal point 
    goal = Point()
    goal.x = 0
    goal.y = 5.0

    # arbitrary far away coordinate from goal
    closest_point = Point()
    closest_point.x = 1000
    closest_point.y = 1000

    # arbitrary large number representing inf distance to goal
    closest_dist = 1000 
    # Variable that stores the coordinate of hit point when you 
    # encounter obstacle for the first time
    hit_point = Point()
    hit_point.x = 0
    hit_point.y = 0
    d = 0.4
    state = 0
    count_time = 0
    count_loop_ = 0
    
    while not rospy.is_shutdown():
        # initialize speed as Twist topic
        

        # TODO:

        # Decide what to do for the robot in each of these states:


        if state == 0:
            # go to goal state.  
            # TODO:
            low_eq = math.sqrt(pow(goal.y, 2) + pow(goal.x, 2))
            # find the x,y distance to the goal from current position
            inc_x = goal.x - odom.x
            inc_y = goal.y - odom.y
     	
     	    # find angle of the goal point wrt GLOBAL frame 
            angle_to_goal = atan2(inc_y,inc_x)
        
            # find the heading angle difference
            angle_diff = angle_to_goal - odom.theta
            # find the distance to the goal from current location
            dist_diff = math.sqrt(((inc_x)*(inc_x))+((inc_y)*(inc_y)))
             
     	    # TODO: 
	    if abs(dist_diff) > 0.050:
     	        if abs(angle_diff) > 0.1:
     		    speed.linear.x = 0.0
     		    speed.angular.z = 0.3
     	    	else:
     		    speed.linear.x = 0.5
     		    speed.angular.z = 0.0
	    else:
	        speed.linear.x = 0.0
	        speed.angular.z = 0.0

            #As the robot encounters the obstacle it will note the hit point and switch to state 1
            if scan.region['front']< 0.2 :
                hit_point.x = odom.x
		hit_point.y = odom.y
                count_time = 0
                state = 1   

            
            print "current state: ", state_dict[state]



        elif state == 1:
            # Follow obstacle until it encounters the line of path. 
            if scan.region['front']< d:
            	speed.linear.x = 0.0
            	speed.angular.z = 0.25
            elif scan.region['front']> d and scan.region['fleft']> d and scan.region['fright']> d and scan.region['right']> d:
            	speed.linear.x = 0.02
            	speed.angular.z = -0.25
            elif scan.region['front']> d and scan.region['fleft']> d and scan.region['fright']< d and scan.region['right']< d:
            	speed.linear.x = 0.25
            	speed.angular.z = 0.0
            #Formuala for distance of a point from the line eq1/eq2
	    eq1 = math.sqrt(pow(goal.y, 2) + pow(goal.x, 2))
            eq2 = math.fabs((goal.y) * odom.x - (goal.x) * odom.y + (goal.x * 0) - (goal.y * 0))
            #The condition for dist of 2nd hit point to be less tan the first hit point
            dist_hit_point_to_goal = math.sqrt((hit_point.x-goal.x)**2 + (hit_point.y-goal.y)**2)
            dist_current_to_goal = math.sqrt((odom.x-goal.x)**2 + (odom.y-goal.y)**2)
	   
	    if count_time > 10 and eq2/eq1 < 0.1 and dist_current_to_goal<dist_hit_point_to_goal :
                state = 0 
            #If robot fails to reach the goal
            if count_time > 20 and odom.x == hit_point.x and odom.y == hit_point.y:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
            print "current state: ", state_dict[state]

        #Count the time so that as the robot reaches hit_point it does not change the state immediately hence a delay condition
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_time = count_time + 1 
            count_loop_ = 0
        print"count_time: ",count_time   

            
        print scan.region
        pub.publish(speed)
        rate.sleep()

# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
