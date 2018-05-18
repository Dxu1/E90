import rospy
import numpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transform2d
import math
import argparse
import pprint
import plan
#import pts_alg as alg
import alg
import graph
import cv2

#global metrics
CONTROL_PERIOD = 0.4 #controller interval
STEP_SIZE = rospy.Duration(CONTROL_PERIOD) 
P_GAIN = 0.4 #gain for proportional controller
I_GAIN = 0.2 #gain for integral controller (outer loop)
PLANT_GAIN = 0.444 #plant model
F2M = 0.3048
P2F = 10

class Controller():
    def __init__(self,img,sx,sy,gx,gy):
	# initiliaze
        rospy.init_node('PathFollow', anonymous=False)

	#parameter initialization
	self.start_pose = None
	self.cur_pose = None
	self.rel_pose_prev = None
	#for smith predictor
	self.e1_lag = None
	self.w_lag = None
	self.w_lag2 = None
	#for KI outer loop
	self.ea_lag = None
	self.wd_lag = None
	#for state machine
	self.state = "approach"

	#test path
	self.pts = path_find(img,sx,sy,gx,gy)
	print(self.pts)
	for i in self.pts:
	    i[0] = p2m(i[0])
	    i[1] = p2m(i[1])

	#rospy.loginfo("pts={}".format(self.pts))
	self.cx = self.pts[0][0]
	self.cy = self.pts[0][1]
	self.nx = self.pts[1][0]
	self.ny = self.pts[1][1]

	#subscriptions
	self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
	'''how to control time interval for publishing velocity'''
	self.odom = rospy.Subscriber('odom', Odometry, self.odom_callback)
	#controller step time is 0.4
	rospy.Timer(STEP_SIZE, self.control_callback)

        # tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")
        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

    def odom_callback(self,data):
	self.cur_pose = transform2d.transform2d_from_ros_pose(data.pose.pose)
        #rospy.loginfo(self.cur_pose)


    def control_callback(self, timer_event=None):
	#find position change
	if self.cur_pose is not None:	
	    if (self.start_pose == None):
	        self.start_pose = self.cur_pose.copy()
	    rel_pose = self.start_pose.inverse() * self.cur_pose
	else:
	    rel_pose = None

	#no waypoint = end
	if len(self.pts) <= 0:
	    self.state = 'stop'

	#define next waypoint
	self.nx = self.pts[1][0]
	self.ny = self.pts[1][1]


	#initialize velocities
	# Twist is a datatype for velocity
	move_cmd = Twist()

	#state determination
	dist = dist_diff(rel_pose.y, rel_pose.x, self.nx, self.ny)
	
	if dist < 0.18:
	    self.state = 'pause'

	if self.state == 'approach':
            move_cmd.linear.x = 0.1

	if self.state == 'pause':
            move_cmd.linear.x = 0.05
	    self.cx = self.nx
	    self.cy = self.ny
	    self.pts.pop(0)
	    self.state = 'approach'
	    #print(self.pts)
	
	if self.state == 'stop':
	    move_cmd.linear.x = 0
	    rospy.loginfo('reach the end')

	#compute desired angle from position change
	#theta_desired = angle_diff(rel_pose.x, rel_pose.y, self.nx, self.ny)
	theta_desired = angle_diff(self.cx, self.cy, self.nx, self.ny)


	#compute actual angular velocity
	if self.rel_pose_prev is not None:
	    angle_actual = rel_pose.theta - self.rel_pose_prev.theta
	    w_actual = angle_actual / CONTROL_PERIOD
	else:
	    w_actual = 0

	#PI Controller to computer desired angular velocity
	angle_error = (math.pi * 0.5 - theta_desired) - rel_pose.theta 
	w_desired = P_GAIN * angle_error + I_GAIN * angle_integrator(self.ea_lag, angle_error, self.wd_lag)


	#use smith predictor to find command angular velocity
	e1 = w_desired - w_actual

	w_command = smith_predictor(e1, self.e1_lag, self.w_lag, self.w_lag2)

	#pass value to next time step
	#for actual angular velocity	
	self.rel_pose_prev = rel_pose
	#for smith predictor
	self.w_lag2 = self.w_lag
	self.w_lag = w_command
	self.e1_lag = e1
	#for outer loop control
	self.wd_lag = w_desired
	self.ea_lag = angle_error

	#publish velocities
	move_cmd.angular.z = w_command
	self.cmd_vel.publish(move_cmd)


	#check output with rospy loginfo
	#rospy.loginfo('orientation={}'.format(rel_pose.theta))
	#rospy.loginfo('rel_prev={}'.format(self.rel_pose_prev))
	#rospy.loginfo('rel={}'.format(rel_pose))
	#rospy.loginfo('nx={}'.format(self.nx))
	#rospy.loginfo('ny={}'.format(self.ny))
	rospy.loginfo('dist={}'.format(dist))
	#rospy.loginfo('theta_desired={}'.format(theta_desired))
	#rospy.loginfo('w_desired={}'.format(w_desired))
	#rospy.loginfo('w_actual={}'.format(w_actual))
	#rospy.loginfo('w_command={}'.format(move_cmd.angular.z))
	#rospy.loginfo('nx={}'.format(self.nx))
	#rospy.loginfo('ny={}'.format(self.ny))
	#rospy.loginfo('state={}'.format(self.state))
	#rospy.loginfo('pts={}'.format(self.pts))

    def run(self):
	rospy.spin()
	


#functions to be called
def smith_predictor(e1, e1_lag, w_lag, w_lag2):
    Ct1 = 1 + 0.5*PLANT_GAIN*CONTROL_PERIOD
    Ct2 = 0.5*CONTROL_PERIOD
    if w_lag2 is None:
	# n=1 case
	if e1_lag is not None and w_lag is not None:
	    w = (Ct2*(e1+e1_lag)+w_lag)/Ct1
	# n=0 case
	else:
	    w = Ct2*e1/Ct1
    # n >= 2 case
    else:
	w = (Ct2*(e1+e1_lag+PLANT_GAIN*w_lag2)+w_lag)/Ct1
    return w


def angle_diff(x1,y1,x2,y2):
    angle_diff = math.atan2((y2-y1),(x2-x1))

    return angle_diff

def dist_diff(x1,y1,x2,y2):
    dist_diff = math.sqrt((x2-x1)**2+(y2-y1)**2)

    return dist_diff

def path_find(img,sx,sy,gx,gy):
    # read image
    img = cv2.imread(img)

    # Dijkstra based on image pixel
    pts, costDict = plan.plan_path(img,sx,sy,gx,gy)
    for i in range(len(pts)):
	pts[i] = pts[i].split(",")
	pts[i][0] = int(pts[i][0])
	pts[i][1] = int(pts[i][1])

    # point agglomeration to get rid of unnecessary waypoints
    newpts = alg.reducePoints(pts)
    newerpts = alg.mergePoints(newpts)

    return newerpts

def angle_integrator(ea_lag, ea, wd_lag):
    if ea_lag is not None and wd_lag is not None:
	wd_i = wd_lag + 0.5*CONTROL_PERIOD*(ea_lag + ea)
    else:
	wd_i = 0.5 * CONTROL_PERIOD * ea

    return wd_i

def p2m(x):
    x = (float(x)/P2F)*F2M

    return x



if __name__ == '__main__':

    #input
    parser = argparse.ArgumentParser()
    parser.add_argument("img", help="Path on Image")
    parser.add_argument("sx", help="Starting position in x", type=int)
    parser.add_argument("sy", help="Starting position in y", type=int)
    parser.add_argument("gx", help="Goal position in x", type=int)
    parser.add_argument("gy", help="Goal position in y", type=int)
    args = parser.parse_args()

    pprint.pprint(args)

    img = args.img
    sx = args.sx
    sy = args.sy
    gx = args.gx
    gy = args.gy

    try:
    	p=Controller(img,sx,sy,gx,gy)
    	p.run()
    except:
        rospy.loginfo("terminated.")
