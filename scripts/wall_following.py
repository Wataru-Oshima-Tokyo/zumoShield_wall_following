#! /usr/bin/env python3

# ROS imports
import rospkg
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

import math

#import time
class WALL_FOLLOW():
    def __init__(self):
        self.TIMEOUT = 0.5 
        self.last_clb_time_ = 0

        self.hz = 20
        self.inf = 5
        self.wall_dist = 0.5
        #max_speed = 0.3
        self.max_speed = 0.1
        self.p = 1
        self.d = 0
        self.angle = 1
        self.direction = -1

        self.e = 0
        self.angle_min = 0
        self.dist_front = 0
        self.diff_e = 0
        self.dist_min = 0

        self.RUN = 0

        self.pub_ = None
        # Sensor regions
        self.regions_ = {
                'bright': 0,
                'right': 0,
                'fright': 0,
                'front': 0,
                'left': 0,
        }

        rospy.init_node('reading_laser')
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        self.start_srv_ = rospy.Service('/wall_follow/start', Empty, self.clbk_start_service)
        self.stop_srv_ = rospy.Service('/wall_follow/stop', Empty, self.clbk_stop_service)

        rate = rospy.Rate(self.hz)

        last_clb_time_ = rospy.get_time()

        while not rospy.is_shutdown():
            msg = Twist()

            if self.RUN is 1:

                time_duration = rospy.get_time() - last_clb_time_
                if time_duration < self.TIMEOUT:
                    print("here")
                    msg = self.following_wall()

                self.pub_.publish(msg)
                rate.sleep()


    def clbk_start_service(self,req):
        print("start wall follow")
        self.RUN = 1
        return EmptyResponse()


    def clbk_stop_service(self,req):
        print("stop wall follow")
        self.RUN = 0
        return EmptyResponse()

    def clbk_laser(self,msg):

        self.last_clb_time_ = rospy.get_time()  

        size = len(msg.ranges)
        # print(size)
        # size 360 , 360deg

        index_n_126 = int(180 + 126)
        index_n_90 = int(180 + 90)
        index_n_54 = int(180 + 54)
        index_n_18 = int(180 + 18)
        index_18 = int(180 - 18)
        index_54 = int(180 - 54)
        index_90 = int(180 - 90)
        index_126 = int(180 - 126)

        if self.direction == -1:
            min_index = 180
            max_index = index_n_126
        elif self.direction == 1:
            min_index = index_126
            max_index = 180

        # Determine values for PD control of distance and P control of angle
        for i in range(min_index, max_index):
            if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.01:
                min_index = i
        self.angle_min = (min_index-size//2)*msg.angle_increment
        dist_min = msg.ranges[min_index]
        self.dist_front = msg.ranges[size//2]
        self.diff_e = min((dist_min - self.wall_dist) - self.e, 100)
        self.e = min(dist_min - self.wall_dist, 100)

        # Determination of minimum distances in each region
        self.regions_ = {
            'bright':  min(min(msg.ranges[index_n_90:index_n_126]), self.inf),
            'right': min(min(msg.ranges[index_n_54:index_n_90]), self.inf),
            'fright':  min(min(msg.ranges[index_n_18:index_n_54]), self.inf),
            'front':  min(min(msg.ranges[index_18:index_n_18]), self.inf),
            'fleft':   min(min(msg.ranges[index_54:index_18]), self.inf),
            'left':   min(min(msg.ranges[index_90:index_54]), self.inf),
            'bleft':   min(min(msg.ranges[index_126:index_90]), self.inf),
        }
        #rospy.loginfo(regions_)


        #print(round(regions_['bleft'],1),round(regions_['left'],1),round(regions_['fleft'],1),round(regions_['front'],1),round(regions_['fright'],1),round(regions_['right'],1),round(regions_['bright'],1))

    def following_wall(self):
        """
        PD control for the wall following state. 
        Returns:
                Twist(): msg with angular and linear velocities to be published
                        msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                        msg.angular.z -> PD controller response
        """
        msg = Twist()
        if self.dist_front < self.wall_dist:
            msg.linear.x = 0
        elif self.dist_front < self.wall_dist*2:
            msg.linear.x = 0.5*self.max_speed
        elif abs(self.angle_min) > 1.75:
            msg.linear.x = 0.4*self.max_speed
        else:
            msg.linear.x = self.max_speed

        #print("e:",e," angle_min:",angle_min)
        msg.angular.z = max(min(self.direction*(self.p*self.e+self.d*self.diff_e) + self.angle*(self.angle_min-((math.pi)/2)*self.direction), 0.2), -0.2)
        print ('Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x))
        return msg


if __name__ == '__main__':
    wf = WALL_FOLLOW() 
