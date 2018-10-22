#!/usr/bin/env python

""" random_walk.py - Version 1.0 2018-10-18

    Control a simple mobile robot to do a random walk avoiding obstacles

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2018 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import numpy as np

class RandomWalk():
    def __init__(self):
        # Give the node a name
        rospy.init_node('random_walk', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
    
        # How fast will we update the robot's movement?
        rate = rospy.get_param('~rate', 5)
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)

        # Set the forward linear speed to 0.15 meters per second 
        linear_speed = rospy.get_param('~linear_speed', 0.1)
        
        # Set the rotation speed in radians per second
        angular_speed = rospy.get_param('~angular_speed', 0.5)
        
        obstacle_distance = rospy.get_param('~obstacle_distance', 0.3)

        # Publisher to control the robot's speed
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/twist', Twist, queue_size=5)
        
        self.sonar_sensors = ['sonar_front_left', 'sonar_front_center', 'sonar_front_right', 'sonar_rear_left', 'sonar_rear_center', 'sonar_rear_right']
        
        self.sonar = 0.0
        #self.ir = 0.0
        
#         for sonar in self.sonar_sensors:
#             rospy.wait_for_message('arduino/sensor/' + sonar, Range)
        
        #rospy.wait_for_message('arduino/sensor/ir_front_center', Range)
        
        self.suscribers = dict()
        self.sonar_range = dict()
        
        for sonar in self.sonar_sensors:
            self.sonar_range[sonar] = obstacle_distance + 1.0
            self.suscribers[sonar] = rospy.Subscriber('arduino/sensor/' + sonar, Range, self.get_sonar, [sonar])
            
        #self.ir_sub = rospy.Subscriber('arduino/sensor/ir_front_center', Range, self.get_front_ir)
        
        # Enter main loop to move the robot
        while not rospy.is_shutdown():
            cmd_vel_msg = Twist()
            
            go_straight = True
            
            for sonar in self.sonar_sensors:
                if "front" in sonar:
                    if (np.isfinite(self.sonar_range[sonar]) and self.sonar_range[sonar] < obstacle_distance):
                        go_straight = False
                        break
                    
            if go_straight:
                cmd_vel_msg.linear.x = linear_speed
            else:
                #if self.sonar_range['sonar_front_right'] < self.sonar_range['sonar_front_left']:
                cmd_vel_msg.angular.z = angular_speed
                #else:
                    #cmd_vel_msg.angular.z = angular_speed

            self.cmd_vel_pub.publish(cmd_vel_msg)
                
            r.sleep()
        
    def get_sonar(self, msg, sonar):
        self.sonar_range[sonar[0]] = msg.range
        
    def get_front_ir(self, msg):
        self.ir = msg.range

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    RandomWalk()

