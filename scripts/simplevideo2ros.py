#!/usr/bin/env python

""" simpleVideo2ros.py - Version 1.1 2013-12-20

    Read in a recorded video file and republish as a ROS Image topic.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class SimpleVideo2ROS:
    def __init__(self):
        rospy.init_node('simplevideo2ros', anonymous=False)

        rospy.on_shutdown(self.cleanup)

        """ Define the input (path to video file) as a ROS parameter so it
            can be defined in a launch file or on the command line """
        self.input = rospy.get_param("~input", "")
        print(self.input)

        """ Define the image publisher with generic topic name "output" so that it can
            be remapped in the launch file. """
        image_pub = rospy.Publisher("output", Image, queue_size='None')

        # The target frames per second for the video
        self.fps = rospy.get_param("~fps", 30)

        self.capture = cv2.VideoCapture(self.input)

        # Create the CvBridge object
        bridge = CvBridge()

        # Enter the main processing loop
        while not rospy.is_shutdown():
            # Get the next frame from the video
            frame = self.get_frame()

            # Convert the frame to ROS format
            try:
                image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError, e:
                print e

            time.sleep(1.0/self.fps)

    def get_frame(self):
        ret, frame = self.capture.read()
        if frame is None:
            self.capture = cv2.VideoCapture(self.input)
            ret, frame = self.capture.read()

        return frame

    def cleanup(self):
            print "Shutting down video2ros node."
            cv2.destroyAllWindows()

def main(args):

    try:
        v2r = SimpleVideo2ROS()
    except KeyboardInterrupt:
        print "Shutting down simpleVideo2ros..."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
