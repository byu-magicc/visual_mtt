#!/usr/bin/env python

import os, subprocess, signal

import rospy
import dynamic_reconfigure.client

from visual_mtt.msg import Stats

class ROSLauncher(object):
    """ROSLauncher

        The object that actually interfaces with the roslaunch shell command.
    """
    def __init__(self, flags):
        super(ROSLauncher, self).__init__()
        
        self.flags = flags

        # Store the roslaunch process
        self.process = None
        

    def run(self):
        """Launch

            Create a subprocess that runs `roslaunch visual_mtt ...`

            See: https://stackoverflow.com/a/4791612/2392520
        """
        cmd = 'roslaunch visual_mtt play_from_recording.launch {} >/dev/null'.format(self.flags)
        print("Running command: {}".format(cmd))
        self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)


    def stop(self):
        # Send a kill signal to all the process groups
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)






class Scenario(object):
    """Scenario

        A Benchmark runs multiple Scenarios. The goal of each scenario is to
        stress test the visual_mtt algorithm under different conditions. For
        example, possible scenarios are:

            - No camera motion, high altitude (homographies should do well)
            - No camera motion, low altitude (homographies should do poorly)
            - Moving platform, high altitude
            - Moving platform, low altitude
            - live camera (test data throughput speeds with algorithm)


        A Scenario is meant to be run multiple times with different parameters.
        Variations in parameters include:

            - frame stride
            - max number of features



    """
    def __init__(self, name, desc, **kwargs):
        super(Scenario, self).__init__()
        self.name = name
        self.desc = desc

        # Build the roslaunch flags
        self.flags = ''

        if 'bag_path' in kwargs:
            self.flags += ' bag_path:={}'.format(kwargs['bag_path'])

        if 'bag_topic' in kwargs:
            self.flags += ' bag_topic:={}'.format(kwargs['bag_topic'])

        if 'cam_info' in kwargs:
            self.flags += ' camera_info_url:=file://{}'.format(kwargs['cam_info'])

        if 'flags' in kwargs:
            self.flags += ' {}'.format(kwargs['flags'])


    def run(self, frame_stride):
        """Run
        """
        self.launch = ROSLauncher(self.flags)
        self.launch.run()

        # Dynamic Reconfigure client
        client = dynamic_reconfigure.client.Client('visual_frontend')

        # Update the frame stride
        params = { 'frame_stride': frame_stride }
        config = client.update_configuration(params)


    def stop(self):
        self.launch.stop()
        
        


class BenchmarkRunner(object):
    """BenchmarkRunner
        
        Create an object that runs the benchmark.

    """
    def __init__(self, scenario, frame_strides):
        super(BenchmarkRunner, self).__init__()

        self.scenario = scenario
        self.frame_strides = frame_strides

        # Store each stats message
        self.stats = []

        # Hook up ROS subscribers
        rospy.Subscriber('/stats', Stats, self._handle_stats)


    def _handle_stats(self, msg):
        self.stats.append(msg)

    def _wait_for_end(self):
        while not rospy.is_shutdown():

            rospy.spin()

    def run(self):
        
        for fs in self.frame_strides:
            self.scenario.run(fs)
            self._wait_for_end()
            self.scenario.stop();
        

if __name__ == '__main__':

    rospy.init_node('benchmark', anonymous=False)

    # # Create a Benchmark Runner. It can be run for both CUDA and non-CUDA.
    # benchmark = BenchmarkRunner({
    #                 'frame_stride': [1, 2, 3, 4, 5],
    #                 'max_features': [100, 200, 300, 400, 500]
    #             })

    # benchmark.run()

    frame_strides = [1, 2, 3, 4, 5]
    frame_strides = [1]


    #
    # Benchmark Scenario 1
    # 

    # Define the scenario
    name = 'Kiwanis Frisbee'
    desc = 'moving platform, low altitude'
    bag = '/home/plusk01/Documents/bags/kiwanis/kiwanis_short.bag'
    topic = '/image_flipped'
    cam_info = '/home/plusk01/Documents/bags/kiwanis/creepercam.yaml'
    flags = 'compressed:=true has_info:=false'
    s1 = Scenario(name, desc, bag_path=bag, bag_topic=topic, cam_info=cam_info, flags=flags)

    benchmark = BenchmarkRunner(s1, frame_strides)
    benchmark.run()

    #
    # Benchmark Scenario 2
    # 

    # Define the scenario
    name = 'Kiwanis Frisbee'
    desc = 'moving platform, low altitude'
    bag = '/home/plusk01/Documents/bags/kiwanis/kiwanis_short.bag'
    topic = '/image_flipped'
    cam_info = '/home/plusk01/Documents/bags/kiwanis/creepercam.yaml'
    flags = 'compressed:=true has_info:=false'
    s2 = Scenario(name, desc, bag_path=bag, bag_topic=topic, cam_info=cam_info, flags=flags)

    # benchmark = BenchmarkRunner(s2, frame_strides)
    # benchmark.run()