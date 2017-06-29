#!/usr/bin/env python

import sys, argparse
import time, datetime, pickle
import os, subprocess, signal

import rospy, rosbag
import dynamic_reconfigure.client

from colorama import init; init(autoreset=True)
from colorama import Fore, Style

import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage, Image
from visual_mtt.msg import Stats

def static_vars(**kwargs):
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate

@static_vars(utilization=0)
def get_utilization(idx, data, stride, fps):
    """Get Utilization
    """

    # Reset utilization when starting a new run
    if idx == 0:
        get_utilization.utilization = 0

    # How much time do I have available for computation?
    t_available = (1.0/fps)*stride

    # How much computation was performed?
    t_computation = 0
    for val in data.values():
        t_computation += val

    # Find the LPF alpha -- converge quickly during the first 30
    alpha = 0.95 if idx < 30 else 1.0/( 1.5/t_available + 1)

    # Compute utilization
    get_utilization.utilization = alpha*(t_computation/t_available) + (1-alpha)*get_utilization.utilization

    return get_utilization.utilization

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
        if not rospy.is_shutdown():
            cmd = 'roslaunch visual_mtt play_from_recording.launch {} > /dev/null 2>&1'.format(self.flags)
            # print("Running command: {}".format(cmd))
            self.process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

        else:
            print("rospy shutting down, could not run roslaunch command")


    def stop(self):
        # Send a kill signal to all the process groups
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

        # Wait a second to let roslaunch cleanly exit
        time.sleep(1.5)



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
        self.kwargs = kwargs

        self.bag_path = ''
        self.bag_topic = ''
        self.bag_topic_type = Image

        self.fps = 0
        
        # Build the roslaunch flags
        self.duration = 0
        self.flags = ''

        if 'bag_path' in kwargs:
            self.bag_path = kwargs['bag_path']
            self.flags += ' bag_path:={}'.format(kwargs['bag_path'])
            bag = rosbag.Bag(kwargs['bag_path'])
            self.duration = round(bag.get_end_time() - bag.get_start_time(), 2)

        if 'bag_topic' in kwargs:
            self.flags += ' bag_topic:={}'.format(kwargs['bag_topic'])
            self.bag_topic = kwargs['bag_topic']

        if 'duration' in kwargs:
            self.duration = kwargs['duration']

        if 'cam_info' in kwargs:
            self.flags += ' camera_info_url:=file://{}'.format(kwargs['cam_info'])

        if 'compressed' in kwargs:
            self.flags += ' compressed:=true'
            self.bag_topic_type = CompressedImage
            self.bag_topic += '/compressed'

        if 'flags' in kwargs:
            self.flags += ' {}'.format(kwargs['flags'])

        if 'silent' in kwargs and kwargs['silent']:
            self.flags += ' tuning:=false show_tracks:=false hide_bag:=true'

        if 'fps' in kwargs:
            self.fps = float(kwargs['fps'])


    def _wait_for_end(self):
        """Wait for end

            This makes the subprocess call blocking -- we don't move on
            until one roslaunch scenario is finished. We know it is
            finished based on duration, which is either calculated from
            the rosbag or given as a parameter when the scenario was
            set up by the user.
        """
        start = time.time()
        while not rospy.is_shutdown():

            # Calculate the elapsed time
            elapsed = time.time() - start

            # Print out our progress
            sys.stdout.write('\tProgress:\t%0.2f/%0.2f s \t [%0.1f%%]\r' % (elapsed, self.duration, elapsed/self.duration*100) )
            sys.stdout.flush()

            if elapsed > self.duration:
                print
                break


    def run(self, frame_stride):
        """Run

            Launch the visual_mtt launch file. Then, use Dynamic Reconfigure
            to set the frame stride for the desired variation on the
            scenario. After waiting for the end of the scenario, kill the
            roslaunch command.
        """
        self.launch = ROSLauncher(self.flags)
        self.launch.run()

        # Dynamic Reconfigure client
        client = dynamic_reconfigure.client.Client('visual_frontend')

        # Update the frame stride
        params = { 'frame_stride': frame_stride }
        config = client.update_configuration(params)

        # Wait until we should stop the scenario
        self._wait_for_end()

        # Kill the launch/node
        self.launch.stop()


    def serialize(self):
        return {
            'name': self.name,
            'desc': self.desc,
            'opts': self.kwargs
        }
        


class BenchmarkRunner(object):
    """BenchmarkRunner
        
        Create an object that runs the benchmark.

    """
    ITERS = 3
    def __init__(self, scenario, frame_strides):
        super(BenchmarkRunner, self).__init__()

        self.scenario = scenario
        self.frame_strides = frame_strides
        self.current_iter = 0

        # the last frame/video message received
        self.fps = scenario.fps
        self.last_frame_time = 0
        self.first_seq = 0

        # Store each stats message each run
        self.runs = [] # { stride: 1, fps: 30, iter1: [], iter2: [], ... }

        # Hook up ROS subscribers
        rospy.Subscriber('/stats', Stats, self._handle_stats)


    def _handle_stats(self, msg):
        # Put the data in a dictionary
        data = {
            't_feature_manager': msg.t_feature_manager,
            't_homography_manager': msg.t_homography_manager,
            't_measurement_generation': msg.t_measurement_generation,
        }

        # Add the data to the iteration in the run
        run = self.runs[-1]
        run['iter%i'%self.current_iter].append(data)


    def run(self):

        # Calculate total seconds
        t_secs = self.scenario.duration * BenchmarkRunner.ITERS * len(self.frame_strides)

        print
        print
        print(Fore.BLUE + Style.DIM + '='*80)
        print(Fore.YELLOW + Style.BRIGHT + 'Benchmark Scenario: ' + Style.DIM + self.scenario.name)
        print(Fore.BLUE + Style.DIM + '='*80)
        print('Description:\t{}'.format(self.scenario.desc))
        print('Bag Path:\t{}'.format(self.scenario.bag_path))
        print('Iterations:\t{}'.format(BenchmarkRunner.ITERS))
        print('Frame Strides:\t{}'.format(self.frame_strides))
        print('Duration:\t{} seconds each run, {} seconds total'.format(self.scenario.duration, t_secs))
        print(Fore.BLUE + Style.DIM + '-'*80)

        
        #
        # Run the scenario and gather data
        #

        for fs in self.frame_strides:

            print(Style.BRIGHT + 'Frame Stride: ' + Style.DIM + str(fs))

            self.runs.append({
                    'stride': fs,
                    'fps': self.fps,
                    'iters': BenchmarkRunner.ITERS
                })

            # Average utilization results over ITERS iterations
            for i in xrange(BenchmarkRunner.ITERS):
                self.runs[-1]['iter%i'%i] = []
                self.current_iter = i

                # Blocking call to run
                self.scenario.run(fs)

        #
        # Process the data to compute utilization
        #

        for run in self.runs:
            fps = run['fps']
            stride = run['stride']
            for i in xrange(run['iters']):
                util = []
                for idx, data in enumerate(run['iter%s'%i]):
                    # Calculate the utilization
                    util.append( get_utilization(idx, data, stride, fps) )

                run['iter%iutil'%i] = util

        return self.runs


class BenchmarkAnalyzer(object):
    """BenchmarkAnalyzer"""
    def __init__(self):
        super(BenchmarkAnalyzer, self).__init__()

        self.data = [] # { scenario: s, results: [] }
        

    def _smooth_utilization(self, run):
        """Smooth Utilization

            Average the utillization across the iterations of
            the same scenario with the same stride.
        """

        util = np.zeros(len(run['iter0util']))

        iters = run['iters']
        for i in xrange(iters):

            newutil = np.array(run['iter%iutil'%i])

            # How long is this utilization array?
            N = len(newutil)

            # Shrink util array if needed
            if len(util) > N:
                util = util[0:N]

            util += newutil[0:len(util)]

        # Average
        util = util/float(iters)

        # Burn the first 30 data points
        return util[30:]




    def add(self, scenario, results):
        """Add

        """
        self.data.append({ 'scenario': scenario, 'results': results })


    def analyze(self):
        for data in self.data:
            for run in data['results']:
                util = self._smooth_utilization(run)
                
                plt.plot(util)
                plt.title(data['scenario'].name + ' w/ stride ' + str(run['stride']))
                plt.xlabel('R-RANSAC Iteration')
                plt.ylabel('Utilization')
                plt.axis([0, np.size(util), 0, max(1.5, np.max(util))])
                plt.show()


    def save(self, filename=None):
        # If no filename supplied
        if filename is None:
            today = datetime.datetime.now().strftime("%d%B%Y_%H%M%S")
            filename = 'benchmarks_{}.pickle'.format(today)

        # Add extension if necessary
        if '.' not in filename:
            filename += '.pickle'

        # Make sure it's a good filename
        filename = filename.lower().replace(' ', '_')

        dump = []
        for data in self.data:
            d = {
                'scenario': data['scenario'].serialize(),
                'results': data['results']
            }
            dump.append(d)

        with open(filename, 'wb') as f:
            pickle.dump(dump, f, protocol=pickle.HIGHEST_PROTOCOL)

    def load(self, filename):
        print(Fore.GREEN + "Opening {}".format(filename))

        with open(filename, 'rb') as f:
            dump = pickle.load(f)

        for data in dump:
            sopts = data['scenario']
            name = sopts['name']
            desc = sopts['desc']
            del sopts['name']
            del sopts['desc']
            s = Scenario(name, desc, **sopts)
            self.add(s, data['results'])


def run_benchmarks(args):
    rospy.init_node('benchmark', anonymous=False)

    # Create a benchmark data analyzer
    analyzer = BenchmarkAnalyzer()

    # Set up common parameters for scenarios
    frame_strides = [1, 2, 3]

    # =========================================================================

    #
    # Benchmark Scenario 1
    # 

    # Define the scenario
    name = 'Kiwanis Frisbee'
    desc = 'moving platform, low altitude'
    opts = {
        'bag_path': '/home/plusk01/Documents/bags/kiwanis/kiwanis_frisbee.split.bag',
        'bag_topic': '/image_flipped',
        'cam_info': '/home/plusk01/Documents/bags/kiwanis/creepercam.yaml',
        'flags': 'has_info:=false pub_output_img:=true',
        'compressed': True,
        'duration': 5,
        'silent': True,
        'fps': 30
    }
    s = Scenario(name, desc, **opts)

    # Benchmark the scenario and add the results
    benchmark = BenchmarkRunner(s, frame_strides)
    results = benchmark.run()
    analyzer.add(s, results)

    #
    # Benchmark Scenario 2
    # 

    # Define the scenario
    name = 'Kiwanis Variety'
    desc = 'moving platform, low altitude'
    opts = {
        'bag_path': '/home/plusk01/Documents/bags/kiwanis/kiwanis_variety.bag',
        'bag_topic': '/image_flipped',
        'cam_info': '/home/plusk01/Documents/bags/kiwanis/creepercam.yaml',
        'flags': 'has_info:=false pub_output_img:=true',
        'compressed': True,
        'duration': 60,
        'silent': True,
        'fps': 30
    }
    s = Scenario(name, desc, **opts)

    # Benchmark the scenario and add the results
    benchmark = BenchmarkRunner(s, frame_strides)
    results = benchmark.run()
    analyzer.add(s, results)

    # =========================================================================

    # Save and return the data analyzer
    analyzer.save(args['output'])
    return analyzer
        


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Benchmark visual_mtt algorithm using rosbag data')
    parser.add_argument('-a', '--analyze', help='Pickled benchmark results to analyze', required=False)
    parser.add_argument('-o', '--output', help='Filename of the benchmark results', required=False)
    args = vars(parser.parse_args())


    if args['analyze'] is not None:
        analyzer = BenchmarkAnalyzer()
        analyzer.load(args['analyze'])
        analyzer.analyze()
    else:
        analyzer = run_benchmarks(args)