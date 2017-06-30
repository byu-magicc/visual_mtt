#!/usr/bin/env python

import sys, argparse, socket
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
from visual_mtt.msg import Tracks

"""

    Data structure:
        - Each pickle file is called a benchmark.
        - A benchmark is made up of scenarios.
        - Each scenario is ran with varying frame strides.
        - each scenario/frame_stride combination is ran for
            BenchmarkRunner.ITERS iterations to average good data
            This combination is called a run
        - Each run ('iter0', 'iter1', etc) is a list of dicts in
            the following form:


"""


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

###############################################################################
###############################################################################

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

        # If you let it run unbounded, you slow down the ROS network!
        # rate = rospy.Rate(20)

        start = time.time()
        while not rospy.is_shutdown():

            # Calculate the elapsed time
            elapsed = time.time() - start

            # Print out our progress
            sys.stdout.write('\tProgress:    %0.2f/%0.2f s \t [%0.1f%%]\r' % (elapsed, self.duration, elapsed/self.duration*100) )
            sys.stdout.flush()

            if elapsed > self.duration:
                print
                break


            # Note that I'm using Python time instead of rospy time because
            # I was getting a 'Time Going Backward' error -- because this
            # Python script goes in and out of ROS land, this seems like
            # the best way to handle that
            time.sleep(0.10) # 10 Hz
            # rate.sleep()


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

        # time.sleep(20)

        # Kill the launch/node
        self.launch.stop()


    def serialize(self):
        return {
            'name': self.name,
            'desc': self.desc,
            'opts': self.kwargs
        }
        
###############################################################################
###############################################################################

class BenchmarkRunner(object):
    """BenchmarkRunner
        
        Create an object that runs the benchmark.

    """
    ITERS = 2
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
        rospy.Subscriber('/tracks', Tracks, self._handle_tracks)


    def _handle_tracks(self, msg):
        # Extract utilization information
        data = {
            'u_feature_manager':        msg.util.feature_manager,
            'u_homography_manager':     msg.util.homography_manager,
            'u_measurement_generation': msg.util.measurement_generation,
            'u_rransac':                msg.util.rransac,
            'u_total':                  msg.util.total,
            'track_count':              len(msg.tracks),
            'measurement_count':        msg.util.number_of_rransac_measurements,
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

        return self.runs

###############################################################################
###############################################################################

class BenchmarkAnalyzer(object):
    """BenchmarkAnalyzer"""
    def __init__(self):
        super(BenchmarkAnalyzer, self).__init__()

        self.benchmarks = [] # { name: '', scenarios: { scenario: s, results: [] } }
        

    def _smooth(self, data, alpha=0.1):
        """Smooth
        """

        if len(data) == 0:
            return data

        # Start with initial value
        x = data[0]

        # LPF data
        smoothed = []
        for d in data:
            x = alpha*d + (1-alpha)*x
            smoothed.append(x)

        return smoothed

    def _avg_run(self, run):
        """Average Utilizations (run)

            Average the utillization across the iterations of
            the same scenario with the same stride.
            (i.e., average iter0, iter1, etc)

            Since there could potentially be a different number
            of datapoints in the same run, this just truncates
            to the smallest length array.
        """

        # Decide which utilizations we want to average/return
        ukeys = ['u_total', 'u_feature_manager', 'u_homography_manager',
                        'u_measurement_generation', 'u_rransac',
                        'track_count', 'measurement_count']
        utils = {u:[] for u in ukeys}

        for key in ukeys:
            util = np.zeros(len(run['iter0']))

            iters = run['iters']
            for i in xrange(iters):

                newutil = np.array([data[key] for data in run['iter%i'%i]])

                # How long is this utilization array?
                N = len(newutil)

                # Shrink util array if needed
                if len(util) > N:
                    util = util[0:N]

                util += newutil[0:len(util)]

            # Average
            util = util/float(iters)

            # Burn the first and last 30 data points
            utils[key] = util[30:-30]

        return utils

    def _avg_utils(self, utils_list):
        """Average Utils

        """

        result_util = {}

        for key in utils_list[0].keys():

            batch_util = np.zeros(len(utils_list[0][key]))

            for util in utils_list:

                # How long is the current util array?
                N = len(util[key])

                # Shrink batch_util array if needed
                if len(batch_util) > N:
                    batch_util = batch_util[0:N]

                batch_util += util[key][0:len(batch_util)]

            # Average
            batch_util = batch_util/float(len(utils_list))

            # Add to the final result
            result_util[key] = batch_util

        return result_util


    def _timeline_plots(self, benchmark):
        """Timeline Plots
        """

        for scenario in benchmark['scenarios']:

            # Create subplots
            f, axarr = plt.subplots(len(scenario['results']), sharex=False)

            for i, run in enumerate(scenario['results']):
                utils = self._avg_run(run)

                u_total = utils['u_total']
                u_fm = utils['u_feature_manager']
                u_hm = utils['u_homography_manager']
                u_mg = utils['u_measurement_generation']
                u_rt = utils['u_rransac']
                tc = self._smooth(utils['track_count'], alpha=0.3)
                mc = self._smooth(utils['measurement_count'])


                axarr[i].plot(u_total, label='Total')
                # axarr[i].plot(u_fm, label='Feature Manager')
                # axarr[i].plot(u_hm, label='Homography Manager')
                # axarr[i].plot(u_mg, label='Measurement Generation')
                # axarr[i].plot(u_rt, label='R-RANSAC Tracker')

                # Second y-axis plots
                ax2 = axarr[i].twinx()
                ax2.plot(tc, 'k:', label='Track Count')
                ax2.plot(mc, 'g:', label='Measurement Count')
                ax2.set_ylabel('Count')

                axarr[i].set_ylabel('Stride {}\nUtilization'.format(run['stride']))
                if np.size(u_total) is not 0:
                    axarr[i].axis([0, np.size(u_total), 0, max(1.0, np.max(u_total))])

                if i == 0:
                    pre = benchmark['name'] + ': ' if len(benchmark['name']) > 0 else ''
                    axarr[i].set_title(pre + scenario['scenario'].name)

                    # added these three lines
                    lns = axarr[i].lines + ax2.lines
                    labs = [l.get_label() for l in lns]
                    axarr[i].legend(lns, labs, loc='best', prop={'size': 8})

                if i == len(axarr)-1:
                    axarr[i].set_xlabel('Iteration')


    def _combine_scenarios(self, benchmark):
        """Combine Scenarios

            ASSUMPTION: All scenarios were ran with the same stride parameters.
        """

        # How many strides are there? Note the assumption that the first scenario is
        # representative (in number or strides) of all scenarios
        strides = [result['stride'] for result in benchmark['scenarios'][0]['results']]


        # Store the results in a dictionary by stride
        utils_by_stride = {}

        for i, stride in enumerate(strides):

            utils_with_same_stride = []

            # For each scenario, average the runs with the same frame stride
            # And keep in a list to average all those.
            for scenario in benchmark['scenarios']:

                run = scenario['results'][i]
                utils = self._avg_run(run)
                utils_with_same_stride.append(utils)

            # Average all the utils with the same stride from different scenarios
            utils_by_stride[stride] = self._avg_utils(utils_with_same_stride)

        return utils_by_stride


    def add(self, scenario, results):
        """Add

            Assumes only one benchmark
        """
        if len(self.benchmarks) == 0:
            self.benchmarks.append({ 'name': '', 'scenarios': []})
        self.benchmarks[-1]['scenarios'].append({ 'scenario': scenario, 'results': results })


    def save(self, filename=None, has_cuda=False):
        # If no filename supplied
        if filename is None:
            hostname = socket.gethostname()
            if has_cuda:
                hostname += '_cuda'
            filename = 'bm_{}'.format(hostname)

        # Make sure it's a good filename
        keepcharacters = ('_')
        filename = "".join(c for c in filename if c.isalnum() or c in keepcharacters).rstrip()

        # Add extension if necessary
        if '.' not in filename:
            filename += '.pickle'

        scenarios = []
        for scenario in self.benchmarks[-1]['scenarios']:
            d = {
                'scenario': scenario['scenario'].serialize(),
                'results': scenario['results']
            }
            scenarios.append(d)

        with open(filename, 'wb') as f:
            # Dump a benchmark (a pickle file == a benchmark)
            pickle.dump(scenarios, f, protocol=pickle.HIGHEST_PROTOCOL)


    def load(self, filenames):

        # each filename corresponds to a benchmark
        for filename in filenames:
            print(Fore.GREEN + "Loading benchmark data from {}".format(filename))

            with open(filename, 'rb') as f:
                dump = pickle.load(f)

            scenarios = []
            for data in dump:
                sopts = data['scenario']
                name = sopts['name']
                desc = sopts['desc']
                del sopts['name']
                del sopts['desc']
                s = Scenario(name, desc, **sopts)

                scenario = { 'scenario': s, 'results': data['results'] }
                scenarios.append(scenario)

            self.benchmarks.append({
                    'name': filename.split('.pickle')[0],
                    'scenarios': scenarios
                })


    def analyze(self, args):
        """Analyze
        """

        strides = []

        bdata = []
        for benchmark in self.benchmarks:
            if args['timeline']:
                self._timeline_plots(benchmark)

            utils_by_stride = self._combine_scenarios(benchmark)
            for stride in utils_by_stride.keys():

                # Save stride for later
                if stride not in strides:
                    strides.append(stride)

                util = utils_by_stride[stride]
                for key in util.keys():
                    util[key] = np.mean(util[key])
            bdata.append({'bm': benchmark, 'utils_by_stride': utils_by_stride, 'cuda': False})


        for stride in strides:
            # Create a plot for each stride
            f, axarr = plt.subplots(1, sharex=False)

            ind = np.arange(len(self.benchmarks))
            width = 0.15

            keys = ['u_feature_manager', 'u_homography_manager', 'u_measurement_generation', 'u_rransac']
            colors = ('#d62728', 'royalblue', 'mediumseagreen', 'darkviolet')
            labels = ('Feature Manager', 'Homography Manager', 'Meas Generation', 'R-RANSAC')
            for i,key in enumerate(keys):
                # Get non-CUDA utilizations by stride
                data = [b['utils_by_stride'][stride][key] if not b['cuda'] else 0 for b in bdata]
                axarr.bar(ind, data, width, color=colors[i], label=labels[i])

                # Get CUDA uttilizations by stride
                data = [b['utils_by_stride'][stride][key] if b['cuda'] else 0 for b in bdata]
                axarr.bar(ind + width, data, width, color=colors[i])

            # add some text for labels, title and axes ticks
            axarr.set_ylabel('Avg. Util.')
            axarr.set_title('Utilization Summary for Stride = {}'.format(stride))
            axarr.set_xticks(ind + width / 2.0)
            axarr.set_xticklabels([b['name'] for b in self.benchmarks])
            axarr.set_ylim([0, 1])
            axarr.legend(loc='best', prop={'size': 10})


        plt.tight_layout()
        plt.show()

###############################################################################
###############################################################################

def run_benchmarks(args):
    rospy.init_node('benchmark', anonymous=False)

    # Create a benchmark data analyzer
    analyzer = BenchmarkAnalyzer()

    # Set up common parameters for scenarios -- must be the same for all
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
        'duration': 20,
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
        'duration': 5,
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
    analyzer.save(args['output'], args['cuda'])
    return analyzer
        


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Benchmark visual_mtt algorithm using rosbag data')
    parser.add_argument('benchmark_files', help='Pickled benchmark results file to analyze', nargs='*')
    parser.add_argument('-o', '--output', help='Name of the benchmark results', required=False)
    parser.add_argument('--cuda', help='Is this benchmark running on a CUDA-enabled build?', action='store_true')
    parser.add_argument('--timeline', help='Plot statistics over time', action='store_true')
    args = vars(parser.parse_args())

    if len(args['benchmark_files']) > 0:
        analyzer = BenchmarkAnalyzer()
        analyzer.load(args['benchmark_files'])
        analyzer.analyze(args)
    else:
        analyzer = run_benchmarks(args)