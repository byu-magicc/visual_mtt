#!/usr/bin/env python

import sys, argparse, socket
import time, datetime, pickle
import os, subprocess, signal
import json

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


    *************************************************************
    *** When running a benchmark and the pickle that is saved ***
    *************************************************************

    benchmark = {
        'name': 'tx2',
        'cuda': False,
        'frame_strides': [1,2,3],
        'iterations': BenchmarkRunner.ITERS,
        'scenarios': [
            {
                'name': 'Kiwanis Frisbee',
                'desc': 'desc',
                # ... other options from sopts
                'statistics': [ # by stride
                    {
                        'stride': 1,
                        'u_total': np.array([]),
                        'u_feature_manager': np.array([]),
                        'u_homography_manager': np.array([]),
                        'u_measurement_generation': np.array([]),
                        'u_rransac': np.array([]),
                        'u_total': np.array([]),
                        'track_count': np.array([]),
                        'measurement_count': np.array([]),
                        'time_available': np.array([]),
                    },
                    { stride: 2, ... }, { stride: 3, ... }
                ],
            },
            { 'name': 'Kiwanis Variety', ... }

        ]
    }

    *************************************************************
    *** When analyzing benchmarks and loading from the pickle ***
    *************************************************************

    benchmark = {
        'name': 'tx2',
        'frame_strides': [1,2,3],
        'iterations': BenchmarkRunner.ITERS,
        'scenarios': {
            'cuda': [
                {
                    'name': 'Kiwanis Frisbee',
                    'desc': 'desc',
                    # ... other options from sopts
                    'statistics': [ # by stride
                        {
                            'stride': 1,
                            'u_total': np.array([]),
                            'u_feature_manager': np.array([]),
                            'u_homography_manager': np.array([]),
                            'u_measurement_generation': np.array([]),
                            'u_rransac': np.array([]),
                            'u_total': np.array([]),
                            'track_count': np.array([]),
                            'measurement_count': np.array([]),
                            'time_available': np.array([]),
                        },
                        { stride: 2, ... }, { stride: 3, ... }
                    ],
                },
                { 'name': 'Kiwanis Variety', ... }
            ],
            'noncuda': [ ... ]
        }
    }

    *************************************************************
    ***    When reading scenarios to run from a JSON file     ***
    *************************************************************
    
    scenarios = [
        {
            "name": "Kiwanis Frisbee", 
            "desc": "moving platform, low altitude", 
            "bag_path": "bms_kiwanis_frisbee.bag",    <-- relative to json file
            "cam_info": "creepercam.yaml",            <-- relative to json file
            "bag_topic": "/image_flipped", 
            "compressed": true, 
            "fps": 30, 
            "duration": 20, 
            "silent": true, 
            "flags": "has_info:=false pub_output_img:=true"
        }
    ]


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

        # Hide the roslaunch output
        self.squelch = False
        

    def run(self):
        """Launch

            Create a subprocess that runs `roslaunch visual_mtt ...`

            See: https://stackoverflow.com/a/4791612/2392520
        """
        if not rospy.is_shutdown():
            cmd = 'roslaunch visual_mtt play_from_recording.launch {}'.format(self.flags)
            if self.squelch:
                cmd += ' > /dev/null 2>&1'
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


class BenchmarkFile(object):
    """BenchmarkFile"""

    @staticmethod
    def save(benchmark):
        """Save
        """

        name = benchmark['name']
        if benchmark['cuda']:
            name += '_cuda'

        # Use the benchmark name to create a filename
        filename = 'bm_{}.pickle'.format(name)
        with open(filename, 'wb') as f:
            # Dump a benchmark (a pickle file == a benchmark)
            pickle.dump(benchmark, f, protocol=pickle.HIGHEST_PROTOCOL)


    @staticmethod
    def load(filenames):
        """Load
        """

        benchmarks = []

        # each filename corresponds to a benchmark
        for filename in filenames:
            print(Fore.GREEN + "Loading benchmark data from {}".format(filename))

            with open(filename, 'rb') as f:
                benchmark = pickle.load(f)

            benchmarks.append(benchmark)

        return benchmarks

    @staticmethod
    def rename(filename, new_name):
        """Rename
        """

        # Add the filename to a list so we can use the BenchmarkFile.load method
        filenames = [filename]

        # Load the current benchmark file into a dict
        bm = BenchmarkFile.load(filenames)[0]

        print(Fore.YELLOW + "Saving {} as {}".format(bm['name'], new_name))

        # Rename the benchmark
        bm['name'] = new_name

        # Saving the benchmark will create a new file with the new_name
        BenchmarkFile.save(bm)

###############################################################################
###############################################################################

class ScenarioRunner(object):
    """ScenarioRunner

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
    def __init__(self, **kwargs):
        super(ScenarioRunner, self).__init__()
        self.name = kwargs['name']
        self.desc = kwargs['desc']
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

        if 'info_topic' in kwargs:
            self.flags += ' info_topic:={}'.format(kwargs['info_topic'])

        if 'params' in kwargs:
            self.flags += ' params:={}'.format(kwargs['params'])

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

            # Calculate percentage and make sure that it is less than 100%
            percent = elapsed/self.duration*100
            percent = percent if percent < 100 else 100

            # Print out our progress
            sys.stdout.write('\tProgress:    %0.2f/%0.2f s \t [%0.1f%%]\r' % (elapsed, self.duration, percent) )
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


    def run(self, frame_stride, fn_handle):
        """Run

            Launch the visual_mtt launch file. Then, use Dynamic Reconfigure
            to set the frame stride for the desired variation on the
            scenario. After waiting for the end of the scenario, kill the
            roslaunch command.
        """
        self.launch = ROSLauncher(self.flags)
        self.launch.run()

        # Register a subscriber for this scenario run
        sub = rospy.Subscriber('/tracks', Tracks, fn_handle)

        # Dynamic Reconfigure client
        client = dynamic_reconfigure.client.Client('visual_frontend')

        # Update the frame stride
        params = { 'frame_stride': frame_stride }
        config = client.update_configuration(params)

        # Wait until we should stop the scenario
        self._wait_for_end()

        # Make sure to unsubscribe from the topic
        sub.unregister()
        
        # Kill the launch
        self.launch.stop()
    
        
###############################################################################
###############################################################################

class BenchmarkRunner(object):
    """BenchmarkRunner
        
        Create an object that runs the benchmark.

        A Benchmark represents a set of scenarios. A benchmark has a name
        and knows whether or not CUDA was enabled.

    """
    ITERS = 2
    def __init__(self, name, has_cuda, scenarios, frame_strides=[3]):
        super(BenchmarkRunner, self).__init__()

        # If no benchmark name is supplied use hostname
        if name is None:
            name = socket.gethostname()

        # Make sure it's a good benchmark name
        name = "".join(c for c in name if c.isalnum() or c in ('_')).rstrip()

        # Keeps track of which timestep of the rosbag we are on.
        # i.e., how many Tracks messages have been received for
        # this iteration?
        self._t = 0

        # Add statistics structure to each scenario dictionary
        for scenario in scenarios:
            scenario['statistics'] = []

        # Initialize the benchmark data structure
        self.benchmark = {
            'name': name,
            'cuda': has_cuda,
            'frame_strides': frame_strides,
            'iterations': BenchmarkRunner.ITERS,
            'scenarios': scenarios
        }

        # Start the ROS Node
        rospy.init_node('benchmark_runner', anonymous=False)


    def _handle_tracks(self, scenario, iteration, msg):

        # Grab the statistics by frame_stride for convenience
        # Note that the current frame_stride will be the latest dict in the list
        stats = scenario['statistics'][-1]

        # On the first iteration, just append each incoming datapoint
        if iteration == 0:
            stats['u_total']                     = np.append(stats['u_total'],                    [msg.util.total])
            stats['u_feature_manager']           = np.append(stats['u_feature_manager'],          [msg.util.feature_manager])
            stats['u_homography_manager']        = np.append(stats['u_homography_manager'],       [msg.util.homography_manager])
            stats['u_measurement_generation']    = np.append(stats['u_measurement_generation'],   [msg.util.measurement_generation])
            stats['u_rransac']                   = np.append(stats['u_rransac'],                  [msg.util.rransac])
            stats['track_count']                 = np.append(stats['track_count'],                [len(msg.tracks)])
            stats['measurement_count']           = np.append(stats['measurement_count'],          [msg.util.number_of_rransac_measurements])
            stats['time_available']              = np.append(stats['time_available'],             [msg.util.time_available])

        # On iterations after the first add, the datapoint to what is already there
        else:

            # If this iteration has more datapoints than an earlier iteration (which should be rare)
            # then just ignore these new measurements
            if len(stats['u_total']) > self._t:
                stats['u_total'][self._t]                    += msg.util.total
                stats['u_feature_manager'][self._t]          += msg.util.feature_manager
                stats['u_homography_manager'][self._t]       += msg.util.homography_manager
                stats['u_measurement_generation'][self._t]   += msg.util.measurement_generation
                stats['u_rransac'][self._t]                  += msg.util.rransac
                stats['track_count'][self._t]                += len(msg.tracks)
                stats['measurement_count'][self._t]          += msg.util.number_of_rransac_measurements
                stats['time_available'][self._t]             += msg.util.time_available


        # Update which timestep of the data we are on
        self._t += 1


    def _run_scenario(self, scenario, frame_strides):
        """Run Scenario

            Runs a scenario from the benchmark with the varying frame strides.
        """

        for fs_idx, fs in enumerate(frame_strides):
            print(Style.BRIGHT + 'Frame Stride: ' + Style.DIM + str(fs))

            # Add a new frame stride statistic dict if needed
            if len(scenario['statistics']) == fs_idx:
                scenario['statistics'].append({
                        'stride': fs,
                        'u_total': np.array([]),
                        'u_feature_manager': np.array([]),
                        'u_homography_manager': np.array([]),
                        'u_measurement_generation': np.array([]),
                        'u_rransac': np.array([]),
                        'u_total': np.array([]),
                        'track_count': np.array([]),
                        'measurement_count': np.array([]),
                        'time_available': np.array([]),
                    })

            # Create the ScenarioRunner object to run the scenario dict
            sobj = ScenarioRunner(**scenario)

            # Average utilization results over ITERS iterations
            for i in xrange(BenchmarkRunner.ITERS):

                # Reset the incoming tracks message counter
                self._t = 0

                # Blocking call to run
                sobj.run(fs, fn_handle=lambda msg: self._handle_tracks(scenario, i, msg))


            # Now average if neccessary
            if BenchmarkRunner.ITERS > 1:

                # Grab the timeline statistics by frame_stride
                stats = scenario['statistics'][fs_idx]

                keys = list(set(stats.keys()) - set(['stride']))
                for key in keys:
                    stats[key] = stats[key] / float(BenchmarkRunner.ITERS)


    def run(self):
        """Run
            Runs the benchmark -- which is made up of scenarios.
        """

        frame_strides = self.benchmark['frame_strides']

        for scenario in self.benchmark['scenarios']:

            # Calculate total seconds
            t_secs = scenario['duration'] * BenchmarkRunner.ITERS * len(frame_strides)

            print
            print
            print(Fore.BLUE + Style.DIM + '='*80)
            print(Fore.YELLOW + Style.BRIGHT + 'Benchmark Scenario: ' + Style.DIM + scenario['name'])
            print(Fore.BLUE + Style.DIM + '='*80)
            print('Description:\t{}'.format(scenario['desc']))
            print('Bag Path:\t{}'.format(scenario['bag_path']))
            print('Iterations:\t{}'.format(BenchmarkRunner.ITERS))
            print('Frame Strides:\t{}'.format(frame_strides))
            print('Duration:\t{} seconds each run, {} seconds total'.format(scenario['duration'], t_secs))
            print(Fore.BLUE + Style.DIM + '-'*80)

            self._run_scenario(scenario, frame_strides)


    def save(self):
        BenchmarkFile.save(self.benchmark)

###############################################################################
###############################################################################

class BenchmarkAnalyzer(object):
    """BenchmarkAnalyzer"""
    def __init__(self):
        super(BenchmarkAnalyzer, self).__init__()

        self.benchmarks = []
        

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


    def _timeline_plots(self, benchmark, all=False):
        """Timeline Plots
        """

        for key in benchmark['scenarios'].keys():
            for scenario in benchmark['scenarios'][key]:

                # Create subplots: 1 for each stride
                numsubplots = len(benchmark['frame_strides'])
                f, axarr = plt.subplots(numsubplots, sharex=False)
                if numsubplots == 1:
                    axarr = [axarr]

                for i, stats in enumerate(scenario['statistics']):

                    n = 30 # burn the first n samples
                    u_total = stats['u_total'][n:]
                    u_fm = stats['u_feature_manager'][n:]
                    u_hm = stats['u_homography_manager'][n:]
                    u_mg = stats['u_measurement_generation'][n:]
                    u_rt = stats['u_rransac'][n:]
                    tc = self._smooth(stats['track_count'][n:], alpha=0.3)
                    mc = self._smooth(stats['measurement_count'][n:])


                    axarr[i].plot(u_total, label='Total')

                    if all:
                        axarr[i].plot(u_fm, label='Feature Manager')
                        axarr[i].plot(u_hm, label='Homography Manager')
                        axarr[i].plot(u_mg, label='Measurement Generation')
                        axarr[i].plot(u_rt, label='R-RANSAC Tracker')

                    # Second y-axis plots
                    ax2 = axarr[i].twinx()
                    ax2.plot(tc, 'k:', label='Track Count')
                    ax2.plot(mc, 'g:', label='Measurement Count')
                    ax2.set_ylabel('Count')

                    axarr[i].set_ylabel('Stride {}\nUtilization'.format(stats['stride']))
                    if np.size(u_total) is not 0:
                        axarr[i].axis([0, np.size(u_total), 0, max(1.0, np.max(u_total))])

                    if i == 0:
                        pre = benchmark['name'] + (' (cuda)' if key == 'cuda' else '') + ': '
                        axarr[i].set_title(pre + scenario['name'])

                        # added these three lines
                        lns = axarr[i].lines + ax2.lines
                        labs = [l.get_label() for l in lns]
                        axarr[i].legend(lns, labs, loc='best', prop={'size': 8})

                    if i == len(axarr)-1:
                        axarr[i].set_xlabel('Iteration')

                plt.tight_layout()


    def _average_stats_by_stride(self, benchmark, cuda=False):
        """Average Statistics by Stride

            Take the corresponding frame_stride statistic dict in each scenario
            and average them across scenarios.

            summary_stats = {
                '1':
                    {
                        'stride': 1,
                        'u_total': 0,
                        'u_feature_manager': 0,
                        'u_homography_manager': 0,
                        'u_measurement_generation': 0,
                        'u_rransac': 0,
                        'u_total': 0,
                        'track_count': 0,
                        'measurement_count': 0,
                        'time_available': 0,
                    },
                '2':
                    { 'stride': 2 }, ...
            }
        """

        # How many strides are there?
        strides = benchmark['frame_strides']


        # Store the summary stat dict in a list
        summary_stats = {}

        key = 'cuda' if cuda else 'noncuda'
        if key not in benchmark['scenarios']:
            return []

        for scenario in benchmark['scenarios'][key]:
            for stats in scenario['statistics']:

                # Does a stats dict already exist in the summary stats?
                if stats['stride'] in summary_stats:

                    existing_stats = summary_stats[stats['stride']]

                    # Average the new stats from a different scenario into the summary stats
                    for key in existing_stats.keys():
                        existing_stats[key]  = (existing_stats[key] + np.mean(stats[key])) / 2.0

                else:
                    summary_stat = {}
                    for key in stats.keys():
                        summary_stat[key] = np.mean(stats[key])

                    summary_stats[summary_stat['stride']] = summary_stat

        return summary_stats



    def analyze(self, args):
        """Analyze
        """

        stats_by_stride = []

        bdata = []
        for benchmark in self.benchmarks:
            if args['timeline']:
                self._timeline_plots(benchmark, args['all'])

            cuda_stats    = self._average_stats_by_stride(benchmark, cuda=True)
            noncuda_stats = self._average_stats_by_stride(benchmark, cuda=False)
            bdata.append({'benchmark': benchmark, 'cuda_summary_stats': cuda_stats, 'noncuda_summary_stats': noncuda_stats})


        for stride in benchmark['frame_strides']:
            # Create a plot for each stride
            f, axarr = plt.subplots(1, sharex=False)

            ind = np.arange(len(self.benchmarks))
            width = 0.25

            keys = ['u_feature_manager', 'u_homography_manager', 'u_measurement_generation', 'u_rransac']
            colors = ('#d62728', 'royalblue', 'mediumseagreen', 'darkviolet')
            labels = ('Feature Manager', 'Homography Manager', 'Meas Generation', 'R-RANSAC')
            for i,key in enumerate(keys):

                # Get non-CUDA utilizations by stride
                data = [b['noncuda_summary_stats'][stride][key] if stride in b['noncuda_summary_stats'] else 0 for b in bdata]
                axarr.bar(ind, data, width, color=colors[i], label=labels[i])

                # Get CUDA uttilizations by stride
                data = [b['cuda_summary_stats'][stride][key] if stride in b['cuda_summary_stats'] else 0 for b in bdata]
                axarr.bar(ind + width, data, width, color=colors[i])

            max_noncuda = [b['noncuda_summary_stats'][stride]['u_total'] if stride in b['noncuda_summary_stats'] else 0 for b in bdata]
            max_cuda    = [b['cuda_summary_stats'][stride]['u_total'] if stride in b['cuda_summary_stats'] else 0 for b in bdata]
            max_utilization = max(max_noncuda + max_cuda)

            # Draw 100% utilization line
            axarr.plot([0., np.max(ind)+1], [1.0, 1.0], "k--")

            # add some text for labels, title and axes ticks
            axarr.set_ylabel('Avg. Util.')
            axarr.set_title('Utilization Summary for Stride = {}'.format(stride))
            axarr.set_xticks(ind + width)
            axarr.set_xticklabels([b['name'] for b in self.benchmarks], fontsize=12)
            axarr.set_ylim([0, max(1, max_utilization)])
            axarr.legend(loc='best', prop={'size': 14})

            plt.tight_layout()
        plt.show()


    def load(self, filenames):
        benchmarks = BenchmarkFile.load(filenames)

        # Combine benchmarks with the same name
        for benchmark in benchmarks:
            key = 'cuda' if benchmark['cuda'] else 'noncuda'

            # If a benchmark with the same name exists in the master list of benchmarks, grab a pointer to it
            existing_benchmark = next((b for b in self.benchmarks if b['name'] == benchmark['name']), None)
            if existing_benchmark:

                # Make sure these benchmarks are CUDA complements of each other
                if benchmark['cuda'] is not existing_benchmark['cuda']:

                    existing_benchmark['scenarios'][key] = benchmark['scenarios']

            # This is a new benchmark
            else:

                # Remove the old scenarios key...
                scenarios = benchmark['scenarios']
                del benchmark['scenarios']

                # ...and add it back under the correct cuda/noncuda designation
                benchmark['scenarios'] = { key: scenarios }

                self.benchmarks.append(benchmark)

###############################################################################
###############################################################################

def run_benchmarks(args):

    file = os.path.expanduser(args['file'])

    # Grab the root of the file since the bags and camera info files are
    # defined in the scenarios JSON w.r.t the current JSON file
    root = os.path.dirname(os.path.abspath(file))

    scenarios = []
    with open(file) as json_data:
        scenarios = json.load(json_data)

    # Prepend the paths with the root
    for scenario in scenarios:
        scenario['bag_path'] = os.path.join(root, scenario['bag_path'])

        if 'cam_info' in scenario and not scenario['cam_info'].startswith('/'):
            scenario['cam_info'] = os.path.join(root, scenario['cam_info'])

        if 'params' in scenario and not scenario['params'].startswith('/'):
            scenario['params'] = os.path.join(root, scenario['params'])


    # Benchmark the scenario and add the results
    benchmark = BenchmarkRunner(args['name'], args['cuda'], scenarios, frame_strides=[1, 2, 3])
    benchmark.run()
    benchmark.save()
    

def rename_benchmarks(args):

    # Grab the user-specified new name
    new_name = args['rename'][0]

    for bm_file in args['benchmark_files']:
        # Expand the filename in case it has a `~`
        bm_file = os.path.expanduser(bm_file)

        BenchmarkFile.rename(bm_file, new_name)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Benchmark visual_mtt algorithm using rosbag data')
    parser.add_argument('benchmark_files', help='Pickled benchmark results file to analyze', nargs='*')
    parser.add_argument('-f', '--file', help='Path to the scenario JSON file', required=False)
    parser.add_argument('-n', '--name', help='Name of the benchmark results', required=False)
    parser.add_argument('--cuda', help='Is this benchmark running on a CUDA-enabled build?', action='store_true')
    parser.add_argument('--timeline', help='Plot statistics over time', action='store_true')
    parser.add_argument('--all', help='Show all statistics over time', action='store_true')
    parser.add_argument('--rename', help='Rename a benchmark', nargs=1, action='store')
    args = vars(parser.parse_args())

    if len(args['benchmark_files']) == 0 and args['file'] is not None:
        run_benchmarks(args)

    elif args['rename']:
        rename_benchmarks(args)

    elif len(args['benchmark_files']) > 0:
        analyzer = BenchmarkAnalyzer()
        analyzer.load(args['benchmark_files'])
        analyzer.analyze(args)

    else:
        print(Fore.RED + Style.BRIGHT + 'I did not understand you... :(')
        print
        parser.print_help()