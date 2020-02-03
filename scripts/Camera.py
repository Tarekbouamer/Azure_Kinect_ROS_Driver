
import math
import numpy as np
import os
from Config.config import load_config, DEFAULTS as DEFAULT_CONFIGS
import rospy
import roslaunch
import  pprint

class Camera(object):

    # K4A Group
    K4A_FPS = 'none'

    K4A_DEPTH_MODE = False
    K4A_DEPTH_FORMAT = 'none'

    K4A_RGB_RECT = 0
    K4A_DEPTH_RECT = 0
    K4A_IR_RECT = 0

    K4A_TF_PREFIX = ''
    K4A_CAMERA_NAME = ''

    K4A_COLOR_Mode = False
    K4A_COLOR_FORMAT = 'none'
    K4A_COLOR_RESOLUTION = 'none'

    K4A_POINT_CLOUD = False
    K4A_RGB_POINT_CLOUD = False
    K4A_POINT_CLOUD_IN_DEPTH_FRAME = False

    K4A_SENSOR_SN = ''

    K4A_COLOR_RECORDING_ENABLED = False
    K4A_RECORDING_FOLDER_PATH = ''

    K4A_VIDEO_LOOP_PATH = False
    K4A_VIDEO_FILE_PATH = ''

    K4A_BODY_TRACKING_ENABLE = False
    K4A_BODY_TRACKING_ENABLE_SMOOTHING_FACTOR = 0.0

    K4A_RESCALE_IR_TO_MONO8 = False
    K4A_IR_MONO8_SCALING_FACTOR = 1.0

    K4A_IMU_RATE_TARGET = 0

    K4A_WIRED_SYNC_MODE = 0
    K4A_SUBORDINATE_DELAY_OFF_MASTER_USEC = 0

    # General __
    MODE = 'none'
    FPS_LIST = ['5', '15', '30']
    DEPTH_FORMAT_LIST = ['NFOV_UNBINNED', 'NFOV_2X2BINNED', 'WFOV_UNBINNED', 'WFOV_2X2BINNED', 'PASSIVE_IR' ]
    COLOR_RESOLUTION_LIST = ['720P', '1080P', '1440P', '1536P', '2160P', '3072P']
    COLOR_FORMAT_LIST = ['bgra', 'jpeg']


    # ROS LAUNCH Group
    ROS_LAUNCH_PATH = ''
    ROS_LAUNCH_PARAM_LIST = ''

    def __init__(self, args, name, sn, sync):
        cfg = self.make_config(args, name)
        self.load_param(cfg)
        self.K4A_CAMERA_NAME = name
        self.K4A_SENSOR_SN = sn
        self.K4A_WIRED_SYNC_MODE = sync
        return


    def load_param(self, config):

        # general section
        self.MODE = config.get('general', 'MODE')
        self.K4A_TF_PREFIX = config.get('general', 'K4A_TF_PREFIX')
        self.K4A_CAMERA_NAME = config.get('general', 'K4A_CAMERA_NAME')
        self.K4A_SENSOR_SN = config.get('general', 'K4A_SENSOR_SN')
        self.K4A_COLOR_RECORDING_ENABLED = config.get('general', 'K4A_COLOR_RECORDING_ENABLED')
        self.K4A_RECORDING_FOLDER_PATH = config.get('general', 'K4A_RECORDING_FOLDER_PATH')

        # depth section
        self.K4A_FPS = config.get('depth', 'K4A_FPS')
        self.K4A_DEPTH_MODE = config.get('depth', 'K4A_DEPTH_MODE')
        self.K4A_DEPTH_FORMAT = config.get('depth', 'K4A_DEPTH_FORMAT')
        self.K4A_DEPTH_RECT = config.get('depth', 'K4A_DEPTH_RECT')

        # color section
        self.K4A_COLOR_Mode = config.get('color', 'K4A_COLOR_Mode')
        self.K4A_COLOR_FORMAT = config.get('color', 'K4A_COLOR_FORMAT')
        self.K4A_COLOR_RESOLUTION = config.get('color', 'K4A_COLOR_RESOLUTION')
        self.K4A_RGB_RECT = config.get('color', 'K4A_RGB_RECT')

        # ir section
        self.K4A_RESCALE_IR_TO_MONO8 = config.get('ir', 'K4A_RESCALE_IR_TO_MONO8')
        self.K4A_IR_MONO8_SCALING_FACTOR = config.get('ir', 'K4A_IR_MONO8_SCALING_FACTOR')
        self.K4A_IR_RECT = config.get('ir', 'K4A_IR_RECT')

        # point cloud section
        self.K4A_POINT_CLOUD = config.get('point_cloud', 'K4A_POINT_CLOUD')
        self.K4A_RGB_POINT_CLOUD = config.get('point_cloud', 'K4A_RGB_POINT_CLOUD')
        self.K4A_POINT_CLOUD_IN_DEPTH_FRAME = config.get('point_cloud', 'K4A_POINT_CLOUD_IN_DEPTH_FRAME')

        # body section
        self.K4A_BODY_TRACKING_ENABLE = config.get('body', 'K4A_BODY_TRACKING_ENABLE')
        self.K4A_BODY_TRACKING_ENABLE_SMOOTHING_FACTOR = config.get('body', 'K4A_BODY_TRACKING_ENABLE_SMOOTHING_FACTOR')

        # imu section
        self.K4A_IMU_RATE_TARGET = config.get('imu', 'K4A_IMU_RATE_TARGET')

        # sync section
        self.K4A_WIRED_SYNC_MODE = config.get('sync', 'K4A_WIRED_SYNC_MODE')
        self.K4A_SUBORDINATE_DELAY_OFF_MASTER_USEC = config.get('sync', 'K4A_SUBORDINATE_DELAY_OFF_MASTER_USEC')

        # launch section
        self.ROS_LAUNCH_PATH = config.get('launch', 'ROS_LAUNCH_PATH')
        self.ROS_LAUNCH_PARAM_LIST = config.get('launch', 'ROS_LAUNCH_PARAM_LIST')

        return

    def make_config(self, args, name):
        # load setup configuration file
        print("Init " + name + " parameters")

        if args.mode == 'driver':
            cfg = load_config(args.config, DEFAULT_CONFIGS["driver"])
        elif args.mode == 'recording':
            cfg = load_config(args.config, DEFAULT_CONFIGS["recording"])
        elif args.mode == 'playback':
            cfg = load_config(args.config, DEFAULT_CONFIGS["playback"])
        else:
            print("ERROR wrong arg mode")
        return cfg

    def ros_launch(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        camera1 = ['/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/record.launch',
                   'file_name:='+self.K4A_CAMERA_NAME,
                   'tf_prefix:='+ self.K4A_CAMERA_NAME,
                   'recording_folder:='+self.K4A_RECORDING_FOLDER_PATH+'/'+self.K4A_CAMERA_NAME+'.mkv',
                   'wired_sync_mode:='+str(self.K4A_WIRED_SYNC_MODE)
                   ]

        roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(camera1)[0]
        roslaunch_args1 = camera1[1:]

        launch_files = [(roslaunch_file1, roslaunch_args1)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

        #_file = self.ROS_LAUNCH_PATH
        #args = 'file_name:=cam1'
        #launch_files = [(_file, args)]
        #launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files) ## Lis of args create a list from loaded args
        launch.start()
        return


    def k4a_param_list(self):
        params = 'file_name:='.join(map(str, self.K4A_CAMERA_NAME))
        return  params
        #for a in dir(self):
         #   if a.startswith("K4A") and not callable(getattr(self, a)):
          #      print("{:40} {}".format(a, getattr(self, a)))
        #print("\n")

    def display(self):
        """Display Configuration."""
        print("\nConfigurations:")

        print("\n AZURE PARAM:")
        for a in dir(self):
            if a.startswith("K4A") and not callable(getattr(self, a)):
                print("{:40} {}".format(a, getattr(self, a)))
        print("\n")

        print("\n GENERAL PARAM:")
        for a in dir(self):
            if not a.startswith("K4A") and not a.startswith("ROS") and not a.startswith("__") and not callable(getattr(self, a)):
                print("{:40} {}".format(a, getattr(self, a)))
        print("\n")

        print("\n ROS LAUNCH PARAM:")
        for a in dir(self):
            if a.startswith("ROS") and not callable(getattr(self, a)):
                print("{:40} {}".format(a, getattr(self, a)))
                print("\n")
        print("\n")
        return

