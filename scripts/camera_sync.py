# !/usr/bin/env python2
import rospy
import rospkg
import yaml

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
import roslaunch
import os
import argparse
from Config.config import load_config, DEFAULTS as DEFAULT_CONFIGS
import time
import Camera as cam


parser = argparse.ArgumentParser(description="Multi Azure Python  Controller")
parser.add_argument("--mode", type=str, default="driver", help="Write logs to the given directory")
parser.add_argument("--nb", type=int, default="1", help="Enter number of cameras used")
parser.add_argument("--config", metavar="FILE", type=str, help="Path to configuration file")
parser.add_argument("--path", metavar="DIR", type=str, help="Path to save experiment mkv video")




def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "spin"
        rate.sleep()
        ##rospy.loginfo(hello_str)
      #  pub.publish(hello_str)
      #  rate.sleep()

def launchNode():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    camera1 = ['/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/record.launch',
               'file_name:=cam1']
    camera2 = ['/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/driver.launch',
               'file_name:=cam2']

    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(camera1)[0]
    roslaunch_args1 = camera1[1:]

    roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(camera2)[0]
    roslaunch_args2 = camera2[1:]

    # launch_files = [(roslaunch_file1, roslaunch_args1), (roslaunch_file2, roslaunch_args2)]
    launch_files = [(roslaunch_file1, roslaunch_args1)]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    launch.start()
    #rospy.loginfo("started")


def go():
    #rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/record.launch"])
    launch.start()


class nodeCamerasController():
    def __init__(self):
        # init cameras controller node
        rospy.init_node('controller')

def load_setup_config(args):
    print("\n Loading setup  configuration :")
    print(" ")
    cfg_setup = load_config(args.config, DEFAULT_CONFIGS["setup"])
    for section in cfg_setup.sections():
        for name, value in cfg_setup.items(section=section):
            print(" {:40} {}".format(name, cfg_setup.get(section, name)))
        print(" ")
    return cfg_setup



def main(args):

    if args.path == '':
        print(" Recoding folder is not set, K4A switch to mode Driver")
        args.mode = 'driver'

    if args.mode == 'driver':
        #
        print("Mode DRIVER is ON")

    if args.mode == 'recording':
        print("Mode RECORDING is ON")
        cameras_list = []
        if os.path.isdir(args.path):
            cfg_setup = load_setup_config(args)

            for x in cfg_setup.sections():
                print(x)
                camera_name = cfg_setup.get(x, 'CAMERA_NAME')
                camera_sn = cfg_setup.get(x, 'SENSOR_SN')
                camera_sync = cfg_setup.getint(x, 'SYNC')
                cameras_list.append(cam.Camera(args, camera_name, camera_sn, camera_sync))

            for x in cameras_list:
                x.ros_launch()

            #talker()








            #for c in args.nb:
               # cameras_list.append(cam.Camera(args))
            #cameras_list[0].display()
            #cam1.ros_launch()
            #talker()

            print("####$$$$$$$$$$$%%%%%%%%%%%%%%%^^^^^^^^^^^^^^^^^^^")

        else:
            print("ERROR ", args.path, " Folder does not exist")

    if args.mode == 'playback':
        print("Mode PLAYBACK is ON")

        #mkv_filepath = os.path.join(args.path, ...)
        if os.path.isdir(args.path):
            camera1 = cam.Camera(args)
            camera1.display()
        else:
            print("ERROR ", args.path, " Folder does not exist")

    #try:
    #    talker()
    #except rospy.ROSInterruptException:
    #    pass


if __name__ == '__main__':
    main(parser.parse_args())
