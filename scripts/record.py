
import roslaunch
import rospy
import sys
import signal

import threading
from std_msgs.msg import String

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')

    c1 = cam1.shutdown()
    print("C1 = ",c1)
    c2= cam2.shutdown()
    print("C2 = ",c2)

    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


rospy.init_node('launcher', anonymous=False)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)


pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(10)  # 10hz

rospy.sleep(5)


### Configuration Camera 1
camera1 = ['/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/record.launch',
           'file_name:=cam_1',
           'sensor_sn:=000221294412',
           'recording_folder:=/home/tarek/K4a_data/cam_1.mkv',
           'wired_sync_mode:=0'
           ]
_file1 = roslaunch.rlutil.resolve_launch_arguments(camera1)[0]
_args1 = camera1[1:]
launch_files1 = [(_file1, _args1)]
cam1 = roslaunch.parent.ROSLaunchParent(uuid, launch_files1)

### Configuration Camera 2
camera2 = ['/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/record.launch',
           'file_name:=cam_2',
           'sensor_sn:=001221194512',
           'recording_folder:=/home/tarek/K4a_data/cam_2.mkv',
           'wired_sync_mode:=0'
           ]
_file2 = roslaunch.rlutil.resolve_launch_arguments(camera2)[0]
_args2 = camera2[1:]
launch_files2 = [(_file2, _args2)]
cam2 = roslaunch.parent.ROSLaunchParent(uuid, launch_files2)


## Starter for 5 seconds
status = "pause"
pub.publish(status)
rospy.sleep(2)

cam1.start()
cam2.start()
rospy.sleep(10)

## Record  for desired duration
status = "record"
pub.publish(status)
rospy.sleep(20)

## Finish   pause before stopping
status = "stop"
pub.publish(status)
rospy.sleep(10)


while True:
    pass













