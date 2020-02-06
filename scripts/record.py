
import roslaunch
import rospy
import sys
import signal

import threading
from std_msgs.msg import String
import datetime

camera_list = []

def signal_handler(cam1, cam2):
    print('You pressed Ctrl+C!, shutdown Cameras')
    cam1.shutdown()
    cam2.shutdown()
    #stop_cam(camera_list)
    rospy.sleep(10)
    sys.exit(0)

def stop_cam(camera_list):
    for cam in reversed(camera_list):
        cam.shutdown()


def launch_cam(uuid, name, sn, sync):
    camera = ['/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/record.launch',
               'file_name:='+str(name),
               'sensor_sn:='+str(sn),
               'recording_folder:=/home/tarek/K4a_data/'+str(name)+'.mkv',
               'wired_sync_mode:='+str(sync)
                ]
    _file = roslaunch.rlutil.resolve_launch_arguments(camera)[0]
    _args = camera[1:]
    launch_files = [(_file, _args)]
    cam = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    return cam


def main():


    experiment = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    print(experiment)

    #start master node launcher
    rospy.init_node('launcher', anonymous=False)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    print("\n Launcher node started with Topic << chatter >> ...")
    rospy.sleep(2)
    status = "pause"
    pub.publish(status)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    print("\n roslaunch configure_logging ...")
    rospy.sleep(1)

    name = 'cam2_' + str(experiment)
    cam2 = launch_cam(uuid, name, '001221194512', '2')   # Subordinates .....
    cam2.start()
    camera_list.append(cam2)
    print("\n Subordinates camera Launched ...")

    rospy.sleep(5)

    name = 'cam1_' + str(experiment)
    cam1 = launch_cam(uuid, name, '000221294412', '1')   # Master
    cam1.start()
    camera_list.append(cam1)
    print("\n Master camera Launched ...")

    rospy.sleep(5)

    status = "record"
    pub.publish(status)
    rospy.sleep(10)

    status = "stop"
    pub.publish(status)
    rospy.sleep(5)

    status = "record"
    pub.publish(status)
    rospy.sleep(10)

    status = "stop"
    pub.publish(status)
    rospy.sleep(5)


    signal.signal(signal.SIGINT, signal_handler(cam1, cam2))

    while True:
        pass


if __name__ == '__main__':
    main()











