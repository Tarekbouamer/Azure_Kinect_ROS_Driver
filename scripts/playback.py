
import roslaunch
import rospy
import sys
import signal
import glob
import os
import shutil


import threading
from std_msgs.msg import String
import datetime

camera_list = []

def signal_handler(cam1,cam2):
    print('You pressed Ctrl+C!, shutdown Cameras')
    #stop_cam(camera_list)
    cam1.shutdown()
    cam2.shutdown()
    rospy.sleep(10)
    sys.exit(0)

def stop_cam(camera_list):
    for cam in reversed(camera_list):
        cam.shutdown()


def launch_cam(uuid, name, sn, sync, cam_folder):
    camera = ['/home/tarek/workspaces/ros/azure_ws/src/Azure_Kinect_ROS_Driver/launch/playback.launch',
               'file_name:='+str(name),
               'sensor_sn:='+str(sn),
               'recording_folder:='+str(cam_folder),
               'recording_file:=/home/tarek/K4a_data/'+str(name)+'.mkv',
               'wired_sync_mode:='+str(sync)
              ]
    _file = roslaunch.rlutil.resolve_launch_arguments(camera)[0]
    _args = camera[1:]
    launch_files = [(_file, _args)]
    cam = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    return cam


def delete_files(p):
    f_list = [os.path.basename(x) for x in glob.glob(p + '*')]
    print(f_list)
    for f in f_list:
        if os.path.isfile(p+f):
            os.remove(p+f)
        elif os.path.isdir(p+f):
            shutil.rmtree(p+f)

def cluster_files(path):

    Depth_list = [os.path.basename(x) for x in glob.glob(path + '*_Depth.png')]
    Depth_pfm_list = [os.path.basename(x) for x in glob.glob(path + '*_Depth.pfm')]
    Depth_Vis_list = [os.path.basename(x) for x in glob.glob(path + '*_Depth_Vis.png')]

    RGB_list = [os.path.basename(x) for x in glob.glob(path + '*_RGB.png')]
    IR_list = [os.path.basename(x) for x in glob.glob(path + '*_IR.png')]

    Registed_list = [os.path.basename(x) for x in glob.glob(path + '*_Depth_Registed*')]

    depth_pth = path + 'depth/'
    image_pth = path + 'image/'
    ir_pth = path + 'ir/'
    registed_pth = path + 'registed/'

    if not os.path.isdir(depth_pth):
        os.mkdir(depth_pth)

    if not os.path.isdir(image_pth):
        os.mkdir(image_pth)

    if not os.path.isdir(ir_pth):
        os.mkdir(ir_pth)

    if not os.path.isdir(registed_pth):
        os.mkdir(registed_pth)

    delete_files(depth_pth)
    delete_files(image_pth)
    delete_files(ir_pth)
    delete_files(registed_pth)

    for dp, df, dv, rgb, ir in zip(Depth_list, Depth_pfm_list, Depth_Vis_list, RGB_list, IR_list):
        shutil.move(path + dp, depth_pth)
        shutil.move(path + df, depth_pth)
        shutil.move(path + dv, depth_pth)

        shutil.move(path + rgb, image_pth)
        shutil.move(path + ir, ir_pth)

    for reg in Registed_list:
        shutil.move(path + reg, registed_pth)



def main():
    experiment = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    print(experiment)

    path = '/home/tarek/K4a_data/'

    cam1_list = [os.path.basename(x) for x in glob.glob('/home/tarek/K4a_data/cam1_*')]
    cam1_list.sort()

    cam2_list = [os.path.basename(x) for x in glob.glob('/home/tarek/K4a_data/cam2_*')]
    cam2_list.sort()

    print(cam1_list)
    print(cam2_list)

    for name1, name2 in zip(cam1_list, cam2_list):

        name1, _ = name1.split(".")
        cam1_name, cam1_date, cam1_time = name1.split("_")

        name2, _ = name2.split(".")
        cam2_name, cam2_date, cam2_time = name2.split("_")

        path = path + cam1_date + cam1_time + '/'
        path1 = path + cam1_name + '/'
        path2 = path + cam2_name + '/'

        if not os.path.isdir(path):
            os.mkdir(path)

        if not os.path.isdir(path1):
            os.mkdir(path1)

        if not os.path.isdir(path2):
            os.mkdir(path2)

        print(path)
        print(path1)
        print(path2)
        print("\n")

        ## empty folder's files
        delete_files(path1)
        delete_files(path2)

        #start master node launcher
        rospy.init_node('launcher', anonymous=False)
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        print("\n Launcher node started with Topic << chatter >> ...")

        status = "record"
        pub.publish(status)
        rospy.sleep(5)
        print("\n roslaunch configure_logging ...")

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cam2 = launch_cam(uuid, name2, '001221194512', '2', path2)   # Subordinates .....
        cam2.start()
        camera_list.append(cam2)
        print("\n Subordinates camera Launched ...")

        rospy.sleep(2)

        cam1 = launch_cam(uuid, name1, '000221294412', '1', path1)   # Master
        cam1.start()
        camera_list.append(cam1)
        print("\n Master camera Launched ...")

        print("\n Files extraction ...")

        rospy.sleep(40)

        status = "stop"
        pub.publish(status)
        rospy.sleep(5)

        cluster_files(path1)
        cluster_files(path2)

        #rename_files(path1)
        #rename_files(path2)






    signal.signal(signal.SIGINT, signal_handler(cam1, cam2))


if __name__ == '__main__':
    main()


