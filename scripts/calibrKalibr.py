

import argparse
from glob import glob
from os import path, mkdir, chdir
import shutil
import subprocess

parser = argparse.ArgumentParser(description="calibrate two azure cameras")

parser.add_argument("--dir", type=str, default=".", help="process directory")
parser.add_argument("--kalibr_dir", type=str, default=".", help="kailbr directory")



def main(args):
    print("\n init calibration ...")

    experiments = glob(path.join(args.dir, '*/'))

    print('\n', experiments)

    for exp in experiments:
        # create kalibr file
        print(exp.split('/')[5], '...')
        kalibr_dir = path.join(exp, "kalibr/")

        if not path.isdir(kalibr_dir):
            mkdir(kalibr_dir)

        # create cam1 cam2  folders
        cam1_dir = path.join(kalibr_dir, "cam0/")
        cam2_dir = path.join(kalibr_dir, "cam1/")

        if not path.isdir(cam1_dir):
            mkdir(cam1_dir)

        if not path.isdir(cam2_dir):
            mkdir(cam2_dir)

        #copy files from cam1/image/* to kalibr/cam1/*

        for file in glob(path.join(exp, 'cam1/image/*.*')):
            shutil.copy(file, cam1_dir)

        for file in glob(path.join(exp, 'cam2/image/*.*')):
            shutil.copy(file, cam2_dir)

    # move to Kalib directory
    chdir(args.kalibr_dir)

    for exp in experiments:

        folder_arg = exp + 'kalibr/'
        output_arg = exp + 'kalibr/images.bag'
        cam0 = '/cam0/image_raw'
        cam1 = '/cam1/image_raw'
        model0 = 'pinhole-equi'
        model1 = 'pinhole-equi'
        target = args.kalibr_dir + 'tags/april_6x6_80x80cm.yaml'
        sync = '0.02'

        subprocess.run(["kalibr_bagcreater",
                        "--folder", folder_arg,
                        "--output-bag", output_arg
                        ])

        subprocess.run(["kalibr_calibrate_cameras",
                        "--bag", output_arg,
                        "--topics", cam0, cam1,
                        "--models", model0, model1,
                        "--target", target,
                        "--approx-sync", sync,
                        "--output", folder_arg,
                        "--no-shuffle",
                        "--dont-show-report",

                        ])
        print("Done ...")


if __name__ == '__main__':
    main(parser.parse_args())