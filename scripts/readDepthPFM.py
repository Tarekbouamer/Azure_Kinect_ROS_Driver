import glob, os
import re
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

import sys

import cv2

import argparse


parser = argparse.ArgumentParser(description="Data Annotation ")
parser.add_argument("data", metavar="DIR", type=str, help="Path to dataset")

def readPFM(file):
    """ Read a pfm file """
    file = open(file, 'rb')

    color = None
    width = None
    height = None
    scale = None
    endian = None

    header = file.readline().rstrip()
    header = str(bytes.decode(header, encoding='utf-8'))
    if header == 'PF':
        color = True
    elif header == 'Pf':
        color = False
    else:
        raise Exception('Not a PFM file.')

    pattern = r'^(\d+)\s(\d+)\s$'
    temp_str = str(bytes.decode(file.readline(), encoding='utf-8'))
    dim_match = re.match(pattern, temp_str)
    if dim_match:
        width, height = map(int, dim_match.groups())
    else:
        temp_str += str(bytes.decode(file.readline(), encoding='utf-8'))
        dim_match = re.match(pattern, temp_str)
        if dim_match:
            width, height = map(int, dim_match.groups())
        else:
            raise Exception('Malformed PFM header: width, height cannot be found')

    scale = float(file.readline().rstrip())
    if scale < 0:  # little-endian
        endian = '<'
        scale = -scale
    else:
        endian = '>'  # big-endian

    data = np.fromfile(file, endian + 'f')
    shape = (height, width, 3) if color else (height, width)

    data = np.reshape(data, shape)
    # DEY: I don't know why this was there.
    file.close()

    return data, scale


def main(args):
    print("python main function")
    depth_pfm_list = glob.glob(args.data + "*_Depth.pfm")
    depth_registed_pfm_list = glob.glob(args.data + "*_Depth_registed.pfm")
    for file in depth_registed_pfm_list:
        print(file)
        img, scale = readPFM(file)

        print(img.shape)
        img = Image.fromarray(img, 'L')
        print('Scale', scale)

        cv2.imshow("Image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        #input("Press Enter to continue...")

    print('Done')


if __name__ == '__main__':
    main(parser.parse_args())