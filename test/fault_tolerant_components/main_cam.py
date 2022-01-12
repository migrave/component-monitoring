#!/usr/bin/python3
import argparse
import logging
from ft_components.components.rgbd_camera.run import run as run_rgbd_camera

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fault tolerant components status',
                                     epilog='EXAMPLE: python3 main_kn.py')
    parser.add_argument('-d', '--debug', help='print debug output', action='store_true')

    args = parser.parse_args()
    # setup logging
    if args.debug:
        logging.basicConfig(level=logging.INFO)
    else:
        logging.basicConfig(level=logging.WARNING)

    run_rgbd_camera()
