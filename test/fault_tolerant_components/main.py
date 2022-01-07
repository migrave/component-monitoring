#!/usr/bin/python3
import argparse
import logging
from ft_components.components.rgbd_camera.run import run as run_rgbd_camera
from ft_components.components.knowledge_base.run import run as run_knowledge_base

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MFault tolerant components status',
                                     epilog='EXAMPLE: python3 main.py')
    parser.add_argument('-d', '--debug', help='print debug output', action='store_true')

    args = parser.parse_args()
    # setup logging
    if args.debug:
        logging.basicConfig(level=logging.INFO)
    else:
        logging.basicConfig(level=logging.WARNING)

    run_knowledge_base()
    #run_rgbd_camera()
