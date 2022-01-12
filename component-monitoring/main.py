#!/usr/bin/env python
import argparse
import multiprocessing

from component_monitoring.config.config_utils import ConfigUtils
from component_monitoring.monitor_manager import MonitorManager

import logging

from component_monitoring.storage.storage_manager import StorageManager

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Monitor component status',
                                     epilog='EXAMPLE: python3 main.py 001 001')
    parser.add_argument('config_file', type=str,
                        default='config/component_monitoring_config',
                        help='Path to a configuration file')
    parser.add_argument('-d', '--debug', help='print debug output', action='store_true')

    args = parser.parse_args()
    # setup logging
    if args.debug:
        logging.basicConfig(level=logging.INFO)
    else:
        logging.basicConfig(level=logging.WARNING)
    config_file_path = args.config_file
    config_data = ConfigUtils.read_config(config_file_path)

    robot_id = config_data['robot_id']
    hw_monitor_config_dir = config_data['config_dirs']['hardware']
    sw_monitor_config_dir = config_data['config_dirs']['software']

    # we read the parameters of the hardware and software monitors
    hw_monitor_config_params = ConfigUtils.get_config_params(hw_monitor_config_dir)
    sw_monitor_config_params = ConfigUtils.get_config_params(sw_monitor_config_dir)

    # we populate the parameters of the configuration utilities
    # to simplify runtime access to the configuration data
    ConfigUtils.config_data = config_data
    ConfigUtils.hw_monitor_config_params = hw_monitor_config_params
    ConfigUtils.sw_monitor_config_params = sw_monitor_config_params
    storage_config_params = config_data['storage']

    manager = multiprocessing.Manager()
    monitors = manager.dict()

    storage_manager = StorageManager(storage_config_params)

    # we initialise a manager for the monitors that will continuously
    # update the component status message
    monitor_manager = MonitorManager(hw_monitor_config_params,
                                     sw_monitor_config_params,
                                     storage_manager)

    # component_network = ComponentNetwork(config_file_path)

    try:
        monitor_manager.start()
        storage_manager.start()
        monitor_manager.join()
        storage_manager.join()

    except (KeyboardInterrupt, SystemExit):
        monitor_manager.log_off()
        print('Component monitors exiting')
