#!/usr/bin/python3

import rospy
from ft_components.components.knowledge_base.knowledge_base_sm import KnowledgeBaseSM
import json
import yaml


def run():
    rospy.init_node('knowledge_base')

    general_message_format = json.load(open('ft_components/schemas/general.json'))
    general_message_schema = json.load(open('ft_components/schemas/general.schema'))
    monitoring_message_schema = json.load(open('ft_components/schemas/monitoring.schema'))
    config = yaml.safe_load(open('ft_components/components/knowledge_base/config/config.yaml'))

    stream_pointcloud = KnowledgeBaseSM(
        component_id=config['id'],
        monitor_manager_id=config['monitoring']['monitor_manager_id'],
        storage_manager_id=config['monitoring']['storage_manager_id'],
        nans_threshold=config['threshold'],
        monitoring_control_topic=config['monitoring']['control_topic'],
        monitoring_pipeline_server=config['monitoring']['pipeline_server'],
        monitors_ids=config['monitoring']['monitors'],
        general_message_format=general_message_format,
        general_message_schema=general_message_schema,
        monitoring_message_schema=monitoring_message_schema,
        data_transfer_timeout=config['data_transfer_timeout']
    )

    try:
        stream_pointcloud.run()
        while stream_pointcloud.is_running and not rospy.is_shutdown():
            rospy.spin()
    except (KeyboardInterrupt, SystemExit):
        print('{0} interrupted; exiting...'.format(stream_pointcloud.name))
        stream_pointcloud.stop()


if __name__ == '__main__':
    run()
