import json
import logging
from multiprocessing import Process
from typing import Union, Dict

from bson import json_util
from jsonschema import validate
from kafka import KafkaProducer, KafkaConsumer, TopicPartition

from component_monitoring.config.config_params import MonitorModeConfig


class MonitorBase(Process):
    """
    Abstract class defining a component monitor on a high level
    """

    def __init__(self, component:str, config_params: MonitorModeConfig, server_address: str, control_channel: str):
        Process.__init__(self)
        self.component = component
        self.mode = config_params.name
        self.config_params = config_params
        self.event_topic = f"{self.component}_{self.config_params.name}_eventbus"
        self.control_topic = control_channel
        self.server_address = server_address
        self.logger = logging.getLogger(f"monitor_{self.config_params.name}")
        self.logger.setLevel(logging.INFO)
        self.producer = None
        self.consumer = None
        with open('component_monitoring/messaging/schemas/event.json', 'r') as schema:
            self.event_schema = json.load(schema)
        self.healthstatus = {}

    def get_status_message_template(self) -> Dict:
        """
        Get the basic event message template
        @return: Basic Event message template
        """
        msg = dict()
        return msg

    def valid_status_message(self, msg: dict) -> bool:
        """
        Validate the event message against the respective schema

        @param msg: JSON dict containing the event message
        @return: True, if the message conforms to the schema, False otheriwse
        """
        try:
            validate(instance=msg, schema=self.event_schema)
            return True
        except:
            return False

    def send_event_msg(self, msg: Union[str, dict, bytes]) -> None:
        """
        Send event message via the Kafka message bus

        @param msg: The event message to send
        @return: None
        """
        if isinstance(msg, bytes):
            self.producer.send(topic=self.event_topic, value=msg)
        else:
            self.producer.send(topic=self.event_topic, value=self.serialize(msg))

    def run(self) -> None:
        """
        Entry point of the base monitor process; Setting up the Kafka consumer and producer

        @return: None
        """
        self.producer = KafkaProducer(bootstrap_servers=self.server_address)
        self.consumer = KafkaConsumer(bootstrap_servers=self.server_address,
                                      client_id=self.config_params.name,
                                      group_id=self.config_params.name,
                                      enable_auto_commit=True,
                                      auto_commit_interval_ms=1000,
                                      auto_offset_reset='latest')

        assignments = [TopicPartition(topic, 0) for topic in [self.event_topic, self.control_topic]]
        self.consumer.assign(assignments)
        for assignment in assignments:
            self.consumer.seek_to_end(assignment)

    def serialize(self, msg: Dict) -> bytes:
        """
        JSON serialize any event message

        @param msg: dict to be JSON serialized
        @return: serialized event message
        """
        return json.dumps(msg, default=json_util.default).encode('utf-8')

    def publish_status(self) -> None:
        """
        Publish the current health status of the component under monitoring via Kafka

        @return: None
        """
        msg = self.get_status_message_template()
        msg["monitorName"] = self.config_params.name
        msg["monitorDescription"] = self.config_params.description
        msg["healthStatus"] = self.healthstatus
        if self.valid_status_message(msg):
            self.send_event_msg(msg)
        else:
            self.logger.error(f"Validation of event message failed in {self.config_params.name}!")
            self.logger.error(msg)
