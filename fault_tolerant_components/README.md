# Component State Machine

This project contains implementation of the abstract class `ComponentSMBase` which is used for creating Fault Tolerant Robot Components. A minimal working example of such component is provided within class `RGBDCameraSM`. It simulates the behaviour of the fault tolerant RGBD camera in the robot. It only simulates because there is no real hardware interface implemented. Instead of this simply the ROS topic with the pointcloud is used.

The fault tolerance of the component is based on two aspects:

* Usage of the [Fault Tolerant State Machine](https://github.com/HBRS-SDP/component-monitoring).
* Direct communication with the [Component Monitoring](https://github.com/HBRS-SDP/component-monitoring).

## Usage of the Fault Tolerant State Machine

The concept of the Fault Tolerant State Machine (FTSM) is explained [here](https://github.com/ropod-project/ftsm). In the current implementation the RGBD camera tries to recover whenever the respective Monitor (see *Communication with Component Monitoring*) detects that the gererated pointcloud is faulty. When the RGBD camera can not generate the pointcloud (no data on the topic `/hsrb/head_rgbd_sensor/depth_registered/points`) it tries to reconfigure itself.

## Communication with Component Monitoring

The concept of the Component Monitoring is explained [here](https://github.com/HBRS-SDP/component-monitoring). The implemented Fault Tolerant Robot Component is transferring data (e.g.: pointcloud) to the Component Monitor with the use of ROS messages. Respective Component Monitors are monitoring the data and generating events with the use of Kafka message bus. Those events are then received by the implemented Fault Tolerant Robot Component and an appriopriate reaction is performed.

Additionally, the Fault Tolerant Robot Component is able to turn off/on the respective monitors. It is done by sending an appriopriate request (via Kafka message bus) to the Monitor Manager which is able to start/stop chosen monitors. 

The Fault Tolerant Robot Component is also able to communicate with the Data Storage and request to store in the database events from the chosen Component Monitors.

The complete communication schema is depicted below with the Internal Block Diagram.

![System architecture](../mas_execution_manager/docs/figures/commponent_sm_comm.png)

The messages sent within the Kafka message bus are in the form of JSON. There are two types of messages:

* general message - the json file and schema are in `general.json` and `general.schema` files. The communication between the Fault Tolerant Robot Component and Monitor Manager/Data Storage is based on that form of the message. The message is composed of the following fields:
  * `from` - unique id of the publisher (e.g. id of the Fault Tolerant Robot Component)
  * `to` - unique id of the receiver (e.g. id of the Monitor Manager)
  * `message` - type of the message (`request`, `response` or `info`)
  * `body` - the content of this part varies, depending on the type of the message.
    * If the message is of type `request` the `body` section contains respective fields:
      * `command` - possible commands are: `activate` / `shutdown` (switch on/off chosen monitors), `start_storage`/`stop_storage` (start/stop data storage for the events produced by the chosen monitors)
      * `monitors` - monitors which we want to be shut down by the Monitor Manager or monitors which are producing events that no longer should be stored by the Data Storage 
    * If the message is of type `response` the `body` section contains respective fields:
      * `code` - possible codes are: `200` / `400` (success/failure)
      * `message` - additional information, mainly for debugging purposes
    * If the message is of type `info` the `body` section contains respective fields:
      * `monitors` - list of the monitors ids
      * `actions`
* event message - the event containing the percentage of the NaN values in the pointcloud generated by the RGBD camera.The format of the message is defined in the `monitoring.json` and `monitoring.schema` and is as follows:
  * `monitorName` - name of the monitor that generated the event
  * `monitorDescription` - description of the monitor
  * `healthStatus` - health status of the component. It contains one filed `nan_ratio`, which is the percentage of the NaN values in the pointcloud generated by the component (here RGBD camera).

## Configuration

The Fault Tolerant Robot Component needs to have assigned certain parameters which are depicted in the configuration file `config/config.yaml`. All of the parameters are explained below:

* `id` - unique id of the Fault Tolerant Robot Component
* `data_input_topics` - ROS topic from which the data is received by the component
* `data_output_topics` - ROS topic to which the data is put from the component
* `data_transfer_timeout` - if component can not publish data for this time, reconfiguration is activated (the unit is second)
* `threshold` - percentage threshold of NaN values (in the pointcloud) that is acceptable. If number of NaN values exceeds this limit, the component starts recovery behaviour. In the current implementation the information about the NaNs in the generated pointcloud is received from the monitor.
* `monitoring` - this section contains all the parameters that are necessary to communicate with the Monitor Manager and Storage Manager (Data Storage). The parameters are:
  * `pipeline_server` - address of the Kafka server
  * `control_topic` - Kafka topic to communicate with the Storage Manager (Data Storage) and Monitor Manager
  * `monitor_manager_id` - unique id of the Monitor Manager
  * `storage_manager_id` - unique id of the Storage Manager (in the Data Storage part)
  * `monitors` - list of unique ids of the monitor modes that are monitoring the component

## Usage

To run the fault tolerant RGBD camera, which is a minimal working example of the Fault Tolerant Robot Component, please use the below command:

```python
cd scripts
python run_rgbd_camera.py
```

Please, bear in mind that the implemented RGBD camera component needs to receive pointcloud data on the ROS topic `/hsrb/head_rgbd_sensor/depth_registered/points`.