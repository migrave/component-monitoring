# component-monitoring
Package for managing monitors of components used in MigrAVE
 Problems:
* How to make two components (`rgbd_camera` and `knowledge_base`) use the same monitor mode in `component_monitoring`? 
* Monitor mode for the `rgbd_camera` is `pointcloud_monitor`.
* I want to use `pointcloud_monitor` for `knowledge_base` as well.
* I can not do this because class `PointcloudMonitor` is defined only for `rgbd_camera` in the `monitors` directory.
* When I pass `rgbd_camera` as dependency and `pointcloud_monitor` as dependency monitor for `knowledge_base` component, the `pointcloud_monitor` is not created.
