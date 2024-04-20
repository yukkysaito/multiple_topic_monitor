# Multiple Topic Monitor

This is a ROS 2 package for monitoring the frequency and delay of multiple topics.

## Installation

Clone this repository into your ROS 2 workspace and build it using colcon:

```bash
cd ~/your_ros2_workspace/src
git clone https://github.com/yukkysaito/multiple_topic_monitor.git
cd ..
colcon build
```

## Usage

### Running the Node

To run the multiple topic monitor node, use the `ros2 run` command followed by the package name and node name, along with the topics you want to monitor:

```bash
ros2 run multiple_topic_monitor multiple_topic_monitor /topic1 /topic2 /topic3
```

You can also specify the window size for calculating averages using the `-w` or `--window` option:

```bash
ros2 run multiple_topic_monitor multiple_topic_monitor /topic1 /topic2 /topic3 -w 10
```

### Output

The node will output the frequency (Hz) and delay (if applicable) for each topic being monitored. The delay is measured in seconds.

```
yukky@yukky:~/workspace/multiple_topic_monitor$ ros2 run multiple_topic_monitor multiple_topic_monitor /planning/hazard_lights_cmd /tf /perception/obstacle_segmentation/pointcloud -w 10
[INFO 1713632632.003018708] [topic_monitor]: Subscribed to /planning/hazard_lights_cmd
[INFO 1713632632.054411974] [topic_monitor]: Subscribed to /tf
[INFO 1713632632.086228617] [topic_monitor]: Subscribed to /perception/obstacle_segmentation/pointcloud
[INFO 1713632632.086962056] [topic_monitor]: No header found for topic /planning/hazard_lights_cmd
[INFO 1713632632.087427722] [topic_monitor]: No header found for topic /tf
[INFO 1713632633.087194077] [topic_monitor]: ----------2024-04-21 02:03:53.086522----------
[INFO 1713632633.087833129] [topic_monitor]: /planning/hazard_lights_cmd Hz: 9.98
[INFO 1713632633.088391419] [topic_monitor]: /tf Hz: 39.62
[INFO 1713632633.088945931] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Hz: 10.00
[INFO 1713632633.089488071] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Delay (s): Avg: 0.021, Min: 0.021, Max: 0.021, Std Dev: 0.00018
[INFO 1713632634.086795272] [topic_monitor]: ----------2024-04-21 02:03:54.086436----------
[INFO 1713632634.087089064] [topic_monitor]: /planning/hazard_lights_cmd Hz: 9.93
[INFO 1713632634.087343683] [topic_monitor]: /tf Hz: 39.92
[INFO 1713632634.087593483] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Hz: 10.00
[INFO 1713632634.087851668] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Delay (s): Avg: 0.021, Min: 0.021, Max: 0.021, Std Dev: 0.00007
[INFO 1713632635.086827859] [topic_monitor]: ----------2024-04-21 02:03:55.086457----------
[INFO 1713632635.087125369] [topic_monitor]: /planning/hazard_lights_cmd Hz: 9.92
[INFO 1713632635.087378595] [topic_monitor]: /tf Hz: 39.97
[INFO 1713632635.087624748] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Hz: 10.00
[INFO 1713632635.087880519] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Delay (s): Avg: 0.021, Min: 0.021, Max: 0.021, Std Dev: 0.00010
[INFO 1713632636.087026327] [topic_monitor]: ----------2024-04-21 02:03:56.086470----------
[INFO 1713632636.087516509] [topic_monitor]: /planning/hazard_lights_cmd Hz: 10.08
[INFO 1713632636.087951777] [topic_monitor]: /tf Hz: 39.94
[INFO 1713632636.088379963] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Hz: 10.00
[INFO 1713632636.088843625] [topic_monitor]: /perception/obstacle_segmentation/pointcloud Delay (s): Avg: 0.021, Min: 0.021, Max: 0.021, Std Dev: 0.00015

```


## License

This software is licensed under the BSD-3-Clause License for the original code, and additional code written by Yukihiro Saito is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for more details.
