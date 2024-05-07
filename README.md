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

To run the multiple topic monitor node, use the `ros2 topic multiple_topic_monitor` command, along with the topics you want to monitor:

```bash
ros2 topic multiple_topic_monitor /topic1 /topic2 /topic3
```

| Option           | Description                                                                     |
| ---------------- | ------------------------------------------------------------------------------- |
| `-w`, `--window` | Specifies the window size for calculating message averages. Default is `10000`. |
| `--csv`          | Enables CSV output mode.                                                        |

### Output

The node will output the frequency (Hz) and delay (if applicable) for each topic being monitored. The delay is measured in seconds.

```
yukky@yukky:~/workspace/multiple_topic_monitor$ ros2 topic multiple_topic_monitor /planning/hazard_lights_cmd /tf /perception/obstacle_segmentation/pointcloud -w 10
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

#### csv format
```
yukky@yukky:~/workspace/multiple_topic_monitor/src/multiple_topic_monitor$ ros2 topic multiple_topic_monitor /planning/hazard_lights_cmd /tf /perception/obstacle_segmentation/pointcloud -w 10 --csv
[INFO 1713635027.749710504] [topic_monitor]: Subscribed to /planning/hazard_lights_cmd
[INFO 1713635027.802425796] [topic_monitor]: Subscribed to /tf
[INFO 1713635027.833324882] [topic_monitor]: Subscribed to /perception/obstacle_segmentation/pointcloud
[INFO 1713635027.834031931] [topic_monitor]: No header found for topic /planning/hazard_lights_cmd
[INFO 1713635027.834483751] [topic_monitor]: No header found for topic /tf
timestamp, /planning/hazard_lights_cmd hz, /tf hz, /perception/obstacle_segmentation/pointcloud hz, /planning/hazard_lights_cmd delay[ms], /tf delay[ms], /perception/obstacle_segmentation/pointcloud delay[ms]
2024-04-21 02:43:48.833518, 10.00, 39.85, 10.00, NA, NA, 20.75
2024-04-21 02:43:49.833592, 10.09, 40.36, 10.00, NA, NA, 20.90
2024-04-21 02:43:50.833570, 10.01, 39.97, 10.00, NA, NA, 20.92
2024-04-21 02:43:51.833562, 10.03, 40.05, 10.00, NA, NA, 21.00
```

## License

This software is licensed under the BSD-3-Clause License for the original code, and additional code written by Yukihiro Saito is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for more details.
