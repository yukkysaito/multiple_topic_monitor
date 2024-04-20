import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sys
from argparse import ArgumentParser, ArgumentTypeError
from collections import deque
import functools
import math
from ros2topic.api import get_msg_class
from datetime import datetime

DEFAULT_WINDOW_SIZE = 10000

def positive_int(value):
    ivalue = int(value)
    if ivalue <= 0:
        raise ArgumentTypeError(f"{value} is an invalid positive int value")
    return ivalue

class TopicMonitor(Node):
    def __init__(self, topics, window_size=10, csv_mode=False):
        super().__init__('topic_monitor')
        self.window_size = window_size
        self.csv_mode = csv_mode
        self.topic_stats = {topic: {"times": deque(maxlen=window_size), "delays": deque(maxlen=window_size)} for topic in topics}

        self.first_row_printed = False

        for topic in topics:
            msg_class = get_msg_class(self, topic)
            if msg_class:
                self.create_subscription(
                    msg_class,
                    topic,
                    functools.partial(self.message_callback, topic=topic),
                    qos_profile_sensor_data
                )
                self.get_logger().info(f"Subscribed to {topic}")
            else:
                self.get_logger().error(f"No message class found for topic {topic}")

        self.create_timer(1.0, self.print_stats)

    def message_callback(self, msg, topic):
        now = self.get_clock().now()
        self.topic_stats[topic]["times"].append((now.nanoseconds, now.to_msg().sec, now.to_msg().nanosec))
        
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            msg_time = msg.header.stamp
            if msg_time:
                delay = now.nanoseconds - (msg_time.sec * 1e9 + msg_time.nanosec)
                self.topic_stats[topic]["delays"].append(delay)
        elif 'header' not in self.topic_stats[topic]:
            self.topic_stats[topic]['header'] = False
            self.get_logger().info(f"No header found for topic {topic}")

    def print_stats(self):
        now = datetime.now()
        if self.csv_mode and not self.first_row_printed:
            # Print header row
            header = "timestamp, " + ", ".join([f"{topic} hz" for topic in self.topic_stats.keys()] +
                                               [f"{topic} delay[ms]" for topic in self.topic_stats.keys()])
            print(header)
            self.first_row_printed = True

        row_values = []
        for topic, stats in self.topic_stats.items():
            times = stats["times"]
            if len(times) > 1:
                dt = times[-1][0] - times[0][0]
                count = len(times) - 1
                hz = count / (dt / 1e9) if dt else 0
                row_values.append(f"{hz:.2f}")
            else:
                row_values.append("NA")

        for topic, stats in self.topic_stats.items():
            delays = stats["delays"]
            if len(delays):
                avg_delay = sum(delays) / len(delays) / 1e6  # Convert nanoseconds to milliseconds
                row_values.append(f"{avg_delay:.2f}")
            else:
                row_values.append("NA")

        timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")
        if self.csv_mode:
            print(f"{timestamp}, {', '.join(row_values)}")
        else:
            self.get_logger().info(f"{timestamp}, {', '.join(row_values)}")

def main(args=None):
    parser = ArgumentParser(description="Measure the frequency and delay of topics")
    parser.add_argument("topics", nargs='+', help="Topic names to monitor")
    parser.add_argument("-w", "--window", type=positive_int, default=DEFAULT_WINDOW_SIZE, help="Number of messages to average over")
    parser.add_argument("--csv", action="store_true", help="Enable CSV output mode")
    args = parser.parse_args(args)

    rclpy.init()
    node = TopicMonitor(args.topics, args.window, args.csv)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv[1:])
