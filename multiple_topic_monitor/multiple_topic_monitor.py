import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sys
from argparse import ArgumentParser, ArgumentTypeError
from collections import deque
import functools
import math
import matplotlib.pyplot as plt
from ros2topic.api import get_msg_class
from datetime import datetime

DEFAULT_WINDOW_SIZE = 10000

def positive_int(value):
    ivalue = int(value)
    if ivalue <= 0:
        raise ArgumentTypeError(f"{value} is an invalid positive int value")
    return ivalue

class TopicMonitor(Node):
    def __init__(self, topics, window_size=10, csv_mode=False, plot_mode=False):
        super().__init__('topic_monitor')
        self.window_size = window_size
        self.csv_mode = csv_mode
        self.plot_mode = plot_mode
        self.topic_stats = {topic: {"times": deque(maxlen=window_size), "delays": deque(maxlen=window_size), "hz_data": [], "delay_data": []} for topic in topics}
        if self.plot_mode:
            plt.ion()
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
            plt.show()

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
                self.topic_stats[topic]['header'] = False
                self.get_logger().info(f"No header found for topic {topic}")

        self.create_timer(1.0, self.print_stats)

    def message_callback(self, msg, topic):
        now = self.get_clock().now()
        self.topic_stats[topic]["times"].append(now.nanoseconds)

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
        if self.csv_mode:
            if not self.first_row_printed:
                # Print header row for CSV
                header = "timestamp," + ", ".join([f"{topic} Hz" for topic in self.topic_stats.keys()] +
                                                  [f"{topic} delay[ms]" for topic in self.topic_stats.keys()])
                print(header)
                self.first_row_printed = True

            row_values = []
            for topic, stats in self.topic_stats.items():
                times = stats["times"]
                if len(times) > 1:
                    dt = times[-1] - times[0]
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
            print(f"{timestamp}, {', '.join(row_values)}")
        else:
            divider = "-" * 10
            self.get_logger().info(f"{divider}{now.strftime('%Y-%m-%d %H:%M:%S.%f')}{divider}")

            for topic, stats in self.topic_stats.items():
                times = stats["times"]
                if len(times) > 1:
                    dt = times[-1] - times[0]
                    count = len(times) - 1
                    hz = count / (dt / 1e9) if dt else 0
                    self.get_logger().info(f"{topic} Hz: {hz:.2f}")

                delays = stats["delays"]
                if len(delays):
                    avg_delay = sum(delays) / len(delays) / 1e6
                    min_delay = min(delays) / 1e6
                    max_delay = max(delays) / 1e6
                    std_dev = math.sqrt(sum((x - avg_delay * 1e6) ** 2 for x in delays) / len(delays)) / 1e6
                    self.get_logger().info(f"{topic} Delay (ms): Avg: {avg_delay:.3f}, Min: {min_delay:.3f}, Max: {max_delay:.3f}, Std Dev: {std_dev:.5f}")
        if self.plot_mode:
            for topic, stats in self.topic_stats.items():
                hz = "NA"
                avg_delay = "NA"
                times = stats["times"]
                if len(times) > 1:
                    dt = times[-1] - times[0]
                    count = len(times) - 1
                    hz = count / (dt / 1e9) if dt else 0
                else:
                    hz = "NA"
                delays = stats["delays"]
                if len(delays):
                    avg_delay = sum(delays) / len(delays) / 1e6  # Convert nanoseconds to milliseconds
                else:
                    avg_delay = "NA"
                self.topic_stats[topic]["delay_data"].append((now, avg_delay))
                self.topic_stats[topic]["hz_data"].append((now, hz))

            self.update_plot()

    def update_plot(self):
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.set_title('Topic Hz')
        self.ax2.set_title('Topic Delay')
        self.ax1.set_xlabel('Timestamp')
        self.ax2.set_xlabel('Timestamp')
        self.ax1.set_ylabel('Hz')
        self.ax2.set_ylabel('Delay (ms)')
        for topic, stats in self.topic_stats.items():
            if stats["hz_data"] and stats["hz_data"][0][1] != "NA":
                timestamps, hzs = zip(*stats["hz_data"])
                self.ax1.plot(timestamps, hzs, label=topic)
            if stats["delay_data"] and stats["delay_data"][0][1] != "NA":
                timestamps, delays = zip(*stats["delay_data"])
                self.ax2.plot(timestamps, delays, label=f"{topic} Delay")
        self.ax1.legend()
        self.ax2.legend()
        plt.pause(0.001)


def main(args=None):
    parser = ArgumentParser(description="Measure the frequency and delay of topics")
    parser.add_argument("topics", nargs='+', help="Topic names to monitor")
    parser.add_argument("-w", "--window", type=positive_int, default=DEFAULT_WINDOW_SIZE, help="Number of messages to average over")
    parser.add_argument("--csv", action="store_true", help="Enable CSV output mode")
    parser.add_argument("--plot", action="store_true", help="Enable real-time plotting")
    args = parser.parse_args(args)

    rclpy.init()
    node = TopicMonitor(args.topics, args.window, args.csv, args.plot)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv[1:])
