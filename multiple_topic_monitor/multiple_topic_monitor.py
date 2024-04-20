# Copyright (c) 2008, Willow Garage, Inc.
# SPDX-License-Identifier: BSD-3-Clause

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Additional code written by Yukihiro Saito under Apache License, Version 2.0


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
    def __init__(self, topics, window_size=10):
        super().__init__('topic_monitor')
        self.window_size = window_size
        self.topic_stats = {topic: {"times": deque(maxlen=window_size), "delays": deque(maxlen=window_size)} for topic in topics}

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

        self.last_print_time = datetime.now()

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
                avg_delay = sum(delays) / len(delays) / 1e9
                min_delay = min(delays) / 1e9
                max_delay = max(delays) / 1e9
                std_dev = math.sqrt(sum((x - avg_delay * 1e9) ** 2 for x in delays) / len(delays)) / 1e9
                self.get_logger().info(f"{topic} Delay (s): Avg: {avg_delay:.3f}, Min: {min_delay:.3f}, Max: {max_delay:.3f}, Std Dev: {std_dev:.5f}")

def main(args=None):
    parser = ArgumentParser(description="Measure the frequency and delay of topics")
    parser.add_argument("topics", nargs='+', help="Topic names to monitor")
    parser.add_argument("-w", "--window", type=positive_int, default=DEFAULT_WINDOW_SIZE, help="Number of messages to average over")
    args = parser.parse_args(args)

    rclpy.init()
    node = TopicMonitor(args.topics, args.window)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv[1:])
