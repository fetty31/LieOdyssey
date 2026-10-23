#!/usr/bin/env python3
"""ROS 2 benchmark node entrypoint."""

from __future__ import annotations

import argparse

import rclpy

from .core.benchmark_runner import BenchmarkRunner


class BenchmarkNode(rclpy.node.Node):
    def __init__(self, config_path: str):
        super().__init__('ins_benchmark_node')
        self.runner = BenchmarkRunner(self, config_path)

    def run(self) -> None:
        self.get_logger().info('Starting benchmark run')
        result = self.runner.run()
        self.get_logger().info('Benchmark run complete: %s', str(result))


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Run an ins_benchmark session.')
    parser.add_argument('--config', type=str, required=True, help='Path to benchmark YAML config')
    args = parser.parse_args(argv)

    rclpy.init()
    node = BenchmarkNode(args.config)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
