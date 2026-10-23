#!/usr/bin/env python3
"""CLI entry point for the ROS 2 benchmark package."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='ins_benchmark',
        description='ROS 2 benchmark runner for ins_ros',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    run_parser = subparsers.add_parser('run', help='Run a benchmark configuration')
    run_parser.add_argument('--config', type=Path, required=True, help='YAML benchmark config')
    run_parser.set_defaults(handler=_handle_run)

    list_parser = subparsers.add_parser('list', help='List available benchmark configs')
    list_parser.set_defaults(handler=_handle_list)
    return parser


def _handle_run(args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.node import Node

    from .core.benchmark_runner import BenchmarkRunner

    rclpy.init()
    node = Node('ins_benchmark_cli')
    try:
        runner = BenchmarkRunner.from_config_file(node, args.config)
        runner.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _handle_list(_: argparse.Namespace) -> int:
    base = Path(__file__).resolve().parent.parent
    config_dir = base / 'config'
    for cfg in sorted(config_dir.rglob('*.yaml')):
        print(cfg.relative_to(base))
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    return args.handler(args)


if __name__ == '__main__':
    raise SystemExit(main())
