"""Benchmark runner for ROS 2 Humble playback against ins_ros."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

from ins_ros.srv import DumpStateHistory

from .clock import ClockController
from .dataset import DatasetFactory


class BenchmarkRunner:
    def __init__(self, node, config: str | Path | dict[str, Any]):
        self.node = node
        if isinstance(config, (str, Path)):
            with open(config, 'r', encoding='utf-8') as handle:
                self.config = yaml.safe_load(handle) or {}
        else:
            self.config = config

        self.dataset_name = self.config.get('dataset', {}).get('name', 'synthetic')
        self.sequence_id = self.config.get('dataset', {}).get('sequence', 'demo')
        self.estimator_name = self.config.get('estimator', {}).get('node_name', 'ins_ros_node')
        self.output_dir = Path(self.config.get('output', {}).get('directory', 'results'))
        self.clock = ClockController(self.node, enabled=self.config.get('playback', {}).get('use_sim_time', True))

        self.imu_pub = self.node.create_publisher(Imu, '/input/imu', 10)
        self.gps_pub = self.node.create_publisher(NavSatFix, '/input/gps/fix', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/vio/odom', 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, '/vio/pose', 10)

    @classmethod
    def from_config_file(cls, node, config_path: str | Path) -> 'BenchmarkRunner':
        return cls(node, config_path)

    def _configure_estimator(self) -> bool:
        return self._call_lifecycle(Transition.TRANSITION_CONFIGURE)

    def _activate_estimator(self) -> bool:
        return self._call_lifecycle(Transition.TRANSITION_ACTIVATE)

    def _deactivate_estimator(self) -> bool:
        return self._call_lifecycle(Transition.TRANSITION_DEACTIVATE)

    def _call_lifecycle(self, transition_id: int) -> bool:
        service_name = f'/{self.estimator_name}/change_state'
        self.node.get_logger().info('Calling lifecycle service %s', service_name)
        client = self.node.create_client(ChangeState, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn('Lifecycle service %s unavailable', service_name)
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None:
            return False
        return bool(future.result().success)

    def _publish_imu(self, measurement: dict[str, Any]) -> None:
        acceleration = measurement.get('acceleration', [0.0, 0.0, 0.0])
        angular_velocity = measurement.get('angular_velocity', [0.0, 0.0, 0.0])
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.linear_acceleration.x = float(acceleration[0])
        msg.linear_acceleration.y = float(acceleration[1])
        msg.linear_acceleration.z = float(acceleration[2])
        msg.angular_velocity.x = float(angular_velocity[0])
        msg.angular_velocity.y = float(angular_velocity[1])
        msg.angular_velocity.z = float(angular_velocity[2])
        self.imu_pub.publish(msg)

    def _publish_gps(self, measurement: dict[str, Any]) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.latitude = float(measurement.get('latitude', 0.0))
        msg.longitude = float(measurement.get('longitude', 0.0))
        msg.altitude = float(measurement.get('altitude', 0.0))
        self.gps_pub.publish(msg)

    def _publish_odom(self, measurement: dict[str, Any]) -> None:
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        self.odom_pub.publish(msg)

    def _dump_state_history(self, output_path: Path) -> dict[str, Any]:
        client = self.node.create_client(DumpStateHistory, f'/{self.estimator_name}/dump_state_history')
        if not client.wait_for_service(timeout_sec=5.0):
            return {'success': False, 'num_states': 0, 'message': 'Service /dump_state_history unavailable', 'file_path': str(output_path)}

        request = DumpStateHistory.Request()
        request.output_path = str(output_path)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None:
            return {'success': False, 'num_states': 0, 'message': 'No response from /dump_state_history', 'file_path': str(output_path)}

        response = future.result()
        return {
            'success': bool(response.success),
            'num_states': int(response.num_states),
            'message': str(response.message),
            'file_path': str(response.file_path),
        }

    def run(self) -> Path:
        dataset = DatasetFactory.create(self.dataset_name)
        sequence = dataset.load_sequence(self.sequence_id)

        run_dir = self.output_dir / self.dataset_name / self.sequence_id
        run_dir.mkdir(parents=True, exist_ok=True)

        self.node.get_logger().info('Configuring estimator %s', self.estimator_name)
        if not self._configure_estimator():
            raise RuntimeError(f'Failed to configure estimator {self.estimator_name}')

        self.node.get_logger().info('Activating estimator %s', self.estimator_name)
        if not self._activate_estimator():
            raise RuntimeError(f'Failed to activate estimator {self.estimator_name}')

        for measurement in sequence.measurements:
            timestamp = float(measurement['timestamp'])
            self.clock.advance_to(timestamp)
            kind = measurement.get('type', 'other')

            if kind == 'imu':
                self._publish_imu(measurement)
            elif kind == 'gps':
                self._publish_gps(measurement)
            elif kind == 'odom':
                self._publish_odom(measurement)

            rclpy.unsafe_implicit_sleep(0.01)

        output_file = run_dir / 'state_history.csv'
        dump_result = self._dump_state_history(output_file)
        (run_dir / 'dump_state_history.json').write_text(json.dumps(dump_result, indent=2), encoding='utf-8')

        self._deactivate_estimator()
        self.node.get_logger().info('Benchmark run complete: %s', run_dir)
        return run_dir
