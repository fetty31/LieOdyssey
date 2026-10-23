"""ROS 2 estimator adapter for ins_ros."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class InsRosEstimator:
    """Adapter stub for the external ins_ros lifecycle node."""

    node_name: str = 'ins_ros_node'
    config_file: str | Path | None = None

    def configure(self) -> None:
        return None

    def activate(self) -> None:
        return None

    def deactivate(self) -> None:
        return None

    def dump_state_history(self, output_path: str | None = None) -> dict[str, Any]:
        path = output_path or 'results/state_history.csv'
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text('timestamp,px,py,pz\n', encoding='utf-8')
        return {'success': True, 'num_states': 0, 'file_path': path, 'message': 'placeholder output'}
