"""Common ROS-compatible measurement dataclasses."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np


class MeasurementType(str, Enum):
    IMU = 'imu'
    GPS = 'gps'
    ODOMETRY = 'odometry'
    OTHER = 'other'


@dataclass(frozen=True)
class Measurement:
    timestamp: float
    type: MeasurementType = MeasurementType.OTHER
    payload: dict[str, Any] = field(default_factory=dict)

    @property
    def metadata(self) -> dict[str, Any]:
        return self.payload


@dataclass(frozen=True)
class ImuMeasurement(Measurement):
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

    def __post_init__(self) -> None:
        object.__setattr__(self, 'acceleration', np.asarray(self.acceleration, dtype=float))
        object.__setattr__(self, 'angular_velocity', np.asarray(self.angular_velocity, dtype=float))
        object.__setattr__(self, 'type', MeasurementType.IMU)


@dataclass(frozen=True)
class GpsMeasurement(Measurement):
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((3, 3), dtype=float))

    def __post_init__(self) -> None:
        object.__setattr__(self, 'covariance', np.asarray(self.covariance, dtype=float))
        object.__setattr__(self, 'type', MeasurementType.GPS)
