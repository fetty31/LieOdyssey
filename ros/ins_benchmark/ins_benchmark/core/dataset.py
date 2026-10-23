"""Dataset abstraction for deterministic ROS 2 benchmark playback."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Iterable, Iterator


@dataclass
class Sequence:
    identifier: str
    metadata: dict[str, Any] = field(default_factory=dict)
    measurements: list[dict[str, Any]] = field(default_factory=list)

    def __iter__(self) -> Iterator[dict[str, Any]]:
        return iter(self.measurements)


class Dataset(ABC):
    """Generic dataset interface."""

    name: str = 'base'

    @abstractmethod
    def sequences(self) -> Iterable[str]:
        ...

    @abstractmethod
    def load_sequence(self, sequence_id: str) -> Sequence:
        ...

    @abstractmethod
    def ground_truth(self, sequence_id: str) -> Any:
        ...


class SyntheticDataset(Dataset):
    name = 'synthetic'

    def __init__(self, sequence_ids: Iterable[str] | None = None):
        self._ids = list(sequence_ids) if sequence_ids is not None else ['demo']

    def sequences(self) -> Iterable[str]:
        return list(self._ids)

    def load_sequence(self, sequence_id: str) -> Sequence:
        if sequence_id not in self._ids:
            raise ValueError(f'Unknown synthetic sequence {sequence_id!r}')

        seq = Sequence(identifier=sequence_id, metadata={'dataset': self.name})
        seq.measurements = [
            {'timestamp': 0.0, 'type': 'imu', 'acceleration': [0.0, 0.0, 9.81], 'angular_velocity': [0.0, 0.0, 0.0]},
            {'timestamp': 0.5, 'type': 'imu', 'acceleration': [0.0, 0.0, 9.81], 'angular_velocity': [0.0, 0.0, 0.0]},
            {'timestamp': 1.0, 'type': 'gps', 'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
        ]
        return seq

    def ground_truth(self, sequence_id: str) -> dict[str, Any]:
        return {'sequence': sequence_id, 'states': []}


class DatasetFactory:
    _registry: dict[str, type[Dataset]] = {'synthetic': SyntheticDataset}

    @classmethod
    def register(cls, name: str, dataset_cls: type[Dataset]) -> None:
        cls._registry[name] = dataset_cls

    @classmethod
    def create(cls, name: str, **kwargs: Any) -> Dataset:
        try:
            return cls._registry[name](**kwargs)
        except KeyError as exc:
            available = ', '.join(sorted(cls._registry))
            raise ValueError(f'Unknown dataset {name!r}. Available: {available}') from exc

    @classmethod
    def available(cls) -> list[str]:
        return sorted(cls._registry)
