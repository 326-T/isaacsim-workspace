from abc import ABC, abstractmethod
from typing import Any, Dict

import numpy as np


class BaseController(ABC):
    @abstractmethod
    def forward(
        self, position: np.ndarray, orientation: np.ndarray
    ) -> tuple[np.ndarray, Any]:
        pass

    @abstractmethod
    def get_gripper_action(self) -> str | None:
        pass

    @abstractmethod
    def get_applied_action(self) -> Dict:
        pass


class ActionBuffer:
    def __init__(self):
        self._action_buffer: Dict[str, Any] = {}

    def push(self, key: str, value: Any) -> None:
        self._action_buffer[key] = value

    def flush(self) -> Dict[str, Any]:
        copy: Dict[str, Any] = self._action_buffer.copy()
        self._action_buffer.clear()
        return copy
