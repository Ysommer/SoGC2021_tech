from enum import Enum

directions_to_coords = {
    "W": (-1, 0),
    "E": (1, 0),
    "N": (0, 1),
    "S": (0, -1),
    "X": (0, 0)  # 'X' represents stay
}


class EnterCellResult(Enum):
    SUCCESS = 0
    OBSTACLE = 1
    ROBOT = 2
    TAIL = 3


class SolutionResult(Enum):
    SUCCESS = 0
    EXCEEDED_MAX_MAKESPAN = 1
    EXCEEDED_MAX_SUM = 2
    INTERRUPTED = 3

