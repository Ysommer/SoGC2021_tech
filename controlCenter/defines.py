from enum import Enum

directions_to_coords = {
    "W": (-1, 0),
    "E": (1, 0),
    "N": (0, 1),
    "S": (0, -1)
    # "X": (0, 0)  # 'X' represents stay
}

def tail_int_to_direction(val: int):
    tail_ints = {
        -2: "W",
        -3: "N",
        -4: "E",
        -5: "S",
    }
    return tail_ints.get(val)

def direction_to_tail_int(direction: str):
    tail_ints = {
        "W": -2,
        "N": -3,
        "E": -4,
        "S": -5,
    }
    return tail_ints.get(direction)


class EnterCellResult(Enum):
    SUCCESS = 0
    OBSTACLE = 1
    ROBOT = 2
    TAIL = 3


class SolutionResult(Enum):
    SUCCESS = 0
    EXCEEDED_MAX_MAKESPAN = 1
    EXCEEDED_MAX_SUM = 2
    STUCK = 3
    RUNNING = 4

