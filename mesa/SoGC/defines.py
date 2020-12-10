from enum import Enum


class RobotType(Enum):
    ROBOT = 0
    TARGET = 1
    OBSTACLE = 2
    ROBOT_ON_TARGET = 3
    TRACKED_ROBOT = 4
    TRACKED_TARGET = 5


def direction_to_val(direction: str):
    directions = {
        "W": [-1, 0],
        "N": [0, 1],
        "E": [1, 0],
        "S": [0, -1],
        "X": [0, 0]
    }
    return directions.get(direction)
