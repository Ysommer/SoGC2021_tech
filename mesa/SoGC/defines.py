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


X_OFFSET = 30
Y_OFFSET = 30

BOUNDARIES_FACTOR = 3

Category = "images"
# Category = "uniform"
Instance_name = "clouds_00001_50x50_40_912"
Sol_name = "clouds_00001_50x50_40_912_OutAndInBFS_default_EXCEEDED_MAX_MAKESPAN_MSPAN36481_SUM50880.json"

while Sol_name.find(".json.json") != -1:
    Sol_name = Sol_name[:(-1)*len(".json")]

