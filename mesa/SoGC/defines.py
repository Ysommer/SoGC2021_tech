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

X_OFFSET = 50
Y_OFFSET = 50

BOUNDARIES_FACTOR = 3

Categories = ["uniform", "images", "manual"]
Category = Categories[1]
Instance_name = "algae_00004_50x50_50_1113"
Sol_name = "algae_00004_50x50_50_1113_OutAndInBFS_default_SUCCESS_MSPAN67606_SUM88719.json"

while Sol_name.find(".json.json") != -1:
    Sol_name = Sol_name[:(-1)*len(".json")]

