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

X_OFFSET = 10
Y_OFFSET = 10

BOUNDARIES_FACTOR = 4

Categories = ["uniform", "images", "manual"]
Category = Categories[0]
Instance_name = "small_free_007_10x10_90_90"
Sol_name = "small_free_007_10x10_90_90_OutAndInByPercentagei_5_j_3_SUCCESS_MSPAN906_SUM1521.json"

while Sol_name.find(".json.json") != -1:
    Sol_name = Sol_name[:(-1)*len(".json")]

