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

X_OFFSET = 25
Y_OFFSET = 25

BOUNDARIES_FACTOR = 3

Categories = ["uniform", "images", "manual"]
Category = Categories[0]
Instance_name = "small_free_012_20x20_60_240"
Sol_name = "small_free_012_20x20_60_240_OutAndInByPercentage_sea_level__BIT__SUCCESS_MSPAN61_SUM5984.json.json.json"

while Sol_name.find(".json.json") != -1:
    Sol_name = Sol_name[:(-1)*len(".json")]

