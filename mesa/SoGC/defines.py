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
Y_OFFSET = 0


BOUNDARIES_FACTOR = 5

Categories = ["uniform", "images", "manual"]
Category = Categories[1]
Instance_name = "buffalo_free_000_25x25_20_125"
Sol_name = "buffalo_free_000_25x25_20_125_LeftPillar_SUCCESS_MSPAN2294_SUM16479.json"

while Sol_name.find(".json.json") != -1:
    Sol_name = Sol_name[:(-1)*len(".json")]

