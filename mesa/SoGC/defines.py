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

Categories = ["uniform", "images", "manual"]
Category = Categories[0]
Instance_name = "medium_007_30x30_90_584"
Sol_name = "medium_007_30x30_90_584_Chill_per_20_STUCK_MSPAN683_SUM5579.json"

while Sol_name.find(".json.json") != -1:
    Sol_name = Sol_name[:(-1)*len(".json")]

