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

BOUNDARIES_FACTOR = 4

Categories = ["uniform", "images", "manual"]
Category = Categories[0]
Instance_name = "small_014_20x20_90_200"
Sol_name = "small_014_20x20_90_200_Chill_per_25_STUCK_MSPAN85_SUM1375.json"

while Sol_name.find(".json.json") != -1:
    Sol_name = Sol_name[:(-1)*len(".json")]

