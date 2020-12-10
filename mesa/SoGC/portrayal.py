from .defines import RobotType

def portrayCell(cell):
    """
    This function is registered with the visualization server to be called
    each tick to indicate how to draw the cell in its current state.
    :param cell:  the cell in the simulation
    :return: the portrayal dictionary.
    """
    assert cell is not None

    colors = {
        RobotType.ROBOT: "green",
        RobotType.TARGET: "red",
        RobotType.OBSTACLE: "black",
        RobotType.ROBOT_ON_TARGET: "yellow",
        RobotType.TRACKED_ROBOT: "blue",
        RobotType.TRACKED_TARGET: "purple"
    }

    portrayal = {
        "Shape": "rect",
        "w": 1,
        "h": 1,
        "Filled": "true",
        "Layer": 1,
        "x": cell.x,
        "y": cell.y,
        "Color": colors[cell.type],
        "text": str(cell.id),
        "text_color": "black"
    }

    if cell.type == RobotType.TARGET or cell.type == RobotType.TRACKED_TARGET or cell.type == RobotType.ROBOT_ON_TARGET:
        portrayal["Layer"] = 0

    if cell.type == RobotType.ROBOT or cell.type == RobotType.TRACKED_ROBOT or cell.type == RobotType.ROBOT_ON_TARGET:
        portrayal["h"] = 0.7
        portrayal["w"] = 0.7


    if cell.type == RobotType.TRACKED_ROBOT:
        portrayal["text_color"] = "white"

    return portrayal