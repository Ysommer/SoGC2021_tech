from mesa import Model
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from .defines import RobotType
import json
from .cell import Cell

been = False

class SoGC(Model):
    """
    Represents the 2-dimensional array of cells in Conway's
    Game of Life.
    """

    def __init__(self, height=10, width=10, paths=None, tracked=None):
        """
        Create a new playing area of (height, width) cells.
        """
        global been
        if not been:
            been = True
            return

        # Use a simple grid, where edges wrap around.
        self.grid = MultiGrid(height, width, torus=False)

        self.schedule = SimultaneousActivation(self)

        # Place a cell at each location, with some initialized to

        data = json.load(open(paths[0], "r"))
        if paths[1] == '':
            sol = None
        else:
            sol = json.load(open(paths[1], "r"))

        for i in range(len(data["starts"])):
            robot = data["starts"][i]
            target = data["targets"][i]
            cell = Cell((int(robot[0]), int(robot[1])), self, i, RobotType.ROBOT, target)
            if i in tracked:
                cell.type = RobotType.TRACKED_ROBOT
            if sol is not None:
                for s in sol["steps"]:
                    if str(i) in s:
                        cell.addStep(s[str(i)])
                    else:
                        cell.addStep("X")

            self.grid.place_agent(cell, (cell.x, cell.y))
            self.schedule.add(cell)

        for i in range(len(data["targets"])):
            target = data["targets"][i]
            cell = Cell((int(target[0]), int(target[1])), self, i, RobotType.TARGET)

            if i in tracked:
                cell.type = RobotType.TRACKED_TARGET
            self.grid.place_agent(cell, (cell.x, cell.y))

        for i in range(len(data["obstacles"])):
            obs = data["obstacles"][i]
            cell = Cell((int(obs[0]), int(obs[1])), self, i, RobotType.OBSTACLE)

            self.grid.place_agent(cell, (cell.x, cell.y))

        self.running = True

    def step(self):
        """
        Have the scheduler advance each cell by one step
        """
        self.schedule.step()