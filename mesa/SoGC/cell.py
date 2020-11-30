from mesa import Agent
from .defines import RobotType
from .defines import Direction


class Cell(Agent):
    """Represents a single ALIVE or DEAD cell in the simulation."""

    def __init__(self, pos, model, _id, _type: RobotType):
        """
        Create a cell, in the given state, at the given x, y position.
        """
        super().__init__(pos, model)
        self.x, self.y = pos
        self.id = _id
        self.type = _type
        self.steps = []

    def addStep(self, step): # Why does a cell takes steps?
        self.steps.append(step)

    def step(self):
        if len(self.steps) <= 0:
            return
        print("step: ", self.steps[0])
        self.x += Direction.getX(self.steps[0])
        self.y += Direction.getY(self.steps[0])

        del self.steps[0] # Takes O(n) and it will bite as in the future, use a pointer to the list (a counter) or collections.deque

        if len(self.steps) == 0: # Final target?
            self.type = RobotType.ROBOT_ON_TARGET

    def advance(self):
        return
