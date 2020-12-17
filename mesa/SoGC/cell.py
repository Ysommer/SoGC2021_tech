from mesa import Agent
from .defines import RobotType
from .defines import *
from collections import deque


class Cell(Agent):

    def __init__(self, pos, model, _id, _type: RobotType, target=None):
        """
        Create a cell, in the given state, at the given x, y position.
        """
        super().__init__(pos, model)
        self.x, self.y = pos
        self.id = _id
        self.type = _type
        self.steps = deque()
        self.target = target

    def addStep(self, step): # Why does a cell takes steps?
        self.steps.append(step)

    def step(self):
        if len(self.steps) <= 0:
            return
        self.x += direction_to_val(self.steps[0])[0]
        self.y += direction_to_val(self.steps[0])[1]

        self.steps.popleft()

        if self.type == RobotType.ROBOT or self.type == RobotType.ROBOT_ON_TARGET:
            if [self.x, self.y] == self.target: # Final target?
                self.type = RobotType.ROBOT_ON_TARGET
            else:
                self.type = RobotType.ROBOT

    def advance(self):
        return
