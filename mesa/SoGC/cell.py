from mesa import Agent
from .defines import RobotType
from .defines import *
from collections import deque
import time

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
        self.to_delay = True

    def addStep(self, step): # Why does a cell takes steps?
        self.steps.append(step)

    def step(self):
        if self.to_delay and self.id == 0:
            for i in range(20, 0, -1):
                print(i)
                time.sleep(1)
            self.to_delay = False

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
