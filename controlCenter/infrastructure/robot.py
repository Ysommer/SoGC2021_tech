from typing import Iterator, Tuple, Optional, Type, NamedTuple, Union, Callable
from defines import directions_to_coords
from utils import *


class Robot:
    def __init__(self, robot_id: int, pos: (int, int), target_pos: (int, int), extra_data=None):
        self.robot_id = robot_id
        self.pos = pos
        self.target_pos = target_pos
        self.extra_data = extra_data
        self.self_sum = 0

    def step(self, direction: str):
        assert(direction in directions_to_coords)
        self.pos = sum_tuples(directions_to_coords[direction], self.pos)
        self.self_sum += 1

    def robot_arrived(self):
        return self.pos == self.target_pos

    def __eq__(self, other):
        assert isinstance(other, Robot)
        return self.robot_id == other.robot_id

    def __str__(self):
        out = "<Robot = (ID: " + str(self.robot_id)
        out += ", Pos: " + str(self.pos)
        out += ", Target: " + str(self.target_pos)
        out += ", Extra data: " + str(self.extra_data)
        out += " )>"
        return out

    def __repr__(self):
        return str(self)