from typing import Iterator, Tuple, Optional, Type, NamedTuple, Union, Callable
from defines import directions_to_coords


class Robot:

    def __init__(self, robot_id: int, pos: (int, int), target_pos: (int, int)):
        self.robot_id = robot_id
        self.pos = pos
        self.target_pos = target_pos

    def __eq__(self, other):
        assert isinstance(other, Robot)
        return self.robot_id == other.robot_id

    def step(self, direction: str):
        assert(direction in directions_to_coords)
        self.pos += directions_to_coords(direction)

    def robot_arrived(self):
        return self.pos == self.target_pos

