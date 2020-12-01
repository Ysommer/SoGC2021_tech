from typing import Iterator, Tuple, Optional, Type, NamedTuple, Union, Callable
from ..defines import directions_to_coords


class Cell:

    def __init__(self, pos: (int, int), target_id: int = None, is_obs: bool = False, dist_to_target = None):
        self.pos = pos
        self.target_id = target_id
        self.occupied = -1 if is_obs else None
        self.tail: (str, int) = None
        self.dist_to_target = dist_to_target

    def __eq__(self, other):
        assert isinstance(other, Cell)
        return self.pos == other.pos

    def is_obs(self) -> bool:
        return self.occupied == -1

    def get_target(self) -> int:
        return self.target_id

    def get_tail(self, current_turn: int):
        if self.tail is None:
            return None
        if self.tail[1] == current_turn:  # if a robot left this cell on this turn
            return self.tail[0]
        else:
            self.tail = None
            return None

    def is_empty(self) -> bool:
        return self.occupied is None

    def has_robot(self):
        if self.occupied is not None and self.occupied >= 0:
            return self.occupied
        return None

    def enter_cell(self, robot_id: int, direction: str, current_turn: int):  # this method is only
        assert direction in directions_to_coords
        assert not self.occupied and self.get_tail(current_turn) is None or self.get_tail(current_turn) == direction
        self.occupied = robot_id
        return

    def exit_cell(self, direction: str, current_turn: int):
        assert direction in directions_to_coords
        self.occupied = None
        self.tail = (direction, current_turn)
