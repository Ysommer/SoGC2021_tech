from typing import Iterator, Tuple, Optional, Type, NamedTuple, Union, Callable
from defines import directions_to_coords, EnterCellResult


class Cell:

    def __init__(self, pos: (int, int)):
        self.pos = pos
        self.target_id = None
        self.occupied = None
        self.tail: (str, int) = None

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

    def enter_cell(self, robot_id: int, direction: str, current_turn: int, advance: bool = True) -> EnterCellResult:  # this method is only
        assert direction in directions_to_coords
        if self.is_obs():   # if cell is obstacle
            return EnterCellResult.OBSTACLE
        if not self.is_empty():  # if cell is taken by other robot
            return EnterCellResult.ROBOT
        if self.get_tail(current_turn) is not None and self.get_tail(current_turn) != direction:  # if can't enter because of tail
            return EnterCellResult.TAIL
        # else can enter
        if advance:
            self.occupied = robot_id
        return EnterCellResult.SUCCESS

    def exit_cell(self, direction: str, current_turn: int):
        assert direction in directions_to_coords
        self.occupied = None
        self.tail = (direction, current_turn)

    def place_robot(self, robot_id: int):  # for grid init only! places robot outside of algo step
        assert self.occupied is None
        self.occupied = robot_id

    def place_target(self, target_id: int):  # for grid init only! places robot outside of algo step
        self.target_id = target_id

    def place_obstacle(self):  # for grid init only! places robot outside of algo step
        self.occupied = -1

