from typing import Iterator, Tuple, Optional, Type, NamedTuple, Union, Callable
from defines import directions_to_coords, EnterCellResult


class Cell:
    def __init__(self, pos: (int, int)):
        self.pos = pos
        self.target_id = -1                 # -1 if no target
        self.occupied = None                # -1 if obs [0, n] robot id, None if empty
        self.tail: (str, int) = ("", -1)    # (direction, turn_updated)
        self.extra_data = -1


    def __eq__(self, other):
        assert isinstance(other, Cell)
        return self.pos == other.pos

    def is_obs(self) -> bool:
        return self.occupied == -1

    def get_target(self) -> int:
        return self.target_id

    def get_tail(self, current_turn: int) -> str:
        if self.tail[1] < current_turn:
            return ""

        return self.tail[0]

    def is_empty(self) -> bool:
        """
        :return: True if robot or obs, False otherwise
        """
        return self.occupied is None

    def has_robot(self) -> bool:
        return self.get_robot() is not None

    def get_robot(self) -> Union[int, None]:
        if self.occupied is not None and self.occupied >= 0:
            return self.occupied
        return None

    def has_robot_on_target(self) -> bool:
        if self.has_robot():
            return self.occupied == self.target_id
        return False

    def enter_cell(self, robot_id: int, direction: str, current_turn: int, advance: bool = True) -> EnterCellResult:  # this method is only way to move robot into a cell
        if self.is_obs():   # if cell is obstacle
            return EnterCellResult.OBSTACLE
        if self.has_robot():  # if cell is taken by other robot
            return EnterCellResult.ROBOT
        if self.get_tail(current_turn) != "" and self.get_tail(current_turn) != direction:    # if can't enter because of tail
            assert direction in directions_to_coords
            return EnterCellResult.TAIL
        # else can enter
        if advance:
            self.occupied = robot_id
        return EnterCellResult.SUCCESS

    def exit_cell(self, direction: str, current_turn: int):
        self.occupied = None
        self.tail = (direction, current_turn)

    # Next functions for grid init only! places robot outside of algo step
    def place_robot(self, robot_id: int):
        assert self.occupied is None
        self.occupied = robot_id

    def place_target(self, target_id: int):  # for grid init only! places robot outside of algo step
        self.target_id = target_id

    def place_obstacle(self):  # for grid init only! places robot outside of algo step
        self.occupied = -1

    # Operators:
    def __eq__(self, other):
        assert isinstance(other, Cell)
        return self.pos == other.pos

    def __str__(self):
        out = "<Cell = (pos:" + str(self.pos)
        if self.has_robot():
            out += ", Robot ID: " + str(self.get_robot())
        if self.get_target() != -1:
            out += ", Target ID:" + str(self.get_target())
        elif self.is_obs():
            out += ", Obs"
        out += ", tail:" + str(self.tail)
        out += " )>"
        return out

    def __repr__(self):
        return str(self)