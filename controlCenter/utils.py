
from .infrastructure.grid import Grid
from .infrastructure.robot import Robot
from .solution.solution import Solution
from .defines import *

def move_robot_to_dir(robot_id: int, grid: Grid, direction: chr, current_turn: int, solution: Solution) -> bool:
    if grid.move_robot(robot_id, chr, current_turn) == EnterCellResult.SUCCESS:
        solution.update_robot(robot_id, chr, current_turn)
        return True

    return False
