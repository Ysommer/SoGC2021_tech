from solution.solution import *
from typing import List
from infrastructure.robot import *
from defines import *

class SolGrid:
    def __init__(self, size: int, offset: int, robots: List[Robot], obsticales: List[(int, int)], solution: Solution, dynamic: bool=False, grid_len: int = 5000):
        self.size = size
        self.offset = offset
        self.robots = robots
        self.obs = obsticales
        self.solution = solution
        self.dynamic = dynamic
        self.grid_len = grid_len
        self.grid = {} if dynamic else []
        self.__robot_pos = {} #robot_id -> robot_pos
        self.__set_robot_pos()
        self.__set_grid()


    def __set_grid(self):
        if self.dynamic:
            self.grid_len = self.__set_dynamic_grid()
        else:
            self.grid_len = self.__set_full_grid()

        for robot, pos in self.__robot_pos.items():
            self.grid[0][pos] = robot

        t = 1  # time

        for step in self.solution.out["steps"]:
            for robot_id, direction in step.items():
                new_pos = self.__robot_pos[robot_id] + directions_to_coords[direction]
                self.grid[t][new_pos] = robot_id  # update robot's pos in time t
                self.grid[t][self.__robot_pos[robot_id]] = direction_to_tail_int(direction)  # update tail
                self.__robot_pos[robot_id] = new_pos
            t += 1

    def __set_dynamic_grid(self):
        pass

    def __set_full_grid(self) -> int:
        assert self.grid == []
        for t in range(len(self.solution.out["steps"]) + 1):
            self.grid.append({})
        return len(self.grid)

    def __set_robot_pos(self):
        for r in self.robots:
            self.__robot_pos[r.robot_id] = r.pos