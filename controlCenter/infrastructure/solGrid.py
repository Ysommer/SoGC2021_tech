from solution.solution import *
from typing import List, Set
from infrastructure.robot import *
from defines import *
from utils import sum_tuples

class SolGrid:
    def __init__(self, robots: List[Robot], obsticales: List[List[int]], solution: Solution,
                 dynamic: bool=False, grid_len: int = 5000, validate: bool = True):
        self.robots = robots
        self.solution = solution
        self.dynamic = dynamic
        self.grid_len = grid_len
        self.validate = validate
        self.grid = {} if dynamic else []
        self.__robot_pos = [] #robot_id -> robot_pos
        self.obs = set()
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self.__set_robot_pos()
        self.__set_obs(obsticales)
        self.__set_grid()


    def __set_grid(self):
        self.append_empty_stage(0)
        for i in range(len(self.__robot_pos)):
            self.grid[0][self.__robot_pos[i]] = i

        t = 1  # time

        for step in self.solution.out["steps"]:
            if len(step) == 0:
                continue
            self.append_empty_stage(t)
            for robot_id, direction in step.items():
                old_pos = self.__robot_pos[int(robot_id)]
                new_pos = sum_tuples(old_pos, directions_to_coords[direction])
                if self.validate:
                    assert self.validate_move(t, robot_id, direction), "illegal move you stupid ass"
                self.grid[t][new_pos] = robot_id  # update robot's pos in time t
                self.__robot_pos[robot_id] = new_pos
            t += 1
            if(self.dynamic and t > self.grid_len):
                break
    """
    def __set_dynamic_grid(self):
        pass

    def __set_full_grid(self) -> int:
        assert self.grid == []
        for t in range(len(self.solution.out["steps"]) + 1):
            self.grid.append({})
        return len(self.grid)
    """

    def __set_robot_pos(self):
        for r in range(len(self.robots)):
            self.__robot_pos.append(self.robots[r].pos)

    def __set_obs(self, obs_list: List[List[int]]):
        for obsticale in obs_list:
            pos = (obsticale[0], obsticale[1])
            self.obs.add(pos)

    def get_cell_content(self, time: int, pos: (int, int)): # returns None if cell is empty
        if pos in self.obs:
            return -1
        return self.grid[time].get(pos)

    def check_move(self, robot_id: int, new_pos: (int,int), time: int, direction: str) -> bool:
        new_cell_content = self.get_cell_content(time, new_pos)
        if new_cell_content is None or new_cell_content == robot_id or \
            (new_cell_content < -1 and solGrid_int_to_str(new_cell_content) == direction):
            return True
        return False

    def validate_move(self, time, robot_id, direction):
        old_pos = self.__robot_pos[robot_id]
        new_pos = sum_tuples(old_pos,directions_to_coords[direction])
        return new_pos not in self.obs and new_pos not in self.grid[time] and \
               (new_pos not in self.grid[time-1] or \
               (sum_tuples(new_pos, directions_to_coords[direction]) in self.grid[time] and    \
               self.grid[time-1].get(new_pos) == self.grid[time].get(sum_tuples(new_pos, directions_to_coords[direction]))))

    def append_empty_stage(self, time):
        if self.dynamic:
            assert time not in self.grid
            self.grid[time] = {}
        else:
            self.grid[time].append({})

    def validate_solution(self, targets:list):
        for robot_id in range(len(self.__robot_pos)):
            if targets[robot_id] != self.__robot_pos[robot_id]:
                return False
        return True
