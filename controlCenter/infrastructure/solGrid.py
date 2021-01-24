from solution.solution import *
from typing import List, Set
from infrastructure.robot import *
from defines import *
from utils import sum_tuples
from infrastructure.robot import *
import sys


class SolGrid:
    def __init__(self, robots: List[Robot], obstacles: List[List[int]], solution: Solution, size: int = -1,
                 dynamic: bool = False, max_grid_len: int = -1, validate: bool = True):
        self.robots = robots
        self.solution = solution
        self.dynamic = dynamic
        self.max_grid_len = max_grid_len
        self.validate = validate
        self.grid = {} if dynamic else []
        self.__robot_pos = []  # robot_id -> robot_pos
        self.obs = set()
        self.size = size
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        self.x_boundaries = {}
        if size > 0:
            for x in range(self.size):
                self.x_boundaries[x] = (0, size-1)
        self.max_time = 0
        self.time_arrived = [-1 for i in range(len(self.robots))] if self.max_grid_len < 0 \
            else [min(solution.out["extra"]["time_arrived"][i], self.max_grid_len) for i in range(len(robots))]
        self.arrival_order = [] if self.max_grid_len < 0 \
            else solution.out["extra"]["arrival_order"]
        assert len(self.arrival_order) > 0 or self.max_grid_len == -1
        assert self.time_arrived[0] != -1 or self.max_grid_len == -1
        self.__set_robot_pos()
        self.__set_obs(obstacles)
        self.__set_grid()

    def __set_grid(self):
        limit = (self.max_grid_len != -1)
        self.append_empty_stage(0)
        for i in range(len(self.__robot_pos)):
            # self.grid[0][self.__robot_pos[i]] = (i, 'X')
            self.write_cell(0, self.__robot_pos[i], i, 'X')

        t = 1  # time

        for step in self.solution.out["steps"]:
            # self.append_empty_stage(t)
            for robot_id in range(len(self.robots)):
                if str(robot_id) in step:
                    direction = step[str(robot_id)]
                    old_pos = self.__robot_pos[robot_id]
                    new_pos = sum_tuples(old_pos, directions_to_coords[direction])
                    # if self.validate:
                    # assert self.validate_move(t, robot_id, direction), "illegal move you stupid ass"
                    self.write_cell(t, new_pos, robot_id, direction)  # update robot's pos in time t
                    self.__robot_pos[robot_id] = new_pos
                    if not limit and self.robots[robot_id].target_pos == new_pos:
                        self.time_arrived[robot_id] = t
                    self.min_x = min(self.min_x, new_pos[0])
                    self.max_x = max(self.max_x, new_pos[0])
                    self.min_y = min(self.min_y, new_pos[1])
                    self.max_y = max(self.max_y, new_pos[1])
                    x_bound = self.x_boundaries.get(new_pos[0], None)
                    if x_bound is not None:
                        self.x_boundaries[new_pos[0]] = (min(x_bound[0], new_pos[1]), max(x_bound[1], new_pos[1]))
                    else:
                        self.x_boundaries[new_pos[0]] = (new_pos[1], new_pos[1])
                else:
                    self.write_cell(t, self.__robot_pos[robot_id], robot_id, 'X')
            t += 1
            if limit and t > self.max_grid_len:
                break

        if not limit:
            self.arrival_order = sorted(range(len(self.time_arrived)), key=lambda x: self.time_arrived[x])

        self.max_time = t - 1

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
        for obstacle in obs_list:
            pos = (obstacle[0], obstacle[1])
            self.obs.add(pos)

    def get_cell_content(self, time: int, pos: (int, int)):  # by default returns None if cell is empty
        if pos in self.obs:
            return -1, None
        # content = self.grid[time].get(pos, return_if_empty)
        content = self.get_cell(time, pos)
        return content

    # get cell content without obs check
    def get_cell_robot(self, time: int, pos: (int, int)):  # by default returns None if cell is empty
        # content = self.grid[time].get(pos, return_if_empty)
        content = self.get_cell(time, pos)
        return content

    '''
    def validate_move(self, time, moves: list):
        return True
        for (robot_id, direction) in moves:
            new_pos = self.__robot_pos[robot_id]
            old_pos = sub_tuples(new_pos, directions_to_coords[direction])
        old_pos = self.__robot_pos[robot_id]
        new_pos = sum_tuples(old_pos, directions_to_coords[direction])
        legal = new_pos not in self.obs and new_pos not in self.grid[time] and \
                (new_pos not in self.grid[time - 1] or
                 (sum_tuples(new_pos, directions_to_coords[direction]) in self.grid[time] and
                  self.grid[time - 1].get(new_pos) == self.grid[time].get(
                             sum_tuples(new_pos, directions_to_coords[direction]))))
        return legal
        
    '''

    def append_empty_stage(self, time):
        if self.dynamic:
            assert time not in self.grid
            self.grid[time] = {}
        else:
            self.grid.append({})

    def validate_solution(self, targets: list):
        for robot_id in range(len(self.__robot_pos)):
            if targets[robot_id] != self.__robot_pos[robot_id]:
                print("Failure:", robot_id)
                return False
        return True

    # Methods to support Optimization_Algos
    def check_move(self, robot_id: int, new_pos: (int, int, int), direction: str, params=None) -> bool:
        time = new_pos[2]

        # check cell is empty
        new_cell_pos = (new_pos[0], new_pos[1])
        new_cell_now = self.get_cell_content(time, new_cell_pos)
        if new_cell_now is not None and new_cell_now != robot_id:
            return False
        if direction == 'X':
            return True

        # check ok with tail on cell
        new_cell_before = self.get_cell_content(time - 1, new_cell_pos)
        new_cell_before_empty = new_cell_before is None or new_cell_before == robot_id
        tail_from_pos = sum_tuples(new_cell_pos, directions_to_coords[direction])
        tail_from_now = self.get_cell_content(time, tail_from_pos)
        tail_from_now_is_robot = tail_from_now is not None and tail_from_now not in [-1, robot_id]
        can_enter = new_cell_before_empty or (tail_from_now_is_robot and new_cell_before == tail_from_now)
        if not can_enter:
            return False

        # check own tail is ok
        old_cell_pos = sub_tuples(new_cell_pos, directions_to_coords[direction])
        old_cell_now = self.get_cell_content(time, old_cell_pos)
        old_cell_now_empty = old_cell_now is None or old_cell_now == robot_id
        tail_to_pos = sub_tuples(old_cell_pos, directions_to_coords[direction])
        tail_to_before = self.get_cell_content(time - 1, tail_to_pos)
        tail_to_before_is_robot = tail_to_before is not None and tail_to_before not in [-1, robot_id]
        can_leave = old_cell_now_empty or (tail_to_before_is_robot and old_cell_now == tail_to_before)
        return can_leave

    # use with Generator.valid_direction_matrix and pushed out check.  should not call with 'X'
    def new_check_move(self, robot_id: int, new_pos: (int, int, int), direction: str, params=None):
        time = new_pos[2]

        # check cell is empty
        new_cell_pos = new_pos[:2]
        new_cell_now = self.get_cell_robot(time, new_cell_pos)
        if new_cell_now[0] is not None and new_cell_now[0] != robot_id:   # new cell has robot
            return False

        # check ok with tail on cell
        new_cell_before = self.get_cell_robot(time - 1, new_cell_pos)
        if new_cell_before[0] is None or new_cell_before[0] == robot_id:
            return True

        # there is a tail, check if right direction
        tail_from_pos = sum_tuples(new_cell_pos, directions_to_coords[direction])
        tail_from_now = self.get_cell_robot(time, tail_from_pos)
        if tail_from_now[0] is not None and tail_from_now[0] == new_cell_before[0] and tail_from_now[1] == direction:
            return True
        return False

    def find_last_step_on_location(self, robot_id: int, target: (int, int), t: int = -1) -> (
            int, int):  # returns last time the target had any robot on it and the robot id
        assert target not in self.obs
        if t < 0 or t > self.max_time:
            t = self.max_time
        content = self.get_cell(t, target)
        while content is None or content[0] in [-1, robot_id] and t >= 0:
            t -= 1
            content =self.get_cell(t, target)
        return t

    def get_robot_location(self, robot_id: int, time: int):
        if time > self.max_time:
            return None
        pos = self.robots[robot_id].pos
        if time == 0:
            return pos
        t = 0
        while t < time:
            step = self.solution.out["steps"][t]
            d = step.get(robot_id, None)
            if d is not None:
                pos = sum_tuples(pos, directions_to_coords[d])
            t += 1
        '''
        for pos, (r_id, d) in self.grid[time].items():
            if robot_id == r_id:
                return pos
        '''
        return pos

    def get_last_moving_robots(self) -> List[int]:
        last_index = -1
        last = []
        while len(last) == 0:
            for robot_id in self.solution.out["steps"][last_index]:
                last.append(int(robot_id))
            last_index -= 1
        return last

    def update_robot_path(self, robot_id: int, start_pos: (int, int), path: list, start_time: int, end_time: int):
        robot_pos = start_pos
        old_pos = start_pos
        # last = False
        # if robot_id in self.get_last_moving_robots():
        # last = True

        # for pos, r_id in self.grid[start_time].items():
        # if r_id == robot_id:
        # robot_pos = pos
        # break

        t = start_time + 1
        # max_t = len(path) + start_time
        while t <= end_time:
            # for pos, r_id in self.grid[t].items():
            # if r_id == robot_id:
            # del self.grid[t][pos]
            # break
            content = self.get_cell(t, old_pos)
            if content[0] is not None and content[0] == robot_id:
                old_pos = old_pos
            else:
                for d in directions_to_coords:
                    pos = sum_tuples(old_pos, directions_to_coords[d])
                    content = self.get_cell(t, pos)
                    if content[0] is not None and content[0] == robot_id:
                        old_pos = pos
                        break
            del self.grid[t][old_pos]
            self.clean_cell(t, old_pos)
            direction = "X"
            if len(path) + start_time >= t:
                direction = path[t - start_time - 1]
                robot_pos = robot_pos if direction == "X" else sum_tuples(robot_pos, directions_to_coords[direction])
            # assert robot_pos not in self.grid[t]
            # self.grid[t][robot_pos] = (robot_id, direction)
            self.write_cell(t, robot_pos, robot_id, direction)
            # if last and len(path) + start_time < t and self.grid[t] != self.grid[t - 1]:
            # max_t = t
            t += 1

        self.time_arrived[robot_id] = start_time + len(path)
        '''
        new_max_time = max(self.time_arrived)
        while self.max_time > new_max_time:
            del self.grid[self.max_time]
            self.max_time -= 1
        '''

        # old_max_time = self.max_time
        # self.max_time = max_t
        # while old_max_time > self.max_time:
        # del self.grid[old_max_time]
        # old_max_time -= 1

    def pushed_out(self, pos, robot_id):
        """
        returns direction if robot is pushed out by other robot
        otherwise None
        :param pos: (x, y, time)
        """
        t = pos[2]
        r_id, direction = self.get_cell(t+1, pos[:2])
        return direction if r_id is not None and r_id != robot_id else None

    def get_arrival_order(self):
        return self.arrival_order

    def get_time_arrived(self):
        return self.time_arrived
