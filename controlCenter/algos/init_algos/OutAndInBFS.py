from algos.initAlgo import InitAlgo
from infrastructure.grid import Grid
from typing import List
from infrastructure.robot import Robot
from dataCollection.preprocess import Preprocess
from defines import *
from utils import *
from random import shuffle, randint
import queue
from collections import deque


class OutAndInBFS(InitAlgo):

    def __init__(self, instance_name: str, grid: Grid, targets: list, max_makespan: int = None, max_sum: int = None, preprocess: Preprocess = None):
        super().__init__(instance_name,grid, targets, max_makespan, max_sum, preprocess, "OutAndInBFS")
        """
            phases:
            0: push all robots outside the board
            1: move robots to targets with a BFS
            
            robot.extra_data = state in {0 - on the way out, 1 - waiting outside, 2 - moving to target}
        """
        self.phase = 0
        self.phases = [
            self.step_phase_0,
            self.step_phase_1
        ]
        self.robots_outside = 0

        self.bfs_list = []
        for i in range(len(self.robots)):
            self.bfs_list.append(self.calc_bfs(i, self.is_pos_out))
            self.robots[i].extra_data = len(self.bfs_list[-1])

        self.permutation = []
        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)

        self.last_index_on_the_road = 0
        self.max_dist_from_zero = 0


    def is_pos_out(self, i, pos):
        return not (0 <= pos[0] < self.grid.size and 0 <= pos[1] < self.grid.size)

    def is_pos_on_target(self, i, pos):
        return pos == tuple(self.targets[i])

    def get_robot_area(self, robot: Robot):
        pos = robot.pos
        if pos[1] < 0:
            return "S"
        if pos[1] >= self.grid.size:
            return "N"
        if pos[0] < 0:
            return "W"
        if pos[0] >= self.grid.size:
            return "E"

        return "X"

    def calc_bfs(self, i: int, key, blocked: list = None) -> deque:
        if blocked is None:
            blocked = []

        parents = {self.robots[i].pos: None}
        # visited = [self.robots[i].pos]
        q = queue.Queue()
        q.put(self.robots[i].pos)

        def construct_path(parents: dict, pos: (int, int)) -> deque:
            path = []
            while parents[pos] is not None:
                path.append(parents[pos])
                pos = sub_tuples(pos, directions_to_coords[parents[pos]])

            return deque(path[::-1])  # return reversed path

        while not q.empty():
            pos = q.get()
            if key(i, pos):
                return construct_path(parents, pos)
            for direction in directions_to_coords:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if next_pos not in parents and next_pos not in blocked and self.legal_bfs_step(next_pos):
                    q.put(next_pos)
                    # visited.append(next_pos)
                    parents[next_pos] = direction

        return deque()

    def calc_bfs_dist_from_zero(self):
        been = []
        current_level = []
        next_level = []
        next_level.append((0, 0))
        dist = 0
        number_of_target_founds = 0

        while len(next_level) > 0:
            current_level.clear()
            current_level = next_level
            next_level = []
            dist += 1
            for pos in current_level:
                for direction in directions_to_coords:
                    next_pos = sum_tuples(pos, directions_to_coords[direction])
                    if next_pos not in been and next_pos and self.legal_bfs_step(next_pos):
                        next_level.append(next_pos)
                        been.append(next_pos)
                        if next_pos in self.targets:
                            self.robots[self.targets.index(next_pos)].extra_data = dist
                            number_of_target_founds += 1
                            if number_of_target_founds == len(self.targets):
                                return

        assert 0

    def legal_bfs_step(self, pos: (int, int)) -> bool:
        next_cell = self.grid.get_cell(pos)
        if next_cell.is_obs() or next_cell.has_robot_on_target():
            return False
        return -10 <= pos[0] <= self.grid.size + 9 and -10 <= pos[1] <= self.grid.size + 9

    def step(self) -> int:
        if self.phase == 0 and self.robots_outside == len(self.robots):
            self.switch_phase_0_to_1()

        return self.phases[self.phase]()

    def step_phase_0(self) -> int:
        """
            Three cases here:
                *robot in pillar: stop moving
                *robot found spot in pillar but haven't arrived yet: keep West
                *robot haven't found a spot: keep North
        """
        moved = 0
        for i in self.permutation:
            robot = self.robots[i]
            area = self.get_robot_area(robot)
            if area == "X":
                assert len(self.bfs_list[i]) > 0
                if InitAlgo.move_robot_to_dir(i, self.grid, (self.bfs_list[i])[0],
                                              self.current_turn, self.solution):
                    self.bfs_list[i].popleft()
                    moved += 1
                    if len(self.bfs_list[i]) == 0:
                        self.robots_outside += 1

            if area == "N":
                if robot.pos[1] == self.grid.size or  \
                        not self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["S"])).is_empty():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution)

            if area == "S":
                if robot.pos[1] == -1 or  \
                        not self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["N"])).is_empty():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution)

            if area == "W":
                if robot.pos[0] == -1 or \
                        not self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["E"])).is_empty():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution)

            if area == "E":
                if robot.pos[0] == self.grid.size or  \
                        not self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["W"])).is_empty():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution)

        return moved

    def step_phase_1(self) -> int:
        """
        Three cases here:
            *robot in pillar: go East to a road
            *robot in road: adjust y to target
            *robot in the way (x>=0) go East
        """
        moved = 0
        if self.robots[self.permutation[self.last_index_on_the_road]].robot_arrived():
            self.last_index_on_the_road += 1

        if InitAlgo.move_robot_to_dir(self.permutation[self.last_index_on_the_road], self.grid, self.bfs_list[self.last_index_on_the_road][0],
                                      self.current_turn, self.solution):
            self.bfs_list[self.last_index_on_the_road].popleft()
            moved += 1

        """for i in self.permutation:
            if self.robots[i].robot_arrived():
                continue
            if InitAlgo.move_robot_to_dir(i, self.grid, self.bfs_list[i][0],
                                          self.current_turn, self.solution):
                self.bfs_list[i].popleft()
                moved += 1
        """

        return moved



    def switch_phase_0_to_1(self):
        self.phase += 1
        blocked = [r.pos for r in self.robots]

        self.permutation.clear()
        self.calc_bfs_dist_from_zero()
        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)
        self.permutation.reverse()
        self.max_dist_from_zero = self.robots[self.permutation[0]].extra_data

        for i in self.permutation:
            self.bfs_list[i] = self.calc_bfs(i, self.is_pos_on_target, blocked)
            blocked.append(self.targets[i])