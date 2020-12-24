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
from dataCollection.Generator import *


class OutAndInBFS(InitAlgo):
    def __init__(self, instance_name: str,
                 grid: Grid,
                 targets: list,
                 max_makespan: int = -1,
                 max_sum: int = -1,
                 preprocess: Preprocess = None,
                 name="",
                 print_info=True,
                 data_bundle=None):
        super().__init__(instance_name, grid, targets, max_makespan, max_sum, preprocess,
                         self.__class__.__name__ + name, print_info)
        """
            phases:
            0: push all robots outside the board
            1: move robots to targets with a BFS
        """
        init_time = Timer(self.name + " init")
        init_time.start()

        self.phase = 0
        self.phases = [
            self.step_phase_0,
            self.step_phase_1
        ]
        self.phases_timers = [
            Timer("phase 0 timer"),
            Timer("phase 1 timer")
        ]

        self.start_fill_from = (0, 0)
        self.reverse_fill = True
        self.boundaries = {
            "N": self.grid.size + 2,
            "E": self.grid.size + 2,
            "W": -3,
            "S": -3,
        }

        if data_bundle is not None:
            if "start_fill_from" in data_bundle:
                self.start_fill_from = data_bundle["start_fill_from"]
            if "reverse_fill" in data_bundle:
                self.reverse_fill = data_bundle["reverse_fill"]
            if "boundaries" in data_bundle:
                self.boundaries = data_bundle["boundaries"]

        self.bfs_list = [None] * len(self.robots)

        Generator.calc_bfs_map(grid=self.grid,
                               boundaries={
                                   "N": self.grid.size,
                                   "E": self.grid.size,
                                   "W": 0,
                                   "S": 0},
                               source_container_func=self.create_boundaries_queue,
                               source_container_params=self.grid.size,
                               check_move_func=CheckMoveFunction.check_free_from_obs)

        for robot in self.robots:
            dist = self.grid.get_cell_distance(robot.pos)
            if dist == -1:
                print("robot", robot.robot_id, "can't find a path from pos", robot.pos)
                assert 0

            robot.extra_data = dist

        self.permutation = []
        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)

        self.last_index_on_the_road = 0
        self.max_dist_from_zero = 0
        self.off_boundaries_groups = {}
        for d in directions_to_coords:
            self.off_boundaries_groups[d] = []

        init_time.end(to_print=self.print_info)

    def step(self) -> int:
        self.phases_timers[self.phase].start()
        moved = self.phases[self.phase]()
        if moved == 0:
            self.phases_timers[self.phase].end(self.print_info)
            if self.phase == 0:
                self.switch_phase_0_to_1()
                return self.phases[self.phase]()

        return moved

    def step_phase_0(self) -> int:
        """
            Three cases here:
                *robot in pillar: stop moving
                *robot found spot in pillar but haven't arrived yet: keep West
                *robot haven't found a spot: keep North
        """

        def move_north_group():
            pass

        moved = 0

        for i in self.off_boundaries_groups["N"]:
            robot = self.robots[i]
            if self.grid.get_cell((robot.pos[0], self.grid.size)).has_robot():
                if robot.pos[1] >= self.boundaries["N"] - 1:
                    if robot.pos[0] % 3 == 1:
                        if InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution):
                            moved += 1
                        else:
                            moved += InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution)
                    elif robot.pos[0] % 3 == 2:
                        if InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution):
                            moved += 1
                        else:
                            moved += InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution)
                    else:
                        moved += InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution)
                else:
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution)
            elif robot.pos[0] % 3 == 0:
                if InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution):
                    moved += 1
            self.boundaries["N"] = max(self.boundaries["N"], robot.pos[1] + 1)

        for i in self.off_boundaries_groups["S"]:
            robot = self.robots[i]
            if self.grid.get_cell((robot.pos[0], -1)).has_robot():
                moved += InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution)
            elif robot.pos[0] % 3 == 0:
                if InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution):
                    moved += 1
            self.boundaries["S"] = min(self.boundaries["S"], robot.pos[1] - 1)

        for i in self.off_boundaries_groups["W"]:
            robot = self.robots[i]
            if self.grid.get_cell((-1, robot.pos[1])).has_robot():
                moved += InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution)
            elif robot.pos[1] % 3 == 0:
                if InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution):
                    moved += 1
            self.boundaries["W"] = min(self.boundaries["W"], robot.pos[0] - 1)

        for i in self.off_boundaries_groups["E"]:
            robot = self.robots[i]
            if self.grid.get_cell((self.grid.size, robot.pos[1])).has_robot():
                moved += InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution)
            elif robot.pos[1] % 3 == 0:
                if InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution):
                    moved += 1
                elif InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution):
                    moved += 1

            self.boundaries["E"] = max(self.boundaries["E"], robot.pos[0] + 1)

        for i in self.permutation:
            robot = self.robots[i]
            if robot.extra_data <= 0:
                continue

            next_direction = Generator.get_next_move_by_dist_and_obs(self.grid, robot.pos)
            if InitAlgo.move_robot_to_dir(i, self.grid, next_direction, self.current_turn, self.solution):
                moved += 1
                robot.extra_data -= 1
                if robot.extra_data == 0:
                    self.off_boundaries_groups[next_direction].append(i)

        return moved

    def switch_phase_0_to_1(self):
        self.phase += 1

        blocked = set()
        for r in self.robots:
            blocked.add(r.pos)

        self.permutation.clear()
        Generator.calc_bfs_map(grid=self.grid,
                               boundaries=self.boundaries,
                               blocked=blocked,
                               source_container=[self.start_fill_from],
                               check_move_func=CheckMoveFunction.check_free_from_obs)

        for r in self.robots:
            r.extra_data = self.grid.get_cell_distance(r.target_pos)

        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)  # sort by dists

        if self.reverse_fill:
            self.permutation.reverse()

        self.max_dist_from_zero = self.robots[self.permutation[0]].extra_data

        for i in self.permutation:
            robot = self.robots[i]

            boundaries = {
                "N": self.grid.size,
                "S": -1,
                "W": -1,
                "E": self.grid.size
            }

            if i in self.off_boundaries_groups["N"]:
                boundaries["N"] = robot.pos[1]
            elif i in self.off_boundaries_groups["S"]:
                boundaries["S"] = robot.pos[1]
            elif i in self.off_boundaries_groups["E"]:
                boundaries["E"] = robot.pos[0]
            elif i in self.off_boundaries_groups["W"]:
                boundaries["W"] = robot.pos[0]
            else:
                assert 0, "Robot out of any off_boundaries_groups"

            self.bfs_list[i] = Generator.get_bfs_path(
                grid=self.grid,
                boundaries=boundaries,
                blocked=blocked,
                source_container=[robot.pos],
                check_if_dest_params=robot.target_pos)

            assert len(self.bfs_list[i]) > 0 , "Step 1: can't find any path for robot"
            blocked.add(robot.target_pos)

    def step_phase_1(self) -> int:
        """
        Three cases here:
            *robot in pillar: go East to a road
            *robot in road: adjust y to target
            *robot in the way (x>=0) go East
        """
        moved = 0

        moving_robot_id = self.permutation[self.last_index_on_the_road]
        robot = self.robots[moving_robot_id]

        if InitAlgo.move_robot_to_dir(moving_robot_id, self.grid, self.bfs_list[moving_robot_id][0],
                                      self.current_turn, self.solution):
            self.bfs_list[moving_robot_id].popleft()
            moved += 1

        if robot.robot_arrived():
            self.last_index_on_the_road += 1

        return moved

    def is_pos_on_target(self, i, pos):
        return pos == tuple(self.targets[i])

    @staticmethod
    def is_pos_out(pos, size):
        return not OutAndInBFS.is_pos_inbound(pos, size)

    @staticmethod
    def is_pos_inbound(pos, size):
        return -1 <= pos[0] <= size and -1 <= pos[1] <= size

    @staticmethod
    def create_boundaries_queue(source_container, source_container_params) -> queue:

        q = queue.Queue()
        # N & S
        for i in range(0, source_container_params):
            q.put((i, -1))
            q.put((i, source_container_params))
            q.put((-1, i))
            q.put((source_container_params, i))

        return q
