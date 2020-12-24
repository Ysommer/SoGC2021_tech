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
from dataCollection.Generator import Generator


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
        super().__init__(instance_name, grid, targets, max_makespan, max_sum, preprocess, "OutAndInBFS" + name,
                         print_info)
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
        sources = []
        # N & S
        for i in range(0, self.grid.size):
            sources.append((i, -1))
            sources.append((i, self.grid.size))
            sources.append((-1, i))
            sources.append((self.grid.size, i))

        Generator.calc_bfs_map(sources=sources,
                               grid=self.grid,
                               boundaries={
                                   "N": self.grid.size,
                                   "E": self.grid.size,
                                   "W": 0,
                                   "S": 0},
                               clear_cell_key=Generator.cell_is_clear_from_obs)

        for robot in self.robots:
            dist = self.grid.get_cell_distance(robot.pos)
            if dist == -1:
                print("robot", robot.robot_id, "can't find a path from pos", robot.pos)
                assert 0

            robot.extra_data = self.grid.get_cell_for_bfs(robot.pos).last_configured_dist

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
                if self.switch_phase_0_to_1():
                    return self.phases[self.phase]()

        return moved

    def step_phase_0(self) -> int:
        """
            Three cases here:
                *robot in pillar: stop moving
                *robot found spot in pillar but haven't arrived yet: keep West
                *robot haven't found a spot: keep North
        """
        moved = 0

        for i in self.off_boundaries_groups["N"]:
            robot = self.robots[i]
            if self.grid.get_cell((robot.pos[0], self.grid.size)).has_robot() is not None:
                if robot.pos[1] >= self.boundaries["N"] - 1 :
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
            if self.grid.get_cell((robot.pos[0], -1)).has_robot() is not None:
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
            if self.grid.get_cell((-1, robot.pos[1])).has_robot() is not None:
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
            if self.grid.get_cell((self.grid.size, robot.pos[1])).has_robot() is not None:
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

            for d in directions_to_coords.keys():
                next_pos = sum_tuples(robot.pos, directions_to_coords[d])
                if Generator.cell_is_not_an_obs(next_pos, self.grid):
                    next_dist = self.grid.get_cell_for_bfs(next_pos).last_configured_dist
                    if next_dist < robot.extra_data:
                        if InitAlgo.move_robot_to_dir(i, self.grid, d,
                                                      self.current_turn, self.solution):
                            moved += 1
                            robot.extra_data -= 1
                            if robot.extra_data == 0:
                                self.off_boundaries_groups[d].append(i)
                            break

        return moved

    def switch_phase_0_to_1(self) -> bool:
        self.phase += 1

        blocked = {}
        for r in self.robots:
            blocked[r.pos] = None

        self.permutation.clear()
        dests = Generator.calc_travel_distance(
            source_pos=self.start_fill_from,
            grid=self.grid,
            dest_params=self.targets,
            boundaries=self.boundaries,
            out_size=len(self.targets),
            blocked=blocked,
            clear_cell_key=Generator.cell_is_clear_ignore_robots_not_on_target
        )

        if dests is None:
            print("dests is None")
            return False

        for i in range(len(dests)):
            self.robots[i].extra_data = dests[i]

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
                """
                if robot.pos[0] % 3 == 1:
                    self.bfs_list[i].append("W")
                elif robot.pos[0] % 3 == 2:
                    self.bfs_list[i].append("E")

                for y in range(robot.pos[1], boundaries["N"]-1, -1):
                    self.bfs_list[i].append("S")"""

                boundaries["N"] = robot.pos[1]
            elif i in self.off_boundaries_groups["S"]:
                boundaries["S"] = robot.pos[1]
            elif i in self.off_boundaries_groups["E"]:
                boundaries["E"] = robot.pos[0]
            elif i in self.off_boundaries_groups["W"]:
                boundaries["W"] = robot.pos[0]
            else:
                print("Robot out of any off_boundaries_groups")
                return False

            self.bfs_list[i] = Generator.get_bfs_path(
                source_pos=robot.pos,
                grid=self.grid,
                dest_params=robot.target_pos,
                boundaries=boundaries,
                blocked=blocked
            )
            if len(self.bfs_list[i]) == 0:
                print("Step 1: can't find any path for robot", str(i))
                return False
            blocked[robot.target_pos] = None

        return True

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

    def get_robot_area(self, robot: Robot):
        pos = robot.pos
        if self.is_pos_inbound(pos, self.grid.size):
            return "X"

        if pos[1] >= self.grid.size:
            return "N"
        if pos[1] < 0:
            return "S"
        if pos[0] < 0:
            return "W"
        if pos[0] >= self.grid.size:
            return "E"

        assert 0

    def legal_bfs_step(self, pos: (int, int)) -> bool:
        next_cell = self.grid.get_cell(pos)
        if next_cell.is_obs() or next_cell.has_robot_on_target():
            return False
        return -10 <= pos[0] <= self.grid.size + 9 and -10 <= pos[1] <= self.grid.size + 9

    @staticmethod
    def is_pos_out(pos, size):
        return not OutAndInBFS.is_pos_inbound(pos, size)

    @staticmethod
    def is_pos_inbound(pos, size):
        return -1 <= pos[0] <= size and -1 <= pos[1] <= size
