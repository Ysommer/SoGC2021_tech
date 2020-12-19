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

    def __init__(self, instance_name: str, grid: Grid, targets: list, max_makespan: int = None, max_sum: int = None, preprocess: Preprocess = None,
                 name="", start_fill_from=(0, 0), reverse_fill=True, boundaries=None):
        super().__init__(instance_name,grid, targets, max_makespan, max_sum, preprocess, "OutAndInBFS" + name)
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
        self.start_fill_from = start_fill_from
        self.reverse_fill = reverse_fill
        if boundaries is None:
            boundaries = {
                "N": self.grid.size + 1,
                "E": self.grid.size + 1,
                "W": -2,
                "S": -2,
            }

        self.boundaries = boundaries

        self.bfs_list = []
        for i in range(len(self.robots)):
            robot = self.robots[i]
            self.bfs_list.append(
                Generator.get_bfs_path(
                    source_pos=robot.pos,
                    dest_params=self.grid.size,
                    clear_cell_params=self.grid,
                    boundaries=self.boundaries,
                    dest_key=self.is_pos_out,
                    clear_cell_key=Generator.cell_is_clear_ignore_robots_not_on_target
                )
            )
            if len(self.bfs_list[-1]) == 0:
                print("Can't find any bfs for robot", i)
                assert 0
            self.robots[i].extra_data = len(self.bfs_list[-1])

        self.permutation = []
        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)

        self.last_index_on_the_road = 0
        self.max_dist_from_zero = 0

    @staticmethod
    def is_pos_out(pos, size):
        return not (-1 <= pos[0] < size + 1 and -1 <= pos[1] < size + 1)

    def is_pos_on_target(self, i, pos):
        return pos == tuple(self.targets[i])

    def get_robot_area(self, robot: Robot):
        pos = robot.pos
        if not self.is_pos_out(pos, self.grid.size):
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

    def step(self) -> int:
        moved = self.phases[self.phase]()
        if self.phase == 0:
            if moved == 0:
                self.switch_phase_0_to_1()
                return self.phases[self.phase]()

            if self.robots_outside == len(self.robots):
                self.switch_phase_0_to_1()

        return moved

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
                if self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["S"])).has_robot():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution)
                elif robot.pos[0] % 3 == 1:
                    if InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution):
                        moved += 1

            elif area == "S":
                if self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["N"])).has_robot():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution)
                elif robot.pos[0] % 3 == 1:
                    if InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution):
                        moved += 1

            elif area == "W":
                if self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["E"])).has_robot():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution)
                elif robot.pos[1] % 3 == 1:
                    if InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "W", self.current_turn, self.solution):
                        moved += 1

            elif area == "E":
                if self.grid.get_cell(sum_tuples(robot.pos, directions_to_coords["W"])).has_robot():
                    moved += InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution)
                elif robot.pos[1] % 3 == 1:
                    if InitAlgo.move_robot_to_dir(i, self.grid, "N", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "S", self.current_turn, self.solution):
                        moved += 1
                    elif InitAlgo.move_robot_to_dir(i, self.grid, "E", self.current_turn, self.solution):
                        moved += 1

            self.boundaries["W"] = min(self.boundaries["W"], robot.pos[0] - 1)
            self.boundaries["S"] = min(self.boundaries["S"], robot.pos[1] - 1)
            self.boundaries["N"] = max(self.boundaries["N"], robot.pos[1] + 1)
            self.boundaries["E"] = max(self.boundaries["E"], robot.pos[0] + 1)

        return moved

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

    def switch_phase_0_to_1(self):
        self.phase += 1
        blocked = [r.pos for r in self.robots]

        self.permutation.clear()
        dests = Generator.calc_travel_distance(
            source_pos=self.start_fill_from,
            dest_params=self.targets,
            clear_cell_params=self.grid,
            boundaries=self.boundaries,
            out_size=len(self.targets),
            blocked=blocked,
            clear_cell_key=Generator.cell_is_clear_ignore_robots_not_on_target
        )

        assert dests is not None

        for i in range(len(dests)):
            self.robots[i].extra_data = dests[i]

        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)     # sort by dists

        if self.reverse_fill:
            self.permutation.reverse()
        self.max_dist_from_zero = self.robots[self.permutation[0]].extra_data

        for i in self.permutation:
            robot = self.robots[i]
            self.bfs_list[i].clear()
            self.bfs_list[i] = Generator.get_bfs_path(
                source_pos=robot.pos,
                dest_params=robot.target_pos,
                clear_cell_params=self.grid,
                boundaries=self.boundaries,
                blocked=blocked
            )
            if len(self.bfs_list[i]) == 0:
                print("Step 2: can't find any path for robot", str(i))
                assert 0
            blocked.append(robot.target_pos)
