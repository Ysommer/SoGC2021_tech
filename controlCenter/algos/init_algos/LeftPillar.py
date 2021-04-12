from algos.initAlgo import InitAlgo
from infrastructure.grid import Grid
from typing import List
from infrastructure.robot import Robot
from dataCollection.preprocess import Preprocess
from defines import *
from utils import *

X_PILLAR_LOC = -3
X_SOUTH_ROAD = -2
X_NORTH_ROAD = -1


class LeftPillar(InitAlgo):

    def __init__(self, instance_name: str, grid: Grid, targets: list, max_makespan: int = None, max_sum: int = None, preprocess: Preprocess = None, name="", print_info=True, data_bundle = None):
        super().__init__(instance_name, grid, targets, max_makespan, max_sum, preprocess, "LeftPillar")
        """
            phases:
            0: push all robots to a left pillar at x = X_PILLAR_LOC
            1: move robots to targets
        """
        self.phase = 0
        self.num_of_robots_arrived_to_pillar = 0
        global X_PILLAR_LOC
        global X_SOUTH_ROAD
        global X_NORTH_ROAD

        # Set robots their y position in the pillar
        robots_dests_temp = preprocess.sort_R_y_x()
        for y in range(len(robots_dests_temp)):
            self.robots[robots_dests_temp[y]].extra_data = y

        self.robots_permutation = preprocess.sort_R_Y_x()

        # For step 1 exits
        self.x_out = 0
        self.in_north_road = 0
        self.in_south_road = 0

    def step(self) -> int:
        if self.phase == 0 and len(self.robots) == self.num_of_robots_arrived_to_pillar:
            self.phase = 1
            self.robots_permutation.clear()
            self.preprocess.generic_robots_sort(self.robots_permutation, "T_X_y", self.robots)
            self.x_out = self.robots[self.robots_permutation[0]].target_pos[0]

        if self.phase == 0:
            return self.step_phase_0()
        else:
            return self.step_phase_1()

    def step_phase_0(self) -> int:
        """
            Three cases here:
                *robot in pillar: stop moving
                *robot found spot in pillar but haven't arrived yet: keep West
                *robot haven't found a spot: keep North
        """
        changed = 0
        for i in range(len(self.robots_permutation)):
            id = self.robots_permutation[i]
            robot = self.robots[id]

            # first case - robot in pillar
            if robot.pos[0] == X_PILLAR_LOC:
                continue

            # second case - robot moves to his spot
            if robot.pos[1] == robot.extra_data:
                if InitAlgo.move_robot_to_dir(id, self.grid, 'W', self.current_turn, self.solution):
                    changed += 1
                    if robot.pos[0] == X_PILLAR_LOC:
                        self.num_of_robots_arrived_to_pillar += 1

            # third case - robot moves north looking for a spot
            elif robot.pos[1] < robot.extra_data:
                if InitAlgo.move_robot_to_dir(id, self.grid, "N", self.current_turn, self.solution):
                    changed += 1
            else:
                if InitAlgo.move_robot_to_dir(id, self.grid, "S", self.current_turn, self.solution):
                    changed += 1

        return changed

    def step_phase_1(self) -> int:
        """
        Three cases here:
            *robot in pillar: go East to a road
            *robot in road: adjust y to target
            *robot in the way (x>=0) go East
        """
        changed = 0

        for i in self.robots_permutation:
            robot = self.robots[i]
            robot_x = robot.pos[0]
            if robot.robot_arrived():
                continue

            if robot_x >= 0:
                # in the way (X >= 0)
                if InitAlgo.move_robot_to_dir(i, self.grid, 'E', self.current_turn, self.solution):
                    changed += 1

            elif robot_x == X_SOUTH_ROAD:
                if robot.pos[1] <= robot.target_pos[1]:
                    if InitAlgo.move_robot_to_dir(i, self.grid, 'E', self.current_turn, self.solution):
                        changed += 1
                        self.in_south_road -= 1
                        self.in_north_road += 1
                else:
                    if InitAlgo.move_robot_to_dir(i, self.grid, 'S', self.current_turn, self.solution):
                        changed += 1

            elif robot_x == X_NORTH_ROAD:
                assert robot.pos[1] <= robot.target_pos[1]
                if robot.pos[1] == robot.target_pos[1]:
                    if InitAlgo.move_robot_to_dir(i, self.grid, 'E', self.current_turn, self.solution):
                        changed += 1
                        self.in_north_road -= 1
                else:
                    if InitAlgo.move_robot_to_dir(i, self.grid, 'N', self.current_turn, self.solution):
                        changed += 1

            elif robot_x == X_PILLAR_LOC:
                # check if roads are empty
                if self.in_north_road == 0 and self.in_south_road == 0:
                    self.x_out = robot.target_pos[0]

                if robot.target_pos[0] < self.x_out:
                    # roads occupied
                    break

                if InitAlgo.move_robot_to_dir(i, self.grid, 'E', self.current_turn, self.solution):
                    changed += 1
                    self.in_south_road += 1

        return changed