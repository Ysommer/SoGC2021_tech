from ..initAlgo import InitAlgo
from ...infrastructure.grid import Grid
from typing import List
from ...infrastructure.robot import Robot
from ...dataCollection.preprocess import Preprocess
from ...defines import *
from ...utils import *


class LeftPillar(InitAlgo):
    X_PILLAR_LOC = -2

    def __init__(self, instance_name: str, grid: Grid, robots: List[Robot], targets: list, max_makespan: int = None, max_sum: int = None, preprocess: Preprocess = None):
        super().__init__(instance_name,grid, robots, targets, max_makespan, max_sum, preprocess)
        """
            phases:
            0: push all robots to a left pillar at x = X_PILLAR_LOC
            1: move robots to targets
        """
        self.phase = 0
        self.robots_permutation = preprocess.sort_robots_by_y_than_by_x(robots)
        self.num_of_robots_arrived_to_pillar = 0
        global X_PILLAR_LOC

    def step(self) -> int:
        if self.phase == 0 and len(self.robots) == self.num_of_robots_arrived_to_pillar:
            self.phase = 1
            self.robots_permutation = self.preprocess.sort_robots_by_target_x().reverse()

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
        for i in self.robots_permutation:
            robot = self.robots[i]

            # first case - robot in pillar
            if robot.pos[0] == X_PILLAR_LOC:
                continue

            # second case - robot moves to his spot
            if robot.pos[1] == i:
                if move_robot_to_dir(i, self.grid, 'W', self.current_turn, self.solution):
                    changed += 1
                    if robot.pos[0] == X_PILLAR_LOC:
                        self.num_of_robots_arrived_to_pillar += 1

            # third case - robot moves north looking for a spot
            elif robot.pos[1] < i:
                if move_robot_to_dir(i, self.grid, 'N', self.current_turn, self.solution):
                    changed += 1
            else:
                if move_robot_to_dir(i, self.grid, 'S', self.current_turn, self.solution):
                    changed += 1

        return changed

    def step_phase_1(self) -> int:
        """
        Three cases here:
            *robot in pillar: go East to road (x = -1)
            *robot in road: adjust y to target
            *robot in the way (x>=0) go East
        """
        changed = 0
        for i in self.robots_permutation:
            robot = self.robots[i]

            # in pillar
            if robot.pos[0] == X_PILLAR_LOC:
                if move_robot_to_dir(i, self.grid, 'E', self.current_turn, self.solution):
                    changed += 1
                break

            # in road
            if robot.pos[0] == X_PILLAR_LOC+1:
                # exits road, new robot can enter
                if robot.pos[1] == robot.target_pos[1]:
                    if move_robot_to_dir(i, self.grid, 'E', self.current_turn, self.solution):
                        changed += 1
                    continue
                # adjusting y
                elif robot.pos[1] > robot.target_pos[1]:
                    if move_robot_to_dir(i, self.grid, 'S', self.current_turn, self.solution):
                        changed += 1
                    break
                else:
                    if move_robot_to_dir(i, self.grid, 'N', self.current_turn, self.solution):
                        changed += 1
                    break

            # in the way (X >= 0)
            if move_robot_to_dir(i, self.grid, 'E', self.current_turn, self.solution):
                changed += 1

        return changed