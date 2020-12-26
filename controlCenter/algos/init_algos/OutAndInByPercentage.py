from algos.initAlgo import InitAlgo
from infrastructure.grid import Grid
from typing import List
from infrastructure.robot import Robot
from dataCollection.preprocess import Preprocess
from defines import *
from utils import *
from random import shuffle, randint, sample, shuffle
import queue
from collections import deque
from dataCollection.Generator import *


class OutAndInByPercentage(InitAlgo):
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

        self.start_fill_from = (-1, -1)
        self.reverse_fill = True
        self.boundaries = {
            "N": self.grid.size + 2,
            "E": self.grid.size + 2,
            "W": -3,
            "S": -3,
        }
        self.percent_to_leave_inside = 0
        self.timeout = 10

        if data_bundle is not None:
            if "start_fill_from" in data_bundle:
                self.start_fill_from = data_bundle["start_fill_from"]
            if "reverse_fill" in data_bundle:
                self.reverse_fill = data_bundle["reverse_fill"]
            if "boundaries" in data_bundle:
                self.boundaries = data_bundle["boundaries"]
            if "percent_to_leave_inside" in data_bundle:
                self.percent_to_leave_inside = data_bundle["percent_to_leave_inside"]

        self.bfs_list = [None] * len(self.robots)
        self.q_by_robot_id = [None] * len(self.robots)

        random_quantity = (len(self.robots) * self.percent_to_leave_inside) // 100
        self.inside_group = sample(range(0, len(self.robots)), random_quantity)

        for i in self.inside_group:
            robot = self.robots[i]
            robot.extra_data = 0

            if robot.robot_arrived():
                self.q_by_robot_id[i] = self.q_arrived
                continue


            self.bfs_list[i] = Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                check_move_func=CheckMoveFunction.cell_free_from_robots_on_target_and_obs,
                calc_configure_value_func=AStarHeuristics.manhattan_distance
            )
            self.q_by_robot_id[i] = self.q_stay_inside

        Generator.calc_bfs_map(grid=self.grid,
                               boundaries={
                                   "N": self.grid.size,
                                   "E": self.grid.size,
                                   "W": 0,
                                   "S": 0},
                               source_container_func=self.create_boundaries_queue,
                               source_container_params=self.grid.size,
                               check_move_func=CheckMoveFunction.check_free_from_obs)

        self.bfs_map_copy = self.grid.get_copy_bfs_map_()


        for robot in self.robots:
            if self.q_by_robot_id[robot.robot_id] in [self.q_stay_inside, self.q_arrived] :
                continue
            robot.extra_data = self.grid.get_cell_distance(robot.pos)
            assert robot.extra_data > 0, "robot" + str(robot) + "can't find a path from pos"
            self.q_by_robot_id[robot.robot_id] = self.q_reaching_out

        # Arranging the robots in a way that the first robot to act will be the ones who want to get out and are closest to the boundaries
        # After all robots who reaching out acted, the ones which stay inside will act
        self.permutation = []
        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)
        permutation_going_out = self.permutation[random_quantity:]
        permutation_stay_in = self.permutation[:random_quantity]
        self.permutation = permutation_going_out + permutation_stay_in
        self.out_permutation = []

        self.last_index_on_the_road = 0

        init_time.end(to_print=self.print_info)

    def step(self) -> int:
        self.phases_timers[self.phase].start()
        moved = self.phases[self.phase]()
        if moved == 0 and len(self.inside_group) == self.grid.numOfRobotsArrived:
            self.phases_timers[self.phase].end(self.print_info)
            if self.phase == 0:
                if self.switch_phase_0_to_1():
                    return self.phases[self.phase]()
                return 0

        return moved

    def step_phase_0(self) -> int:
        """
            Three cases here:
                *robot in pillar: stop moving
                *robot found spot in pillar but haven't arrived yet: keep West
                *robot haven't found a spot: keep North
        """
        moved = 0
        for i in self.out_permutation:
            moved += self.q_by_robot_id[i](self.robots[i])
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0

        temp_permutation = []
        for i in self.permutation:
            moved += self.q_by_robot_id[i](self.robots[i])
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0
            if self.q_by_robot_id[i] == self.q_outside_in_main_road:
                self.out_permutation.append(i)
            else:
                temp_permutation.append(i)

        self.permutation = temp_permutation

        if moved == 0 and self.grid.numOfRobotsArrived < len(self.inside_group):
            moved += self.shake_map()

        return moved

    def shake_robot(self, i: int) -> bool:
        robot = self.robots[i]
        if robot.robot_arrived():
            return True

        self.bfs_list[i] = Generator.calc_a_star_path(
            grid=self.grid,
            boundaries=self.boundaries,
            source_pos=robot.pos,
            dest_pos=robot.target_pos,
            check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs,
            calc_configure_value_func=AStarHeuristics.manhattan_distance
        )
        if self.bfs_list[i] is None:
            self.bfs_list[i] = Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                check_move_func=CheckMoveFunction.cell_free_from_robots_on_target_and_obs,
                calc_configure_value_func=AStarHeuristics.manhattan_distance
            )
            if self.bfs_list[i] is None:
                print("Robot" + str(robot) + "can't find any valid path")
                self.q_by_robot_id[i] = self.q_stuck
                return False

        robot.extra_data //= 2
        return True

    def shake_map(self):
        moved = 0
        shuffle(self.inside_group)
        for i in self.inside_group:
            robot = self.robots[i]
            if robot.robot_arrived():
                continue

            if not self.shake_robot(i):
                return 0

            moved += self.q_by_robot_id[i](self.robots[i])
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0

        return moved

    def switch_phase_0_to_1(self) -> bool:
        self.phase += 1

        blocked = set()
        for r in self.robots:
            blocked.add(r.pos)

        self.permutation.clear()
        if self.grid.has_robot_on_target(self.start_fill_from):
            print("The point to start fill from is full")
            return False

        Generator.calc_bfs_map(grid=self.grid,
                               boundaries=self.boundaries,
                               blocked=blocked,
                               source_container=[self.start_fill_from],
                               check_move_func=CheckMoveFunction.cell_free_from_robots_on_target_and_obs)

        for r in self.robots:
            if r.robot_arrived():
                r.extra_data = -2
                continue
            r.extra_data = self.grid.get_cell_distance(r.target_pos)
            if r.extra_data == -1:
                print(r, "has no clear path to target")
                return False

        self.preprocess.generic_robots_sort(self.permutation, "EXTRA", self.robots)  # sort by dists
        self.permutation = self.permutation[self.grid.numOfRobotsArrived:] # cut the robots that arrived

        if self.reverse_fill:
            self.permutation.reverse()

        for i in self.permutation:
            robot = self.robots[i]
            # TODO: Narrow boundaries
            self.bfs_list[i] = Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                blocked=blocked,
                calc_configure_value_func=AStarHeuristics.manhattan_distance
            )

            # Because distance is not -1, there must be a way
            assert self.bfs_list[i] is not None, "Step 1: can't find any path for robot:" + str(i)
            blocked.add(robot.target_pos)

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

    # states
    def q_reaching_out(self, robot: Robot) -> int:
        next_direction = Generator.get_next_move_by_dist_and_obs_from_bfs_map_copy(self.bfs_map_copy, robot.pos)
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, next_direction, self.current_turn, self.solution):
            robot.extra_data -= 1
            if robot.extra_data == 0:
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_main_road
                robot.extra_data = next_direction
            return 1
        return 0

    def q_outside_in_main_road(self, robot: Robot) -> int:
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, robot.extra_data, self.current_turn, self.solution):
            col = self.get_col_by_direction(robot.extra_data, robot.pos)
            if col % 3 == 0:
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_side_road
            elif col % 3 == 1:
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_left
            else:
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_right
            return 1
        return 0

    def q_outside_in_side_road(self, robot: Robot) -> int:
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "L"), self.current_turn, self.solution):
            self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_right
            return 1
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "R"), self.current_turn, self.solution):
            self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_left
            return 1
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "U"), self.current_turn, self.solution):
            return 1
        return 0

    def q_outside_in_place_right(self, robot: Robot) -> int:
        col = self.get_col_by_direction(robot.extra_data, robot.pos)
        row = self.get_row_by_direction(robot.extra_data, robot.pos)
        assert col % 3 == 2
        moved = 0
        d = robot.extra_data
        if self.grid.has_robot(self.get_main_road_pos(d, robot.pos)) and self.grid.has_robot(sum_tuples(robot.pos,directions_to_coords[self.get_relative_side(d, "D")])):
            # Need to move, to make space:
            # Try to move to the left
            if row == self.boundaries[d] \
                    and not self.grid.has_robot(sum_tuples(robot.pos, sum_tuples(
                            directions_to_coords[self.get_relative_side(d, "L")], directions_to_coords[self.get_relative_side(d, "D")]))) \
                    and InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "L"), self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_left
                moved = 1
            # Moving up
            else:
                moved += InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "U"), self.current_turn, self.solution)

        stretch_param = self.get_row_by_direction(d, robot.pos)
        if d in ["N", "E"]:
            stretch_func = max
        else:
            stretch_func = min

        self.boundaries[d] = stretch_func(self.boundaries[d], stretch_param)
        return moved

    def q_outside_in_place_left(self, robot: Robot) -> int:
        col = self.get_col_by_direction(robot.extra_data, robot.pos)
        row = self.get_row_by_direction(robot.extra_data, robot.pos)
        assert col % 3 == 1
        moved = 0
        d = robot.extra_data
        if self.grid.has_robot(self.get_main_road_pos(d, robot.pos)) and self.grid.has_robot(sum_tuples(robot.pos,directions_to_coords[self.get_relative_side(d, "D")])):
            # Need to move, to make space:
            # Try to move to the Right
            if row == self.boundaries[d] \
                    and not self.grid.has_robot(sum_tuples(robot.pos, sum_tuples(
                            directions_to_coords[self.get_relative_side(d, "R")], directions_to_coords[self.get_relative_side(d, "D")]))) \
                    and InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "R"), self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_right
                moved = 1
            else:
                moved += InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "U"), self.current_turn, self.solution)

        stretch_param = self.get_row_by_direction(d, robot.pos)
        if d in ["N", "E"]:
            stretch_func = max
        else:
            stretch_func = min

        self.boundaries[d] = stretch_func(self.boundaries[d], stretch_param)
        return moved

    def q_stay_inside(self, robot: Robot) -> int:
        """
        The group that starts inside
        robot.extra data = the number of consecutive turns without moving
        :param robot:
        :return:
        """
        i = robot.robot_id
        if robot.extra_data >= self.timeout:
            if not self.shake_robot(i):
                return 0

        if InitAlgo.move_robot_to_dir(i, self.grid, self.bfs_list[i][0], self.current_turn, self.solution):
            self.bfs_list[i].popleft()
            robot.extra_data = 0
            if not self.bfs_list[i]:
                #Empty bfs_list
                assert robot.robot_arrived(), "Robot " + str(robot) + " is With empty bfs list and didn't hit the target."
                self.q_by_robot_id[i] = self.q_arrived
            return 1

        robot.extra_data += 1
        return 0

    def q_arrived(self, robot: Robot) -> int:
        return 0

    def q_stuck(self, robot: Robot) -> int:
        return 0

    # Helpers
    @staticmethod
    def is_pos_out(pos, size):
        return not OutAndInByPercentage.is_pos_inbound(pos, size)

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

    @staticmethod
    def get_relative_side(direction: str, side: str) -> str:
        res_directions = ["N", "W", "S", "E"]
        direction_value = {
            "N": 0,
            "W": 1,
            "S": 2,
            "E": 3
        }
        side_value = {
            "U": 0,
            "L": 1,
            "D": 2,
            "R": 3
        }
        return res_directions[(direction_value[direction] + side_value[side]) % len(res_directions)]

    def get_col_by_direction(self, direction: str, pos) -> int:
        if direction == "N":
            return pos[0]
        if direction == "S":
            return self.grid.size - pos[0]
        if direction == "W":
            return pos[1]
        if direction == "E":
            return self.grid.size - pos[1]

        assert 0, "get_col_by_direction: direction not valid"

    def get_row_by_direction(self, direction: str, pos) -> int:
        if direction == "N":
            return pos[1]
        if direction == "S":
            return (-1)*pos[1]
        if direction == "E":
            return pos[0]
        if direction == "W":
            return (-1)*pos[0]

    def get_main_road_pos(self, direction: str, pos) -> (int, int):
        road_by_direction = {
            "N": (pos[0], self.grid.size),
            "S": (pos[0], -1),
            "E": (self.grid.size, pos[1]),
            "W": (-1, pos[1])
        }
        return road_by_direction[direction]