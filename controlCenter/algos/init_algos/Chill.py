from algos.initAlgo import InitAlgo
from infrastructure.grid import Grid
from typing import List
from infrastructure.robot import Robot
from dataCollection.preprocess import Preprocess
from defines import *
from utils import *
from random import shuffle, randint, sample, shuffle, random
import queue
from collections import deque
from dataCollection.Generator import *


class Chill(InitAlgo):
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

        # Parse data bundle
        if data_bundle is None:
            data_bundle = {}

        self.boundaries = data_bundle.get("boundaries", {
            "N": self.grid.size + 2,
            "E": self.grid.size + 2,
            "W": -3,
            "S": -3,
        })
        self.percent_to_leave_inside = data_bundle.get("percent_to_leave_inside", 0)
        self.secondary_order = data_bundle.get("secondary_order", "")
        self.descending_order = data_bundle.get("descending_order", False)
        self.break_time = data_bundle.get("break_time", 100)
        self.calcs_per_high = data_bundle.get("calcs_per_high", 10)

        # Init phases
        self.phase = 0
        self.phases = [
            self.step_phase_0,
            self.step_phase_1,
            self.step_phase_2
        ]

        # Set Name
        # All Phases params
        if self.percent_to_leave_inside > 0:
            self.name += "_per_" + str(self.percent_to_leave_inside)

        if self.secondary_order != "":
            self.name += "_" + self.secondary_order

        if self.descending_order:
            self.name += "_desc"

        self.solution.out["algo_name"] = self.name

        # Set robots to groups
        self.q_by_robot_id = [self.q_reaching_out for i in range(len(self.robots))]
        self.inside_group = []
        self.inside_group_set = set()
        self.out_of_boundaries_permutation = []
        self.reaching_out_permutation = []
        self.in_way_to_target = []
        self.get_inside_algos = []
        self.get_inside_groups = self.get_inside_groups = [[] for i in range(self.grid.size)]
        self.current_group = 0
        self.current_robot = 0

        self.max_height = -1
        self.set_inside_group()

        for i in self.inside_group:
            robot = self.robots[i]
            robot.extra_data = 0
            self.inside_group_set.add(robot.pos)  # For BFS MAP
            self.q_by_robot_id[i] = self.q_stay_inside

        # Calc BFS map outside
        Generator.calc_bfs_map(grid=self.grid,
                               boundaries={
                                   "N": self.grid.size,
                                   "E": self.grid.size,
                                   "W": 0,
                                   "S": 0},
                               blocked=self.inside_group_set,
                               source_container_func=self.create_boundaries_queue,
                               source_container_params=self.grid.size,
                               check_move_func=CheckMoveFunction.check_free_from_obs)

        self.bfs_map_copy = self.grid.get_copy_bfs_map()
        temp_robot_list = []

        for robot in self.robots:
            # Ignoring the robots that stay inside
            if self.q_by_robot_id[robot.robot_id] in [self.q_stay_inside, self.q_arrived]:
                continue

            # Check every robot is reachable
            assert self.grid.get_cell_distance(robot.pos) > 0, str(robot) + " is unreachable"

            # Extra data = distance
            robot.extra_data = self.grid.get_cell_distance(robot.pos)
            temp_robot_list.append(robot)

        # Arranging by distance in increasing order
        self.preprocess.generic_robots_sort(self.reaching_out_permutation, "EXTRA", temp_robot_list)

        self.bfs_path = [[] for i in self.robots]

    def set_inside_group(self):
        num_of_robots_inside = (len(self.robots) * self.percent_to_leave_inside) // 100

        Generator.calc_sea_level_bfs_map(grid=self.grid,
                                         boundaries=self.boundaries,
                                         check_move_func=CheckMoveFunction.check_free_from_obs)

        temp_robots = []

        for robot in self.robots:
            robot.extra_data = self.get_extra_data(robot)
            assert robot.extra_data[0] > -1, str(robot) + "has no clear path to target"

            # For later
            self.get_inside_groups[robot.extra_data[0]].append(robot.robot_id)
            if self.max_height < robot.extra_data[0] or self.max_height == -1:
                self.max_height = robot.extra_data[0]


            temp_robots.append(robot)



        self.preprocess.generic_robots_sort(self.inside_group, "EXTRA", temp_robots)  # sort by highs
        self.inside_group.reverse()
        self.inside_group = self.inside_group[:num_of_robots_inside]

        self.get_inside_groups = self.get_inside_groups[:self.max_height + 1]
        self.get_inside_groups.reverse()

    def step(self) -> int:
        moved = self.phases[self.phase]()
        if moved == 0 and self.phase < len(self.phases) - 1:
            self.switch_phase()
            return self.step()
        return moved

    def switch_phase(self):
        if self.phase == 0:
            self.switch_phase_0_to_1()
        else:
            self.switch_phase_1_to_2()

        self.phase += 1

    def step_phase_0(self) -> int:
        """
            Three cases here:
                *robot in pillar: stop moving
                *robot found spot in pillar but haven't arrived yet: keep West
                *robot haven't found a spot: keep North
        """
        moved = 0
        for i in self.out_of_boundaries_permutation:
            moved += self.q_by_robot_id[i](self.robots[i])
            # Is this check necessary?
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0

        temp_permutation = []  # To delete the robots that went outside fast
        for i in self.reaching_out_permutation:
            moved += self.q_by_robot_id[i](self.robots[i])
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0
            if self.q_by_robot_id[i] == self.q_outside_in_main_road:
                self.out_of_boundaries_permutation.append(i)
            else:
                temp_permutation.append(i)

        self.reaching_out_permutation = temp_permutation

        return moved

    def switch_phase_0_to_1(self):
        pass

    def step_phase_1(self) -> int:
        all_clear = True
        for i in self.inside_group:
            robot = self.robots[i]
            self.bfs_path[i] = Generator.calc_a_star_path(self.grid,
                                                          self.boundaries,
                                                          source_pos=robot.pos,
                                                          dest_pos=robot.target_pos,
                                                          calc_configure_value_func=AStarHeuristics.manhattan_distance,
                                                          check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs)
            if self.bfs_path[i] is None:
                all_clear = False
                break

        if all_clear:
            return 0

        moved = 0
        for i in self.inside_group:
            moved += self.q_by_robot_id[i](self.robots[i])
        return moved

    def switch_phase_1_to_2(self):
        def take_a_break():
            for i in range(self.break_time):
                self.solution.out["steps"].append({})

            self.current_turn += self.break_time

        def get_group_sum(group, blocked) -> int:
            sum = 0
            for robot_id in group:
                robot = self.robots[robot_id]
                self.bfs_path[robot_id] = Generator.calc_a_star_path(self.grid,
                                                                     self.boundaries,
                                                                     source_pos=robot.pos,
                                                                     dest_pos=robot.target_pos,
                                                                     blocked=blocked,
                                                                     calc_configure_value_func=AStarHeuristics.manhattan_distance,
                                                                     check_move_func=CheckMoveFunction.check_free_from_obs)

                blocked.add(robot.target_pos)
                assert self.bfs_path[i] is not None, "switch_phase_1_to_2: robot" + str(robot)
                sum += len(self.bfs_path[robot_id])

            return sum

        def sort_group_by_extra_data(group):
            temp = []
            for robot_id in group:
                robot = self.robots[robot_id]
                robot.extra_data = self.get_extra_data(robot)
                temp.append(robot)

            self.preprocess.generic_robots_sort(group, "EXTRA", temp)  # sort by highs

        take_a_break()

        algo_preference = [
            ("dist_from_target", True),
            ("", False),
            ("dist_from_grid", False),
            ("dist_BFS", True),
            ("dist_from_grid", True),
            ("rand", False)
        ]

        blocked = set()

        for group_id in range(len(self.get_inside_groups)):
            group = self.get_inside_groups[group_id]
            min_sum = -1
            min_algo = algo_preference[0]
            for i in range(self.calcs_per_high):
                # Clean blocked from current group
                for robot_id in group:
                    try:
                        blocked.remove(self.robots[robot_id].target_pos)
                    except:
                        break

                temp_algo = algo_preference[min(i, len(algo_preference) - 1)]
                self.secondary_order = temp_algo[0]
                self.descending_order = temp_algo[1]

                sort_group_by_extra_data(group)
                temp_sum = get_group_sum(group, blocked)

                if temp_sum < min_sum or min_sum == -1:
                    min_sum = temp_sum
                    min_algo = temp_algo

            self.get_inside_algos.append(min_algo)

        # To calculate BFS lists
        blocked.clear()
        for group_id in range(len(self.get_inside_groups)):
            group = self.get_inside_groups[group_id]
            self.secondary_order = self.get_inside_algos[group_id][0]
            self.descending_order = self.get_inside_algos[group_id][1]
            sort_group_by_extra_data(group)
            for robot_id in group:
                robot = self.robots[robot_id]
                self.bfs_path[robot_id] = Generator.calc_a_star_path(self.grid,
                                                                     self.boundaries,
                                                                     source_pos=robot.pos,
                                                                     dest_pos=robot.target_pos,
                                                                     blocked=blocked,
                                                                     calc_configure_value_func=AStarHeuristics.manhattan_distance,
                                                                     check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs)

                blocked.add(robot.target_pos)
                assert self.bfs_path[robot_id] is not None, "switch_phase_1_to_2: robot" + str(robot)

    def step_phase_2(self) -> int:
        if self.current_group == len(self.get_inside_groups):
            return 0

        moved = 0
        moving_robot_id = self.get_inside_groups[self.current_group][self.current_robot]
        robot = self.robots[moving_robot_id]

        # Making sure BFS list isn't empty nor None
        assert self.bfs_path[moving_robot_id] and self.bfs_path[moving_robot_id] is not None

        if InitAlgo.move_robot_to_dir(moving_robot_id, self.grid, self.bfs_path[moving_robot_id][0],
                                      self.current_turn, self.solution):
            self.bfs_path[moving_robot_id].popleft()
            moved = 1

        if robot.robot_arrived():
            self.current_robot += 1
            if self.current_robot == len(self.get_inside_groups[self.current_group]):
                self.current_robot = 0
                self.current_group += 1

            self.q_by_robot_id[moving_robot_id] = self.q_arrived
            self.arrived_order.append(robot.robot_id)
            self.time_arrived[robot.robot_id] = self.current_turn + 1

        return moved

    # states
    def q_reaching_out(self, robot: Robot) -> int:
        next_directions = Generator.get_next_move_by_dist_and_obs_from_bfs_map_copy(self.bfs_map_copy, robot.pos)
        for next_dir in next_directions:
            if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, next_dir, self.current_turn, self.solution):
                robot.extra_data -= 1
                if robot.extra_data == 0:
                    self.q_by_robot_id[robot.robot_id] = self.q_outside_in_main_road
                    robot.extra_data = next_dir
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
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "L"),
                                      self.current_turn, self.solution):
            self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_right
            return 1
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "R"),
                                      self.current_turn, self.solution):
            self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_left
            return 1
        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "U"),
                                      self.current_turn, self.solution):
            self.stretch_boundaries(robot)
            return 1
        return 0

    def q_outside_in_place_right(self, robot: Robot) -> int:
        col = self.get_col_by_direction(robot.extra_data, robot.pos)
        row = self.get_row_by_direction(robot.extra_data, robot.pos)
        assert col % 3 == 2
        moved = 0
        d = robot.extra_data
        if self.grid.has_robot(self.get_main_road_pos(d, robot.pos)) and self.grid.has_robot(
                sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "D")])):
            # Need to move, to make space:
            # Try to move to the left
            if row == self.boundaries[d] \
                    and not self.grid.has_robot(sum_tuples(robot.pos, sum_tuples(
                directions_to_coords[self.get_relative_side(d, "L")],
                directions_to_coords[self.get_relative_side(d, "D")]))) \
                    and InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "L"),
                                                   self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_left
                moved = 1
            # Moving up
            else:
                moved += InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "U"),
                                                    self.current_turn, self.solution)

        self.stretch_boundaries(robot)
        return moved

    def q_outside_in_place_left(self, robot: Robot) -> int:
        col = self.get_col_by_direction(robot.extra_data, robot.pos)
        row = self.get_row_by_direction(robot.extra_data, robot.pos)
        assert col % 3 == 1
        moved = 0
        d = robot.extra_data
        if self.grid.has_robot(self.get_main_road_pos(d, robot.pos)) and self.grid.has_robot(
                sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "D")])):
            # Need to move, to make space:
            # Try to move to the Right
            if row == self.boundaries[d] \
                    and not self.grid.has_robot(sum_tuples(robot.pos, sum_tuples(
                directions_to_coords[self.get_relative_side(d, "R")],
                directions_to_coords[self.get_relative_side(d, "D")]))) \
                    and InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "R"),
                                                   self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_right
                moved = 1
            else:
                moved += InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "U"),
                                                    self.current_turn, self.solution)

        self.stretch_boundaries(robot)
        return moved

    def q_stay_inside(self, robot: Robot) -> int:
        if self.phase == 1:
            next_directions = Generator.get_next_move_by_dist_and_obs_from_bfs_map_copy(self.bfs_map_copy, robot.pos)
            for next_dir in next_directions:
                return InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, next_dir, self.current_turn, self.solution)
        return 0

    def q_arrived(self, robot: Robot) -> int:
        return 0

    def q_stuck(self, robot: Robot) -> int:
        return 0

    def q_waiting_outside(self, robot: Robot) -> int:
        return 0

    # Helpers
    def get_extra_data(self, robot: Robot) -> (int, int):
        def get_dist_from_grid(robot: Robot) -> int:
            res = {
                "W": abs(robot.pos[0]),
                "S": abs(robot.pos[1]),
                "E": abs(robot.pos[0] - self.grid.size),
                "N": abs(robot.pos[1] - self.grid.size)
            }
            try:
                return res[robot.extra_data]
            except:
                return 0

        def get_dist_from_target(robot: Robot) -> int:
            return AStarHeuristics.manhattan_distance(robot.pos, robot.pos, robot.target_pos)

        def get_bfs_dist(robot: Robot) -> int:
            return len(Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                calc_configure_value_func=AStarHeuristics.manhattan_distance,
                check_move_func=CheckMoveFunction.cell_free_from_robots_on_target_and_obs))

        # Set secondary order
        extra_data = (self.grid.get_cell_distance(robot.target_pos), 0)
        if self.secondary_order == "rand":
            extra_data = (extra_data[0], random())
        elif self.secondary_order == "dist_from_grid":
            extra_data = (extra_data[0], get_dist_from_grid(robot))
        elif self.secondary_order == "dist_from_target":
            extra_data = (extra_data[0], get_dist_from_target(robot))
        elif self.secondary_order == "dist_BFS":
            extra_data = (extra_data[0], get_bfs_dist(robot))
        else:
            extra_data = (extra_data[0], 0)

        if not self.descending_order:
            extra_data = (extra_data[0], (-1) * extra_data[1])

        return extra_data

    @staticmethod
    def is_pos_out(pos, size):
        return not Chill.is_pos_inbound(pos, size)

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
            return (-1) * pos[1]
        if direction == "E":
            return pos[0]
        if direction == "W":
            return (-1) * pos[0]

    def get_main_road_pos(self, direction: str, pos) -> (int, int):
        road_by_direction = {
            "N": (pos[0], self.grid.size),
            "S": (pos[0], -1),
            "E": (self.grid.size, pos[1]),
            "W": (-1, pos[1])
        }
        return road_by_direction[direction]

    def stretch_boundaries(self, robot):
        if robot.extra_data == "N":
            self.boundaries["N"] = max(self.boundaries["N"], robot.pos[1])
        elif robot.extra_data == "W":
            self.boundaries["W"] = min(self.boundaries["W"], robot.pos[0])
        elif robot.extra_data == "S":
            self.boundaries["S"] = min(self.boundaries["S"], robot.pos[1])
        elif robot.extra_data == "E":
            self.boundaries["E"] = max(self.boundaries["E"], robot.pos[0])
