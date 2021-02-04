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

        self.secondary_order = data_bundle.get("secondary_order", "")
        self.descending_order = data_bundle.get("descending_order", False)

        self.break_time = data_bundle.get("break_time", 0)
        self.calcs_per_high = data_bundle.get("calcs_per_high", 10)

        self.percent_to_leave_inside = data_bundle.get("percent_to_leave_inside", 0)

        self.dynamic_percent_to_leave_inside = data_bundle.get("dynamic_percent_to_leave_inside", False)
        self.lower_percent_to_leave_inside = data_bundle.get("lower_percent_to_leave_inside", 0)
        self.upper_percent_to_leave_inside = data_bundle.get("upper_percent_to_leave_inside", 100)
        self.binary_search_iteration = data_bundle.get("binary_search_iteration", 10)
        self.factor_on_binary_search_result = data_bundle.get("factor_on_binary_search_result", 1)

        # Init phases
        self.phase = 0
        self.phases = [
            self.step_phase_0,
            self.step_phase_1,
            self.step_phase_2
        ]

        # Set Name
        # All Phases params

        if self.secondary_order != "":
            self.name += "_" + self.secondary_order

        if self.descending_order:
            self.name += "_desc"

        # Set robots to groups
        self.q_by_robot_id = [self.q_reaching_out for i in range(len(self.robots))]
        self.inside_group = []
        self.inside_group_set = set()       # the positions of the inside group for bfs
        self.out_of_boundaries_permutation = []
        self.reaching_out_permutation = []
        self.get_inside_algos = []
        self.get_inside_groups = self.get_inside_groups = [[] for i in range(self.grid.size)]
        self.bfs_path = [[] for i in self.robots]
        self.bfs_map_copy = None
        self.topo_map_copy = None

        self.max_height = -1
        self.current_group = 0
        self.current_robot = 0

        self.set_maps()
        self.set_groups()

        if self.dynamic_percent_to_leave_inside:
            self.percent_to_leave_inside = self.binary_search_for_best_percent_to_leave_inside()
            if self.percent_to_leave_inside == -1:
                self.force_stop = True
                return
            self.percent_to_leave_inside = int(self.factor_on_binary_search_result * self.percent_to_leave_inside)
            print("percent to leave inside:", self.percent_to_leave_inside)

        self.set_inside_group()
        if not self.check_for_solution():
            self.force_stop = True
            return

        if self.percent_to_leave_inside > 0:
            self.name += "_per_" + str(self.percent_to_leave_inside)

        self.solution.out["algo_name"] = self.name

        # Generator.print_bfs_map_copy(self.bfs_map_copy, self.grid.size)
        # Generator.print_bfs_map_copy_state(self.topo_map_copy, self.grid.size, self.inside_group_set)

    def set_maps(self):
        Generator.calc_sea_level_bfs_map(grid=self.grid,
                                         boundaries=self.boundaries,
                                         check_move_func=CheckMoveFunction.check_free_from_obs)
        self.topo_map_copy = self.grid.get_copy_bfs_map(False)

    def set_groups(self):
        for robot in self.robots:
            dist = self.topo_map_copy[robot.target_pos]
            assert dist > -1, str(robot) + "has no clear path to target"

            # For later
            self.get_inside_groups[dist].append(robot.robot_id)
            if self.max_height < dist or self.max_height == -1:
                self.max_height = dist

        self.get_inside_groups = self.get_inside_groups[1:self.max_height+1]
        self.get_inside_groups.reverse()

    def set_inside_group(self):
        # Clear
        for i in range(len(self.robots)):
            robot = self.robots[i]
            self.q_by_robot_id[i] = self.q_reaching_out

        self.inside_group_set.clear()

        # Calculate num of robots
        num_of_robots_inside = (len(self.robots) * self.percent_to_leave_inside) // 100
        temp_robots = []
        current_group = 0
        current_robot = 0

        # pick robots
        for i in range(num_of_robots_inside):
            robot_id = self.get_inside_groups[current_group][current_robot]
            robot = self.robots[robot_id]
            robot.extra_data = self.get_extra_data(robot)
            temp_robots.append(robot)

            current_robot += 1
            if current_robot == len(self.get_inside_groups[current_group]):
                current_robot = 0
                current_group += 1

        self.preprocess.generic_robots_sort(self.inside_group, "EXTRA", temp_robots)  # sort by highs
        self.inside_group.reverse()

        assert len(self.inside_group) == num_of_robots_inside

        for i in self.inside_group:
            robot = self.robots[i]
            robot.extra_data = 0
            self.inside_group_set.add(robot.pos)        # For BFS MAP
            self.q_by_robot_id[i] = self.q_stay_inside

    def check_for_solution(self) -> bool:
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

        self.bfs_map_copy = self.grid.get_copy_bfs_map(False)
        temp_robot_list = []

        for robot in self.robots:
            # Ignoring the robots that stay inside
            if self.q_by_robot_id[robot.robot_id] == self.q_stay_inside:
                continue

            # Check every robot is reachable
            if self.bfs_map_copy[robot.pos] == -1:
                # Generator.print_bfs_map_copy(self.bfs_map_copy, self.grid.size)
                # Generator.print_bfs_map_copy_state(self.topo_map_copy, self.grid.size, self.inside_group_set, [robot.pos])
                return False

            # Extra data = distance
            robot.extra_data = self.grid.get_cell_distance(robot.pos)
            temp_robot_list.append(robot)

        # Arranging by distance in increasing order
        self.preprocess.generic_robots_sort(self.reaching_out_permutation, "EXTRA", temp_robot_list)
        return True

    def binary_search_for_best_percent_to_leave_inside(self):
        up = self.upper_percent_to_leave_inside
        down = self.lower_percent_to_leave_inside
        last_success = -1
        for i in range(self.binary_search_iteration):
            if up <= down:
                return last_success
            mid = (up + down) // 2
            self.percent_to_leave_inside = mid
            self.set_inside_group()
            if self.check_for_solution():
                last_success = mid
                down = mid + 1
            else:
                up = mid - 1

        return last_success

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
        # Calc BFS map outside
        Generator.calc_bfs_map(grid=self.grid,
                               boundaries={
                                   "N": self.grid.size,
                                   "E": self.grid.size,
                                   "W": 0,
                                   "S": 0},
                               source_container_func=self.create_boundaries_queue,
                               source_container_params=self.grid.size,
                               check_move_func=CheckMoveFunction.check_free_from_obs)

        self.bfs_map_copy.clear()
        self.bfs_map_copy = self.grid.get_copy_bfs_map(False)

    def step_phase_1(self) -> int:
        def find_next_robots_to_move_by_order(occupied_target) -> Union[list, None]:
            out = []
            pos = occupied_target
            while pos != None and self.bfs_map_copy[pos] > 0:
                out.append(self.grid.get_robot_id_by_pos(pos))
                next_directions = Generator.get_next_move_by_dist_and_obs_from_bfs_map_copy(self.bfs_map_copy, pos)

                next_pos = None
                for next_dir in next_directions:
                    next_pos = sum_tuples(directions_to_coords[next_dir], pos)
                    if CheckMoveFunction.cell_free_from_robots_and_obs(next_pos, self.grid):
                        out.reverse()
                        return out

                pos = next_pos
            return None

        moved = 0
        robots_to_move = None
        all_clear = True
        for i in self.inside_group:
            robot = self.robots[i]
            if not CheckMoveFunction.cell_free_from_robots_and_obs(robot.target_pos, self.grid):
                robots_to_move = find_next_robots_to_move_by_order(robot.target_pos)
                all_clear = False
                break

        if all_clear:
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
            for i in range(len(self.robots)):
                if self.q_by_robot_id[i] in [self.q_outside_in_side_road, self.q_outside_in_place_right, self.q_outside_in_place_left, self.q_outside_in_main_road]:
                    moved += self.q_by_robot_id[i](self.robots[i])

            return moved

        if robots_to_move is not None:
            for i in robots_to_move:
                robot = self.robots[i]
                if self.q_by_robot_id[i](robot):
                    moved += 1
        else:
            robots_to_move = []

        for i in self.inside_group:
            if i in robots_to_move:
                continue
            robot = self.robots[i]
            if self.q_by_robot_id[i] == self.q_stay_inside and self.topo_map_copy[robot.target_pos] <= self.topo_map_copy[robot.pos]:
                moved += self.q_by_robot_id[i](self.robots[i])

        if moved > 0:
            return moved

        for i in range(len(self.robots)):
            if self.q_by_robot_id[i] in [self.q_outside_in_side_road, self.q_outside_in_place_right,
                                         self.q_outside_in_place_left, self.q_outside_in_main_road]:
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
                blocked.remove(robot.pos)
                self.bfs_path[robot_id] = Generator.calc_a_star_path(self.grid,
                                                                     self.boundaries,
                                                                     source_pos=robot.pos,
                                                                     dest_pos=robot.target_pos,
                                                                     blocked=blocked,
                                                                     calc_configure_value_func=AStarHeuristics.manhattan_distance,
                                                                     check_move_func=CheckMoveFunction.check_free_from_obs)

                if self.bfs_path[robot_id] is None:
                    return -1
                blocked.add(robot.target_pos)
                sum += len(self.bfs_path[robot_id])

            return sum

        def sort_group_by_extra_data(group):
            temp = []
            for robot_id in group:
                robot = self.robots[robot_id]
                robot.extra_data = self.get_extra_data(robot)
                temp.append(robot)

            self.preprocess.generic_robots_sort(group, "EXTRA", temp)  # sort by highs

            return group

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
            min_order = []

            for i in range(self.calcs_per_high):
                # Clean blocked from current group's targets
                for robot_id in group:
                    try:
                        blocked.remove(self.robots[robot_id].target_pos)
                    except:
                        pass
                # add blocked from current group
                for robot_id in group:
                    blocked.add(self.robots[robot_id].pos)

                temp_algo = algo_preference[min(i, len(algo_preference) - 1)]
                self.secondary_order = temp_algo[0]
                self.descending_order = temp_algo[1]

                sort_group_by_extra_data(group)
                temp_sum = get_group_sum(group, blocked)
                if temp_sum == -1:
                    continue

                if temp_sum < min_sum or min_sum == -1:
                    min_sum = temp_sum
                    min_order = group.copy()

            self.get_inside_groups[group_id] = min_order

        # Calculate first BFS list
        robot_id = self.get_inside_groups[0][0]
        robot = self.robots[robot_id]
        self.bfs_path[robot_id] = Generator.calc_a_star_path(self.grid,
                                                             self.boundaries,
                                                             source_pos=robot.pos,
                                                             dest_pos=robot.target_pos,
                                                             calc_configure_value_func=AStarHeuristics.manhattan_distance,
                                                             check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs)

        assert self.bfs_path[robot_id] is not None, "switch_phase_1_to_2: robot" + str(robot)

    def step_phase_2(self) -> int:
        moved = 0
        moving_robot_id = self.get_inside_groups[self.current_group][self.current_robot]
        robot = self.robots[moving_robot_id]
        # Making sure BFS list isn't empty nor None

        # assert self.bfs_path[moving_robot_id] is not None
        if self.bfs_path[moving_robot_id] is None:
            Generator.print_bfs_map_copy_state(self.bfs_map_copy, self.grid.size, set())
            print(str(robot))
            return 0

        if InitAlgo.move_robot_to_dir(moving_robot_id, self.grid, self.bfs_path[moving_robot_id][0],
                                      self.current_turn, self.solution):
            self.bfs_path[moving_robot_id].popleft()
            moved = 1

        if robot.robot_arrived():

            self.q_by_robot_id[moving_robot_id] = self.q_arrived
            self.arrived_order.append(robot.robot_id)
            self.time_arrived[robot.robot_id] = self.current_turn + 1

            self.current_robot += 1

            if self.current_robot == len(self.get_inside_groups[self.current_group]):
                self.current_robot = 0
                self.current_group += 1

            if self.current_group == len(self.get_inside_groups):
                return moved

            # Calc the next possible BFS
            for i in range(self.current_robot, len(self.get_inside_groups[self.current_group])):
                next_to_calc = self.get_inside_groups[self.current_group][i]
                next_robot = self.robots[next_to_calc]
                self.bfs_path[next_to_calc] = Generator.calc_a_star_path(self.grid,
                                                                     self.boundaries,
                                                                     source_pos=next_robot.pos,
                                                                     dest_pos=next_robot.target_pos,
                                                                     calc_configure_value_func=AStarHeuristics.manhattan_distance,
                                                                     check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs)

                if self.bfs_path[next_to_calc] is not None:
                    self.get_inside_groups[self.current_group][i] = self.get_inside_groups[self.current_group][self.current_robot]
                    self.get_inside_groups[self.current_group][self.current_robot] = next_to_calc

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
                if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, next_dir, self.current_turn, self.solution):
                    if self.bfs_map_copy[robot.pos] == 0:
                        self.q_by_robot_id[robot.robot_id] = self.q_outside_in_main_road
                        robot.extra_data = next_dir
                    return 1
        return 0

    def q_arrived(self, robot: Robot) -> int:
        return 0

    def q_stuck(self, robot: Robot) -> int:
        return 0

    def q_waiting_outside(self, robot: Robot) -> int:
        return 0

    # Helpers
    def get_extra_data(self, robot: Robot, extra_data: int = -1) -> (int, int):
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
        if extra_data == -1:
            extra_data = self.topo_map_copy[robot.target_pos]


        if robot.robot_id in self.inside_group:
            extra_data = (extra_data, 0)
        else:
            extra_data = (extra_data, 1)


        if self.secondary_order == "rand":
            extra_data = (extra_data[0], extra_data[1], random())
        elif self.secondary_order == "dist_from_grid":
            extra_data = (extra_data[0], extra_data[1], get_dist_from_grid(robot))
        elif self.secondary_order == "dist_from_target":
            extra_data = (extra_data[0], extra_data[1], get_dist_from_target(robot))
        elif self.secondary_order == "dist_BFS":
            extra_data = (extra_data[0], extra_data[1], get_bfs_dist(robot))
        else:
            extra_data = (extra_data[0], extra_data[1], 0)

        if not self.descending_order:
            extra_data = (extra_data[0], extra_data[1], (-1) * extra_data[2])

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
