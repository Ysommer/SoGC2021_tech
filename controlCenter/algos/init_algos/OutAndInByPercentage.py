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
import time as ttt
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

        # Time init
        init_time = Timer(self.name + " init")
        init_time.start()

        # Parse data bundle
        if data_bundle is None:
            data_bundle = {}

        self.start_fill_from = data_bundle.get("start_fill_from", (-1, -1))
        self.reverse_fill = data_bundle.get("reverse_fill", True)
        self.boundaries = data_bundle.get("boundaries", {
            "N": self.grid.size + 2,
            "E": self.grid.size + 2,
            "W": -3,
            "S": -3,
        })
        self.percent_to_leave_inside = data_bundle.get("percent_to_leave_inside", 0)
        self.timeout = data_bundle.get("timeout", 10)
        self.sync_insertion = data_bundle.get("sync_insertion", True)
        self.secondary_order = data_bundle.get("secondary_order", "")
        self.descending_order = data_bundle.get("descending_order", False)

        self.empty_spots_to_move_in_pillar = data_bundle.get("empty_spots_to_move_in_pillar", 1)
        self.empty_spots_to_jump_pillar = data_bundle.get("empty_spots_to_jump_pillar", 2)

        # Init phases
        self.phase = 0
        self.phases = [
            self.step_phase_0,
            self.step_phase_1
        ]
        if self.sync_insertion:
            self.phases[1] = self.step_phase_1_simultaneously_v2
            self.name += "_sync_insertion"

        self.phases_timers = [
            Timer("phase 0 timer"),
            Timer("phase 1 timer")
        ]

        # All Phases params
        if self.percent_to_leave_inside > 0:
            self.name += "_per_" + str(self.percent_to_leave_inside)

        if self.secondary_order != "":
            self.name += "_" + self.secondary_order

        if self.descending_order:
            self.name += "_desc"

        if self.empty_spots_to_move_in_pillar != 1:
            self.name += "_ESTMP"+str(self.empty_spots_to_move_in_pillar)

        if self.empty_spots_to_jump_pillar != 2:
            self.name += "_ESTJP"+str(self.empty_spots_to_jump_pillar)

        self.bfs_list = [None] * len(self.robots)
        self.q_by_robot_id = [self.q_reaching_out for i in range(len(self.robots))]
        self.inside_group = []
        self.out_of_boundaries_permutation = []
        self.reaching_out_permutation = []

        # Phase 0 params
        random_quantity = (len(self.robots) * self.percent_to_leave_inside) // 100
        self.inside_group = sample(range(0, len(self.robots)), random_quantity)
        shuffle(self.inside_group)

        # Calc paths for the inside group
        for i in self.inside_group:
            robot = self.robots[i]
            robot.extra_data = 0

            if robot.robot_arrived():
                self.q_by_robot_id[i] = self.q_arrived
                self.arrived_order.append(robot.robot_id)
                self.time_arrived[robot.robot_id] = self.current_turn + 1
                continue

            self.bfs_list[i] = Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                check_move_func=CheckMoveFunction.check_free_from_obs,
                calc_configure_value_func=AStarHeuristics.manhattan_distance
            )

            assert self.bfs_list[i] is not None, "Can't find any path without obstacles"
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

        self.bfs_map_copy = self.grid.get_copy_bfs_map()
        temp_robot_list = []

        for robot in self.robots:
            # Ignoring the robots that stay inside
            if self.q_by_robot_id[robot.robot_id] in [self.q_stay_inside, self.q_arrived]:
                continue

            # Check every robots and target are reachable
            assert self.grid.get_cell_distance(robot.pos) > 0, str(robot) + " is unreachable"
            assert self.grid.get_cell_distance(robot.target_pos) > 0, str(robot) + "'s target is unreachable"

            # Extra data = distance
            robot.extra_data = self.grid.get_cell_distance(robot.pos)
            temp_robot_list.append(robot)

        # Arranging by distance in increasing order
        self.preprocess.generic_robots_sort(self.reaching_out_permutation, "EXTRA", temp_robot_list)

        self.last_index_on_the_road = 0
        self.phase_1_turns = 0

        self.solution.out["algo_name"] = self.name

        init_time.end(to_print=self.print_info)

    def step(self) -> int:
        self.phases_timers[self.phase].start()
        moved = self.phases[self.phase]()
        if moved == 0 and len(self.inside_group) == self.grid.numOfRobotsArrived:
            self.phases_timers[self.phase].end(self.print_info)
            if self.phase == 0:
                if self.switch_phase_0_to_1_v2():
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
        for i in self.out_of_boundaries_permutation:
            moved += self.q_by_robot_id[i](self.robots[i])
            # Is this check necessary?
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0

        temp_permutation = []
        for i in self.reaching_out_permutation:
            moved += self.q_by_robot_id[i](self.robots[i])
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0
            if self.q_by_robot_id[i] == self.q_outside_in_main_road:
                self.out_of_boundaries_permutation.append(i)
            else:
                temp_permutation.append(i)

        self.reaching_out_permutation = temp_permutation


        for i in self.inside_group:
            moved += self.q_by_robot_id[i](self.robots[i])
            if self.q_by_robot_id[i] == self.q_stuck:
                return 0
            else:
                temp_permutation.append(i)

        if moved == 0 and self.grid.numOfRobotsArrived < len(self.inside_group):
            moved += self.shake_map()

        return moved

    def switch_phase_0_to_1(self) -> bool:
        self.phase += 1

        blocked = set()

        if self.grid.has_robot_on_target(self.start_fill_from):
            print("The point to start fill from is full")
            return False

        Generator.calc_bfs_map(grid=self.grid,
                               boundaries=self.boundaries,
                               blocked=blocked,
                               source_container=[self.start_fill_from],
                               check_move_func=CheckMoveFunction.cell_free_from_robots_on_target_and_obs)

        temp_robots = []
        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]
            robot.extra_data = self.grid.get_cell_distance(robot.target_pos)
            if robot.extra_data == -1:
                print(robot, "has no clear path to target")
                return False
            temp_robots.append(robot)

        self.preprocess.generic_robots_sort(self.out_of_boundaries_permutation, "EXTRA", temp_robots)  # sort by dists

        if self.reverse_fill:
            self.out_of_boundaries_permutation.reverse()

        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]

            self.bfs_list[i] = Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                blocked=blocked,
                calc_configure_value_func=AStarHeuristics.manhattan_distance,
                check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs)

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

        moving_robot_id = self.out_of_boundaries_permutation[self.last_index_on_the_road]
        robot = self.robots[moving_robot_id]

        # Making sure BFS list isn't empty nor None
        assert self.bfs_list[moving_robot_id] and self.bfs_list[moving_robot_id] is not None
        if InitAlgo.move_robot_to_dir(moving_robot_id, self.grid, self.bfs_list[moving_robot_id][0],
                                      self.current_turn, self.solution):
            self.bfs_list[moving_robot_id].popleft()
            moved += 1

        if robot.robot_arrived():
            self.last_index_on_the_road += 1
            self.q_by_robot_id[moving_robot_id] = self.q_arrived
            self.arrived_order.append(robot.robot_id)
            self.time_arrived[robot.robot_id] = self.current_turn + 1

        return moved

    def get_min_offset(self, robot) -> int:
        i = robot.robot_id
        min_offset = 0
        next_pos = robot.pos
        for d_index in range(len(self.bfs_list[i])):
            dir = self.bfs_list[i][d_index]
            next_pos = sum_tuples(next_pos, directions_to_coords[dir])
            last_turn_cell_in_use = self.grid.get_cell(next_pos).extra_data
            if last_turn_cell_in_use + 1 > min_offset + d_index:    # +1 For tail
                min_offset = last_turn_cell_in_use + 2 - d_index
        return min_offset

    def tag_cells(self):
        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]
            min_offset = self.get_min_offset(robot)
            robot.extra_data = min_offset
            next_pos = robot.pos

            for d_index in range(len(self.bfs_list[i])):
                dir = self.bfs_list[i][d_index]
                next_pos = sum_tuples(next_pos, directions_to_coords[dir])
                self.grid.get_cell(next_pos).extra_data = min_offset + d_index + 1

    def get_min_offset_v2(self, robot) -> int:
        i = robot.robot_id

        try:
            min_offset = max(self.grid.get_cell(robot.target_pos).extra_data) - len(self.bfs_list[i]) + 1
        except:
            min_offset = 0

        while True:
            found = True
            next_pos = robot.pos

            for d_index in range(len(self.bfs_list[i])):
                dir = self.bfs_list[i][d_index]
                next_pos = sum_tuples(next_pos, directions_to_coords[dir])
                turn_to_pass = min_offset + d_index

                occupied_turns = self.grid.get_cell(next_pos).extra_data

                try:
                    if turn_to_pass in occupied_turns:
                        min_offset += 1
                        found = False
                        break
                except:
                    self.grid.get_cell(next_pos).extra_data = set()

            if found:
                return min_offset

    def tag_cells_v2(self):
        print("Start tagging cells", ttt.strftime("%d/%m/%Y-%H:%M:%S"))
        for pos in self.grid.grid:
            self.grid.grid[pos].extra_data = set()

        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]
            min_offset = self.get_min_offset_v2(robot)
            robot.extra_data = min_offset
            next_pos = robot.pos

            for d_index in range(len(self.bfs_list[i])):
                dir = self.bfs_list[i][d_index]
                next_pos = sum_tuples(next_pos, directions_to_coords[dir])

                assert min_offset + d_index not in self.grid.get_cell(next_pos).extra_data, str(robot) + "\n" + str(min_offset + d_index)
                self.grid.get_cell(next_pos).extra_data.add(min_offset + d_index - 1)
                self.grid.get_cell(next_pos).extra_data.add(min_offset + d_index)
                self.grid.get_cell(next_pos).extra_data.add(min_offset + d_index + 1)

        print("Done tagging cells", ttt.strftime("%d/%m/%Y-%H:%M:%S"))

    def switch_phase_0_to_1_v2(self):
        def get_dist_from_grid(robot: Robot) -> int:
            res = {
                "W": abs(robot.pos[0]),
                "S": abs(robot.pos[1]),
                "E": abs(robot.pos[0] - self.grid.size),
                "N": abs(robot.pos[1] - self.grid.size)
            }
            return res[robot.extra_data]

        def get_dist_from_target(robot: Robot) -> int:
            return AStarHeuristics.manhattan_distance(robot.pos, robot.pos, robot.target_pos)

        def get_bfs_dist(robot: Robot) -> int:
            return len(Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                calc_configure_value_func=AStarHeuristics.manhattan_distance,
                check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs))

        def get_extra_data(robot: Robot) -> (int, int):
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
                extra_data = (extra_data[0], (-1)*extra_data[1])

            return extra_data

        print("Switch phase - Current Turn:", self.current_turn)
        self.phase += 1
        blocked = set()

        Generator.calc_sea_level_bfs_map(grid=self.grid,
                               boundaries=self.boundaries,
                               check_move_func=CheckMoveFunction.cell_free_from_robots_on_target_and_obs)

        temp_robots = []
        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]
            robot.extra_data = get_extra_data(robot)
            if robot.extra_data[0] == -1:
                print(robot, "has no clear path to target")
                return False
            temp_robots.append(robot)

        self.preprocess.generic_robots_sort(self.out_of_boundaries_permutation, "EXTRA", temp_robots)  # sort by highs

        self.out_of_boundaries_permutation.reverse()

        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]
            robot.extra_data = robot.extra_data[0]

            self.bfs_list[i] = Generator.calc_a_star_path(
                grid=self.grid,
                boundaries=self.boundaries,
                source_pos=robot.pos,
                dest_pos=robot.target_pos,
                blocked=blocked,
                calc_configure_value_func=AStarHeuristics.manhattan_distance,
                check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs)

            # Because distance is not -1, there must be a way
            if self.bfs_list[i] is None:
                print(i)
                print(self.robots[i])
                for d in directions_to_coords:
                    print(self.grid.get_cell(sum_tuples(robot.target_pos, directions_to_coords[d])))
                print(self.out_of_boundaries_permutation)
                return False

            assert self.bfs_list[i] is not None, "Step 1: can't find any path for robot:" + str(i)
            blocked.add(robot.target_pos)

        if self.sync_insertion:
            self.tag_cells_v2()
            self.preprocess.generic_robots_sort(self.out_of_boundaries_permutation, "EXTRA", temp_robots)  # sort by turn offsets

        return True

    def unplug_jam(self, robot_id: int) -> bool:
        r = self.robots[robot_id]
        next_pos = sum_tuples(r.pos, directions_to_coords[self.bfs_list[robot_id][0]])
        if self.grid.has_robot(next_pos):
            other_robot_id = self.grid.get_cell(next_pos).get_robot()
            other_robot_next_pos = sum_tuples(next_pos, directions_to_coords[self.bfs_list[other_robot_id][0]])
            if other_robot_next_pos == r.pos:
                self.bfs_list[robot_id] = Generator.calc_a_star_path(
                    grid=self.grid,
                    boundaries=self.boundaries,
                    source_pos=r.pos,
                    dest_pos=r.target_pos,
                    blocked={next_pos},
                    calc_configure_value_func=AStarHeuristics.manhattan_distance,
                    check_move_func=CheckMoveFunction.cell_free_from_robots_and_obs)
                assert self.bfs_list[robot_id] is not None, "unplug_jam failed"
                return True

        return False

    def step_phase_1_simultaneously(self):
        moved = 0

        # Set states
        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]
            if robot.robot_arrived():
                continue

            if robot.extra_data > self.phase_1_turns:
                break

            if self.q_getting_inside(robot):
                moved += 1
            else:
                print(robot)
                return 0

        self.phase_1_turns += 1
        return moved

    def step_phase_1_simultaneously_v2(self):
        moved = 0

        # Set states
        for i in self.out_of_boundaries_permutation:
            robot = self.robots[i]
            if robot.robot_arrived():
                continue

            if robot.extra_data > self.phase_1_turns:
                continue

            if self.q_getting_inside(robot):
                moved += 1
            else:
                print(robot)
                return 0

        self.phase_1_turns += 1
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
        col = self.get_col_by_direction(robot.extra_data, robot.pos)
        row = self.get_row_by_direction(robot.extra_data, robot.pos)
        assert col % 3 == 0
        moved = 0
        d = robot.extra_data
        left_pos = sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "L")])
        left_main_pos = self.get_main_road_pos(d, left_pos)
        left_cell_clear = self.grid.move_robot(robot.robot_id, self.get_relative_side(d, "L"), self.current_turn, False) == EnterCellResult.SUCCESS
        left_empty_cells = 0
        right_pos = sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "R")])
        right_main_pos = self.get_main_road_pos(d, right_pos)
        right_cell_clear = self.grid.move_robot(robot.robot_id, self.get_relative_side(d, "R"), self.current_turn, False) == EnterCellResult.SUCCESS
        right_empty_cells = 0

        temp_pos = left_pos
        while temp_pos != left_main_pos:
            temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
            if not self.grid.has_robot(temp_pos):
                left_empty_cells += 1

        temp_pos = right_pos
        while temp_pos != right_main_pos:
            temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
            if not self.grid.has_robot(temp_pos):
                right_empty_cells += 1

        left_cell_clear = left_cell_clear and left_empty_cells > 0
        right_cell_clear = right_cell_clear and right_empty_cells > 0

        if left_cell_clear and right_cell_clear:
            left_cell_clear = left_cell_clear > right_cell_clear
            right_cell_clear = not left_cell_clear

        if left_cell_clear:
            if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "L"),
                                          self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_right
                return 1
            else:
                assert 0

        if right_cell_clear:
            if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(robot.extra_data, "R"),
                                          self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_left
                return 1
            else:
                assert 0

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
        main_pos = self.get_main_road_pos(d, robot.pos)
        temp_pos = robot.pos

        if not self.grid.has_robot(sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "R")])):
            while temp_pos != main_pos:
                temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
                if not self.grid.has_robot(temp_pos):
                    return 0

        # Have to move
        # Check move to left pillar
        top_robot = not self.grid.has_robot(sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "U")]))

        left_pos = sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "L")])
        left_cell_clear = self.grid.move_robot(robot.robot_id, self.get_relative_side(d, "L"), self.current_turn, False) == EnterCellResult.SUCCESS
        free_slots_counter_in_left = 0
        move_left = False

        if top_robot and left_cell_clear:
            # Check if left pillar need to rise
            temp_pos = left_pos
            left_main_pos = self.get_main_road_pos(d, left_pos)

            while temp_pos != left_main_pos:
                temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
                if not self.grid.has_robot(temp_pos):
                    free_slots_counter_in_left += 1
                    if free_slots_counter_in_left == self.empty_spots_to_move_in_pillar:
                        move_left = True

        jump_pillar = False
        next_pillar_clear = top_robot and self.grid.move_robot(robot.robot_id, self.get_relative_side(d, "R"), self.current_turn, False) == EnterCellResult.SUCCESS
        free_slots_counter_in_next_pillar = 0

        right_side_pos = sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "R")])
        right_down_side_pos = sum_tuples(right_side_pos, directions_to_coords[self.get_relative_side(d, "D")])
        right_right_next_pillar_pos = sum_tuples(right_side_pos, directions_to_coords[self.get_relative_side(d, "R")])
        right_right_down_next_pillar_pos = sum_tuples(right_right_next_pillar_pos, directions_to_coords[self.get_relative_side(d, "D")])
        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(right_side_pos)
        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(right_down_side_pos)
        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(right_right_next_pillar_pos)
        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(right_right_down_next_pillar_pos)

        if next_pillar_clear:
            # Check if left pillar need to rise
            temp_pos = right_right_down_next_pillar_pos
            right_next_pillar_main_pos = self.get_main_road_pos(d, right_right_down_next_pillar_pos)

            while temp_pos != right_next_pillar_main_pos:
                temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
                if not self.grid.has_robot(temp_pos):
                    free_slots_counter_in_next_pillar += 1
                    if free_slots_counter_in_next_pillar == self.empty_spots_to_jump_pillar:
                        jump_pillar = True

            jump_pillar = jump_pillar and free_slots_counter_in_next_pillar > 0 # To make sure not last pillar


        if jump_pillar and move_left:
            jump_pillar = free_slots_counter_in_next_pillar - self.empty_spots_to_jump_pillar > free_slots_counter_in_left - self.empty_spots_to_move_in_pillar
            move_left = not jump_pillar

        if move_left:
            if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "L"),
                                               self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_left
                return 1
            else:
                assert 0

        if jump_pillar:
            if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "R"),
                                               self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_side_road
                return 1
            else:
                assert 0

        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "U"),
                                           self.current_turn, self.solution):
            moved = 1
            self.stretch_boundaries(robot)

        return moved

    def q_outside_in_place_left(self, robot: Robot) -> int:
        col = self.get_col_by_direction(robot.extra_data, robot.pos)
        row = self.get_row_by_direction(robot.extra_data, robot.pos)
        assert col % 3 == 1
        moved = 0
        d = robot.extra_data
        main_pos = self.get_main_road_pos(d, robot.pos)
        temp_pos = robot.pos

        if not self.grid.has_robot(sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "L")])):
            while temp_pos != main_pos:
                temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
                if not self.grid.has_robot(temp_pos):
                    return 0

        # Have to move
        # Check move to right pillar
        top_robot = not self.grid.has_robot(sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "U")]))

        right_pos = sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "R")])
        right_cell_clear = self.grid.move_robot(robot.robot_id, self.get_relative_side(d, "R"), self.current_turn, False) == EnterCellResult.SUCCESS
        free_slots_counter_in_right = 0
        move_right = False

        if top_robot and right_cell_clear:
            # Check if left pillar need to rise
            temp_pos = right_pos
            right_main_pos = self.get_main_road_pos(d, right_pos)

            while temp_pos != right_main_pos:
                temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
                if not self.grid.has_robot(temp_pos):
                    free_slots_counter_in_right += 1
                    if free_slots_counter_in_right == self.empty_spots_to_move_in_pillar:
                        move_right = True
                        break

        jump_pillar = False
        next_pillar_clear = top_robot and self.grid.move_robot(robot.robot_id, self.get_relative_side(d, "L"), self.current_turn, False) == EnterCellResult.SUCCESS
        free_slots_counter_in_next_pillar = 0

        left_side_pos = sum_tuples(robot.pos, directions_to_coords[self.get_relative_side(d, "L")])
        left_down_side_pos = sum_tuples(left_side_pos, directions_to_coords[self.get_relative_side(d, "D")])
        left_left_next_pillar_pos = sum_tuples(left_side_pos, directions_to_coords[self.get_relative_side(d, "L")])
        left_left_down_next_pillar_pos = sum_tuples(left_left_next_pillar_pos, directions_to_coords[self.get_relative_side(d, "D")])

        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(left_side_pos)
        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(left_down_side_pos)
        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(left_left_next_pillar_pos)
        next_pillar_clear = next_pillar_clear and not self.grid.has_robot(left_left_down_next_pillar_pos)

        if next_pillar_clear:
            # Check if left pillar need to rise
            temp_pos = left_left_down_next_pillar_pos
            left_next_pillar_main_pos = self.get_main_road_pos(d, left_left_down_next_pillar_pos)

            while temp_pos != left_next_pillar_main_pos:
                temp_pos = sum_tuples(temp_pos, directions_to_coords[self.get_relative_side(d, "D")])
                if not self.grid.has_robot(temp_pos):
                    free_slots_counter_in_next_pillar += 1
                    if free_slots_counter_in_next_pillar == self.empty_spots_to_jump_pillar:
                        jump_pillar = True

            jump_pillar = jump_pillar and free_slots_counter_in_next_pillar > 0  # To make sure not last pillar

        if jump_pillar and move_right:
            jump_pillar = free_slots_counter_in_next_pillar - self.empty_spots_to_jump_pillar > free_slots_counter_in_right - self.empty_spots_to_move_in_pillar
            move_right = not jump_pillar

        if move_right:
            if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "R"),
                                          self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_place_right
                return 1
            else:
                assert 0

        if jump_pillar:
            if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "L"),
                                          self.current_turn, self.solution):
                self.q_by_robot_id[robot.robot_id] = self.q_outside_in_side_road
                return 1
            else:
                assert 0

        if InitAlgo.move_robot_to_dir(robot.robot_id, self.grid, self.get_relative_side(d, "U"),
                                      self.current_turn, self.solution):
            moved = 1
            self.stretch_boundaries(robot)

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

        # Making sure BFS list isn't empty nor None
        assert self.bfs_list[i] and self.bfs_list[i] is not None
        if InitAlgo.move_robot_to_dir(i, self.grid, self.bfs_list[i][0], self.current_turn, self.solution):
            self.bfs_list[i].popleft()
            robot.extra_data = 0
            if not self.bfs_list[i]:
                #Empty bfs_list
                assert robot.robot_arrived(), "Robot " + str(robot) + " is With empty bfs list and didn't hit the target."
                self.q_by_robot_id[i] = self.q_arrived
                self.arrived_order.append(robot.robot_id)
                self.time_arrived[robot.robot_id] = self.current_turn + 1
            return 1

        robot.extra_data += 1
        return 0

    def q_arrived(self, robot: Robot) -> int:
        return 0

    def q_stuck(self, robot: Robot) -> int:
        return 0

    def q_getting_inside(self, robot: Robot) -> int:
        i = robot.robot_id
        if InitAlgo.move_robot_to_dir(i, self.grid, self.bfs_list[i][0],
                                      self.current_turn, self.solution):
            self.bfs_list[i].popleft()
            if not self.bfs_list[i]:
                # Empty bfs_list
                assert robot.robot_arrived(), "Robot " + str(
                    robot) + " is With empty bfs list and didn't hit the target."
                self.q_by_robot_id[i] = self.q_arrived
                self.arrived_order.append(robot.robot_id)
                self.time_arrived[robot.robot_id] = self.current_turn + 1
            return 1

        return 0

    def q_waiting_outside(self, robot: Robot) -> int:
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

        robot.extra_data = 0
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

    def stretch_boundaries(self, robot):
        if robot.extra_data == "N":
            self.boundaries["N"] = max(self.boundaries["N"], robot.pos[1])
        elif robot.extra_data == "W":
            self.boundaries["W"] = min(self.boundaries["W"], robot.pos[0])
        elif robot.extra_data == "S":
            self.boundaries["S"] = min(self.boundaries["S"], robot.pos[1])
        elif robot.extra_data == "E":
            self.boundaries["E"] = max(self.boundaries["E"], robot.pos[0])
