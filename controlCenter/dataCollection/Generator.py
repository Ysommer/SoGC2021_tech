from typing import Union, List

from infrastructure.cell import Cell
from infrastructure.robot import Robot
from infrastructure.grid import Grid
from collections import deque
from utils import *
from defines import *
from random import randint
import queue
import heapq
from infrastructure.solGrid import SolGrid
from utils import Timer


class AStarHeuristics:
    @staticmethod
    def manhattan_distance(pos: (int, int), source_pos: (int, int), dest_pos: (int, int),
                           calc_configure_value_params=None) -> int:
        manhattan_distance_val = abs(pos[0] - dest_pos[0]) + abs(pos[1] - dest_pos[1])
        return manhattan_distance_val

    @staticmethod
    def manhattan_distance_with_time(pos: (int, int, int), source_pos: (int, int, int), dest_pos: (int, int, int),
                                     calc_h_value_params=None) -> int:
        manhattan_distance_val = abs(pos[0] - dest_pos[0]) + abs(pos[1] - dest_pos[1]) + abs(pos[2] - dest_pos[2])
        return manhattan_distance_val


class CheckMoveFunction:
    @staticmethod
    def check_free_from_obs(pos, grid: Grid, check_move_params=None) -> bool:
        return not grid.is_obs(pos)

    @staticmethod
    def cell_free_from_robots_on_target_and_obs(pos, grid: Grid, check_move_params=None) -> bool:
        return not (grid.is_obs(pos) or grid.has_robot_on_target(pos))

    @staticmethod
    def cell_free_from_robots_and_obs(pos, grid: Grid, check_move_params=None) -> bool:
        return grid.grid.get(pos, Cell(pos)).occupied is None


class CheckIfDestFunction:
    @staticmethod
    def check_if_dest_single_destination(pos, dest_params):
        return pos == dest_params

    @staticmethod
    def check_if_dest_multi_destinations(pos, dest_params):
        return pos in dest_params


class SourceContainerFunction:
    @staticmethod
    def get_sources(source_container, source_container_params) -> queue:
        d = queue.Queue()
        for s in source_container:
            d.put(s)
        return d


class Generator:
    """
        A class with a generic functions
    """

    @staticmethod
    def check_if_in_boundaries(pos, boundaries):
        return boundaries["N"] >= pos[1] >= boundaries["S"] and \
               boundaries["W"] <= pos[0] <= boundaries["E"]

    @staticmethod
    def get_valid_directions_matrix(source_pos, obstacles: set, x_axis_min_max: dict, margin: int = 3):
        def check_boundaries(pos):
            for x in range((-1) * margin, margin + 1):
                if pos[0] + x in x_axis_min_max:
                    if x_axis_min_max[pos[0] + x][0] - margin <= pos[1] <= x_axis_min_max[pos[0] + x][1] + margin:
                        return True
            """
            for y in range((-1) * margin, margin):
                if pos[1] + y in y_axis_min_max:
                    if y_axis_min_max[pos[1] + y][0] - margin <= pos[0] <= y_axis_min_max[pos[1] + y][1] + margin:
                        print("Flag")
                        return True"""

            return False

        matrix = dict()

        q = queue.Queue()
        q.put(source_pos)

        while not q.empty():
            pos = q.get()
            if pos in matrix:
                continue
            matrix[pos] = []
            preferred_direction_order = Generator.get_preferred_direction_order(pos)
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if next_pos not in obstacles and check_boundaries(next_pos):
                    q.put(next_pos)
                    matrix[pos].append((next_pos, direction))
            matrix[pos].append((pos, 'X'))

        return matrix

    @staticmethod
    def construct_path(grid: Grid, pos: (int, int)) -> deque:
        path = []
        parent = grid.get_cell_parent(pos)
        while parent is not "":
            path.append(parent)
            pos = sub_tuples(pos, directions_to_coords[parent])
            parent = grid.get_cell_parent(pos)

        return deque(path[::-1])  # return reversed path

    @staticmethod
    def get_bfs_path(
            grid: Grid,
            boundaries,
            blocked: set = None,
            source_container=None,
            source_container_func=SourceContainerFunction.get_sources,
            source_container_params=None,
            check_if_dest_function=CheckIfDestFunction.check_if_dest_single_destination,
            check_if_dest_params=None,
            check_move_func=CheckMoveFunction.check_free_from_obs,
            check_move_params=None,
            preferred_direction_order=None) -> deque:

        if source_container is None:
            source_container = [(0, 0)]

        if blocked is None:
            blocked = set()

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords.keys()

        q = source_container_func(source_container, source_container_params)
        grid.start_bfs(q)

        for pos in blocked:
            grid.check_cell_for_bfs(pos)

        while not q.empty():
            pos = q.get()
            if check_if_dest_function(pos, check_if_dest_params):
                return Generator.construct_path(grid, pos)
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if Generator.check_if_in_boundaries(next_pos, boundaries) \
                        and check_move_func(next_pos, grid, check_move_params) \
                        and grid.check_cell_for_bfs(next_pos, parent=direction):
                    q.put(next_pos)
                    # visited.append(next_pos)

        return deque()

    @staticmethod
    def calc_bfs_map(
            grid: Grid,
            boundaries,
            blocked: set = None,
            source_container=None,
            source_container_func=SourceContainerFunction.get_sources,
            source_container_params=None,
            check_move_func=CheckMoveFunction.check_free_from_obs,
            check_move_params=None,
            preferred_direction_order=None):

        if source_container is None:
            source_container = [(0, 0)]

        if blocked is None:
            blocked = set()

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords.keys()

        q = source_container_func(source_container, source_container_params)

        grid.start_bfs(q)

        for pos in blocked:
            grid.check_cell_for_bfs(pos)

        while not q.empty():
            pos = q.get()
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if Generator.check_if_in_boundaries(next_pos, boundaries) \
                        and check_move_func(next_pos, grid, check_move_params) \
                        and grid.check_cell_for_bfs(next_pos, parent=direction, dist=grid.get_cell_distance(pos) + 1):
                    q.put(next_pos)

    @staticmethod
    def calc_sea_level_bfs_map(
            grid: Grid,
            boundaries,
            blocked: set = None,
            check_move_func=CheckMoveFunction.check_free_from_obs,
            check_move_params=None):

        if blocked is None:
            blocked = set()

        preferred_direction_order = directions_to_coords

        deq = deque()
        temp_q = queue.Queue()
        # N & S
        for i in range(0, grid.size):
            deq.append((i, -1))
            temp_q.put((i, -1))
            deq.append((i, grid.size))
            temp_q.put((i, grid.size))
            deq.append((-1, i))
            temp_q.put((-1, i))
            deq.append((grid.size, i))
            temp_q.put((grid.size, i))

        grid.start_bfs(temp_q)

        for pos in blocked:
            grid.check_cell_for_bfs(pos)

        while len(deq) > 0:
            pos = deq.popleft()
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if Generator.check_if_in_boundaries(next_pos, boundaries) \
                        and check_move_func(next_pos, grid, check_move_params):
                    if grid.is_completely_clean_cell(next_pos):
                        # No target
                        if grid.check_cell_for_bfs(next_pos, parent=direction, dist=grid.get_cell_distance(pos)):
                            deq.appendleft(next_pos)
                    else:
                        # Target
                        if grid.check_cell_for_bfs(next_pos, parent=direction, dist=grid.get_cell_distance(pos) + 1):
                            deq.append(next_pos)

    @staticmethod
    def get_next_move_by_dist_and_obs(
            grid: Grid,
            pos: (int, int),
            preferred_direction_order=None):

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords.keys()

        current_dist = grid.get_cell_distance(pos)

        for d in preferred_direction_order:
            next_pos = sum_tuples(pos, directions_to_coords[d])
            if not grid.is_obs(next_pos):
                next_dist = grid.get_cell_distance(next_pos)
                if next_dist < current_dist:
                    return d

        assert 0, "get_next_move_by_dist_and_obs: Failed to find next"

    @staticmethod
    def get_preferred_direction_order(pos) -> List[str]:
        if pos[0] % 2 == 0:
            if pos[1] % 2 == 0:
                return ["E", "N", "S", "W"]
            else:
                return ["E", "S", "N", "W"]
        else:
            if pos[1] % 2 == 0:
                return ["W", "N", "S", "E"]
            else:
                return ["W", "S", "N", "E"]

    @staticmethod
    def get_next_move_by_dist_and_obs_from_bfs_map_copy(
            map: dict,
            pos: (int, int),
            preferred_direction_order=None) -> List[str]:

        if preferred_direction_order is None:
            preferred_direction_order = Generator.get_preferred_direction_order(pos)

        current_dist = map[pos]
        res = []

        for d in preferred_direction_order:
            next_pos = sum_tuples(pos, directions_to_coords[d])
            next_dist = map.get(next_pos, -1)

            if -1 < next_dist < current_dist:
                res.append(d)

        assert len(res) > 0, "get_next_move_by_dist_and_obs: Failed to find next from pos: " + str(pos)
        return res

    @staticmethod
    def calc_a_star_path(
            grid: Grid,
            boundaries,
            source_pos,
            dest_pos,
            blocked: set = None,
            calc_configure_value_func=AStarHeuristics.manhattan_distance,
            calc_configure_value_params=None,
            check_move_func=CheckMoveFunction.check_free_from_obs,
            check_move_params=None) -> deque:
        if blocked is None:
            blocked = set()

        q = queue.Queue()
        q.put(source_pos)
        grid.start_bfs(q, is_a_star=True)

        for pos in blocked:
            grid.check_cell_for_a_star(pos, parent="", g_value=0)

        g_val = 0
        h_val = calc_configure_value_func(pos=source_pos, source_pos=source_pos, dest_pos=dest_pos,
                                          calc_configure_value_params=calc_configure_value_params)
        f_val = g_val + h_val

        h = [(f_val, g_val, source_pos)]
        heapq.heapify(h)

        while len(h) > 0:
            (f_val, g_val, pos) = heapq.heappop(h)
            g_val *= (-1)
            if not grid.check_if_cell_is_open(pos, g_val):
                continue
            if pos == dest_pos:
                return Generator.construct_path(grid, pos)
            preferred_direction_order = Generator.get_preferred_direction_order(pos)
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if Generator.check_if_in_boundaries(next_pos, boundaries) \
                        and check_move_func(next_pos, grid, check_move_params) \
                        and grid.check_cell_for_a_star(next_pos, parent=direction, g_value=g_val + 1):
                    next_g_val = g_val + 1
                    next_h_val = calc_configure_value_func(pos=next_pos, source_pos=source_pos, dest_pos=dest_pos,
                                                           calc_configure_value_params=calc_configure_value_params)
                    next_f_val = next_g_val + next_h_val
                    heapq.heappush(h, (next_f_val, (-1) * next_g_val, next_pos))

        # print("calc_a_star_path: Couldn't find a from", str(source_pos), "to", str(dest_pos))
        return None

    @staticmethod
    def calc_a_star_path_in_time(
            grid: SolGrid,
            valid_direction_matrix,
            source_pos,
            dest_pos,
            robot_id: int,
            noise: int,
            last_step: int,
            blocked_N_visited: set = None,
            calc_h_value_func=AStarHeuristics.manhattan_distance_with_time,
            calc_h_value_params=None,
            check_move_func=None,
            check_move_params=None,
            preferred_direction_order=None):  # list of moves

        debug_info = False
        if debug_info:
            timer = Timer("Astar ")
            timer.start()

        if check_move_func is None:
            check_move_func = grid.new_check_move

        open = {}
        close = {}
        parents = {source_pos: None}

        g_val = (0, 0)
        h_val = calc_h_value_func(pos=source_pos, source_pos=source_pos, dest_pos=dest_pos,
                                  calc_h_value_params=calc_h_value_params)
        f_val = -g_val[0] + h_val

        h = [(f_val, g_val, source_pos)]
        open[source_pos] = (f_val, g_val)
        heapq.heapify(h)

        def construct_path(parents: dict, pos_t: (int, int, int)) -> list:
            path = []
            while parents[pos_t] is not None:
                path.append(parents[pos_t])
                pos_t = sub_tuples_with_time(pos_t, directions_to_coords_with_time[parents[pos_t]])

            return path[::-1]  # return reversed path

        def internal_check_move(next_pos: (int, int, int), direction: str) -> bool:
            can_make_it = dest_pos[2] - next_pos[2] >= \
                          AStarHeuristics.manhattan_distance((next_pos[:2]), (next_pos[:2]), (dest_pos[:2]))
            legal_move = can_make_it and (
                        direction == 'X' or check_move_func(robot_id, next_pos, direction, check_move_params))
            return legal_move

        if noise > 0:
            while len(h) > 0:
                (f_val, g_val, pos) = heapq.heappop(h)
                if pos not in open or open[pos] != (f_val, g_val):
                    continue
                open.pop(pos)
                close[pos] = (f_val, g_val)

                # if pos == dest_pos:
                # return construct_path(parents, pos)
                if pos[0] == dest_pos[0] and pos[1] == dest_pos[1] and pos[2] > last_step:
                    if debug_info:
                        print("robot_id: ", robot_id, "SUCCEEDED with close size: ", len(close))
                        timer.end(True)
                    return construct_path(parents, pos)
                pushed_out_direction = grid.pushed_out(pos, robot_id)
                if pushed_out_direction is not None:
                    if len([x for x in valid_direction_matrix[pos[:2]] if x[1] == pushed_out_direction]) == 0:
                        continue
                    next_poses = [(sum_tuples_with_time(pos, directions_to_coords_with_time[pushed_out_direction]),
                                   pushed_out_direction)]
                else:
                    next_poses_timeless = valid_direction_matrix[pos[:2]]
                    next_poses = [((np[0] + (pos[2] + 1,),
                                    np[1])) for np in next_poses_timeless]
                for next_pos_t, direction in next_poses:
                    if internal_check_move(next_pos_t, direction):
                        next_g_val = sub_tuples(g_val,
                                                ((1, 0) if direction != 'X' else (0, 12 + noise * randint(0, 1))))
                        next_h_val = calc_h_value_func(pos=next_pos_t, source_pos=next_pos_t, dest_pos=dest_pos,
                                                       calc_h_value_params=calc_h_value_params)
                        next_f_val = -next_g_val[0] + next_h_val
                        if next_pos_t in open:
                            if next_g_val > open[next_pos_t][1]:
                                open[next_pos_t] = (next_f_val, next_g_val)
                                heapq.heappush(h, (next_f_val, next_g_val, next_pos_t))
                                parents[next_pos_t] = direction
                        elif next_pos_t in close:
                            if next_g_val > close[next_pos_t][1]:
                                close.pop(next_pos_t)
                                open[next_pos_t] = (next_f_val, next_g_val)
                                heapq.heappush(h, (next_f_val, next_g_val, next_pos_t))
                                parents[next_pos_t] = direction
                        else:
                            parents[next_pos_t] = direction
                            open[next_pos_t] = (next_f_val, next_g_val)
                            heapq.heappush(h, (next_f_val, next_g_val, next_pos_t))
        else:
            while len(h) > 0:
                (f_val, g_val, pos) = heapq.heappop(h)
                if pos not in open or open[pos] != (f_val, g_val):
                    continue
                open.pop(pos)
                close[pos] = (f_val, g_val)

                # if pos == dest_pos:
                # return construct_path(parents, pos)
                if pos[0] == dest_pos[0] and pos[1] == dest_pos[1] and pos[2] > last_step:
                    if debug_info:
                        print("robot_id: ", robot_id, "SUCCEEDED with close size: ", len(close))
                        timer.end(True)
                    return construct_path(parents, pos)
                pushed_out_direction = grid.pushed_out(pos, robot_id)
                if pushed_out_direction is not None:
                    if len([x for x in valid_direction_matrix[pos[:2]] if x[1] == pushed_out_direction]) == 0:
                        continue
                    next_poses = [(sum_tuples_with_time(pos, directions_to_coords_with_time[pushed_out_direction]),
                                   pushed_out_direction)]
                else:
                    next_poses_timeless = valid_direction_matrix[pos[:2]]
                    next_poses = [((np[0] + (pos[2] + 1,),
                                    np[1])) for np in next_poses_timeless]
                for next_pos_t, direction in next_poses:
                    if internal_check_move(next_pos_t, direction):
                        next_g_val = sub_tuples(g_val, ((1, 0) if direction != 'X' else (0, -1)))
                        next_h_val = calc_h_value_func(pos=next_pos_t, source_pos=next_pos_t, dest_pos=dest_pos,
                                                       calc_h_value_params=calc_h_value_params)
                        next_f_val = -next_g_val[0] + next_h_val
                        if next_pos_t in open:
                            if next_g_val > open[next_pos_t][1]:
                                open[next_pos_t] = (next_f_val, next_g_val)
                                heapq.heappush(h, (next_f_val, next_g_val, next_pos_t))
                                parents[next_pos_t] = direction
                        elif next_pos_t in close:
                            if next_g_val > close[next_pos_t][1]:
                                close.pop(next_pos_t)
                                open[next_pos_t] = (next_f_val, next_g_val)
                                heapq.heappush(h, (next_f_val, next_g_val, next_pos_t))
                                parents[next_pos_t] = direction
                        else:
                            parents[next_pos_t] = direction
                            open[next_pos_t] = (next_f_val, next_g_val)
                            heapq.heappush(h, (next_f_val, next_g_val, next_pos_t))

        # print("calc_a_star_path: Couldn't find a from", str(source_pos), "to", str(dest_pos))
        if debug_info:
            print("robot_id:", robot_id, "FAILED with close size: ", len(close))
            timer.end(True)
        return None

    def calc_a_star_path_in_time_old(
            grid: SolGrid,
            valid_direction_matrix,
            source_pos,
            dest_pos,
            robot_id: int,
            epsilon: float,
            last_step: int,
            blocked_N_visited: set = None,
            calc_h_value_func=AStarHeuristics.manhattan_distance_with_time,
            calc_h_value_params=None,
            check_move_func=None,
            check_move_params=None,
            preferred_direction_order=None):  # list of moves

        debug_info = False
        if debug_info:
            timer = Timer("Astar ")
            timer.start()

        if check_move_func is None:
            check_move_func = grid.new_check_move

        open = {}
        close = {}
        parents = {source_pos: None}

        g_val = 0
        h_val = calc_h_value_func(pos=source_pos, source_pos=source_pos, dest_pos=dest_pos,
                                  calc_h_value_params=calc_h_value_params)
        f_val = g_val + h_val

        h = [(f_val, g_val, source_pos)]
        open[source_pos] = (f_val, g_val)
        heapq.heapify(h)

        def construct_path(parents: dict, pos_t: (int, int, int)) -> list:
            path = []
            while parents[pos_t] is not None:
                path.append(parents[pos_t])
                pos_t = sub_tuples_with_time(pos_t, directions_to_coords_with_time[parents[pos_t]])

            return path[::-1]  # return reversed path

        def internal_check_move(next_pos: (int, int, int), direction: str) -> bool:
            can_make_it = dest_pos[2] - next_pos[2] >= \
                          AStarHeuristics.manhattan_distance((next_pos[:2]), (next_pos[:2]), (dest_pos[:2]))
            legal_move = can_make_it and (
                        direction == 'X' or check_move_func(robot_id, next_pos, direction, check_move_params))
            return legal_move

        while len(h) > 0:
            (f_val, g_val, pos) = heapq.heappop(h)
            g_val = (-1) * g_val
            if pos not in open or open[pos] != (f_val, g_val):
                continue
            open.pop(pos)
            close[pos] = (f_val, g_val)

            # if pos == dest_pos:
            # return construct_path(parents, pos)
            if pos[0] == dest_pos[0] and pos[1] == dest_pos[1] and pos[2] > last_step:
                if debug_info:
                    print("robot_id: ", robot_id, "SUCCEEDED with close size: ", len(close))
                    timer.end(True)
                return construct_path(parents, pos)
            pushed_out_direction = grid.pushed_out(pos, robot_id)
            if pushed_out_direction is not None:
                if len([x for x in valid_direction_matrix[pos[:2]] if x[1] == pushed_out_direction]) == 0:
                    continue
                next_poses = [(sum_tuples_with_time(pos, directions_to_coords_with_time[pushed_out_direction]),
                               pushed_out_direction)]
            else:
                next_poses_timeless = valid_direction_matrix[pos[:2]]
                next_poses = [((np[0] + (pos[2] + 1,),
                                np[1])) for np in next_poses_timeless]
            for next_pos_t, direction in next_poses:
                if internal_check_move(next_pos_t, direction):
                    next_g_val = g_val + (1 if direction != 'X' else epsilon)
                    next_h_val = calc_h_value_func(pos=next_pos_t, source_pos=next_pos_t, dest_pos=dest_pos,
                                                   calc_h_value_params=calc_h_value_params)
                    next_f_val = next_g_val + next_h_val
                    if next_pos_t in open:
                        if next_g_val < open[next_pos_t][1]:
                            open[next_pos_t] = (next_f_val, next_g_val)
                            heapq.heappush(h, (next_f_val, (-1) * next_g_val, next_pos_t))
                            parents[next_pos_t] = direction
                    elif next_pos_t in close:
                        if next_g_val < close[next_pos_t][1]:
                            close.pop(next_pos_t)
                            open[next_pos_t] = (next_f_val, next_g_val)
                            heapq.heappush(h, (next_f_val, (-1) * next_g_val, next_pos_t))
                            parents[next_pos_t] = direction
                    else:
                        parents[next_pos_t] = direction
                        open[next_pos_t] = (next_f_val, next_g_val)
                        heapq.heappush(h, (next_f_val, (-1) * next_g_val, next_pos_t))

        # print("calc_a_star_path: Couldn't find a from", str(source_pos), "to", str(dest_pos))
        if debug_info:
            print("robot_id:", robot_id, "FAILED with close size: ", len(close))
            timer.end(True)
        return None

    @staticmethod
    def print_bfs_map_copy(map: dict, size):
        for y in range(size - 1, -1, -1):
            for x in range(size):
                height = str(map.get((x, y), -1)) + "  "
                if len(height) == 3:
                    height = " " + height
                print(height, end="\t")
            print(end="\n")

    @staticmethod
    def print_bfs_map_copy_state(map: dict, size: int, blocked: set, to_follow=None):
        if to_follow is None:
            to_follow = set()
        for y in range(size - 1, -1, -1):
            for x in range(size):
                height = str(map.get((x, y), -1))
                if len(height) == 1:
                    height = " " + height

                print(height, end=":")
                if (x, y) in to_follow:
                    print("#", end="\t")
                elif (x, y) in blocked:
                    print("@", end="\t")
                else:
                    print("_", end="\t")

            print(end="\n")
    '''
    @staticmethod
    def calc_a_star_path_itersum(
            grid,
            boundaries,
            source_pos,
            dest_pos,
            sum_limit,
            target_to_robot_dict,
            blocked: set = None,
            calc_configure_value_func=AStarHeuristics.manhattan_distance,
            calc_configure_value_params=None):

        if blocked is None:
            blocked = set()

        open = {}
        close = {}
        parents = {source_pos: None}

        g_val = 0
        h_val = calc_configure_value_func(pos=source_pos, source_pos=source_pos, dest_pos=dest_pos,
                                          calc_configure_value_params=calc_configure_value_params)
        f_val = g_val + h_val

        h = [(f_val, g_val, source_pos)]
        open[source_pos] = (f_val, g_val)
        heapq.heapify(h)

        def construct_path(parents: dict, pos_t: (int, int, int)) -> list:
            path = []
            while parents[pos_t] is not None:
                path.append(parents[pos_t])
                pos_t = sub_tuples_with_time(pos_t, directions_to_coords_with_time[parents[pos_t]])

            return path[::-1]  # return reversed path

        while len(h) > 0:
            (f_val, g_val, pos) = heapq.heappop(h)
            if pos not in open or open[pos] != (f_val, g_val):
                continue
            open.pop(pos)
            close[pos] = (f_val, g_val)

            if pos == dest_pos:
                return construct_path(parents, pos)
            preferred_direction_order = Generator.get_preferred_direction_order(pos)
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if Generator.check_if_in_boundaries(next_pos, boundaries) \
                        and check_move_func(next_pos, grid, check_move_params) \
                        and grid.check_cell_for_a_star(next_pos, parent=direction, g_value=g_val + 1):
                    next_g_val = g_val + 1
                    next_h_val = calc_configure_value_func(pos=next_pos, source_pos=source_pos, dest_pos=dest_pos,
                                                           calc_configure_value_params=calc_configure_value_params)
                    next_f_val = next_g_val + next_h_val
                    heapq.heappush(h, (next_f_val, (-1) * next_g_val, next_pos))

        # print("calc_a_star_path: Couldn't find a from", str(source_pos), "to", str(dest_pos))
        return None
    '''
