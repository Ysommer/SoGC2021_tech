from typing import Union, List

from infrastructure.robot import Robot
from infrastructure.grid import Grid
from collections import deque
from utils import *
from defines import *
import queue
import heapq
from infrastructure.solGrid import SolGrid


class AStarHeuristics:
    @staticmethod
    def manhattan_distance(pos: (int, int), source_pos: (int, int), dest_pos: (int, int), calc_configure_value_params=None) -> int:
        manhattan_distance_val = abs(pos[0]-dest_pos[0]) + abs(pos[1]-dest_pos[1])
        return manhattan_distance_val

    def manhattan_distance_with_time(pos: (int, int, int),source_pos: (int, int, int), dest_pos: (int, int, int), calc_h_value_params=None) -> int:
        manhattan_distance_val = abs(pos[0]-dest_pos[0]) + abs(pos[1]-dest_pos[1]) + abs(pos[2]- dest_pos[2])
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
        return not (grid.is_obs(pos) or grid.has_robot(pos))


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
                        and grid.check_cell_for_bfs(next_pos, parent=direction, dist= grid.get_cell_distance(pos) + 1):
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

        assert len(res) > 0, "get_next_move_by_dist_and_obs: Failed to find next"
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
        grid.start_bfs(q)

        for pos in blocked:
            grid.check_cell_for_bfs(pos)

        g_val = 0
        h_val = calc_configure_value_func(pos=source_pos, source_pos=source_pos, dest_pos=dest_pos,
                                          calc_configure_value_params=calc_configure_value_params)
        f_val = g_val+h_val

        h = [(f_val, g_val, source_pos)]
        heapq.heapify(h)

        while len(h) > 0:
            (f_val, g_val, pos) = heapq.heappop(h)
            if pos == dest_pos:
                return Generator.construct_path(grid, pos)
            preferred_direction_order = Generator.get_preferred_direction_order(pos)
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if Generator.check_if_in_boundaries(next_pos, boundaries) \
                        and check_move_func(next_pos, grid, check_move_params) \
                        and grid.check_cell_for_bfs(next_pos, parent=direction):
                    next_g_val = (-1)*g_val + 1
                    next_h_val = calc_configure_value_func(pos=next_pos, source_pos=source_pos, dest_pos=dest_pos,
                                                           calc_configure_value_params=calc_configure_value_params)
                    next_f_val = next_g_val + next_h_val
                    heapq.heappush(h, (next_f_val, (-1)*next_g_val, next_pos))

        # print("calc_a_star_path: Couldn't find a from", str(source_pos), "to", str(dest_pos))
        return None

    @staticmethod
    def calc_a_star_path_in_time(
            grid: SolGrid,
            boundaries,
            source_pos,
            dest_pos,
            robot_id: int,
            max_time: int,
            blocked_N_visited: set = None,
            calc_h_value_func=AStarHeuristics.manhattan_distance_with_time,
            calc_h_value_params=None,
            check_move_func=SolGrid.check_move,
            check_move_params=None,
            preferred_direction_order=None): #list of moves
        if blocked_N_visited is None:
            blocked_N_visited = set()

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords_with_time.keys()

        open = {}
        close = {}
        parents = {source_pos: None}

        g_val = 0
        h_val = calc_h_value_func(pos=source_pos, source_pos=source_pos, dest_pos=dest_pos,
                                  calc_h_value_params=calc_h_value_params)
        f_val = g_val+h_val

        h = [(f_val, g_val, source_pos)]
        open[source_pos] = (f_val, g_val)
        heapq.heapify(h)

        def construct_path(parents: dict, pos: (int, int, int))-> list:
            path = []
            while parents[pos] is not None:
                path.append(parents[pos])
                pos = sub_tuples_with_time(pos, directions_to_coords_with_time[parents[pos]])

            return path[::-1]  # return reversed path

        def internal_check_move(next_pos: (int, int, int), direction: str) -> bool:
            boundaries_check = Generator.check_if_in_boundaries(next_pos, boundaries)
            can_make_it = dest_pos[2] - next_pos[2] >= \
                          AStarHeuristics.manhattan_distance((next_pos[0], next_pos[1]), (next_pos[0], next_pos[1]), (dest_pos[0], dest_pos[1]))
            not_blocked = next_pos not in blocked_N_visited
            legal_move = check_move_func(grid ,robot_id, next_pos, direction, check_move_params)
            return boundaries_check and can_make_it and not_blocked and legal_move

        while len(h) > 0:
            (f_val, g_val, pos) = heapq.heappop(h)
            g_val = (-1)*g_val
            if open[pos] != (f_val, g_val):
                (f_val, g_val) = open[pos]
            open.pop(pos)
            close[pos] = (f_val, g_val)

            if pos == dest_pos:
                return construct_path(parents, pos)
            for direction in preferred_direction_order:
                next_pos = sum_tuples_with_time(pos, directions_to_coords_with_time[direction])
                if internal_check_move(next_pos, direction):
                    next_g_val = g_val + (1 if direction != 'X' else (1/(max_time+1)))
                    next_h_val = calc_h_value_func(pos=next_pos, source_pos=source_pos, dest_pos=dest_pos,
                                                   calc_h_value_params=calc_h_value_params)
                    next_f_val = next_g_val + next_h_val
                    if next_pos in open:
                        if next_g_val < open[next_pos][1]:
                            open[next_pos] = (next_g_val + next_h_val, next_g_val)
                            parents[next_pos] = direction
                    elif next_pos in close:
                        if next_g_val < close[next_pos][1]:
                            close[next_pos] = (next_g_val + next_h_val, next_g_val)
                            open[next_pos] = close.pop(next_pos)
                            heapq.heappush(h, (next_f_val, (-1)*next_g_val, next_pos))
                            parents[next_pos] = direction
                    else:
                        parents[next_pos] = direction
                        open[next_pos] = (next_g_val + next_h_val, next_g_val)
                        heapq.heappush(h, (next_f_val, (-1)*next_g_val, next_pos))

        # print("calc_a_star_path: Couldn't find a from", str(source_pos), "to", str(dest_pos))
        return None
