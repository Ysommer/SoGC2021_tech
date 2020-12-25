from typing import Union

from infrastructure.robot import Robot
from infrastructure.grid import Grid
from collections import deque
from utils import *
from defines import *
import queue
import heapq


class AStarHeuristics:
    @staticmethod
    def manhattan_distance(pos: (int, int), source_pos: (int, int), dest_pos: (int, int), calc_configure_value_params=None) -> int:
        manhattan_distance_val = abs(pos[0]-dest_pos[0]) + abs(pos[1]-dest_pos[1])
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
    def calc_a_star_path(
            grid: Grid,
            boundaries,
            source_pos,
            dest_pos,
            blocked: set = None,
            calc_configure_value_func=AStarHeuristics.manhattan_distance,
            calc_configure_value_params=None,
            check_move_func=CheckMoveFunction.check_free_from_obs,
            check_move_params=None,
            preferred_direction_order=None):
        if blocked is None:
            blocked = set()

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords.keys()

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

        print("calc_a_star_path: Couldn't find a from", str(source_pos), "to", str(dest_pos))
        return None
