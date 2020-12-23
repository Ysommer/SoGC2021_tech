from infrastructure.robot import Robot
from infrastructure.grid import Grid
from collections import deque
from utils import *
from defines import *
import queue


class Generator:
    """
        A class with a generic functions
    """

    @staticmethod
    def default_bfs_single_dest_key(source_pos, dest_params):
        return source_pos == dest_params

    @staticmethod
    def default_bfs_many_dest_keys(source_pos, dest_params):
        if source_pos in dest_params:
            return dest_params.index(source_pos)
        return -1

    @staticmethod
    def default_cell_is_clear_key(pos, grid: Grid, clear_cell_params=None):
        """
        :param pos:
        :param clear_cell_params: grid
        :return: if cell is clear
        """
        return grid.check_if_cell_is_free(pos)

    @staticmethod
    def cell_is_clear_from_obs(pos, grid: Grid, clear_cell_params=None):
        return not grid.get_cell(pos).is_obs()

    @staticmethod
    def cell_is_clear_from_robots_on_target_and_obs(pos, grid: Grid, clear_cell_params=None):
        pass

    @staticmethod
    def cell_is_clear_from_robots_and_obs(pos, grid: Grid, clear_cell_params=None):
        pass


    @staticmethod
    def cell_is_clear_ignore_robots_not_on_target(pos, grid: Grid, clear_cell_params=None):
        return grid.check_if_cell_is_free(pos) or \
                (grid.get_cell(pos).has_robot() is not None and not grid.get_cell(pos).has_robot_on_target())

    @staticmethod
    def cell_is_not_an_obs(pos, grid: Grid, clear_cell_params=None):
        return grid.check_if_cell_is_free(pos) or grid.get_cell(pos).has_robot() is not None

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
            source_pos: tuple,
            grid: Grid,
            dest_params,
            boundaries,
            blocked=None,
            dest_key=None,
            clear_cell_params=None,
            clear_cell_key=None,
            preferred_direction_order=None) -> deque:
        """
        calculate a bfs path from one source position to generic dest positions (one-to-many)
        :param source_pos: the position to calc from - a single position
        :param grid: the grid
        :param dest_params: the params related to destination - will be passed to dest key
        :param blocked: a list of blocked positions
        :param dest_key: a function(pos, dest_params)->bool that return if pos in dests
        :param clear_cell_key: a function(pos, clear_cell_params)->bool to check if cell is empty
        :param clear_cell_params: params which passed to clear_cell_key
        :param preferred_direction_order: preferred_direction_order: a list of directions (e.g ["N", "W", "S", "E"] or ["S", "W"])
        :param boundaries: a dictionary of allowed boundaries e.g ["N": grid.size, ...]
        :return: a dequeue of bfs path
        """
        if blocked is None:
            blocked = {}

        if clear_cell_key is None:
            clear_cell_key = Generator.default_cell_is_clear_key

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords

        if dest_key is None:
            dest_key = Generator.default_bfs_single_dest_key

        grid.start_bfs([source_pos])
        for pos in blocked.keys():
            grid.check_cell_for_bfs(pos)

        # visited = [self.robots[i].pos]
        q = queue.Queue()
        q.put(source_pos)

        while not q.empty():
            pos = q.get()
            if dest_key(pos, dest_params):
                return Generator.construct_path(grid, pos)
            for direction in preferred_direction_order:
                next_pos = sum_tuples(pos, directions_to_coords[direction])
                if Generator.check_if_in_boundaries(next_pos, boundaries) \
                        and clear_cell_key(next_pos, grid, clear_cell_params) \
                        and grid.check_cell_for_bfs(next_pos, parent=direction):
                    q.put(next_pos)
                    # visited.append(next_pos)

        return deque()

    @staticmethod
    def calc_travel_distance(
            source_pos: tuple,
            grid: Grid,
            boundaries,
            out_size=1,
            blocked: dict = None,
            dest_params=None,
            dest_key=None,
            clear_cell_params=None,
            clear_cell_key=None,
            preferred_direction_order=None):
        """

        :param source_pos: the position to calc from - a single position
        :param grid: the grid
        :param dest_params: the params related to destination
        :param clear_cell_params: params which passed to clear_cell_key
        :param boundaries: a dictionary of allowed boundaries e.g ["N": grid.size, ...]
        :param out_size: the size of the output list
        :param blocked: a list of blocked positions
        :param dest_key: a function(pos, dest_params)->int that returns: -1 if pos not a dest or dest_index if is
                                                         ### NOTE: -1 <= dest_key output must me < out_size ###
        :param clear_cell_key: a function(pos, clear_cell_params)->bool to check if cell is empty
        :param preferred_direction_order: a list of directions (e.g ["N", "W", "S", "E"] or ["S", "W"])
        :return: a list of distances
        """

        if blocked is None:
            blocked = {}

        if clear_cell_key is None:
            clear_cell_key = Generator.default_cell_is_clear_key

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords

        if dest_key is None:
            if out_size > 1:
                dest_key = Generator.default_bfs_many_dest_keys
            else:
                dest_key = Generator.default_bfs_single_dest_key

        grid.start_bfs([source_pos])
        for pos in blocked.keys():
            grid.check_cell_for_bfs(pos)

        current_level = []
        next_level = []
        next_level.append(source_pos)
        number_of_found_dests = 0
        dist = 0
        dists = [-1] * out_size

        while len(next_level) > 0:
            current_level.clear()
            current_level = next_level
            next_level = []
            dist += 1
            for pos in current_level:
                dest_index = dest_key(pos, dest_params)
                if dest_index != -1:
                    dists[dest_index] = dist
                    number_of_found_dests += 1
                    if number_of_found_dests == out_size:
                        return dists

                for direction in preferred_direction_order:
                    next_pos = sum_tuples(pos, directions_to_coords[direction])
                    if Generator.check_if_in_boundaries(next_pos, boundaries)   \
                            and clear_cell_key(next_pos, grid, clear_cell_params)     \
                            and grid.check_cell_for_bfs(next_pos):
                        next_level.append(next_pos)

        # reaches here only if number_of_found_dests < out_size
        for i in range(len(dists)):
            if dists[i] == -1:
                print("Couldn't find a path for robot", str(i))

        return None

    @staticmethod
    def calc_bfs_map(
            sources: list,
            grid: Grid,
            boundaries,
            blocked: dict = None,
            clear_cell_params=None,
            clear_cell_key=None,
            preferred_direction_order=None):

        sources = sources.copy()

        if blocked is None:
            blocked = {}

        if clear_cell_key is None:
            clear_cell_key = Generator.cell_is_clear_ignore_robots_not_on_target

        if preferred_direction_order is None:
            preferred_direction_order = directions_to_coords

        dist = 0

        grid.start_bfs(sources)
        for pos in blocked.keys():
            grid.check_cell_for_bfs(pos)

        next_level = sources
        current_level = []

        while len(next_level) > 0:
            current_level.clear()
            current_level = next_level
            next_level = []
            dist += 1
            for pos in current_level:
                for direction in preferred_direction_order:
                    next_pos = sum_tuples(pos, directions_to_coords[direction])
                    if Generator.check_if_in_boundaries(next_pos, boundaries)       \
                            and clear_cell_key(next_pos, grid, clear_cell_params)   \
                            and grid.check_cell_for_bfs(next_pos, dist=dist):
                        next_level.append(next_pos)
