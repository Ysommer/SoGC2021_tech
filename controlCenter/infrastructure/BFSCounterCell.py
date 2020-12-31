class BFSCounterCell:
    def __init__(self, last_bfs_counter=0, last_bfs_parent: str = "", last_configured_dist=-1):
        # BFS support
        self.last_bft_counter = last_bfs_counter    # Index of last BFS
        self.last_bfs_parent = last_bfs_parent

        # Distance support
        self.last_configured_dist = last_configured_dist

        # A-Star support
        self.g_value = -1

    def check_move(self, new_bfs_counter: int, parent: str = "", dist=-1, to_update=True) -> True:
        if self.last_bft_counter < new_bfs_counter:
            if to_update:
                self.last_bfs_parent = parent
                self.last_bft_counter = new_bfs_counter
                self.last_configured_dist = dist
            return True
        return False

    def check_move_a_star(self, new_bfs_counter: int, parent: str, g_value, to_update=True) -> True:
        if self.last_bft_counter < new_bfs_counter or self.g_value > g_value:
            if to_update:
                self.last_bfs_parent = parent
                self.last_bft_counter = new_bfs_counter
                self.g_value = g_value
            return True
        return False

    def check_if_open(self, new_bfs_counter: int, g_value: int):
        return self.last_bft_counter < new_bfs_counter or g_value <= self.g_value

    def get_parent(self, new_bfs_counter) -> str:
        if self.last_bft_counter == new_bfs_counter:
            return self.last_bfs_parent
        return ""

    def get_distance(self, new_bfs_counter) -> int:
        if self.last_bft_counter == new_bfs_counter:
            return self.last_configured_dist
        return -1