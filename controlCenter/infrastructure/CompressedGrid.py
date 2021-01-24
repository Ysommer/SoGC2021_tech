import random


class CompressedGrid:
    def __init__(self):
        self.grid = dict()
        self.CLEAN_UP_MASKS = [
            0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000,
            0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000FFFFF,
            0xFFFFFFFFFFFFFFFFFFFFFFFFF00000FFFFFFFFFF,
            0xFFFFFFFFFFFFFFFFFFFF00000FFFFFFFFFFFFFFF,
            0xFFFFFFFFFFFFFFF00000FFFFFFFFFFFFFFFFFFFF,
            0xFFFFFFFFFF00000FFFFFFFFFFFFFFFFFFFFFFFFF,
            0xFFFFF00000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF,
            0x00000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
        ]
        self.tail_to_bits = {
            "X": 0,
            "N": 1,
            "E": 2,
            "S": 3,
            "W": 4,
        }
        self.bits_to_tail = ["X", "N", "E", "S", "W"]

    def get_cell(self,time, pos):
        pos_dict = self.grid.get(pos, {})
        cell = (pos_dict.get(time//8, 0) >> (20*(time % 8))) & 0xFFFFF
        if cell % 2 == 0:
            return (None, None)
        cell >>= 1
        tail = cell & 0x7
        return (cell >> 3, self.bits_to_tail[tail])

    def write_cell(self, time, pos, id, tail):
        cell = (id << 3) + self.tail_to_bits[tail]
        cell <<= 1
        cell += 1
        cell <<= 20*(time % 8)

        pos_grid = self.grid.get(pos, {})
        old_cell = pos_grid.get(time//8, 0)
        old_cell &= self.CLEAN_UP_MASKS[time % 8]
        old_cell |= cell
        pos_grid[time//8] = old_cell
        self.grid[pos] = pos_grid

    def clean_cell(self, time, pos):
        offset = 20*(time % 8)
        pos_dict = self.grid.get(pos, {})
        cell = pos_dict.get(time // 8, 0)
        temp_cell = cell >> offset
        if temp_cell % 2 == 0:
            return

        cell ^= ( 1 << offset)
        pos_dict[time // 8] = cell
        self.grid[pos] = pos_dict

def test1():
    random.seed()
    t = CompressedGrid()

    tests = 100000
    rand_id = []
    rand_pos = []
    rand_time = []
    rand_tail = []

    used = set()
    i = 0
    while i < tests:
        rand_time.append(random.randint(0, 10000))
        rand_pos.append((random.randint(0, 100), random.randint(0, 100)))
        if (rand_time[i], rand_pos[i]) in used:
            rand_time.pop()
            rand_pos.pop()
            continue
        used.add((rand_time[i], rand_pos[i]))
        rand_id.append(random.randint(0, 10000))
        rand_tail.append(t.bits_to_tail[random.randint(0, 4)])
        t.write_cell(rand_time[i], rand_pos[i], rand_id[i], rand_tail[i])
        i += 1

    for i in range(tests):
        expected = (rand_id[i], rand_tail[i])
        got = t.get_cell(rand_time[i], rand_pos[i])
        assert got == expected, "i:" + str(i) + "   got:" + str(got) + "   expected:" + str(expected)

    for i in range(0, tests, 2):
        t.clean_cell(rand_time[i], rand_pos[i])

    for i in range(tests):
        expected = (rand_id[i], rand_tail[i])
        got = t.get_cell(rand_time[i], rand_pos[i])
        if i % 2 == 0:
            expected = (None, None)
        assert got == expected, "i:" + str(i) + "   got:" + str(got) + "   expected:" + str(expected)