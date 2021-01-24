import sys

"""
cell = 1

print(sys.getsizeof(cell))

cell = 42 << 4 + 2
print(str(cell))
print(sys.getsizeof(cell))
"""


TIME_PER_CELL = 8
BITS_PER_TIME = 20
MASK = 0xFFFFF              # 2^20
TAIL_MASK = 0x7

CLEAN_UP_MASKS = [
    0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000,
    0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000FFFFF,
    0xFFFFFFFFFFFFFFFFFFFFFFFFF00000FFFFFFFFFF,
    0xFFFFFFFFFFFFFFFFFFFF00000FFFFFFFFFFFFFFF,
    0xFFFFFFFFFFFFFFF00000FFFFFFFFFFFFFFFFFFFF,
    0xFFFFFFFFFF00000FFFFFFFFFFFFFFFFFFFFFFFFF,
    0xFFFFF00000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF,
    0x00000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
]


def get_tail(time):
    pass

def get_id(time):
    pass


def get_cell(cells, time):
    cell = (cells[time//TIME_PER_CELL] >> (BITS_PER_TIME*(time % TIME_PER_CELL))) & MASK
    tail = cell & TAIL_MASK
    tail_transfer = ["X", "W","S","N","E"]
    return (cell >> 3, tail_transfer[tail])


def write_cell(cells, time, id, tail):
    cell = ((id << 3) + tail) << (BITS_PER_TIME*(time % TIME_PER_CELL))
    cells[time//TIME_PER_CELL] &= CLEAN_UP_MASKS[time % TIME_PER_CELL]
    cells[time//TIME_PER_CELL] |= cell


time = 39
id = 463
tail = 1

cells = [0] * 100
write_cell(cells, time, id, tail)
print(get_cell(cells, time))

for i in range(1000000):
    get_cell(cells, time)