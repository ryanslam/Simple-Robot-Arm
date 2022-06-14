from enum import IntEnum

class PrimaryColors(IntEnum):
    RED = 1
    YELLOW = 2
    BLUE = 3

class Rainbow(PrimaryColors):
    ORANGE = 4

r = Rainbow()