def phi(x, y, w):
    return y + w * x

import math

def phi_xdp(x, y, w):
    return (y + (2 * w - 1) * x + math.sqrt((y-x) ** 2 + 4 * w * y * x)) / (2 * w)

def phi_xup(x, y, w):
    return (y + x + math.sqrt((y + x) ** 2 + 4 * w * (w-1) * x * x)) / (2 * w)

def diagonal_distance(i1, j1, i2, j2):
    x = abs(i1 - i2)
    y = abs(j1 - j2)
    return min(x, y) * math.sqrt(2) + abs(x - y)