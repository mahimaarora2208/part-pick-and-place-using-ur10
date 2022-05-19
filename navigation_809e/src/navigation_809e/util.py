#!/usr/bin/env python


from math import sqrt


def compute_distance(x1, y1, x2, y2):
    return sqrt(((x2-x1)**2) + ((y2-y1)**2))
