"""
Small utilities used across nodes (logging wrappers, timing, simple helper funcs).
"""
import time


def now_ms():

    return int(time.time() * 1000)
