## @file Garbage_Collector.py
#  Cooperative task that periodically triggers MicroPython garbage collection
#  to reduce fragmentation during long runs.
#
#  @author Antonio Ventimiglia
#  @author Caiden Bonney
#  @date   2025-Dec-12
#  @copyright GPLv3

from gc import collect


## Task wrapper that invokes @c gc.collect() on a cooperative schedule.
class GarbageCollector:
    ## Initialize the garbage collector task.
    def __init__(self):
        pass

    ## Generator that runs garbage collection each time it's scheduled.
    #
    #  Yields after collecting so the scheduler can continue running other tasks.
    def run(self):
        while True:
            collect()
            yield
