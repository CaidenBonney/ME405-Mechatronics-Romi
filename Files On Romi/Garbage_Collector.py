import gc


class GarbageCollector:
    def __init__(self):
        pass

    def run(self):
        """
        Collects garbage data for defragmentation
        """
        while True:
            gc.collect()
            yield
