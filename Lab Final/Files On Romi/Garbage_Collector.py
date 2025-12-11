from gc import collect  # , mem_alloc, mem_free


class GarbageCollector:
    def __init__(self):
        pass

    # Collects garbage data for defragmentation
    def run(self):
        while True:
            collect()
            # print("Free:", mem_free())
            # print("Allocated:", mem_alloc())
            yield
