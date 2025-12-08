class RollingQueue:
    def __init__(self, size):
        self.size = size
        self.buf = [0] * size  # underlying storage
        self.index = 0  # next write position
        self.count = 0  # how many values are actually stored
        self.sum = 0  # running sum of values

    # Add a new value, replacing the oldest when full.
    def push(self, value):
        if value == 0:
            return

        if self.count < self.size:
            # Still filling the buffer
            self.buf[self.index] = value
            self.sum += value
            self.count += 1
        else:
            # Buffer is full, overwrite the oldest value
            old = self.buf[self.index]
            self.sum += value - old
            self.buf[self.index] = value

        # Advance circular index
        self.index += 1
        if self.index >= self.size:
            self.index = 0

    # Return the average of all values in the queue
    def average(self):
        if self.count == 0:
            return 0
        return self.sum / self.count

    # Return a list of the values in the queue, starting at the oldest and
    # continuing until the end of the buffer.
    def values(self):
        if self.count < self.size:
            # Only part of the buffer is used
            return self.buf[: self.count]
        # Full buffer, wrap from index to end, then start to index
        return self.buf[self.index :] + self.buf[: self.index]
