import time


def fibonaci(n):
    if type(n) is not int:
        if isinstance(n, str):
            try:
                n = int(n)
            except ValueError:
                raise ValueError("Incorrect input value. String inputs must contain only integers.")
        else:
            raise TypeError("Incorrect type inputted. Input type must be an integer or string of an integer.")       
    if n <= 0:
        raise ValueError("Input must be a positive non-zero integer.")
    a, b = 0, 1
    for _ in range(n):
        yield a
        a, b = b, a + b



if __name__ == "__main__":
    start_time = time.perf_counter_ns()
    fib_length = 7

    total = sum(                            
    map(    
        lambda pair: pair[1],
        filter(
            lambda pair: (pair[0]) % 3 == 0,
            enumerate(fibonaci(fib_length))
            )
        )
    )

    # enumerate makes lazy list of tuples (index, value)
    # filter grabs every third tuple
    # map extracts the value from each tuple
    # sum adds them all up

    end_time = time.perf_counter_ns()
    time_elapsed = (end_time - start_time) / 1_000_000_000  # Convert nanoseconds to seconds
    print(f"Sum of every third value: {total}")
    print(f"Execution time: {time_elapsed} seconds")
