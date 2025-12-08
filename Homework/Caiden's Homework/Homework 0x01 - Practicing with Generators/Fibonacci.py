import time, sys


# Part 1 - Fibonacci sequence generator
def fibonacci(n):
    if type(n) is not int:
        raise TypeError(f"Input, {n} ({type(n)}), input must be {type(0)}.")
    if n < 0:
        raise ValueError(f"Input, {n}, must be a positive integer.")

    a, b = 0, 1
    for _ in range(n):
        yield a
        a, b = b, a + b


# Part 2 Testing the Fibonacci generator
if __name__ == "__main__":
    print("Testing Type Error with string input:")
    try:
        next(fibonacci("10"))
    except TypeError as te:

        print(f"TypeError: {te}\n")
    print("Testing Type Error with float input:")
    try:
        next(fibonacci(1.0))
    except TypeError as te:
        print(f"TypeError: {te}\n")

    print("Testing Value Error with negative input:")
    try:
        next(fibonacci(-5))
    except ValueError as ve:
        print(f"ValueError: {ve}\n")

    # Testing the generator with valid inputs
    print("Fibonacci sequence for 0 terms:")
    for num in fibonacci(0):
        print(num)
    print()

    print("Fibonacci sequence for 1 term:")
    for num in fibonacci(1):
        print(num)
    print()

    print("Fibonacci sequence for 10 terms:")
    for num in fibonacci(10):
        print(num)
    print()

    # Set limit to handle large integers for part 3's commented print statements (Exact needed: 20790)
    sys.set_int_max_str_digits(21000)

    # Part 3 - Add every third number in the Fibonacci sequence
    print("Sum of every third Fibonacci number for 100 terms:")
    time_start = time.perf_counter_ns()
    sum_every_third_100 = int(0)
    for num in fibonacci(100):
        if num % 3 == 0:
            sum_every_third_100 += num
    time_end = time.perf_counter_ns()
    print(f"Sum: {sum_every_third_100}")
    time_diff = time_end - time_start
    print(f"Elapsed time: {time_diff/1_000_000_000} seconds")
    print()

    print("Sum of every third Fibonacci number for 100,000 terms:")
    time_start = time.perf_counter_ns()
    sum_every_third = int(0)
    for num in fibonacci(100_000):
        if num % 3 == 0:
            sum_every_third += num
    time_end = time.perf_counter_ns()
    print("Sum for 100,000 terms is very large. Uncomment print statement in code to see it.")
    # print(f"Sum: {sum_every_third}")  # Uncomment to see the sum
    time_diff = time_end - time_start
    print(f"Elapsed time: {time_diff/1_000_000_000} seconds")
    print()

    print("*********************************************")
    print("  Part 3 Extra Credit - Without using loops  ")
    print("*********************************************\n")

    # Extra Credit - Add every third number in the Fibonacci sequence without using loops
    print("Sum of every third Fibonacci number  for 100 terms without loops:")
    time_start = time.perf_counter_ns()
    sum_every_third_no_loop_100 = sum(filter(lambda x: x % 3 == 0, fibonacci(100)))
    time_end = time.perf_counter_ns()
    print(f"Sum: {sum_every_third_no_loop_100}")
    time_diff = time_end - time_start
    print(f"Elapsed time: {time_diff/1_000_000_000} seconds")
    print()

    # Extra Credit - Add every third number in the Fibonacci sequence without using loops
    print("Sum of every third Fibonacci number  for 100,000 terms without loops:")
    time_start = time.perf_counter_ns()
    sum_every_third_no_loop = sum(filter(lambda x: x % 3 == 0, fibonacci(100_000)))
    time_end = time.perf_counter_ns()
    print("Sum for 100,000 terms is very large. Uncomment print statement in code to see it.")
    # print(f"Sum: {sum_every_third_no_loop}")  # Uncomment to see the sum
    time_diff = time_end - time_start
    print(f"Elapsed time: {time_diff/1_000_000_000} seconds")
    print()

    print("Verifying both methods yield the same result for 100 terms:")
    print(sum_every_third_100 == sum_every_third_no_loop_100)
    print()

    print("Verifying both methods yield the same result for 100,000 terms:")
    print(sum_every_third == sum_every_third_no_loop)
    print()

    # Reset to default max str digits
    sys.set_int_max_str_digits(sys.int_info.default_max_str_digits)
