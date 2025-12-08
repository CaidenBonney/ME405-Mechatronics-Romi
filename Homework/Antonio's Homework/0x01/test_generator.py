import unittest
from generator import fibonaci


class TestFibonacci(unittest.TestCase):
    def test_fibonaci_sequence(self):
        """Check that the Fibonacci generator yields the correct sequence."""
        n = 10
        expected = [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
        result = list(fibonaci(n))
        self.assertEqual(result, expected)

    def test_fibonaci_non_integer_input(self):
        """Ensure non-integer input raises the correct exception."""
        with self.assertRaises(TypeError):
            list(fibonaci(3.5))
        with self.assertRaises(ValueError):
            list(fibonaci("abc"))
        with self.assertRaises(TypeError):
            list(fibonaci([1, 2, 3]))
        with self.assertRaises(TypeError):
            list(fibonaci(None))
        with self.assertRaises(ValueError):
            list(fibonaci("12.34"))
        with self.assertRaises(ValueError):
            list(fibonaci("12abc")) 
        with self.assertRaises(ValueError):
            list(fibonaci(""))
        with self.assertRaises(TypeError):
            list(fibonaci(False))

    def test_fibonaci_string_integer_input(self):
        """Check that string integer input works."""
        result = list(fibonaci("5"))
        self.assertEqual(result, [0, 1, 1, 2, 3])

    def test_fibonaci_negative_or_zero(self):
        """Check for zero or negative input."""
        with self.assertRaises(ValueError):
            list(fibonaci(0))
        with self.assertRaises(ValueError):
            list(fibonaci(-10))
        with self.assertRaises(ValueError):
            list(fibonaci("0"))
        with self.assertRaises(ValueError):
            list(fibonaci("-5"))


    def test_sum_every_third_value(self):
        """Verify the sum of every third Fibonacci number for small n."""
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
        # Fibonacci sequence up to n=7 → [0,1,1,2,3,5,8,13,21,34]
        # Every 3rd value: 0 (index 0), 2 (3), 8 (6)
        expected_sum = 0 + 2 + 8
        self.assertEqual(total, expected_sum)


if __name__ == "__main__":
    unittest.main()
