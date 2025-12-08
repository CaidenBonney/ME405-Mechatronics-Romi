from pyb import UART  # pyright: ignore


class DataTransfer:
    """
    Streams a CSV header once per host connection, then CSV rows:
        tl,tr,lp,rp,lv,rv
    Assumes the provided queues already yield correct Python ints (u32/s32 mapping done upstream).
    """

    def __init__(self):
        self.uart = UART(5, 115200)
        self.sent_header = False
        self.queues_were_full = False

    def reset(self, data_transfer_s):
        # all queues empty
        self.uart.write(b"Test Data Transfer Complete\r\n")
        # reset header for next test
        self.sent_header = False
        self.queues_were_full = False
        data_transfer_s.put(0)

    def run(self, shares):
        """
        Generator for your cooperative scheduler.
        'shares' should unpack to:
            data_transfer_s, tl_q, tr_q, lp_q, rp_q, lv_q, rv_q
        where each *_q has .any() and .get().
        """
        (
            data_transfer_s,
            test_complete_s,
            lt_q,
            rt_q,
            lp_q,
            rp_q,
            lv_q,
            rv_q,
        ) = shares
        queues = (lt_q, rt_q, lp_q, rp_q, lv_q, rv_q)
        self.uart.write(b"Bluetooth Connection Established\r\n")
        while True:
            # # Header first (once)
            # if not self.sent_header:
            #     self.uart.write(b"lt,rt,lp,rp,lv,rv\r\n")
            #     self.sent_header = True
            #     yield
            #     continue

            # # Not streaming
            # if not data_transfer_s.get():
            #     yield
            #     continue

            # # queues are either empty after they filled or empty after test completed -> restarting the testing
            # # -------- We should remove the queues were full thing now that we have a test complete flag that raises when the queues are full anyway
            # if (lt_q.empty() or rt_q.empty() or lp_q.empty() or rp_q.empty() or lv_q.empty() or rv_q.empty()) and (
            #     self.queues_were_full or test_complete_s.get()
            # ):
            #     self.reset(data_transfer_s)
            #     yield
            #     continue

            # if lt_q.full() or rt_q.full() or lp_q.full() or rp_q.full() or lv_q.full() or rv_q.full():
            #     self.queues_were_full = True

            # if lt_q.any() and rt_q.any() and lp_q.any() and rp_q.any() and lv_q.any() and rv_q.any():
            #     # Pop one sample from each queue
            #     lt = lt_q.get()
            #     rt = rt_q.get()
            #     lp = lp_q.get()
            #     rp = rp_q.get()
            #     lv = lv_q.get()
            #     rv = rv_q.get()
            #     # Queue a row and try to flush a bit
            #     self.uart.write(f"{lt},{rt},{lp},{rp},{lv},{rv}\r\n".encode("utf-8"))
            pass
            yield  # keep cooperative pace
