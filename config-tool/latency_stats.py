#!/usr/bin/env python3

# Prints Switch Pro Controller mode latency counters once per interval.
# Times run from a host input report arriving at the remapper to the
# Pro report that carries it being read by the console (or computer).

from common import *

import struct
import sys
import time

interval = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
samples = int(sys.argv[2]) if len(sys.argv) > 2 else 0  # 0 = run until interrupted

device = get_device()


def read_stats():
    data = struct.pack("<BBB26B", REPORT_ID_CONFIG, CONFIG_VERSION, GET_LATENCY_STATS, *([0] * 26))
    device.send_feature_report(add_crc(data))
    data = get_feature_report(device, REPORT_ID_CONFIG, CONFIG_SIZE + 1)
    fields = struct.unpack("<BLLHHHHLHHLL", data)
    check_crc(data, fields[-1])
    return fields[1:-1]


def avg(total, count):
    return total / count if count else 0.0


read_stats()  # Clear counters
start = time.monotonic()
while True:
    samples -= 1
    time.sleep(interval)
    now = time.monotonic()
    elapsed, start = now - start, now
    (
        inputs,
        reports,
        early,
        input_gap_max,
        input_count,
        input_max,
        input_sum,
        button_count,
        button_max,
        button_sum,
    ) = read_stats()
    print(
        f"in {inputs / elapsed:6.1f}/s gap<={input_gap_max}us | "
        f"out {reports / elapsed:6.1f}/s early {early} | "
        f"input->read avg {avg(input_sum, input_count):6.0f}us max {input_max}us | "
        f"button->read n={button_count} avg {avg(button_sum, button_count):6.0f}us max {button_max}us",
        flush=True,
    )
    if samples == 0:
        break
