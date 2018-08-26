"""
Read power net frequency data from Arduino and write data to stdout.

Usage: readout.py /dev/ttyN

Print one line for each captured reference pulse, formatted as:
  P <systemtime> <boardtime>

Print one line for each rising zero-crossing, formatted as:
  Z <systemtime> <boardtime> <sample> <nsamples> <sum> <sumsq>
"""

import sys
import time
import serial
import struct


def main():

    if len(sys.argv) != 2:
        print >>sys.stderr, __doc__
        sys.exit(1)

    devname = sys.argv[1]

    dev = serial.Serial(devname, 115200, 8, 'N', 1)
    dev.flushInput()
    dev.read(64)
    dev.flushInput()

    # Align with data stream
    s = dev.read(5)
    retry = 0
    while s[0] not in "\x01\x02":
        retry += 1
        if retry > 16:
            print >>sys.stderr, "Failed to align with data stream"
            sys.exit(1)
        s = s[1:] + dev.read(1)

    # Read data
    while 1:
        if s[0] == '\x01':
            # pulse
            tt = time.time()
            (ts,) = struct.unpack('<I', s[1:])
            print 'P %.6f %u' % (tt, ts)
        elif s[0] == '\x02':
            # zero-crossing
            s += dev.read(10)
            tt = time.time()
            (ts, v, vn, vsum, vsumsq) = struct.unpack('<IhHhi', s[1:])
            print 'Z %.6f %u %d %d %d %d' % (tt, ts, v, vn, vsum, vsumsq)
        else:
            # bad
            print >>sys.stderr, "Lost alignment with data stream"
            sys.exit(1)
        sys.stdout.flush()
        s = dev.read(5)

if __name__ == "__main__":
    main()

