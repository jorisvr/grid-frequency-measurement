#!/usr/bin/env python3

"""
Program which reads raw Netfreq v3 data from the Arduino and writes
the decoded data in text format to screen or file.

Usage:
  netfreq_readout.py --port /dev/ttyXX [options]

Options:
  --port /dev/ttyXX         Serial port to read data from Arduino.
  --outfile filename        Write output to specified file (default: stdout).
  --logfile filename        Write logging to specified file (default: stderr).
  --stats                   Print statistics to stderr.
  --duration N              Stop after N seconds (default: run forever).
"""

import sys
import argparse
import time

import netfreq_utils


def main():

    # Parse command line arguments.
    parser = argparse.ArgumentParser()

    parser.format_usage = lambda: __doc__ + "\n"
    parser.format_help = lambda: __doc__ + "\n"

    parser.add_argument("--port", action="store", type=str, required=True)
    parser.add_argument("--outfile", action="store", type=str)
    parser.add_argument("--logfile", action="store", type=str)
    parser.add_argument("--stats", action="store_true")
    parser.add_argument("--duration", action="store", type=int)

    args = parser.parse_args()

    # Start logging.
    netfreq_utils.init_logging(logfile=args.logfile)

    # Open output file.
    if args.outfile:
        outf = open(args.outfile, "w")
    else:
        outf = sys.stdout

    # Setup reading from serial port.
    reader = netfreq_utils.SerialFrameReader(args.port)

    # Prepare NMEA parser.
    nmea_parser = netfreq_utils.NmeaParser()

    # Align to binary data stream.
    reader.align()

    nr_pulses = 0
    first_pulse = 0
    last_pulse = 0

    t0 = time.monotonic()
    while True:

        # Read next message from Arduino.
        msg = reader.read_message()

        # Write as text.
        txt = netfreq_utils.frame_to_text(msg)
        print(txt, file=outf)
        outf.flush()

        # Keep track of AC frequency.
        if isinstance(msg, netfreq_utils.AcPulseFrame):
            if nr_pulses == 0:
                first_pulse = msg.start_time
            nr_pulses += 1
            last_pulse = msg.start_time

        # Parse NMEA information and show statistics.
        if isinstance(msg, netfreq_utils.NmeaData):
            gpsinfo = nmea_parser.process(msg)
            if args.stats and gpsinfo is not None:
                if nr_pulses > 10:
                    interval_us = (last_pulse - first_pulse) & 0xffffffff
                    freq = (nr_pulses - 1) * 1.0e6 / interval_us
                else:
                    freq = 0
                status = "Status: "
                status += "GPS time %02d:%02d:%02d, " % gpsinfo.utc_time
                if gpsinfo.gps_fix > 0:
                    status += "GPS fix, "
                else:
                    status += "no fix,  " 
                status += "%2d satellites, " % gpsinfo.nr_satellites
                status += "frequency ~ %.4f Hz" % freq
                sys.stderr.write(status + "  \r")
                sys.stderr.flush()
                nr_pulses = 0

        # Stop after requested duration.
        if args.duration is not None:
            t = time.monotonic()
            if t > t0 + args.duration:
                break

    if args.stats:
        print(file=sys.stderr)

    # Close serial port.
    reader.close()

    # Close output file.
    if args.outfile:
        outf.close()


if __name__ == "__main__":
    main()

