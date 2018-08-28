#!/usr/bin/python

"""
Read power net frequency data from Arduino and write data to stdout.

Usage: readout.py /dev/ttyACMx

Print one line for each captured timestamp packet, formatted as:
  T <systemtime> <boardtime>

Print one line for each captured PPS packet, formatted as:
  P <systemtime> <boardtime>

Print one line for each AC mains cycle, formatted as:
  A <systemtime> <starttime> <endtime>

Print one line for each NMEA line, formatted as:
  G <systemtime> <nmea_data>
"""

import sys
import time
import serial
import struct


pktHeartbeatId = 0x10
pktAcPulseId   = 0x11
pktPpsPulseId  = 0x12
pktNmeaId      = 0x13


class UnwrapTime(object):

    def __init__(self):

        self.laststamp = None

    def unwrap(self, stamp):

        wrap = (1<<32)
        mask = wrap- 1
        maxstep = (1<<30)

        assert stamp < wrap

        laststamp = self.laststamp
        if laststamp is None:
            laststamp = stamp

        stamp += laststamp - (laststamp & mask)

        if stamp > laststamp + maxstep:
            stamp -= wrap
        if stamp + maxstep < laststamp:
            stamp += wrap

        if stamp + maxstep < laststamp or stamp > laststamp + maxstep:
            print "W unexpected wrap"

        self.laststamp = stamp
        return stamp


def readPacket(dev, initAlign=False):
    """Read one packet from the serial port.

    Return (pktHeartbeatId, systime, boardtime)
        or (pktAcPulseId, systime, starttime, endtime)
        or (pktPpsPulseId, systime, boardtime)
        or (pktNmeaId, systime, data)
        or () in case of invalid data
        or None in case of timeout.
    """

    s = dev.read(1)
    ts = time.time()
    if len(s) != 1:
        return None

    if initAlign:
        # During initial alignment, ignore mismatching type markers.
        while ord(s) not in (pktHeartbeatId,
                             pktPpsPulseId,
                             pktAcPulseId,
                             pktNmeaId):
            s = dev.read(1)
            ts = time.time()
            if len(s) != 1:
                return None

    (typ,) = struct.unpack('B', s)
            
    if typ == pktHeartbeatId or typ == pktPpsPulseId:
        s = dev.read(5)
        if len(s) != 5:
            return ()
        (csum, boardtime) = struct.unpack('<BI', s)
        if ((typ + sum(map(ord, s)))) & 0xff != 0xff:
            return ()
        return (typ, ts, boardtime)

    elif typ == pktAcPulseId:
        s = dev.read(9)
        if len(s) != 9:
            return ()
        (csum, starttime, endtime) = struct.unpack('<BII', s)
        if ((typ + sum(map(ord, s))) & 0xff) != 0xff:
            return ()
        return (typ, ts, starttime, endtime)

    elif typ == pktNmeaId:
        s = dev.read(1)
        if len(s) != 1:
            return ()
        (datalen,) = struct.unpack('B', s)
        if datalen < 1 or datalen > 60:
            return ()
        s = dev.read(datalen)
        if len(s) != datalen:
            return ()
        return (typ, ts, s)

    else:
        # Invalid type code.
        return ()


def main():

    if len(sys.argv) != 2:
        print >>sys.stderr, __doc__
        sys.exit(1)

    devname = sys.argv[1]

    dev = serial.Serial(devname, 115200, 8, 'N', 1, timeout=5)
    dev.flushInput()
    dev.read(2)
    dev.flushInput()

    print >>sys.stderr, "Aligning with data stream (1)"

    # Align with data stream
    retry = 0
    ngood = 0
    while ngood < 3 and retry < 10:

        pkt = readPacket(dev, initAlign=(ngood == 0))
        if pkt is None:
            print >>sys.stderr, "Timeout while reading from serial port"
            ngood = 0
            retry += 1

        elif pkt == ():
            ngood = 0
            retry += 1

        else:
            if ngood == 0:
                print >>sys.stderr, "Aligning with data stream (2)"
            ngood += 1

    if ngood == 0:
        # Abort after too many failures.
        print >>sys.stderr, "ERROR: Failed to align with data stream"
        sys.exit(1)

    print >>sys.stderr, "Aligned to data stream"

    unwrapper = UnwrapTime()
    nmeabuffer = ""

    # Read data
    while True:

        pkt = readPacket(dev)

        if pkt is None:
            print >>sys.stderr, "ERROR: Timeout while reading from serial port"
            break

        elif pkt == ():
            break

        elif pkt[0] == pktHeartbeatId:
            print "T %.6f %u" % (pkt[1], pkt[2])
            sys.stdout.flush()

        elif pkt[0] == pktAcPulseId:
            print "A %.6f %u %u" % (pkt[1], pkt[2], pkt[3])
            sys.stdout.flush()

        elif pkt[0] == pktPpsPulseId:
            print "P %.6f %u" % (pkt[1], pkt[2])
            sys.stdout.flush()

        elif pkt[0] == pktNmeaId:
            nmeabuffer += pkt[2]
            if nmeabuffer.endswith("\n"):
                print "G %.6f %s" % (pkt[1], nmeabuffer.rstrip())
                sys.stdout.flush()
                nmeabuffer = ""

        else:
            break

    print >>sys.stderr, "ERROR: Lost alignment with data stream"
    sys.exit(1)

if __name__ == "__main__":
    main()

