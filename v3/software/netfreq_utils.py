"""
Routines for netfreq v3 data processing.
"""

import sys
import logging
import time
import serial
import struct
from collections import namedtuple


# Global logger.
_log = logging.getLogger("netfreq")


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


class FrameReader:
    """Read binary data packets from the Arduino sketch via serial port."""

    # Frame types returned by frame reader.
    Heartbeat   = namedtuple('HeartbeatFrame', ['host_time', 'board_time'])
    AcPulse     = namedtuple('AcPulseFrame',   ['host_time', 'start_time', 'end_time'])
    PpsPulse    = namedtuple('PpsFrame',       ['host_time', 'board_time'])
    NmeaData    = namedtuple('NmeaFrame',      ['host_time', 'data'])
    SyncLost    = namedtuple('SyncLost',       ['host_time', 'message'])

    # Serial port settings.
    BAUDRATE = 115200
    TIMEOUT = 5.0

    # Alignment parameters.
    ALIGN_MIN_GOOD = 3
    ALIGN_MAX_SCAN_BYTES = 80

    # Frame type IDs.
    FRAME_ID_HEARTBEAT  = 0x10
    FRAME_ID_ACPULSE    = 0x11
    FRAME_ID_PPSPULSE   = 0x12
    FRAME_ID_NMEA       = 0x13

    def __init__(self, port):
        self._dev = serial.Serial(port, baudrate=self.BAUDRATE, timeout=self.TIMEOUT)
        self._dev.reset_input_buffer()

    def align(self):
        """Align to the binary data stream from the microprocessor.

        Return True when alignment succeeded.
        Return False after timeout or repeated failure to align.
        """

        _log.info("Aligning to input stream")

        # Discard pending received data.
        self._dev.reset_input_buffer()

        # Temporary read-ahead buffer.
        buf = bytearray()

        # Try alignment at successive start positions.
        for scanpos in range(self.ALIGN_MAX_SCAN_BYTES):

            pos = scanpos
            ngood = 0

            # Try to detect frames at current position.
            while ngood < self.ALIGN_MIN_GOOD or pos < len(buf):

                # Get first two bytes of frame.
                if pos + 2 > len(buf):
                    recv = self._dev.read(pos + 2 - len(buf))
                    if pos + 2 > len(buf) + len(recv):
                        _log.warning("Timeout while aligning to input stream")
                        return False
                    buf += recv

                # Determine frame length.
                ptyp = buf[pos]
                if ptyp == self.FRAME_ID_HEARTBEAT:
                    plen = 6
                elif ptyp == self.FRAME_ID_ACPULSE:
                    plen = 10
                elif ptyp == self.FRAME_ID_PPSPULSE:
                    plen = 6
                elif ptyp == self.FRAME_ID_NMEA:
                    plen = buf[pos+1]
                    if plen > self.NMEA_MAX_LENGTH:
                        # Bad NMEA frame length - retry alignment.
                        break
                else:
                    # Bad frame type - retry alignment.
                    break

                # Get remaining bytes of frame.
                if pos + plen > len(buf):
                    recv = self._dev.read(pos + plen - len(buf))
                    if pos + plen > len(buf) + len(recv):
                        _log.warning("Timeout while aligning to input stream")
                        return False
                    buf += recv

                if ptyp != self.FRAME_ID_NMEA:
                    # Verify checksum.
                    if sum(buf[pos:pos+plen]) & 0xff != 0xff:
                        # Checksum mismatch.
                        break

                # Got good frame.
                pos += plen
                ngood += 1

            if ngood >= self.ALIGN_MIN_GOOD:
                # Succesful alignment.
                _log.info("Aligned to input stream")
                return True

        # Alignment failed at ALIGN_MAX_SCAN_BYTES succesive positions.
        _log.warning("Failed to align to input stream")
        return False


    def read_frame(self):
        """Read one data frame from the microprocessor.

        If a correct frame is received, return a corresponding frame instance
        (Heartbeat, AcPulse, PpsPulse, NmeaData).

        If an invalid frame is received or timeout occurs,
        return a SyncLost instance.
        """

        # Read first two bytes of frame.
        rawframe = self._dev.read(2)
        host_time = time.time()

        if len(rawframe) != 2:
            _log.warning("Timeout while reading frame")
            return FrameReader.SyncLost(host_time=host_time,
                                        message="Timeout")

        # Determine frame length.
        ptyp = rawframe[0]
        if ptyp == self.FRAME_ID_HEARTBEAT:
            plen = 6
        elif ptyp == self.FRAME_ID_ACPULSE:
            plen = 10
        elif ptyp == self.FRAME_ID_PPSPULSE:
            plen = 6
        elif ptyp == self.FRAME_ID_NMEA:
            plen = hdr[1]
            if plen > self.NMEA_MAX_LENGTH:
                # Bad NMEA frame length.
                _log.warning("Lost alignment to input stream")
                return FrameReader.SyncLost(host_time=host_time,
                                            message="Alignment lost")
        else:
            # Bad frame type
            _log.warning("Lost alignment to input stream")
            return FrameReader.SyncLost(host_time=host_time,
                                        message="Alignment lost")

        # Read remaining bytes of frame.
        tail = self._dev.read(plen - 2)
        if len(tail) != plen - 2:
            _log.warning("Timeout while reading frame")
            return FrameReader.SyncLost(host_time=host_time,
                                        message="Timeout")

        # Verify checksum.
        if ptyp != self.FRAME_ID_NMEA:
            if (sum(hdr) + sum(tail)) & 0xff != 0xff:
                # Checksum mismatch.
                _log.warning("Lost alignment to input stream")
                return FrameReader.SyncLost(host_time=host_time,
                                            message="Alignment lost")

        # Decode frame.
        if ptyp == self.FRAME_ID_HEARTBEAT:
            (board_time,) = struct.unpack('<I', tail)
            return FrameReader.Heartbeat(host_time=host_time,
                                         board_time=board_time)
        elif ptyp == self.FRAME_ID_ACPULSE:
            (start_time, end_time) = struct.unpack('<II', tail)
            return FrameReader.AcPulse(host_time=host_time,
                                       start_time=start_time,
                                       end_time=end_time)
        elif ptyp == self.FRAME_ID_PPSPULSE:
            (board_time,) = struct.unpack('<I', tail)
            return FrameReader.PpsPulse(host_time=host_time,
                                        board_time=board_time)
        elif ptyp == self.FRAME_ID_NMEA:
            return FrameReader.NmeaData(host_time=host_time,
                                        data=tail)


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

