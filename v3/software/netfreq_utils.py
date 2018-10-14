"""
Routines for netfreq v3 data processing.
"""

import sys
import logging
import re
import serial
import struct
import time
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


class FrameSyncLost(Exception):
    """Raised by FrameReader when alignment with input stream is lost."""
    pass


class HeartbeatFrame(namedtuple('HeartbeatFrame',
                                ('host_time', 'board_time'))):
    """Represents a heartbeat frame from the microprocessor."""

    def __str__(self):
        return "T %.6f %u" % (self.host_time, self.board_time)


class AcPulseFrame(namedtuple('AcPulseFrame',
                              ('host_time', 'start_time', 'end_time'))):
    """Represents an AC pulse frame from the microprocessor."""

    def __str__(self):
        return "A %.6f %u %u" % (self.host_time,
                                 self.start_time,
                                 self.end_time)


class PpsFrame(namedtuple('PpsFrame',
                          ('host_time', 'board_time'))):
    """Represents a PPS frame from the microprocessor."""

    def __str__(self):
        return "P %.6f %u" % (self.host_time, self.board_time)


class NmeaData(namedtuple('NmeaData',
                          ('host_time', 'data'))):
    """Represents an NMEA string from the microprocessor."""

    def __str__(self):
        msg = self.data.decode('ascii', errors='replace')
        msg = msg.rstrip("\r\n")
        return 'G %.6f %s' % (self.host_time, msg)


class SerialFrameReader:
    """Read binary data frames from the microprocessor via the serial port."""

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
        self._partial_nmea_data = None

    def close(self):
        self._dev.close()

    def align(self):
        """Align to the binary data stream from the microprocessor.

        Return True when alignment succeeded.
        Return False after timeout or repeated failure to align.
        """

        _log.info("Aligning to input stream")

        # Discard pending received data.
        self._dev.reset_input_buffer()
        self._partial_nmea_data = None

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

        # Alignment failed at ALIGN_MAX_SCAN_BYTES successive positions.
        _log.warning("Failed to align to input stream")
        return False

    def read_frame(self):
        """Read one frame from the microprocessor.

        If a correct frame is received, return a corresponding frame instance
        (HeartbeatFrame, AcPulseFrame, PpsFrame, NmeaData).

        If an invalid frame is received or timeout occurs,
        raise FrameSyncError.
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
                _log.warning("Lost alignment to input stream (bad data length")
                raise FrameSyncError("Alignment lost")
        else:
            # Bad frame type
            _log.warning("Lost alignment to input stream (bad frame type)")
            raise FrameSyncError("Alignment lost")

        # Read remaining bytes of frame.
        tail = self._dev.read(plen - 2)
        if len(tail) != plen - 2:
            _log.warning("Timeout while reading frame")
            raise FrameSyncError("Timeout")

        # Verify checksum.
        if ptyp != self.FRAME_ID_NMEA:
            if (sum(hdr) + sum(tail)) & 0xff != 0xff:
                # Checksum mismatch.
                _log.warning("Lost alignment to input stream " +
                             "(checksum mismatched)")
                raise FrameSyncError("Alignment lost")

        # Decode frame.
        if ptyp == self.FRAME_ID_HEARTBEAT:
            (board_time,) = struct.unpack('<I', tail)
            return HeartbeatFrame(host_time=host_time, board_time=board_time)
        elif ptyp == self.FRAME_ID_ACPULSE:
            (start_time, end_time) = struct.unpack('<II', tail)
            return AcPulseFrame(host_time=host_time,
                                start_time=start_time,
                                end_time=end_time)
        elif ptyp == self.FRAME_ID_PPSPULSE:
            (board_time,) = struct.unpack('<I', tail)
            return PpsFrame(host_time=host_time, board_time=board_time)
        elif ptyp == self.FRAME_ID_NMEA:
            return NmeaData(host_time=host_time, data=tail)

    def read_message(self):
        """Read one message from the microprocessor.

        A message corresponds to one frame (HeartbeatFrame, AcPulseFrame,
        PpsFrame) except in the case of NMEA data, where multiple frames
        may be concatenated to reconstruct one NMEA message.

        If an invalid frame is received or timeout occurs,
        raise FrameSyncError.
        """

        while True:

            frame = self.read_frame()
            if isinstance(frame, NmeaData):
                if self._partial_nmea_data is not None:
                    self._partial_nmea_data += frame.data
                    if frame.data.endswith(b"\n"):
                        frame.data = self._partial_nmea_data.rstrip(b"\r\n")
                        self._partial_nmea_data = b""
                        return frame
                elif frame.data.endswith(b"\n"):
                    self._partial_nmea_data = b""
            else:
                return frame


class FileFrameReader:
    """Read pulse frames and NMEA messages from a text file."""

    def __init__(self, filename):
        self._file = open(filename, "r")

    def close(self):
        self._file.close()

    def read_message(self):
        """Read one message from the input file.

        Return a corresponding frame instance (HeartbeatFrame, AcPulseFrame,
        PpsFrame, NmeaData).

        Raise IOError if reading from the file fails.
        Raise ValueError if the file contains an invalid data format.
        Raise FrameSyncLost if the end of the file is reached.
        """

        s = self._file.readline()
        if not s.endswith("\n"):
            # Reached end of file.
            raise FrameSyncLost("End of file")

        m = re.match(r"^T ([0-9.]+) ([0-9]+)\s*$", s)
        if m:
            return HeartbeatFrame(host_time=float(m.group(1)),
                                  board_time=int(m.group(2)))

        m = re.match(r"^A ([0-9.]+) ([0-9]+) ([0-9]+)\s*$", s)
        if m:
            return AcPulseFrame(host_time=float(m.group(1)),
                                start_time=int(m.group(2)),
                                end_time=int(m.group(3)))

        m = re.match(r"^P ([0-9.]+) ([0-9]+)\s*$", s)
        if m:
            return PpsFrame(host_time=float(m.group(1)),
                            board_time=int(m.group(2)))

        m = re.match(r"^G ([0-9.]+) (\$.*)\n$", s)
        if m:
            return NmeaData(host_time=float(m.group(1)),
                            data=m.group(2).rstrip("\r\n").encode('ascii'))

        raise ValueError("Invalid message format in input")


class NmeaParser:
    """Parse NMEA messages and extract relevant fields."""

    def process(self, msg):
        """Process one NmeaData message.

        If the NMEA message is a correctly formatted GPGGA message,
        extract the time stamp and satellite count and return a corresponding
        GpsInfo instance.

        Otherwise, if the NMEA message is incorrect or irrelevant, return None.
        """
        assert isinstance(msg, NmeaData)

        if msg.data.startswith(b"$GPGGA"):



            return GpsInfo(host_time=msg.host_time,
                           utc_time=utc_time,
                           gps_fix=gps_fix,
                           nr_satellites=nr_sattellites)
        pass


GpsInfo = namedtuple('GpsInfo',
                     ('host_time', 'utc_time', 'gps_fix', 'nr_satellites'))


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

