"""
Routines for netfreq v3 data processing.
"""

import sys
import logging
import re
import serial
import struct
import time

from typing import NamedTuple, Optional, Tuple, Union


# Global logger.
_log = logging.getLogger("netfreq")


class FrameSyncError(Exception):
    """Raised by FrameReader when alignment with input stream is lost."""
    pass


# Represents a heartbeat frame from the front-end microprocessor.
HeartbeatFrame = NamedTuple('HeartbeatFrame', [
    ('host_time', float),   # Host timestamp (seconds since Epoch)
    ('board_time', int)     # Board timestamp (microseconds, 32-bit)
])

# Represents an AC pulse frame from the front-end microprocessor.
AcPulseFrame = NamedTuple('AcPulseFrame', [
    ('host_time', float),   # Host timestamp (seconds since Epoch)
    ('start_time', int),    # Start of AC pulse (microseconds, 32-bit)
    ('end_time', int)       # End of AC pulse (microseconds, 32-bit)
])

# Represents a PPS frame from the front-end microprocessor.
PpsFrame = NamedTuple('PpsFrame', [
    ('host_time', float),   # Host timestamp (seconds since Epoch)
    ('board_time', int)     # Board timestamp (microseconds, 32-bit)
])

# Represents an NMEA string from the front-end microprocessor.
NmeaData = NamedTuple('NmeaData', [
    ('host_time', float),   # Host timestamp (seconds since Epoch)
    ('data', bytes)         # Sequence of NMEA data bytes
])

# GPS fix data extracted from an NMEA $GPGGA message.
GpsInfo = NamedTuple('GpsInfo', [
    ('host_time', float),   # Host timestamp (seconds since Epoch)
    ('utc_time', Tuple[int, int, int]),  # GPS timestamp as tuple (h,m,s) UTC
    ('gps_fix', int),       # 0 = no fix, 1 = GPS fix, 2 = DGPS fix
    ('nr_satellites', int)  # Number of satellites in use
])

# Union of all data frame types.
_FrameType = Union[HeartbeatFrame, AcPulseFrame, PpsFrame, NmeaData]


def init_logging(logfile=None, level=logging.INFO):
    """Initialize Python logging framework."""

    if logfile is not None:
        h = logging.FileHandler(logfile, "a")
    else:
        h = logging.StreamHandler(sys.stderr)
    fmt = logging.Formatter(
        fmt="[%(asctime)s] %(name)s %(levelname)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S")
    fmt.converter = time.gmtime
    h.setFormatter(fmt)
    logging.root.addHandler(h)
    logging.root.setLevel(level)


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

    NMEA_MAX_LENGTH     = 64

    def __init__(self, port) -> None:
        self._dev = serial.Serial(port,
                                  baudrate=self.BAUDRATE,
                                  timeout=self.TIMEOUT)
        self._dev.reset_input_buffer()
        self._partial_nmea_data = bytearray()

    def close(self) -> None:
        self._dev.close()

    def align(self) -> bool:
        """Align to the binary data stream from the microprocessor.

        Return True when alignment succeeded.
        Return False after timeout or repeated failure to align.
        """

        _log.info("Aligning to input stream")

        # Discard pending received data.
        self._dev.reset_input_buffer()
        self._partial_nmea_data = bytearray()

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
                    datalen = buf[pos+1]
                    plen = 2 + datalen
                    if datalen > self.NMEA_MAX_LENGTH:
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

    def read_frame(self) -> _FrameType:
        """Read one frame from the microprocessor.

        If a correct frame is received, return a corresponding frame instance
        (HeartbeatFrame, AcPulseFrame, PpsFrame, NmeaData).

        If an invalid frame is received or timeout occurs,
        raise FrameSyncError.
        """

        # Read first two bytes of frame.
        hdr = self._dev.read(2)
        host_time = time.time()

        if len(hdr) != 2:
            _log.warning("Timeout while reading frame")
            raise FrameSyncError("Timeout")

        # Determine frame length.
        ptyp = hdr[0]
        if ptyp == self.FRAME_ID_HEARTBEAT:
            # 6 bytes
            # byte 0     = 0x10
            # byte 1     = checksum
            # bytes 2..5 = (uint32_le) timestamp
            plen = 6
        elif ptyp == self.FRAME_ID_ACPULSE:
            # 10 bytes
            # byte 0     = 0x11
            # byte 1     = checksum
            # bytes 2..5 = (uint32_le) timestamp of pulse start
            # bytes 5..9 = (uint32_le) timestamp of pulse end
            plen = 10
        elif ptyp == self.FRAME_ID_PPSPULSE:
            # 6 bytes
            # byte 0     = 0x12
            # byte 1     = checksum
            # bytes 2..5 = (uint32_le) timestamp of PPS pulse
            plen = 6
        elif ptyp == self.FRAME_ID_NMEA:
            # 3..62 bytes
            # byte 0     = 0x13
            # byte 1     = data length (range 1..60)
            # byte 2..   = 1..60 bytes NMEA string data
            datalen = hdr[1]
            plen = 2 + datalen
            if datalen > self.NMEA_MAX_LENGTH:
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

    def read_message(self) -> _FrameType:
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
                self._partial_nmea_data += frame.data
                if frame.data.endswith(b"\n"):
                    nmea_data = bytes(self._partial_nmea_data.rstrip(b"\r\n"))
                    self._partial_nmea_data = bytearray()
                    return NmeaData(host_time=frame.host_time,
                                    data=nmea_data)
            else:
                return frame


def frame_to_text(frame: _FrameType) -> str:
    """Serialize a data frame to a single-line ASCII string."""

    if isinstance(frame, HeartbeatFrame):
        return "T %.6f %u" % (frame.host_time, frame.board_time)
    elif isinstance(frame, AcPulseFrame):
        return "A %.6f %u %u" % (frame.host_time,
                                 frame.start_time,
                                 frame.end_time)
    elif isinstance(frame, PpsFrame):
        return "P %.6f %u" % (frame.host_time, frame.board_time)
    elif isinstance(frame, NmeaData):
        msg = frame.data.rstrip(b"\r\n").decode('ascii', errors='replace')
        return 'G %.6f %s' % (frame.host_time, msg)
    else:
        assert False


class FileFrameReader:
    """Read pulse frames and NMEA messages from a text file."""

    def __init__(self, filename) -> None:
        self._file = open(filename, "r")

    def close(self) -> None:
        self._file.close()

    def read_message(self) -> _FrameType:
        """Read one message from the input file.

        Return a corresponding frame instance (HeartbeatFrame, AcPulseFrame,
        PpsFrame, NmeaData).

        Raise IOError if reading from the file fails.
        Raise ValueError if the file contains an invalid data format.
        Raise FrameSyncError if the end of the file is reached.
        """

        s = self._file.readline()
        if not s.endswith("\n"):
            # Reached end of file.
            raise FrameSyncError("End of file")

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

    @staticmethod
    def _verify_checksum(data: bytes) -> Tuple[bool, bytes]:
        """Verify NMEA checksum and remove checksum from NMEA message."""
        m = re.match(br"^\$([^*]*)\*([0-9a-fA-F]{2})$", data)
        if m:
            msg = m.group(1)
            got_checksum = int(m.group(2), 16)
            calc_checksum = 0
            for b in msg:
                calc_checksum ^= b
            return ((calc_checksum == got_checksum), msg)
        return (False, b"")

    def process(self, msg: NmeaData) -> Optional[GpsInfo]:
        """Process one NmeaData message.

        If the NMEA message is a correctly formatted GPGGA message,
        extract the time stamp and satellite count and return a corresponding
        GpsInfo instance.

        Otherwise, if the NMEA message is incorrect or irrelevant, return None.
        """
        assert isinstance(msg, NmeaData)

        if not msg.data.startswith(b"$GPGGA,"):
            # Not a GPGGA message.
            return None

        (checksum_ok, msg_stripped) = self._verify_checksum(msg.data)
        if not checksum_ok:
            _log.warning("Checksum mismatch in NMEA message %s", msg.data)
            return None

        msg_fields = msg_stripped.split(b",")
        assert msg_fields[0] == b"GPGGA"
        if len(msg_fields) != 15:
            _log.warning("Invalid GPGGA format in NMEA message %s", msg.data)
            return None

        if len(msg_fields[1]) < 6:
            _log.warning("Invalid timestamp in NMEA message %s", msg.data)
            return None

        try:
            utc_hour = int(msg_fields[1][0:2])
            utc_min = int(msg_fields[1][2:4])
            utc_sec = int(msg_fields[1][4:6])
            gps_fix = int(msg_fields[6])
            nr_satellites = int(msg_fields[7])
        except ValueError:
            _log.warning("Invalid GPGGA format in NMEA message %s", msg.data)
            return None

        if ((utc_hour < 0) or (utc_hour > 23)
                or (utc_min < 0) or (utc_min > 59)
                or (utc_sec < 0) or (utc_sec > 60)):
            _log.warning("Invalid timestamp in NMEA message %s", msg.data)
            return None

        return GpsInfo(host_time=msg.host_time,
                       utc_time=(utc_hour, utc_min, utc_sec),
                       gps_fix=gps_fix,
                       nr_satellites=nr_satellites)

