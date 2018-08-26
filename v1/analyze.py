"""
Analyze power net frequency data.

Usage: analyze.py datafile.dat
"""

import sys
import time
import numpy
import numpy.fft
import scipy.stats
import scipy.signal.filter_design as scipy_fd


# Frequency of Arduino board clock
boardclock_hz = 16.0e6

# Sample frequency of sine wave
samplefreq_hz = 1.0/0.000228

# Estimated slope of sine wave at zero crossing
# = 2 * PI * 50 Hz / samplefreq_hz * sqrt(2) * 315 lsb rms
# = 32 lsb/sample
zcross_sine_slope = 32.0

# Gain of transformer, resistor network, ADC
volt_per_lsb  = 0.7261

# Number of averages in power spectrum
pwelch_navg = 7


def timestr(t):
    return time.strftime('%F %T (UTC)', time.gmtime(t))


def readfile(fname):
    """Read pulse and zero-cross data from the specified file.
    Also unwrap the board time counter.
    Return (pulse, zcross) where
      pulse is (systime[], boardtime[]) and
      zcross is (systime[], boardtime[], sample[], nsamples[], sum[], sumsq[]).
    """

    f = file(fname, 'r')

    # Count nr of lines
    npulse  = 0
    nzcross = 0
    nline   = 0
    s = f.readline()
    s1 = f.readline()
    while s1:
        nline += 1
        if s.startswith('P'):
            npulse += 1
        elif s.startswith('Z'):
            nzcross += 1
        else:
            print >>sys.stderr, '\nInvalid format on line', nline
            sys.exit(1)
        if nline % 10000 == 0:
            print '\rScanning line', nline,
            sys.stdout.flush()
        s = s1
        s1 = f.readline()
    print '... done'

    f.seek(0)

    # Allocate arrays
    psystime   = numpy.zeros(npulse,  numpy.float64)
    pboardtime = numpy.zeros(npulse,  numpy.float64)
    zsystime   = numpy.zeros(nzcross, numpy.float64)
    zboardtime = numpy.zeros(nzcross, numpy.float64)
    zsample    = numpy.zeros(nzcross, numpy.int32)
    znsamples  = numpy.zeros(nzcross, numpy.int32)
    zsum       = numpy.zeros(nzcross, numpy.int32)
    zsumsq     = numpy.zeros(nzcross, numpy.int32)

    # Read data
    ip = 0
    iz = 0
    i = 0
    timehigh = 0
    for i in xrange(nline):

        s = f.readline()

        v = s.strip().split()
        if (v[0] != 'P' or len(v) != 3) and (v[0] != 'Z' or len(v) != 7):
            print >>sys.stderr, '\nERROR: Invalid format on line', i+1
            sys.exit(1)

        # Unwrap board time counter and check for anomalies.
        # This fails if there is no activity for more than 67 seconds.
        skip = 0
        timelow = int(v[2])
        if i:
            if timelow < prevtime - (1<<24):
                # positive overflow on time counter
                timehigh += 1<<32
                if timelow > prevtime - (1<<32) + (1<<30):
                    skip = 1    # too big forward skip
            elif timelow > prevtime + (1<<30):
                # negative overflow on time counter (or too big forward skip)
                timehigh -= 1<<32
                if timelow < prevtime + (1<<32) - (1<<24):
                    skip = 1    # too big forward/backward skip
        if skip:
            print '\nERROR: Boardtime skips from', prevtime, 'to', timelow, 'on line', (i+1)
        boardtime = timehigh + timelow
        prevtime  = timelow

        if v[0] == 'P':
            psystime[ip]   = float(v[1])
            pboardtime[ip] = boardtime
            ip += 1
        elif v[0] == 'Z':
            zsystime[iz]   = float(v[1])
            zboardtime[iz] = boardtime
            zsample[iz]    = int(v[3])
            znsamples[iz]  = int(v[4])
            zsum[iz]       = int(v[5])
            zsumsq[iz]     = int(v[6])
            iz += 1

        if (i+1) % 10000 == 0:
            print '\rReading line', i+1, '/', nline,
            sys.stdout.flush()

    print '... done'
    print 'Got', npulse, 'pulses and', nzcross, 'zero-cossings'
    print

    f.close()

    return ( (psystime, pboardtime),
             (zsystime, zboardtime, zsample, znsamples, zsum, zsumsq) )


def checkBoardclock(systime, boardtime):
    """Verify rate of board clock against system time."""

    sdelta = systime[1:] - systime[:-1]
    bdelta = boardtime[1:] - boardtime[:-1]

    # Check that board clock is strictly monotonous.
    if numpy.any(bdelta <= 0):
        i = numpy.where(bdelta <= 0)[0][0]
        print 'ERROR: Non-monotonous step in board clock:'
        print '    systime=%.6f  boardtime=%.0f' % (systime[i], boardtime[i])
        print '    systime=%.6f  boardtime=%.0f  bdelta=%.0f' % (systime[i+1], boardtime[i+1], bdelta[i])
        sys.exit(1) 

    err = numpy.abs(sdelta * boardclock_hz - bdelta)
    i = numpy.argmax(err)

    if err[i] > 4.0 * boardclock_hz:
        print 'ERROR: Anomaly in board clock time:'
        print '    systime=%.6f  boardtime=%.0f' % (systime[i], boardtime[i])
        print '    systime=%.6f  boardtime=%.0f  sdelta=%.6f  bdelta=%.0f' % (systime[i+1], boardtime[i+1], sdelta[i], bdelta[i])
        print '    systime=%.6f  boardtime=%.0f  sdelta=%.6f  bdelta=%.0f' % (systime[i+2], boardtime[i+2], sdelta[i+1], bdelta[i+1])
        sys.exit(1)


def checkVoltage(zsystime, znsamples, zsum, zsumsq):
    """Check basic statistics of grid voltage."""

    nsamples = numpy.sum(znsamples.astype(numpy.float64))
    offset   = numpy.sum(zsum.astype(numpy.float64)) / nsamples
    rms      = numpy.sqrt(numpy.sum(zsumsq.astype(numpy.float64)) / nsamples - offset**2)

    print 'Voltage statistics:'
    print '    offset     = % 8.3f lsb' % offset
    print '    AC voltage = % 8.3f lsb rms  ( ~%8.3f Vrms )' % (rms, rms * volt_per_lsb)

    # Look for irregular 50 Hz period
    ix = numpy.where( (znsamples < (0.5 * samplefreq_hz / 50.0)) |
                      (znsamples > (2.0 * samplefreq_hz / 50.0)) )
    for i in ix[0]:
        print '    STRANGE period (%d samples) at %s' % (znsamples[i], timestr(zsystime[i]))

    # Look for out-of-range offset
    ix = numpy.where( (zsum < (offset - 200) * znsamples) |
                      (zsum > (offset + 200) * znsamples) )
    for i in ix[0]:
        print '    STRANGE offset (%.1f lsb) at %s' % (float(zsum[i]) / float(znsamples[i]), timestr(zsystime[i]))

    # Look for out-of-range AC voltage
    ix = numpy.where( (zsumsq < (rms**2 + offset**2 - 10000) * znsamples) |
                      (zsumsq > (rms**2 + offset**2 + 10000) * znsamples) )
    for i in ix[0]:
        print '    STRANGE voltage at %s' % timestr(zsystime[i])

    print


def showPulseStats(ptime):
    """Report clock rate and jitter."""

    pdelta    = ptime[1:] - ptime[:-1]
    avgrate   = numpy.mean(pdelta)
    jitperrms = numpy.std(pdelta) / avgrate
    jitccmax  = numpy.max(numpy.abs(pdelta[1:] - pdelta[:-1])) / avgrate

    print '    Average board clock rate:   %8.3f Hz' % avgrate
    print '    Min/max board clock rate:   %8.3f Hz ... %8.3f Hz' % (numpy.min(pdelta), numpy.max(pdelta))
    print '    PPS period jitter (RMS):    %.3f us rms' % (1.0e6 * jitperrms)
    print '    Peak pulse-to-pulse jitter: %.3f us' % (1.0e6 * jitccmax)


def sanitizePulse(systime, boardtime):
    """Check for spurious or missing pulses."""

    mindelta = boardclock_hz - boardclock_hz // 10
    maxdelta = boardclock_hz + boardclock_hz // 10

    pdelta   = boardtime[1:] - boardtime[:-1]

    # Count spurious pulses and missing pulses.
    # This fails if there are no pulses for more than 268 seconds.
    spurious = 0
    missing  = 0
    for i in xrange(len(pdelta)):
        d = pdelta[i]
        if d < mindelta or d > maxdelta:
            # there is something funny about this pulse
            k = int((d - mindelta) / boardclock_hz)
            if k > 0:
                missing += k
                d -= k * boardclock_hz
                print 'MISSING %d PULSE(S) at %s' % (k, timestr(systime[i]))
            if d < mindelta or d > maxdelta:
                spurious += 1
                print 'SPURIOUS PULSE (dev=%d) at %s' % (d - boardclock_hz, timestr(systime[i]))

    if spurious or missing:
        sys.exit(1)

    # Use median-of-three selection to reject jitter.
    # The first two pulses and last two pulses will be lost.
    ptime_0 = boardtime[2:-2]
    ptime_f = 2 * boardtime[1:-3] - boardtime[:-4]
    ptime_b = 2 * boardtime[3:-1] - boardtime[4:]
    ptime = numpy.median([ ptime_0, ptime_f, ptime_b ], axis=0)

    return systime[2:-2], ptime


def spectrum(ydata, fsample, fname):
    """Compute amplitude spectrum (NOT spectral density) and write to data file."""

    blocklen = (2 * len(ydata)) // (pwelch_navg + 1)
    fftlen   = 256 * (blocklen // 256)

    window   = numpy.hanning(fftlen)

    ydata    = ydata - numpy.mean(ydata)

    spec = numpy.zeros(1 + fftlen // 2)
    for i in xrange(pwelch_navg):
        p = i * blocklen // 2
        q = numpy.fft.rfft(ydata[p:p+fftlen] * window)
        spec += numpy.abs(numpy.square(q))
        del q
    spec[1:] *= 2

    spec = numpy.sqrt(spec / pwelch_navg) / fftlen / numpy.mean(window)

    f = file(fname, 'w')
    for i in xrange(1, len(spec)-1):
        print >>f, i * fsample / float(fftlen), spec[i]
    f.close()


def timeplot(ydata, fsample, toffset, downsample, fname):
    """Dump time domain data to output file.

    Data is optionally downsampled by an integer factor. In this case some
    samples are lost at the start and end of the sequence due to filtering.
    """

    skip = 0
    if downsample != 1:
        # Windowed sinc filter, cutoff=0.5*fsample/downsample, a=2
        skip = 2 * downsample
        lpfilter = scipy_fd.firwin(2 * skip + 1, 1.0/downsample)
        ydata = numpy.convolve(ydata, lpfilter[::-1], mode='same')

    f = file(fname, 'w')
    for i in xrange(skip, len(ydata) - skip, downsample):
        print >>f, toffset + i / float(fsample), ydata[i]
    f.close()


def main():

    if len(sys.argv) != 2:
        print >>sys.stderr, __doc__
        sys.exit(1)

    # Read data file.
    ( (psystime, pboardtime),
      (zsystime, zboardtime, zsample, znsamples, zsum, zsumsq)
        ) = readfile(sys.argv[1])

    # Sanity check on board clock rate.
    checkBoardclock(psystime, pboardtime)
    checkBoardclock(zsystime, zboardtime)

    # Check basic stats on grid voltage.
    checkVoltage(zsystime, znsamples, zsum, zsumsq)

    print 'Board clock vs GPS (before jitter filter):'
    showPulseStats(pboardtime)
    print

    # Sanitize pulse-per-second.
    psystime, pboardtime = sanitizePulse(psystime, pboardtime)

    # Plot board clock frequency over time (in Hz).
    pboardfreq = pboardtime[1:] - pboardtime[:-1]
    timeplot(pboardfreq, 1.0, 0, 5, 'bclockfreq_time.dat')

    # Frequency spectrum of board clock frequency (in Hz RMS).
    spectrum(pboardfreq, 1.0, 'bclockfreq_spectrum.dat')
    del pboardfreq

    # Apply low-pass filter, cutoff at 0.1 Hz.
    # The first 10 pulses and last 10 pulses will be lost.
    pboardtime = numpy.convolve(
        pboardtime,
        scipy_fd.firwin(21, 0.2),
        mode='valid')
    psystime = psystime[10:-10]
    assert len(psystime) == len(pboardtime)

    spectrum(pboardtime[1:] - pboardtime[:-1], 1.0, 'bclockfreq_spectrum_filtered.dat')

    print 'Board clock vs GPS after filter:'
    showPulseStats(pboardtime)
    print

    # Print system time corresponding to first pulse.
    print 'GPS time 0 =', timestr(psystime[0])

    # Analyze rate of system clock.
    sysclockRate = (psystime[-1] - psystime[0]) / float(len(psystime) - 1)
    print 'Average system clock rate = %.8f' % sysclockRate
    print

    # Free some memory.
    del psystime
    del zsystime

    print "Analyzing zero-crossings ..."

    # Improve accuracy of zero crossing times, using the first sample
    # value after zero crossing and a fixed estimate of the signal slope.
    zboardtime = zboardtime - numpy.clip(zsample / float(zcross_sine_slope), 0.0, 1.0) * boardclock_hz / samplefreq_hz
    del zsample

    # Apply 57-period moving average filter to offset and squared sum.
    # (Sampling is coherent over 57 full periods, due to the relation between
    #  the sampling frequency and 50 Hz.)
    tfilter = numpy.ones(57)
    tnsamples = numpy.convolve(znsamples, tfilter, mode='same')
    del znsamples
    toffset   = numpy.convolve(zsum, tfilter, mode='same') / tnsamples
    zvoltage  = numpy.sqrt(numpy.convolve(zsumsq, tfilter, mode='same') / tnsamples - numpy.square(toffset))
    del tfilter
    del tnsamples
    del toffset
    del zsum
    del zsumsq

    # Determine a range in the data where we are sure that there are valid
    # pulses and zero-crossings and no boundary effects from FIR filters.
    p0 = 5
    while pboardtime[p0-1] < zboardtime[200]: p0 += 1
    pn = len(pboardtime) - 5 - p0
    while pboardtime[p0+pn] > zboardtime[-200]: pn -= 1

    # Use piecewise linear interpolation to resample the voltage data
    # on the exact 1 Hz grid of GPS pulses.
    pvoltage = numpy.interp(pboardtime, zboardtime, zvoltage)
    del zvoltage

    # Voltage statistics
    voltage_avg = numpy.mean(pvoltage[p0:p0+pn])
    voltage_stddev = numpy.std(pvoltage[p0:p0+pn])
    print "Voltage:"
    print "    Average AC signal:  %8.3f lsb rms      ( ~%8.3f Vrms )" % (voltage_avg, voltage_avg * volt_per_lsb)
    print "    Voltage variation:  %8.3f lsb rms rms  ( ~%8.3f Vrms rms )" % (voltage_stddev, voltage_stddev * volt_per_lsb)
    print

    # Plot voltage over time (in Vrms)
    timeplot(pvoltage[p0:p0+pn] * volt_per_lsb, 1.0, p0, 1, 'voltage_time.dat')

    # Frequency spectrum of voltage (in Vrms rms).
    spectrum(pvoltage[p0:p0+pn] * volt_per_lsb, 1.0, 'voltage_spectrum.dat')

    # Use piecewise linear interpolation to expand the 1 Hz (GPS)
    # board clock samples into 5 Hz (GPS) board clock samples.
    # Note: pboardtime[i] == kboardtime[5*i]
    kboardtime = numpy.interp(numpy.arange(5*(len(pboardtime)-1)) / 5.0,
                              numpy.arange(len(pboardtime)), pboardtime)
    k0 = 5 * p0
    kn = 5 * pn

    # Use piecewise linear interpolation to sample the sine fringe+phase
    # on the perfect 5 Hz timeline.
    kphase = numpy.interp(kboardtime,
                          zboardtime, numpy.arange(len(zboardtime), dtype=numpy.float64))

    del pboardtime
    del kboardtime
    del zboardtime

    # Analyze grid frequency.
    kfreq       = 5.0 * (kphase[1:] - kphase[:-1])
    freq_avg    = 5.0 * (kphase[k0+kn] - kphase[k0]) / float(kn - 1)
    freq_stddev = numpy.std(kfreq[k0:k0+kn])
    timedev     = (kphase / 50.0 - numpy.arange(len(kphase)) / 5.0)
    timedev     = timedev - numpy.mean(timedev)
    maxtimedev  = numpy.max(numpy.abs(timedev[k0:k0+kn]))
    freq24      = (kphase[24*3600*5::5] - kphase[:-24*3600*5:5]) / (24.0 * 3600.0)
    freq24_min  = numpy.min(freq24)
    freq24_max  = numpy.max(freq24)
    del kphase

    print 'Frequency:'
    print '    Average frequency:    %9.6f Hz' % freq_avg
    print '    Standard deviation:   %9.6f Hz' % freq_stddev
    print '    24-hour average:      %9.6f Hz - %9.6f Hz' % (freq24_min, freq24_max)
    print '    Max absolute time deviation: %8.6f s' % maxtimedev
    print

    # Plot frequency over time (in Hz)
    timeplot(kfreq[k0:k0+kn], 5.0, p0, 5, 'frequency_time.dat')

    # Plot 24-hour average frequency over time (in Hz)
    timeplot(freq24[p0:p0+pn-24*3600], 1.0, p0+12*3600, 1, 'freq24_time.dat')

    # Frequency spectrum of frequency (in Hz rms)
    spectrum(kfreq[k0:k0+kn], 5.0, 'frequency_spectrum.dat')

    # Plot absolute time error over time (in seconds)
    timeplot(timedev[k0:k0+kn], 5.0, p0, 5, 'timedev_time.dat')

    # Find correlation between amplitude and frequency.
    pfreq = numpy.convolve(kfreq,
                           scipy_fd.firwin(21, 1/5.0),
                           mode='same')[::5]
    del kfreq
    coeff = numpy.linalg.lstsq(
        numpy.matrix([ pfreq[p0:p0+pn], numpy.ones(pn) ]).transpose(),
        pvoltage[p0:p0+pn])[0][0]
    (r, pval) = scipy.stats.pearsonr(pfreq[p0:p0+pn], pvoltage[p0:p0+pn])
    print 'Correlation between frequency and voltage:'
    print '    Least-squares correlation:       %g Vrms/Hz' % (coeff * volt_per_lsb)
    print '    Pearson correlation coefficient: % 7.3f' % r
    print '    Confidence (two-sided p-value):  %5.1f %%' % pval
    print

    # Plot amplitude vs frequency (frequency in Hz, amplitude in Vrms).
    f = file('amplitude_vs_frequency.dat', 'w')
    for i in xrange(p0, p0+pn):
        print >>f, pfreq[i], pvoltage[i] * volt_per_lsb
    f.close()


if __name__ == "__main__":
    main()

