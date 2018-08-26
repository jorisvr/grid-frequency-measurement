set terminal png giant size 1024,768
set grid
unset key

set output 'arduinofreq.png'
set title "Arduino clock frequency vs GPS"
set ylabel "Frequency deviation [Hz]" offset 1
set xlabel "Time [hours]"
unset logscale
set xrange [0:192]
set xtics 24
plot 'bclockfreq_time.dat' using ($1/3600.0):($2-16000000) with lines

set output 'arduinofreqspectrum.png'
set title "Arduino clock frequency spectrum"
set ylabel "Frequency variation [Hz rms]" offset 1
set xlabel "Period"
set logscale
set xrange [0.000005:1]
set yrange [0.0002:2]
set xtics ("1 day" 1.1574e-5, "1 hour" 2.7778e-4, "1 min" 0.016667, "15 sec" 0.066667, "2 sec" 0.5)
plot 'bclockfreq_spectrum.dat' with lines

set output 'freq.png'
set title "Momentary power grid frequency"
set ylabel "Frequency [Hz]" offset 1
set xlabel "Time [days]"
unset logscale
set xrange [0:8]
set yrange [49.85:51.0]
set xtics offset 4 ("Fri" 0, "Sat" 1, "Sun" 2, "Mon" 3, "Tue" 4, "Wed" 5, "Thu" 6, "Fri" 7)
set ytics 0.1
plot 'frequency_time.dat' using ($1/3600.0/24.0):2 with lines

set output 'freq24.png'
set title "24-hour average frequency"
set ylabel "Frequency [Hz]" offset 1
set xlabel "Time [days]"
unset logscale
set xrange [0.5:7.5]
set yrange [49.98:50.01]
set xtics offset 4 ("Sat" 1, "Sun" 2, "Mon" 3, "Tue" 4, "Wed" 5, "Thu" 6, "" 7)
set ytics 0.005
plot 'freq24_time.dat' using ($1/3600.0/24.0):2 with lines

set output 'freqspectrum.png'
set title "Power grid frequency spectrum"
set ylabel "Frequency variation [Hz rms]" offset 1
set xlabel "Period"
set logscale
set xrange [5.0e-6:1]
set yrange [*:*]
set xtics offset 0 ("1 day" 1.1574e-5, "1 hour" 2.7778e-4, "15 min" 0.0011111, "1 min" 0.016667, "1 sec" 1.0)
set ytics ("1e-3" 0.001, "1e-4" 0.0001, "1e-5" 1.0e-5)
plot 'frequency_spectrum.dat' with lines

set output 'timedev.png'
set title "Time deviation of a synchronous clock"
set ylabel "Time deviation [s]"
set xlabel "Time [days]"
unset logscale
set xrange [0:8]
set yrange [*:*]
set xtics offset 4 ("Fri" 0, "Sat" 1, "Sun" 2, "Mon" 3, "Tue" 4, "Wed" 5, "Thu" 6, "Fri" 7)
set ytics 5.0
plot 'timedev_time.dat' using ($1/3600.0/24.0):2 with lines

set output 'voltage.png'
set title "AC voltage"
set ylabel "Voltage [Vrms]"
set xlabel "Time [hours]"
unset logscale
set xrange [0:192]
set yrange [*:*]
set xtics offset 0 24
set ytics
plot 'voltage_time.dat' using ($1/3600.0):2 with lines

set output 'voltagespectrum.png'
set title "Voltage spectrum"
set ylabel "Voltage variation [Vrms rms]"
set xlabel "Period"
set logscale
set xrange [5.0e-6:0.5]
set yrange [5.0e-5:0.5]
set xtics ("1 day" 1.1574e-5, "1 hour" 2.7778e-4, "1 min" 0.016667, "2 sec" 0.5)
set ytics (1.0e-4, 1.0e-3, 0.01, 0.1, 0.5)
plot 'voltage_spectrum.dat' with lines

set output 'voltagevsfreq.png'
set title "Voltage vs frequency"
set ylabel "Voltage [Vrms]"
set xlabel "Frequency [Hz]"
unset logscale
set xrange [49.8:50.1]
set yrange [*:*]
set xtics 0.05
set ytics 2
plot 'amplitude_vs_frequency.dat' with points pointsize 0.1

set terminal png small size 380,264
set lmargin 8
set tmargin 2

set output 'freq_small.png'
set title "Momentary power grid frequency"
unset xlabel
set ylabel "Frequency [Hz]" offset 1.6
unset logscale
set xrange [0:8]
set yrange [49.85:50.25]
set xtics offset 3 ("Fri" 0, "Sat" 1, "Sun" 2, "Mon" 3, "Tue" 4, "Wed" 5, "Thu" 6, "Fri" 7)
set ytics 0.1
plot 'frequency_time.dat' using ($1/3600.0/24.0):2 with lines

set output 'freq24_small.png'
set title "24-hour average frequency"
unset xlabel
set ylabel "Frequency [Hz]" offset 2
unset logscale
set xrange [0.5:7.5]
set yrange [49.98:50.01]
set xtics offset 3 ("Sat" 1, "Sun" 2, "Mon" 3, "Tue" 4, "Wed" 5, "Thu" 6, "" 7)
set ytics 0.01
plot 'freq24_time.dat' using ($1/3600.0/24.0):2 with lines

set output 'freqspectrum_small.png'
set title "Power grid frequency spectrum"
set ylabel "Frequency variation [Hz rms]" offset 1
unset xlabel
set logscale
set xrange [5.0e-6:1]
set yrange [*:*]
set xtics offset 0 ("1 day" 1.1574e-5, "1 hour" 2.7778e-4, "1 min" 0.016667, "1 sec" 1.0)
set ytics ("1e-3" 0.001, "1e-4" 0.0001, "1e-5" 1.0e-5)
plot 'frequency_spectrum.dat' with lines

set lmargin 7
set output 'timedev_small.png'
set title "Time deviation of a synchronous clock"
unset xlabel
set ylabel "Time deviation [s]" offset 1
unset logscale
set xrange [0:8]
set yrange [*:*]
set xtics offset 3 ("Fri" 0, "Sat" 1, "Sun" 2, "Mon" 3, "Tue" 4, "Wed" 5, "Thu" 6, "Fri" 7)
set ytics 5.0
plot 'timedev_time.dat' using ($1/3600.0/24.0):2 with lines
set lmargin 8

set output 'voltage_small.png'
set title "AC voltage"
set ylabel "Voltage [Vrms]" offset 1
set xlabel "Time [hours]"
unset logscale
set xrange [0:192]
set yrange [*:*]
set xtics offset 0 24
set ytics
plot 'voltage_time.dat' using ($1/3600.0):2 with lines

set output 'voltagevsfreq_small.png'
set title "Voltage vs frequency"
unset logscale
set ylabel "Voltage [Vrms]" offset 1
set xlabel "Frequency [Hz]"
set xrange [49.9:50.1]
set yrange [220:235]
set xtics offset 0 0.05
set ytics 5
plot 'amplitude_vs_frequency.dat' with points pointsize 0.1

