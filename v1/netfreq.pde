/*
 * netfreq
 * Arduino sketch for measuring AC frequency of power net.
 *
 * Inputs:
 *   analog input 0   = low-voltage 50 Hz AC signal
 *   digital output 4 = pull low to enable GPS receiver
 *   digital input 8  = GPS pulse per second
 *
 * Serial output:
 *
 *   Pulse captured (5 bytes):
 *     byte 0      = 0x01
 *     bytes 1..4  = (uint32_t) time
 *
 *   Rising zero-crossing captured (15 bytes):
 *     byte 0      = 0x02
 *     bytes 1..4  = (uint32_t) time
 *     bytes 5..6  = (int16_t)  sampled value after zero crossing
 *     bytes 7..8  = (uint16_t) number of samples in past full period
 *     bytes 9..10 = (int16_t)  sum of samples in past full period
 *     bytes 11..14 = (int32_t) sum of squared samples in past full period
 *
 *  Multi-byte values are transmitted with LSB first.
 */

const int16_t zcross_zero_value = 0;    // detect zero-crossing at this sample value
const int16_t zcross_low_value  = -256; // detect negative half below this sample value

volatile uint16_t timecnt = 0;          // bits 16..31 of global time counter

volatile uint8_t pulse_pending = 0;     // 1 if a captured pulse is pending
volatile uint32_t pulse_time = 0;       // time of last captured pulse

volatile uint8_t zcross_pending = 0;    // 1 if a zero-crossing is pending
volatile uint32_t zcross_time = 0;      // time of last zero-crossing
volatile int16_t zcross_value = 0;      // sample value after last zero-crossing
volatile uint16_t zcross_nsample = 0;   // number of samples in past full period
volatile int16_t  zcross_sum = 0;       // sum of samples in past full period
volatile int32_t  zcross_sumsq = 0;     // sum of squared samples in past full period

uint8_t  zc_neghalf = 0;                // 1 if signal is in negative half
uint16_t zc_cur_nsample = 0;            // number of samples in current full period
int16_t zc_cur_sum = 0;                 // sum of samples in current full period
int32_t zc_cur_sumsq = 0;               // sum of squared samples in current full period


// Timer1 input capture interrupt
ISR(TIMER1_CAPT_vect) {

  // get time of captured event
  uint8_t  t0 = ICR1L;
  uint8_t  t1 = ICR1H;
  uint16_t tcnt = timecnt;

  // compensate for pending overflow interrupt
  if (t1 < 0x40 && (TIFR1 & bit(TOV1)))
    tcnt++;
  
  // store time of captured event
  pulse_time = ((uint32_t)tcnt << 16) | ((uint32_t)t1 << 8) | t0;

  // set flag to indicate pulse captured
  pulse_pending = 1;
}


// Timer1 overflow interrupt
ISR(TIMER1_OVF_vect) {
  // increment time counter
  timecnt++;
}


// ADC conversion done interrupt
ISR(ADC_vect) {

  // fetch current time stamp
  uint8_t  t0 = TCNT1L;
  uint8_t  t1 = TCNT1H;
  uint16_t tcnt = timecnt;

  // compensate for pending overflow interrupt
  if (t1 < 0x40 && (TIFR1 & bit(TOV1)))
    tcnt++;

  // fetch sampled value
  uint8_t v0 = ADCL;
  uint8_t v1 = ADCH;
  int16_t v  = (((uint16_t)v1 << 8) | v0) - 512;

  if (zc_neghalf && v >= zcross_zero_value) {
    // this is a zero-cross event
    zcross_time    = ((uint32_t)tcnt << 16) | ((uint32_t)t1 << 8) | t0;
    zcross_value   = v;           // store first value after zero-cross
    zcross_nsample = zc_cur_nsample;
    zcross_sum     = zc_cur_sum;
    zcross_sumsq   = zc_cur_sumsq;
    zcross_pending = 1;           // set flag to indicate zero-cross was detected
    zc_neghalf  = 0;              // now in positive half
    zc_cur_nsample = 0;           // reset statistics
    zc_cur_sum = 0;
    zc_cur_sumsq = 0;
  } else if (v < zcross_low_value) {
    // the signal is in the negative half
    zc_neghalf = 1;
  }

  // keep track of average and rms sampled values
  zc_cur_nsample++;
  zc_cur_sum += v;
  zc_cur_sumsq += (uint32_t)v * v;

  // clear interrupt flag of timer0 to prepare for next ADC trigger
  TIFR0 = bit(OCF0A);
}


// TODO : maybe I need to enable TIMER0_COMPA_vect) and implement an empty handler ???


void setup() {

  // serial port to 115200 bps
  Serial.begin(115200);

  // pull pin 4 low to enable the GPS receiver
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  // disable analog comparator, use ICFP1 for input capturing
  ACSR  = bit(ACD);
  
  // setup timer0 for ADC triggering
  TIMSK0 = 0;                     // disable Arduino timer interrupt
  TCCR0A = bit(WGM01);            // select clear-timer-on-compare mode
  TCCR0B = bit(CS01)|bit(CS00);   // select CK/64 prescaling factor (4 us)
  OCR0A  = 56;                    // set period to 57 * 4 us = 228 us (4.386 kHz)

  // setup timer1 for time keeping and input capturing
  TCCR1A = 0;                     // normal mode, count to 0xffff
  TCCR1B = bit(ICNC1)|bit(ICES1)|bit(CS10); // no prescaling, input capture on rising edge
  TIMSK1 = bit(ICIE1)|bit(TOIE1); // enable interrupt on input capture and on overflow

  // setup ADC
  ADMUX  = bit(REFS1)|bit(REFS0); // select internal 1.1V reference, select input channel 0
  ADCSRB = bit(ADTS1)|bit(ADTS0); // trigger ADC on timer0 output compare match
  ADCSRA = bit(ADEN) |            // enable ADC
           bit(ADATE) |           // enable auto-triggering
           bit(ADIE) |            // enable interrupt on conversion done
           bit(ADPS2)|bit(ADPS1)|bit(ADPS0);  // select prescaling factor 128 (125 kHz clk)
  DIDR0  = bit(ADC0D);            // disable digital input buffer for analog input pin 0

}


// write a 16-bit integer to the serial port (LSB first)
static void serial_write_int16(uint16_t v)
{
    Serial.write(v & 0xff);
    Serial.write((v >> 8) & 0xff);
}


// write a 32-bit integer to the serial port (LSB first)
static void serial_write_int32(uint32_t v)
{
    Serial.write(v & 0xff);
    Serial.write((v >> 8) & 0xff);
    Serial.write((v >> 16) & 0xff);
    Serial.write((v >> 24) & 0xff);
}


void loop() {
  // write pending events to serial port
  while (1) {
    if (pulse_pending) {
      Serial.write(0x01);
      serial_write_int32(pulse_time);
      pulse_pending = 0;
    }
    if (zcross_pending) {
      Serial.write(0x02);
      serial_write_int32(zcross_time);
      serial_write_int16(zcross_value);
      serial_write_int16(zcross_nsample);
      serial_write_int16(zcross_sum);
      serial_write_int32(zcross_sumsq);
      zcross_pending = 0;
    }
  }
}

