/*
 *  netfreq_stm32
 *
 *  Measure AC mains frequency.
 *  Arduino sketch for STM32 Blue Pill board.
 *
 *  Connections:
 *    PB9 (input)  = Output signal from opto-coupler.
 *                   Low when opto-coupler in on state (during peak of positive half-period).
 *                   High when opto-coupler in off state.
 *
 *    PB6 (input)  = PPS signal from GPS receiver.
 *                   Rising edge on start of every second.
 *
 *    PA10 (input) = USART1_RX = Serial port signal from GPS receiver to STM32.
 *                   9600 bps NMEA stream.
 *
 *    PA9 (output) = USART1_TX = Serial port signal from STM32 to GPS receiver.
 *
 *  Serial output via USB (115200 bps):
 *    
 *    Heartbeat packet, 6 bytes, sent once per second when PPS not active:
 *      byte 0    = 0x10
 *      byte 1    = checksum
 *      byte 2..5 = (uint32_t) timestamp
 *    
 *    Mains AC packet, 10 bytes, sent once per AC cycle:
 *      byte 0    = 0x11
 *      byte 1    = checksum
 *      byte 2..5 = (uint32_t) timestamp of opto-coupler switch-on
 *      byte 6..9 = (uint32_t) timestamp of opto-coupler switch-off
 *
 *    PPS timestamp, 6 bytes, sent on each PPS pulse:
 *      byte 0    = 0x12
 *      byte 1    = checksum
 *      byte 2..5 = (uint32_t) timestamp of PPS pulse
 *
 *    NMEA data, 3..62 bytes:
 *      byte 0    = 0x13
 *      byte 1    = data length, min 1 max 60
 *      byte 2..  = up to 60 bytes NMEA character data
 *
 *    Multi-byte values are transmitted with LSB first.
 *    All timestamps are in microseconds.
 */

// Send heartbeat packet every second according to 1 MHz counter.
const uint32_t heartbeat_interval = 1000000;
const uint32_t pps_interval = 1000000;

// Don't do time consuming stuff within the last 1 ms before the next heartbeat/PPS.
const uint32_t heartbeat_prepare_time = 1000;

// Wait 0.2 ms after capturing an edge before restarting the capture function.
const uint32_t acpulse_dead_time = 200;

timer_dev* const timer_main = &timer4;
const uint8_t    timer_channel_acpulse = 4;  // PB9 = TIM4_CH4
const uint8_t    timer_channel_pps = 1;      // PB6 = TIM4_CH1

volatile uint16_t timestamp_msb;          // bits 16..31 of global 1 MHz time counter

volatile uint32_t ac_pulse_start_ts;      // timestamp of captured start of AC pulse
volatile uint32_t ac_pulse_end_ts;        // timestamp of captured end of AC pulse
volatile bool     ac_pulse_got_start;     // true when start of pulse captured
volatile bool     ac_pulse_got_end;       // true when end of pulse captured

volatile uint32_t pps_pulse_ts;           // timestamp of captured PPS pulse
volatile bool     pps_pulse_received;     // true when PPS pulse captured

uint32_t next_heartbeat_ts;               // timestamp when to send next heartbeat packet
bool     pps_active;                      // true when receiving PPS pulse every second
uint8_t  led_counter;                     // blink LED once every 50 AC cycles

HardwareSerial* const gps_serial = &Serial1; // USART1 (serial port to GPS receiver)
uint8_t gps_data[64];                     // NMEA data buffer
uint8_t gps_data_len;                     // NMEA data length
const uint8_t gps_data_offset = 2;        // start of NMEA data within gps_data[] array
const uint8_t gps_data_max_len = 60;


// Read current value of 1 MHz timestamp counter.
uint32_t get_timestamp()
{
   uint16_t ts_lsb, ts_msb;

   while (true) {

        // Read timer overflow counter and timer counter.
        ts_msb = timestamp_msb;
        ts_lsb = timer_get_count(timer_main);

        // Check pending overflow interrupt.
        bool pending_ovf = ((timer_main->regs.gen->SR & TIMER_SR_UIF) != 0);

        // Re-read overflow counter and check that it has not changed (and no change pending).
        if ((!pending_ovf) && ts_msb == timestamp_msb) {
            break;
        }
    }

    return (((uint32_t)ts_msb) << 16) | ts_lsb;
}


// Extend a 16-bit event timestamp to a 32-bit timestamp based on the
// current value of the overflow counter. This function may only be
// called by input capture interrupt handlers.
uint32_t get_extended_event_timestamp(uint16_t ts_lsb)
{
    // Get current timer overflow counter.
    uint16_t ts_msb = timestamp_msb;

    // Check for pending timer overflow interrupt.
    // Note that capture interrupt and overflow interrupt are dispatched through
    // the same IRQ, so the overflow handler will never interrupt the capture handler.
    bool pending_ovf = ((timer_main->regs.gen->SR & TIMER_SR_UIF) != 0);

    if (pending_ovf) {
        // Pending overlow interrupt; overflow counter not yet updated.
        // The timer wrapped either shortly before or shortly after the captured event.
        // If the timer wrapped before the captured event, we need to adjust
        // the value of the overflow counter. Look at the capture timestamp to
        // guess whether the wrap occurred before or after event.
        if (ts_lsb < 0x8000) {
            ts_msb++;
        }
    } else {
        // No pending overflow; overflow counter is accurate.
        // Note that capture handler and overflow handler are dispatched through
        // the same IRQ and capture handler runs BEFORE overflow handler.
        // We can thus be sure that the overflow counter will not incorrectly
        // reflect a timer wrap that occurred after the captured event.
    }

    uint32_t event_ts = (((uint32_t)ts_msb) << 16) | ts_lsb;
    return event_ts;
}


// AC pulse input capture interrupt.
void handleAcCapture() {

    // Get capture timestamp.
    uint16_t ts_lsb = timer_get_compare(timer_main, timer_channel_acpulse);

    // Extend timestamp to 32 bits.
    uint32_t event_ts = get_extended_event_timestamp(ts_lsb);

    if (ac_pulse_got_start) {
        // Captured end of AC pulse.
        ac_pulse_end_ts = event_ts;
        ac_pulse_got_end = true;
    } else {
        // Captured start of AC pulse.
        ac_pulse_start_ts = event_ts;
        ac_pulse_got_start = true;
    }

    // Stop capturing AC pulse events.
    timer_cc_disable(timer_main, timer_channel_acpulse);
}


// PPS input capture interrupt.
void handlePpsCapture() {

    // Get capture timestamp.
    uint16_t ts_lsb = timer_get_compare(timer_main, timer_channel_pps);

    // Extend timestamp to 32 bits.
    uint32_t event_ts = get_extended_event_timestamp(ts_lsb);

    // Store received PPS timestamp (unless previous timestamp still pending).
    if (!pps_pulse_received) {
        pps_pulse_ts = event_ts;
        pps_pulse_received = true;
    }
}


// Timer4 overflow interrupt
void handleTimerOverflow() {

    // Increment timer MSB counter.
    timestamp_msb++;

}


// Write a 32-bit integer to a byte buffer (LSB first).
void put_uint32(uint8_t *buf, uint32_t v)
{
    buf[0] = v & 0xff;
    buf[1] = (v >> 8) & 0xff;
    buf[2] = (v >> 16) & 0xff;
    buf[3] = (v >> 24) & 0xff;
}


// Calculate checksum.
static inline uint8_t checksum(const uint8_t *pkt, size_t n)
{
    uint8_t csum = 0;
    for (size_t p = 0; p < n; p++) {
        csum += pkt[p];
    }
    return csum ^ 0xff;
}

// Send heartbeat packet.
void serial_send_heartbeat(uint32_t ts)
{
    uint8_t pkt[6];
    pkt[0] = 0x10;
    pkt[1] = 0x00;
    put_uint32(pkt + 2, ts);
    pkt[1] = checksum(pkt, 6);
    Serial.write(pkt, 6);
}


// Send mains AC packet.
void serial_send_acpulse()
{
    uint8_t pkt[10];
    uint8_t csum;
    pkt[0] = 0x11;
    pkt[1] = 0x00;
    put_uint32(pkt + 2, ac_pulse_start_ts);
    put_uint32(pkt + 6, ac_pulse_end_ts);
    pkt[1] = checksum(pkt, 10);
    Serial.write(pkt, 10);
}


// Send PPS packet.
void serial_send_pps(uint32_t ts)
{
    uint8_t pkt[6];
    pkt[0] = 0x12;
    pkt[1] = 0x00;
    put_uint32(pkt + 2, ts);
    pkt[1] = checksum(pkt, 6);
    Serial.write(pkt, 6);
}


void setup() {

    // Initialize state.
    timestamp_msb = 0;
    ac_pulse_got_start = false;
    ac_pulse_got_end = false;
    pps_pulse_received = false;
    next_heartbeat_ts = heartbeat_interval;
    pps_active = false;
    led_counter = 0;
    gps_data_len = 0;

    // Serial port (USB) to 115200 bps.
    Serial.begin(115200);

    // USART1 serial port (GPS) to 9600 bps.
    gps_serial->begin(9600);

    // Enable control of on-board LED.
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, 1); // turn LED off

    // Setup timer 4 for 1 MHz counting with full 16-bit range.
    timer_set_prescaler(timer_main, 71);      // 72 MHz input clock prescaled by factor 72
    timer_set_reload(timer_main, 0xffff);     // overflow at 65535

    // Set interrupt on timer overflow.
    timer_attach_interrupt(timer_main, TIMER_UPDATE_INTERRUPT, handleTimerOverflow);

    // Set pin PB9 to read input from optocoupler.
    pinMode(PB9, INPUT);

    // Set pin PB6 to read input from GPS PPS signal.
    pinMode(PB6, INPUT);

    // Setup input capture on pin PB9 (TIM4_CH4).
    // Set interrupt on input capture.
    timer_attach_interrupt(timer_main, timer_channel_acpulse, handleAcCapture);

    // Set input mode with input filter to 8-sample window (0x30 -> IC4F = 0011 -> N=8).
    timer_oc_set_mode(timer_main, timer_channel_acpulse, (timer_oc_mode)0, TIMER_IC_INPUT_DEFAULT | 0x30);

    // Set to capture on falling edge and enable capture.
    timer_cc_set_pol(timer_main, timer_channel_acpulse, 1);
    timer_cc_enable(timer_main, timer_channel_acpulse);

    // Setup input capture on pin PB6 (TIM4_CH1).
    // Set interrupt on input capture.
    timer_attach_interrupt(timer_main, timer_channel_pps, handlePpsCapture);

    // Set input filter to 8-sample window (0x30 -> IC1F = 0011 -> N=8).
    timer_oc_set_mode(timer_main, timer_channel_pps, (timer_oc_mode)0, TIMER_IC_INPUT_DEFAULT | 0x30);

    // Set to capture on rising edge and enable capture.
    timer_cc_set_pol(timer_main, timer_channel_pps, 0);
    timer_cc_enable(timer_main, timer_channel_pps);
}


void loop() {

    uint32_t ts_now = get_timestamp();

    if (pps_pulse_received) {
        // Received PPS pulse.

        // Send PPS packet to PC.
        serial_send_pps(pps_pulse_ts);

        // Set to expect next PPS pulse in one second.
        next_heartbeat_ts = pps_pulse_ts + pps_interval + heartbeat_prepare_time / 2;
        pps_active = true;
        pps_pulse_received = false;
    }

    if ((uint32_t)(ts_now - next_heartbeat_ts) < 0x10000000) {
        // Reached time to send next heartbeat packet.
        // Send heartbeat packet, unless PPS pulse is active.
        if (!pps_active) {
            serial_send_heartbeat(ts_now);
        }
        pps_active = false;
        next_heartbeat_ts += heartbeat_interval;
    }

    if (ac_pulse_got_start && !ac_pulse_got_end) {
        // Captured start of AC pulse (falling edge).

        // Wait until some time has passed after the captured edge (to avoid transition glitches).
        if ((uint32_t)(ts_now - ac_pulse_start_ts) > acpulse_dead_time) {
          
            // Set timer to capture on rising edge.
            timer_cc_set_pol(timer_main, timer_channel_acpulse, 0);
            timer_cc_enable(timer_main, timer_channel_acpulse);

            // Check if end of pulse already occurred.
            if (digitalRead(PB9)) {

                // Signal already high. Stop capturing.
                timer_cc_disable(timer_main, timer_channel_acpulse);

                // If the rising edge was not captured, flag it as captured with zero pulse duration.
                if (!ac_pulse_got_end) {
                    ac_pulse_end_ts = ac_pulse_start_ts;
                    ac_pulse_got_end = true;
                }
            }
        }
    }

    if ((uint32_t)(next_heartbeat_ts - ts_now) > heartbeat_prepare_time) {
        // Postpone this when the next heartbeat is about to occur.

        if (ac_pulse_got_start && ac_pulse_got_end) {
            // Captured end of AC pulse (rising edge).

            // Wait until some time has passed after the captured edge.
            if ((uint32_t)(ts_now - ac_pulse_end_ts) > acpulse_dead_time) {

                // Send event packet.
                serial_send_acpulse();

                ac_pulse_got_start = false;
                ac_pulse_got_end = false;

                // Set timer to capture on falling edge for next cycle.
                timer_cc_set_pol(timer_main, timer_channel_acpulse, 1);
                timer_cc_enable(timer_main, timer_channel_acpulse);

                // Blink LED once every 50 AC cycles.
                led_counter++;
                if (led_counter == 30) {
                    // Turn LED on.
                    digitalWrite(PC13, 0);
                } else if (led_counter >= 50) {
                    // Turn LED off.
                    digitalWrite(PC13, 1);
                    led_counter = 0;
                }
            }

        } else if (gps_serial->available()) {
            // Received NMEA byte from GPS receiver.

            // Store received byte in NMEA buffer.
            uint8_t c = gps_serial->read();
            gps_data[gps_data_offset + gps_data_len] = c;
            gps_data_len++;

            // When buffer full or EOL received, send NMEA buffer to PC.
            if (gps_data_len >= gps_data_max_len || c == '\n') {
                gps_data[0] = 0x13;
                gps_data[1] = gps_data_len;
                Serial.write(gps_data, 2 + gps_data_len);
                gps_data_len = 0;
            }
        }
    }

}

