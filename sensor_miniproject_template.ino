/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packetsnick_new.h"
#include "serial_driver.h"


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */

ISR(INT0_vect)
{
    uint8_t val = (PINE & (1<<0));   // read pin 2 directly
    if(buttonState == STATE_RUNNING && val){
        buttonState = STATE_STOPPED;
        stateChanged = true;
    }
    if(buttonState == STATE_STOPPED && !val){
        buttonState = STATE_RUNNING;
        stateChanged = true;
    }
}

// =============================================================
// Color sensor (TCS3200)
// =============================================================
#define S0 PB3   // pin 4
#define S1 PB4   // pin 5
#define S2 PB5  // pin 6
#define S3 PB6   // pin 7
#define COLOR PE1 // pin 8

static uint32_t measureChannel(uint8_t s2, uint8_t s3)
{
    if(s2)
        PORTB |= (1<<PB5);
    else
        PORTB &= ~(1<<PB5);

    if(s3)
        PORTB |= (1<<PB6);
    else
        PORTB &= ~(1<<PB6);

    uint32_t count = 0;
    uint16_t i;
    for(i=0;i<10000;i++)
    {
        if(PINE & (1<<PE1))
        {
            count++;
            while(PINE & (1<<PE1));
        }
    }
    return count;
}
static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b)
{
    *r = measureChannel(0,0) * 10;
    *g = measureChannel(1,1) * 10;
    *b = measureChannel(0,1) * 10;
}

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */


// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.

static void handleCommand(const TPacket *cmd)
{
    if (cmd->packetType != PACKET_TYPE_COMMAND)
        return;

    switch (cmd->command)
    {
        case COMMAND_ESTOP:
        {
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            TPacket pkt;
            memset(&pkt,0,sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command = RESP_OK;
            strncpy(pkt.data,"Debug: EStop activated",31);
            sendFrame(&pkt);
            sendStatus(STATE_STOPPED);
            break;
        }
        case COMMAND_COLOR:
        {
            uint32_t r,g,b;
            readColorChannels(&r,&g,&b);

            TPacket pkt;
            memset(&pkt,0,sizeof(pkt));

            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command = RESP_COLOR;

            pkt.params[0] = r;
            pkt.params[1] = g;
            pkt.params[2] = b;

            sendFrame(&pkt);

            break;
        }
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================
void setup()
{
    DDRB |= (1<<PB3);    // S0
    DDRB |= (1<<PB4);    // S1
    DDRB |= (1<<PB5);    // S2
    DDRB |= (1<<PB6);    // S3

    DDRE &= ~(1<<PE1);   // COLOR sensor output input

    PORTB |= (1<<PB3);   // S0 HIGH
    PORTB &= ~(1<<PB4);  // S1 LOW
    
    DDRD &= ~(1<<PD0);
    PORTD |= (1<<PD0);

    // interrupt config
    EICRA |= (1<<ISC00);
    EIMSK |= (1<<INT0);

    usartInit(103);

    sei();
}
void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
