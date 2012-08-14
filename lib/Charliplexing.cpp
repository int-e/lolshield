/*
  Charliplexing.cpp - Using timer2 with 1ms resolution
  
  Alex Wenger <a.wenger@gmx.de> http://arduinobuch.wordpress.com/
  Matt Mets <mahto@cibomahto.com> http://cibomahto.com/
  
  Timer init code from MsTimer2 - Javier Valencia <javiervalencia80@gmail.com>
  Misc functions from Benjamin Sonnatg <benjamin@sonntag.fr>
  
  History:
    2009-12-30 - V0.0 wrote the first version at 26C3/Berlin
    2010-01-01 - V0.1 adding misc utility functions 
      (Clear, Vertical,  Horizontal) comment are Doxygen complaints now
    2010-05-27 - V0.2 add double-buffer mode
    2010-08-18 - V0.9 Merge brightness and grayscale

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <inttypes.h>
#include <math.h>
#include <avr/interrupt.h>
#include "Charliplexing.h"

volatile unsigned int LedSign::tcnt2;


struct videoPage {
    uint8_t pixels[SHADES-1][48];
}; 

/* -----------------------------------------------------------------  */
/** Table for the LED multiplexing cycles
 * Each frame is made of 24 bytes (for the 24 display cycles)
 * There are SHADES frames per buffer in grayscale mode (one for each brigtness)
 * and twice that many to support double-buffered grayscale.
 */
videoPage leds[2];

/// Determines whether the display is in single or double buffer mode
uint8_t displayMode = SINGLE_BUFFER;

/// Flag indicating that the display page should be flipped as soon as the
/// current frame is displayed
volatile boolean videoFlipPage = false;

/// Pointer to the buffer that is currently being displayed
videoPage* displayBuffer;

/// Pointer to the buffer that should currently be drawn to
videoPage* workBuffer;

/// Flag indicating that the timer buffer should be flipped as soon as the
/// current frame is displayed
volatile boolean videoFlipTimer = false;


// Timer counts to display each page for, plus off time
typedef struct timerInfo {
    uint8_t counts[SHADES];
    uint8_t prescaler[SHADES];
};

// Double buffer the timing information, of course.
timerInfo* frontTimer;
timerInfo* backTimer;

timerInfo* tempTimer;

timerInfo timer[2];

// Record a slow and fast prescaler for later use
typedef struct prescalerInfo {
    uint8_t relativeSpeed;
    uint8_t TCCR2;
};

// TODO: Generate these based on processor type and clock speed
prescalerInfo slowPrescaler = {1, 0x03};
//prescalerInfo fastPrescaler = {32, 0x01};
prescalerInfo fastPrescaler = {4, 0x02};

static bool initialized = false;

/// Uncomment to set analog pin 5 high during interrupts, so that an
/// oscilloscope can be used to measure the processor time taken by it
#define MEASURE_ISR_TIME
#ifdef MEASURE_ISR_TIME
uint8_t statusPIN = 19;
#endif

/* -----------------------------------------------------------------  */
// for each pixel, give byte offset and mask for corresponding bit
static const uint8_t pixelAddr[][16][2] PROGMEM = {
    { 31, 0x20, 33, 0x20, 35, 0x20, 37, 0x20, 39, 0x20, 41, 0x20, 43, 0x20,
      45, 0x20, 29, 0x20, 22, 0x10, 27, 0x20, 46, 0x08, 25, 0x20, 22, 0x04, },
    {  7, 0x10,  9, 0x10, 11, 0x10, 13, 0x10, 15, 0x10, 17, 0x10, 19, 0x10,
      23, 0x10,  5, 0x10, 20, 0x10,  3, 0x10, 44, 0x08,  1, 0x10, 20, 0x04, },
    { 31, 0x08, 33, 0x08, 35, 0x08, 37, 0x08, 39, 0x08, 41, 0x08, 45, 0x08,
      47, 0x08, 29, 0x08, 18, 0x10, 27, 0x08, 42, 0x08, 25, 0x08, 18, 0x04, },
    {  7, 0x04,  9, 0x04, 11, 0x04, 13, 0x04, 15, 0x04, 19, 0x04, 21, 0x04,
      23, 0x04,  5, 0x04, 16, 0x10,  3, 0x04, 40, 0x08,  1, 0x04, 16, 0x04, },
    { 31, 0x02, 33, 0x02, 35, 0x02, 37, 0x02, 41, 0x02, 43, 0x02, 45, 0x02,
      47, 0x02, 29, 0x02, 14, 0x10, 27, 0x02, 38, 0x08, 25, 0x02, 14, 0x04, },
    {  7, 0x01,  9, 0x01, 11, 0x01, 15, 0x01, 17, 0x01, 19, 0x01, 21, 0x01,
      23, 0x01,  5, 0x01, 12, 0x10,  3, 0x01, 36, 0x08,  1, 0x01, 12, 0x04, },
    { 30, 0x80, 32, 0x80, 36, 0x80, 38, 0x80, 40, 0x80, 42, 0x80, 44, 0x80,
      46, 0x80, 28, 0x80, 10, 0x10, 26, 0x80, 34, 0x08, 24, 0x80, 10, 0x04, },
    {  6, 0x40, 10, 0x40, 12, 0x40, 14, 0x40, 16, 0x40, 18, 0x40, 20, 0x40,
      22, 0x40,  4, 0x40,  8, 0x10,  2, 0x40, 32, 0x08,  0, 0x40,  8, 0x04, },
    { 32, 0x20, 34, 0x20, 36, 0x20, 38, 0x20, 40, 0x20, 42, 0x20, 44, 0x20,
      46, 0x20, 28, 0x20,  6, 0x10, 26, 0x20, 30, 0x08, 24, 0x20,  6, 0x04, },
};
// resulting display phases
//
// frames 0-5                          frames 6-11
// . . . . . . . . . . . . . .         . . . . . . . . . 5 . . . 5
// 3 4 5 . . . . . 2 . 1 . 0 .         . . . 0 1 2 3 5 . 4 . . . 4
// . . . . . . . . . . . . . .         . . . . . . . . . 3 . . . 3
// 3 4 5 . . . . . 2 . 1 . 0 .         . . . 0 1 3 4 5 . 2 . . . 2
// . . . . . . . . . . . . . .         . . . . . . . . . 1 . . . 1
// 3 4 5 . . . . . 2 . 1 . 0 .         . . . 1 2 3 4 5 . 0 . . . 0
// . . . . . . . . . 5 . . . 5         . . . . . . . . . . . . . .
// 3 5 . . . . . . 2 4 1 . 0 4         . . 0 1 2 3 4 5 . . . . . .
// . . . . . . . . . 3 . . . 3         . . . . . . . . . . . . . .
//
// frames 12-17                         frames 18-23
// 3 4 5 . . . . . 2 . 1 . 0 .          . . . 0 1 2 3 4 . . . 5 . .
// . . . . . . . . . . . . . .          . . . . . . . . . . . 4 . .
// 3 4 5 . . . . . 2 . 1 . 0 .          . . . 0 1 2 4 5 . . . 3 . .
// . . . . . . . . . . . . . .          . . . . . . . . . . . 2 . .
// 3 4 5 . . . . . 2 . 1 . 0 .          . . . 0 2 3 4 5 . . . 1 . .
// . . . . . . . . . . . . . .          . . . . . . . . . . . 0 . .
// 3 4 . . . . . . 2 . 1 5 0 .          . . 0 1 2 3 4 5 . . . . . .
// . . . . . . . . . . . 4 . .          . . . . . . . . . . . . . .
// 4 5 . . . . . . 2 . 1 3 0 .          . . 0 1 2 3 4 5 . . . . . .

/* -----------------------------------------------------------------  */
/** Constructor : Initialize the interrupt code. 
 * should be called in setup();
 */
void LedSign::Init(uint8_t mode)
{
#ifdef MEASURE_ISR_TIME
    pinMode(statusPIN, OUTPUT);
    digitalWrite(statusPIN, LOW);
#endif

	float prescaler = 0.0;
	
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)
	TIMSK2 &= ~(1<<TOIE2);
	TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
	TCCR2B &= ~(1<<WGM22);
	ASSR &= ~(1<<AS2);
	TIMSK2 &= ~(1<<OCIE2A);
	
	if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL)) {	// prescaler set to 64
		TCCR2B |= ((1<<CS21) | (1<<CS20));
		TCCR2B &= ~(1<<CS22);
		prescaler = 32.0;
	} else if (F_CPU < 1000000UL) {	// prescaler set to 8
		TCCR2B |= (1<<CS21);
		TCCR2B &= ~((1<<CS22) | (1<<CS20));
		prescaler = 8.0;
	} else { // F_CPU > 16Mhz, prescaler set to 128
		TCCR2B |= (1<<CS22);
		TCCR2B &= ~((1<<CS21) | (1<<CS20));
		prescaler = 64.0;
	}
#elif defined (__AVR_ATmega8__)
	TIMSK &= ~(1<<TOIE2);
	TCCR2 &= ~((1<<WGM21) | (1<<WGM20));
	TIMSK &= ~(1<<OCIE2);
	ASSR &= ~(1<<AS2);
	
	if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL)) {	// prescaler set to 64
		TCCR2 |= (1<<CS22);
		TCCR2 &= ~((1<<CS21) | (1<<CS20));
		prescaler = 64.0;
	} else if (F_CPU < 1000000UL) {	// prescaler set to 8
		TCCR2 |= (1<<CS21);
		TCCR2 &= ~((1<<CS22) | (1<<CS20));
		prescaler = 8.0;
	} else { // F_CPU > 16Mhz, prescaler set to 128
		TCCR2 |= ((1<<CS22) && (1<<CS20));
		TCCR2 &= ~(1<<CS21);
		prescaler = 128.0;
	}
#elif defined (__AVR_ATmega128__)
	TIMSK &= ~(1<<TOIE2);
	TCCR2 &= ~((1<<WGM21) | (1<<WGM20));
	TIMSK &= ~(1<<OCIE2);
	
	if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL)) {	// prescaler set to 64
		TCCR2 |= ((1<<CS21) | (1<<CS20));
		TCCR2 &= ~(1<<CS22);
		prescaler = 64.0;
	} else if (F_CPU < 1000000UL) {	// prescaler set to 8
		TCCR2 |= (1<<CS21);
		TCCR2 &= ~((1<<CS22) | (1<<CS20));
		prescaler = 8.0;
	} else { // F_CPU > 16Mhz, prescaler set to 256
		TCCR2 |= (1<<CS22);
		TCCR2 &= ~((1<<CS21) | (1<<CS20));
		prescaler = 256.0;
	}
#endif

	tcnt2 = 256 - (int)((float)F_CPU * 0.0005 / prescaler);

    // Record whether we are in single or double buffer mode
    displayMode = mode;
    videoFlipPage = false;

    // Point the display buffer to the first physical buffer
    displayBuffer = &leds[0];

    // If we are in single buffered mode, point the work buffer
    // at the same physical buffer as the display buffer.  Otherwise,
    // point it at the second physical buffer.
    if( displayMode & DOUBLE_BUFFER ) {
        workBuffer = &leds[1];
    }
    else {
        workBuffer = displayBuffer;
    }

    // Set up the timer buffering
    frontTimer = &timer[0];
    backTimer = &timer[1];

    videoFlipTimer = false;
    SetBrightness(127);
	
    // Clear the buffer and display it
    LedSign::Clear(0);
    LedSign::Flip(false);

    // Then start the display
	TCNT2 = tcnt2;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__)
	TIMSK2 |= (1<<TOIE2);
#elif defined (__AVR_ATmega128__) || defined (__AVR_ATmega8__)
	TIMSK |= (1<<TOIE2);
#endif

    // If we are in double-buffer mode, wait until the display flips before we
    // return
    if (displayMode & DOUBLE_BUFFER)
    {
        while (videoFlipPage) {
            delay(1);
        }
    }

    initialized = true;
}


/* -----------------------------------------------------------------  */
/** Signal that the front and back buffers should be flipped
 * @param blocking if true : wait for flip before returning, if false :
 *                 return immediately.
 */
void LedSign::Flip(bool blocking)
{
    if (displayMode & DOUBLE_BUFFER)
    {
        // Just set the flip flag, the buffer will flip between redraws
        videoFlipPage = true;

        // If we are blocking, sit here until the page flips.
        while (blocking && videoFlipPage) {
            delay(1);
        }
    }
}


/* -----------------------------------------------------------------  */
/** Clear the screen completely
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Clear(int set) {
    for(int x=0;x<14;x++)  
        for(int y=0;y<9;y++) 
            Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Clear an horizontal line completely
 * @param y is the y coordinate of the line to clear/light [0-8]
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Horizontal(int y, int set) {
    for(int x=0;x<14;x++)  
        Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Clear a vertical line completely
 * @param x is the x coordinate of the line to clear/light [0-13]
 * @param set if 1 : make all led ON, if not set or 0 : make all led OFF
 */
void LedSign::Vertical(int x, int set) {
    for(int y=0;y<9;y++)  
        Set(x,y,set);
}


/* -----------------------------------------------------------------  */
/** Set : switch on and off the leds. All the position #for char in frameString:
 * calculations are done here, so we don't need to do in the
 * interrupt code
 */
void LedSign::Set(uint8_t x, uint8_t y, uint8_t c)
{
    uint8_t bufferNum = pgm_read_byte_near(&pixelAddr[y][x][0]);
    uint8_t work      = pgm_read_byte_near(&pixelAddr[y][x][1]);

    // If we aren't in grayscale mode, just map any pin brightness to max
    if (c > 0 && !(displayMode & GRAYSCALE)) {
        c = SHADES-1;
    }

    for (int i = 0; i < SHADES-1; i++) {
        if( c > i ) {
            workBuffer->pixels[i][bufferNum] |= work;   // ON
        }
        else {
            workBuffer->pixels[i][bufferNum] &= ~work;   // OFF
        }
    }
}


/* Set the overall brightness of the screen
 * @param brightness LED brightness, from 0 (off) to 127 (full on)
 */

void LedSign::SetBrightness(uint8_t brightness)
{
    // An exponential fit seems to approximate a (perceived) linear scale
    float brightnessPercent = ((float)brightness / 127)*((float)brightness / 127);
    uint8_t difference = 0;

    /*   ---- This needs review! Please review. -- thilo  */
    // set up page counts
    // TODO: make SHADES a function parameter. This would require some refactoring.
    int start = 15;
    int max = 255;
    float scale = 1.5;
    float delta = pow( max - start , 1.0 / scale) / (SHADES - 1);
    uint8_t pageCounts[SHADES]; 

    pageCounts[0] = max - start;
    for (uint8_t i=1; i<SHADES; i++) {
        pageCounts[i] = max - ( pow( i * delta, scale ) + start );
    }
    Serial.end();

    if (! initialized) {
       // set front timer defaults
        for (int i = 0; i < SHADES; i++) {
            frontTimer->counts[i] = pageCounts[i];
            // TODO: Generate this dynamically
            frontTimer->prescaler[i] = slowPrescaler.TCCR2;
        }
    }

    // Wait until the previous brightness request goes through
    while( videoFlipTimer ) {
        delay(1);
    }

    // Compute on time for each of the pages
    // Use the fast timer; slow timer is only useful for < 3 shades.
    for (uint8_t i = 0; i < SHADES - 1; i++) {
        uint8_t interval = 255 - pageCounts[i];

        backTimer->counts[i] = 255 -    brightnessPercent 
                                      * interval 
                                      * fastPrescaler.relativeSpeed;
        backTimer->prescaler[i] = fastPrescaler.TCCR2;
        difference += backTimer->counts[i] - pageCounts[i];
    }

    // Compute off time
    backTimer->counts[SHADES - 1] = 255 - difference;
    backTimer->prescaler[SHADES - 1] = slowPrescaler.TCCR2;

    /*   ---- End of "This needs review! Please review." -- thilo  */

    // Have the ISR update the timer registers next run
    videoFlipTimer = true;
}


/* -----------------------------------------------------------------  */
/** The Interrupt code goes here !  
 */
ISR(TIMER2_OVF_vect)
{
    DDRD  = 0x0;
    DDRB  = 0x0;
#ifdef MEASURE_ISR_TIME
    digitalWrite(statusPIN, HIGH);
#endif

    // For each cycle, we have potential SHADES pages to display.
    // Once every page has been displayed, then we move on to the next
    // cycle. The last page is blank, for dimming.

    // Each page consists of two half-pages that are displayed in the same way.

    // 12 cycles of matrix (one half-page)
    static uint8_t cycle = 0;

    // 2*SHADES half-pages to display
    static uint8_t page = 0;

    TCCR2B = frontTimer->prescaler[page/2];
    TCNT2 = frontTimer->counts[page/2];

    // Cast 'pixels' buffer to half-page addressable layout.
    uint8_t (*pixels)[12][2] = (uint8_t (*)[12][2])displayBuffer->pixels;
    uint8_t bitsLow  = pixels[page][cycle][0];
    uint8_t bitsHigh = pixels[page][cycle][1];

    PORTD = bitsLow;
    PORTB = bitsHigh;

    if (page < 2*(SHADES - 1)) {
        if (cycle < 6) {
            bitsLow  |= _BV(cycle+2);
        } else {
            bitsHigh |= _BV(cycle-6);
        }
        DDRD = bitsLow;
        DDRB = bitsHigh;
    }

    page++;

    if (page >= 2*SHADES) {
        page = 0;
        cycle++;
    }

    if (cycle >= 12) {
        cycle = 0;

        // If the page should be flipped, do it here.
        if (videoFlipPage && (displayMode & DOUBLE_BUFFER))
        {
            // TODO: is this an atomic operation?
            videoFlipPage = false;

            videoPage* temp = displayBuffer;
            displayBuffer = workBuffer;
            workBuffer = temp;
        }

        if (videoFlipTimer) {
            videoFlipTimer = false;

            tempTimer = frontTimer;
            frontTimer = backTimer;
            backTimer = tempTimer;
        }
    }

#ifdef MEASURE_ISR_TIME
    digitalWrite(statusPIN, LOW);
#endif
}
