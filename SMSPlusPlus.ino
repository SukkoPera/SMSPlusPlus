/*******************************************************************************
 * This file is part of SMS++.                                                 *
 *                                                                             *
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>                    *
 *                                                                             *
 * SMS++ is free software: you can redistribute it and/or modify               *
 * it under the terms of the GNU General Public License as published by        *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * SMS++ is distributed in the hope that it will be useful,                    *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with SMS++. If not, see <http://www.gnu.org/licenses/>.               *
 *******************************************************************************
 *
 * SMS++ - 50/60 Hz switch and In-Game-Reset (IGR) for Sega Master System.
 *
 * Please refer to the GitHub page and wiki for any information:
 * https://github.com/SukkoPera/SMSPlusPlus
 */


// http://www.smspower.org/Development/PeripheralPorts


/*******************************************************************************
 * PLATFORM SELECTION
 ******************************************************************************/

#if defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168__) || defined (__AVR_ATmega8__)

/*
 * Arduino Uno/Nano/Standalone, but these have different configurations, so the
 * board type must be defined MANUALLY
 */
//~ #define ARDUINO_UNO
#define ARDUINO_NANO

#if defined (ARDUINO_UNO)

#warning "Compiling for Arduino Uno"

/*
 *
 *                      +----[PWR]-------------------| USB |--+
 *                      |                            +-----+  |
 *                      |         GND/RST2  [ ][ ]            |
 *                      |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |
 *                      |          5V/MISO2 [ ][ ]  A4/SDA[ ] |
 *                      |                             AREF[ ] |
 *                      |                              GND[ ] |
 *                      | [ ]N/C                    SCK/13[X] | PB5 Pad Port Pin 9 (SMS: TR/B2 - MD: C/Start)
 *                      | [ ]IOREF                 MISO/12[X] | PB4 Pad Port Pin 6 (SMS: TL/B1/Trigger - MD: B/A)
 *                      | [ ]RST                   MOSI/11[X]~| PB3 Pad Port Pin 4 (SMS: Right - MD: Right/Mode)
 *                      | [ ]3V3    +---+               10[X]~| PB2 Pad Port Pin 3 (SMS: Left - MD: Left/X)
 *                  +5V | [X]5v    -| A |-               9[X]~| PB1 Pad Port Pin 2 (SMS: Down - MD: Down/Y)
 *                  GND | [X]GND   -| R |-               8[X] | PB0 Pad Port Pin 1 (SMS: Up - MD: Up/Z)
 *                      | [ ]GND   -| D |-                    |
 *                      | [ ]Vin   -| U |-               7[X] | PD7 Pad Port Pin 7 (SMS: TH/Light Sensor - MD: Select)
 *                      |          -| I |-               6[X]~| Pad Port Trace 7
 * Pad Port Trace 1 PC0 | [X]A0    -| N |-               5[X]~| Pause/Reset In
 * Pad Port Trace 2 PC1 | [X]A1    -| O |-               4[X] | Pause Out
 * Pad Port Trace 3 PC2 | [X]A2     +---+           INT1/3[X]~| Reset Out
 * Pad Port Trace 4 PC3 | [X]A3                     INT0/2[X] | Video Mode
 * Pad Port Trace 6 PC4 | [X]A4/SDA  RST SCK MISO     TX>1[ ] | (Led Green)
 * Pad Port Trace 9 PC5 | [X]A5/SCL  [ ] [ ] [ ]      RX<0[ ] | (Led Red)
 *                      |            [ ] [ ] [ ]              |
 *                      |  UNO_R3    GND MOSI 5V  ____________/
 *                      \_______________________/
 */

/* We don't have enough pins to connect both the Reset and Pause buttons. Anyway
 * we don't really need them both. Actually we only need one of them to switch
 * between 50/60 Hz modes from the console itself. If switching from the
 * controller is enough, don't enable/connect any of them.
 *
 * By default we expect Pause to be connected, since it is the only physical
 * button available on the SMS2. If it is connected, it can be turned into a
 * Reset button enabling RESET_ON_PAUSE below.
 */
#define PAUSE_IN_PIN 5
//#define RESET_IN_PIN 5

// Other pin definitions
#define PAUSE_OUT_PIN 4
#define RESET_OUT_PIN 3
#define VIDEOMODE_PIN 2

/* If leds are enabled, the serial console (useful for debugging) will be
 * disabled
 */
#define MODE_LED_R_PIN 0
#define MODE_LED_G_PIN 1

// Controller port
#define PDREG_PAD_PORT DDRB
#define PDREG_PAD_BITS ((1 << DDB5) | (1 << DDB4) | (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0))
#define PIREG_PAD PINB
#define POREG_PAD PORTB

// Select signal
#define PDREG_SELECT_PORT DDRD
#define PDREG_SELECT_BIT DDD7
#define POREG_SELECT PORTD

// Select signal is on a different port
#define PIREG_SELECT PIND

// Traces port
#define PDREG_TRACES_PORT DDRC
#define PDREG_TRACES_BITS ((1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0))
#define POREG_TRACES PORTC

// Select trace is on a different port
#define PDREG_TRACE7_PORT DDRD
#define PDREG_TRACE7_BIT DDD6
#define POREG_TRACE7 PORTD

#if !defined (MODE_LED_R_PIN) && !defined (MODE_LED_G_PIN)
	#define ENABLE_SERIAL_DEBUG
#else
	#warning "Serial debugging disabled"
#endif
//~ #define DEBUG_PAD

#elif defined (ARDUINO_NANO)

#warning "Compiling for Arduino Nano"

/*
 * This configuration is almost identical to that of the Uno, except that we use
 * the Nano extra pin A6 to emulate a digital input to sample the Pause/Reset
 * button. This frees up pin 5, which we can then use as Controller Type Out.
 *
 *                                               +-----+
 *                                  +------------| USB |------------+
 *                                  |            +-----+            |
 *                   Pad Port Pin 9 | [X]D13/SCK        MISO/D12[X] | Pad Port Pin 6
 *                                  | [ ]3.3V           MOSI/D11[X]~| Pad Port Pin 4
 *                                  | [ ]V.ref     ___    SS/D10[X]~| Pad Port Pin 3
 *                 Pad Port Trace 1 | [X]A0       / N \       D9[X]~| Pad Port Pin 2
 *                 Pad Port Trace 2 | [X]A1      /  A  \      D8[X] | Pad Port Pin 1
 *                 Pad Port Trace 3 | [X]A2      \  N  /      D7[X] | Pad Port Pin 7
 *                 Pad Port Trace 4 | [X]A3       \_0_/       D6[X]~| Pad Port Trace 7
 *                 Pad Port Trace 6 | [X]A4/SDA               D5[X]~| Controller Type Out
 *                 Pad Port Trace 9 | [X]A5/SCL               D4[X] | Pause Out
 *                   Pause/Reset In | [X]A6              INT1/D3[X]~| Reset Out
 *                                  | [ ]A7              INT0/D2[X] | Video Mode
 *                              +5V | [X]5V                  GND[X] | GND
 *                                  | [ ]RST                 RST[ ] |
 *                                  | [ ]GND   5V MOSI GND   TX1[X] | (Led Green)
 *                                  | [ ]Vin   [ ] [ ] [ ]   RX0[X] | (Led Red)
 *                                  |          [ ] [ ] [ ]          |
 *                                  |          MISO SCK RST         |
 *                                  | NANO-V3                       |
 *                                  +-------------------------------+
 */

/* We don't have enough pins to connect both the Reset and Pause buttons. Anyway
 * we don't really need them both. Actually we only need one of them to switch
 * between 50/60 Hz modes from the console itself. If switching from the
 * controller is enough, don't enable/connect any of them.
 *
 * By default we expect Pause to be connected, since it is the only physical
 * button available on the SMS2. If it is connected, it can be turned into a
 * Reset button enabling RESET_ON_PAUSE below.
 */
#define PAUSE_IN_PIN A6
//~ #define RESET_IN_PIN A6

// Threshold to read analog inputs as HIGH
#define ANALOG_IN_THRESHOLD 950

/* Controller Type Out: this will be HIGH when a MegaDrive 3- or 6-button pad is
 * detected. Useful for driving a multiplexer, see wiki.
 */
#define PAD_TYPE_PIN 5

#define PAUSE_OUT_PIN 4
#define RESET_OUT_PIN 3
#define VIDEOMODE_PIN 2

/* If leds are enabled, the serial console (useful for debugging) will be
 * disabled
 */
#define MODE_LED_R_PIN 0
#define MODE_LED_G_PIN 1

// Controller port
#define PDREG_PAD_PORT DDRB
#define PDREG_PAD_BITS ((1 << DDB5) | (1 << DDB4) | (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0))
#define PIREG_PAD PINB
#define POREG_PAD PORTB

// Select signal
#define PDREG_SELECT_PORT DDRD
#define PDREG_SELECT_BIT DDD7
#define POREG_SELECT PORTD

// Select signal is on a different por
#define PIREG_SELECT PIND

// Traces port
#define PDREG_TRACES_PORT DDRC
#define PDREG_TRACES_BITS ((1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0))
#define POREG_TRACES PORTC

// Select trace is on a different port
#define PDREG_TRACE7_PORT DDRD
#define PDREG_TRACE7_BIT DDD6
#define POREG_TRACE7 PORTD

#if !defined (MODE_LED_R_PIN) && !defined (MODE_LED_G_PIN)
	#define ENABLE_SERIAL_DEBUG
#else
	#warning "Serial debugging disabled"
#endif

#else
	#error "Unsupported Arduino platform!"
#endif

#else
	#error "Unsupported platform!"
#endif


/*******************************************************************************
 * BUTTON COMBO SETTINGS
 ******************************************************************************/

/* DON'T TOUCH THIS! Just look at it for the button names you can use below!
 *
 * Technical note: This has been organized (together with the controller port
 * wiring) to minimize bit twiddling in the controller reading function.
 */
enum MdButton {
	MD_BTN_MODE =  1 << 11,
	MD_BTN_X =     1 << 10,
	MD_BTN_Y =     1 << 9,
	MD_BTN_Z =     1 << 8,
	MD_BTN_START = 1 << 7,
	MD_BTN_A =     1 << 6,
	MD_BTN_C =     1 << 5,
	MD_BTN_B =     1 << 4,
	MD_BTN_RIGHT = 1 << 3,
	MD_BTN_LEFT =  1 << 2,
	MD_BTN_DOWN =  1 << 1,
	MD_BTN_UP =    1 << 0
};

// Master System Buttons - For internal use only
enum SmsButton {
	SMS_BTN_TH =    1 << 6,
	SMS_BTN_TR =    1 << 5,
	SMS_BTN_TL =    1 << 4,
	SMS_BTN_RIGHT = 1 << 3,
	SMS_BTN_LEFT =  1 << 2,
	SMS_BTN_DOWN =  1 << 1,
	SMS_BTN_UP =    1 << 0,

	// Commodity aliases
	SMS_BTN_B1 =    SMS_BTN_TL,
	SMS_BTN_B2 =    SMS_BTN_TR
};

/* Button combo that enables the other combos
 *
 * Note: That vertical bar ("pipe") means that the buttons must be pressed
 *       together.
 */
#define COMBO_TRIGGER (MD_BTN_START | MD_BTN_B)

/* Button combos to perform other actions. These are to be considered in
 * addition to TRIGGER_COMBO.
 */
#define COMBO_RESET (MD_BTN_A | MD_BTN_C)

// Combos for video modes
#define COMBO_50HZ MD_BTN_LEFT
#define COMBO_60HZ MD_BTN_RIGHT

/* Combos to switch among autofire modes. These are NOT in addition to
 * TRIGGER_COMBO.
 */
#define COMBO_AUTOFIRE_X (MD_BTN_START | MD_BTN_X)
#define COMBO_AUTOFIRE_Y (MD_BTN_START | MD_BTN_Y)
#define COMBO_AUTOFIRE_Z (MD_BTN_START | MD_BTN_Z)

// Define this to use A as B+C. When padUseAB is enabled, C = A+B.
#define PAD_USE_THIRD_BTN_AS_2BTNS


/*******************************************************************************
 * ADVANCED SETTINGS
 ******************************************************************************/

/* Offset in the EEPROM at which the current mode should be saved. Undefine to
 * disable mode saving.
 */
#define MODE_ROM_OFFSET 42

// Time to wait after mode change before saving the new mode (milliseconds)
#define MODE_SAVE_DELAY 3000L

/* Colors to use to indicate the video mode, in 8-bit RGB componentes. You can
 * use any value here if your led is connected to PWM-capable pins, otherwise
 * values specified here will be interpreted as either fully off (if 0) or fully
 * on (if anything else).
 *
 * Note that using PWM-values here sometimes causes unpredictable problems. This
 * happened to me on an ATtiny861, and it's probably due to how pins and timers
 * interact. It seems to work fine on a full Arduino, but unless you really want
 * weird colors, use only 0x00 and 0xFF.
 *
 * We only have two LED pins, so let's use a dual-color led.
 */

#define MODE_LED_50HZ_COLOR {0xFF, 0x00}  // Red
#define MODE_LED_60HZ_COLOR {0x00, 0xFF}  // Green

// Define this if your led is common-anode, comment out for common-cathode
//#define MODE_LED_COMMON_ANODE

/* Use a single led to indicate the video mode. Since this does NOT disable the
 * dual led, it can be used together with it, provided that you have a free pin.
 *
 * Basically, the single led is blinked 1-2 times according to which mode is set
 * (1 is 50 Hz, see VideoMode below).
 */
//#define MODE_LED_SINGLE_PIN 1

/* Use a led to indicate when a button press is detected. Useful for making sure
 * that all button presses are registered correctly.
 */
//#define PAD_LED_PIN 0

// Print the controller status on serial. Only useful for debugging.
#ifdef ENABLE_SERIAL_DEBUG
//~ #define DEBUG_PAD
#endif

/* Reset the console when the pause button on the console itself is pressed.
 * This might be useful on the SMS2, since it has no RESET button. Now that you
 * can trigger PAUSE from your controller, the PAUSE button on the console is
 * pretty useless, isn't it?
 */
#define RESET_ON_PAUSE

/* Presses of the reset button longer than this amount of milliseconds will
 * switch to the next mode, shorter presses will reset the console.
 */
#define LONGPRESS_LEN 700

/* Time to ignore combos for after one has been detected. Soft of acts as a
 * debouncing mechanism for combos.
 */
#define IGNORE_COMBO_MS LONGPRESS_LEN

// Debounce duration for the reset/pause button
#define DEBOUNCE_MS 20

// Duration of the reset/pause pulse (milliseconds)
#define PULSE_LEN 250

// Interval between pulses for reading the 6-button pad (microseconds)
#define SIXMD_BTN_PULSE_INTERVAL 30

/*******************************************************************************
 * END OF SETTINGS
 ******************************************************************************/


#ifdef MODE_ROM_OFFSET
	#include <EEPROM.h>
#endif

enum VideoMode {
	VID_50HZ = 0,
	VID_60HZ,
	VID_MODES_NO	// Leave at end
};

enum PadType {
	PAD_SMS,		// Master System
	PAD_MD,			// Mega Drive/Genesis
	PAD_MD_6BTN		// Mega Drive/Genesis 6-Button
};

enum AutoFireRate {
	AF_OFF = 0,
	AF_SLOW,
	AF_MEDIUM,
	AF_QUICK,
	AF_MODES_NO		// Leave at end
};

const byte autofireHitsPerSec[AF_MODES_NO] = {
	0,				// Well, this is just ignored
	3,
	6,
	9
};

struct AutoFireButton {
	AutoFireRate rate;
	unsigned long pressStart;
};

PadType padType = PAD_SMS;

VideoMode current_mode = VID_50HZ;

AutoFireButton afStatusX = {AF_OFF, 0};
AutoFireButton afStatusY = {AF_OFF, 0};
AutoFireButton afStatusZ = {AF_OFF, 0};

boolean padUseAB = false;

// This will be handy
#if (defined MODE_LED_R_PIN || defined MODE_LED_G_PIN)

#define ENABLE_MODE_LED_DUAL

const byte mode_led_colors[][VID_MODES_NO] = {
	MODE_LED_50HZ_COLOR,
	MODE_LED_60HZ_COLOR
};
#endif

unsigned long mode_last_changed_time;


#ifdef ENABLE_SERIAL_DEBUG
	#define debug(...) Serial.print (__VA_ARGS__)
	#define debugln(...) Serial.println (__VA_ARGS__)
#else
	#define debug(...)
	#define debugln(...)
#endif

/* These functions set the RESET line to the desired state. Note that RESET is
 * an active-low signal.
 *
 * We drive the RESET line emulating an open-collector output.
 */
#ifdef RESET_OUT_PIN
inline void enableReset () {
	/* No explicit setting to LOW is needed, pins are LOW by default when first
	 * set as OUTPUTs.
	 */
	pinMode (RESET_OUT_PIN, OUTPUT);
}

inline void disableReset () {
	/* Switch to INPUT, pin will go to HI-Z and the pull-up resistor we're
	 * soldered to will bring the line high
	 */
	pinMode (RESET_OUT_PIN, INPUT);
}
#endif

/* Ditto for the PAUSE line
 */
#ifdef PAUSE_OUT_PIN
inline void enablePause () {
	pinMode (PAUSE_OUT_PIN, OUTPUT);
}

inline void disablePause () {
	pinMode (PAUSE_OUT_PIN, INPUT);
}
#endif

void update_mode_leds () {
#ifdef ENABLE_MODE_LED_DUAL
	const byte *colors = mode_led_colors[current_mode];
	byte c;

#ifdef MODE_LED_R_PIN
	c = colors[0];
#ifdef MODE_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite (MODE_LED_R_PIN, c);
#endif

#ifdef MODE_LED_G_PIN
	c = colors[1];
#ifdef MODE_LED_COMMON_ANODE
	c = 255 - c;
#endif
	digitalWrite (MODE_LED_G_PIN, c);
#endif

#endif  // ENABLE_MODE_LED_DUAL

#ifdef MODE_LED_SINGLE_PIN
	// WARNING: This loop must be reasonably shorter than LONGPRESS_LEN in the worst case!
	for (int i = 0; i < current_mode + 1; ++i) {
		digitalWrite (MODE_LED_SINGLE_PIN, LOW);
		delay (40);
		digitalWrite (MODE_LED_SINGLE_PIN, HIGH);
		delay (80);
	}
#endif
}

void save_mode () {
#ifdef MODE_ROM_OFFSET
	if (mode_last_changed_time > 0 && millis () - mode_last_changed_time >= MODE_SAVE_DELAY) {
		debug (F("Saving video mode to EEPROM: "));
		debugln (current_mode);
		byte saved_mode = EEPROM.read (MODE_ROM_OFFSET);
		if (current_mode != saved_mode) {
			EEPROM.write (MODE_ROM_OFFSET, static_cast<byte> (current_mode));
		} else {
			debugln (F("Mode unchanged, not saving"));
		}
		mode_last_changed_time = 0;    // Don't save again

		// Blink led to tell the user that mode was saved
#ifdef ENABLE_MODE_LED_DUAL
		byte c = 0;

#ifdef MODE_LED_COMMON_ANODE
		c = 255 - c;
#endif

#ifdef MODE_LED_R_PIN
		digitalWrite (MODE_LED_R_PIN, c);
#endif

#ifdef MODE_LED_G_PIN
		digitalWrite (MODE_LED_G_PIN, c);
#endif

		// Keep off for a bit
		delay (200);

		// Turn led back on
		update_mode_leds ();
#endif  // ENABLE_MODE_LED_DUAL

#ifdef MODE_LED_SINGLE_PIN
		// Make one long flash
		digitalWrite (MODE_LED_SINGLE_PIN, LOW);
		delay (500);
		digitalWrite (MODE_LED_SINGLE_PIN, HIGH);
#endif
	}
#endif  // MODE_ROM_OFFSET
}

void set_mode (VideoMode m) {
	switch (m) {
		default:
		case VID_50HZ:
			digitalWrite (VIDEOMODE_PIN, HIGH);    // PAL 50Hz
			break;
		case VID_60HZ:
			digitalWrite (VIDEOMODE_PIN, LOW);	// PAL 60Hz
	}

	current_mode = m;
	update_mode_leds ();

	mode_last_changed_time = millis ();
}

void change_mode (int increment) {
	// This also loops in [0, VID_MODES_NO) backwards
	VideoMode new_mode = static_cast<VideoMode> ((current_mode + increment + VID_MODES_NO) % VID_MODES_NO);
	set_mode (new_mode);
}

void next_mode () {
	change_mode (+1);
}

void prev_mode () {
	change_mode (-1);
}

// Reset is active low on SMS
void handle_reset_button () {
#ifdef RESET_IN_PIN
	static byte debounce_level = LOW;
	static bool pressed_before = false;
	static long last_int = 0, last_pressed = 0;
	static unsigned int hold_cycles = 0;

#ifdef ARDUINO_NANO
	/* We use A6/A7 on this platform, which are only analog inputs, so we must
	 * read them as such
	 */
	byte pressed_now = (analogRead (RESET_IN_PIN) > ANALOG_IN_THRESHOLD) ? HIGH : LOW;
#else
	byte pressed_now = digitalRead (RESET_IN_PIN);
#endif

	if (pressed_now != debounce_level) {
		// Reset debouncing timer
		last_int = millis ();
		debounce_level = pressed_now;
	} else if (millis () - last_int > DEBOUNCE_MS) {
		// OK, button is stable, see if it has changed
		if (pressed_now == LOW && !pressed_before) {
			// Button just pressed
			last_pressed = millis ();
			hold_cycles = 0;
		}
		else if (pressed_now == HIGH && pressed_before) {
			// Button released
			if (hold_cycles == 0) {
				debugln (F("Reset button pushed for a short time"));
				reset_console ();
		}
	} else {
			// Button has not just been pressed/released
			if (pressed_now == LOW && millis () % last_pressed >= LONGPRESS_LEN * (hold_cycles + 1)) {
				// Reset has been hold for a while
				debugln (F("Reset button held"));
				++hold_cycles;
				next_mode ();
			}
		}

		pressed_before = (pressed_now == LOW);
	}
#else
	#warning "RESET button handling disabled"
#endif
}

// Pause is active low on SMS
void handle_pause_button () {
#ifdef PAUSE_IN_PIN
	static byte debounce_level = LOW;
	static bool pressed_before = false;
	static long last_int = 0, last_pressed = 0;
	static unsigned int hold_cycles = 0;

#ifdef ARDUINO_NANO
	// See above
	byte pressed_now = (analogRead (PAUSE_IN_PIN) > ANALOG_IN_THRESHOLD) ? HIGH : LOW;
#else
	byte pressed_now = digitalRead (PAUSE_IN_PIN);
#endif

	if (pressed_now != debounce_level) {
		// Reset debouncing timer
		last_int = millis ();
		debounce_level = pressed_now;
	} else if (millis () - last_int > DEBOUNCE_MS) {
		// OK, button is stable, see if it has changed
		if (pressed_now == LOW && !pressed_before) {
			// Button just pressed
			last_pressed = millis ();
			hold_cycles = 0;
		}
		else if (pressed_now == HIGH && pressed_before) {
			// Button released
			if (hold_cycles == 0) {
				debugln (F("Pause button pushed for a short time"));
#ifdef RESET_ON_PAUSE
				reset_console ();
#else
				pause_console ();
#endif
		}
	} else {
			// Button has not just been pressed/released
			if (pressed_now == LOW && millis () % last_pressed >= LONGPRESS_LEN * (hold_cycles + 1)) {
				// Reset has been hold for a while
				debugln (F("Pause button held"));
				++hold_cycles;
				next_mode ();
			}
		}

		pressed_before = (pressed_now == LOW);
	}
#else
	#warning "PAUSE button handling disabled"
#endif
}


void reset_console () {
	debugln (F("Resetting console"));

	enableReset ();
	delay (PULSE_LEN);
	disableReset ();
}

void pause_console () {
	debugln (F("Pausing console"));

	enablePause ();
	delay (PULSE_LEN);
	disablePause ();
}

// Set the level of the SELECT signal of the first controller port
inline void setSelect (byte level) {
	if (level)
		POREG_SELECT |= (1 << PDREG_SELECT_BIT);
	else
		POREG_SELECT &= ~(1 << PDREG_SELECT_BIT);
}

// Returns the state of the first controller port
inline byte readPadPort () {
	return PIREG_PAD & PDREG_PAD_BITS;
}

#ifdef PIREG_SELECT
inline boolean readPadPin7 () {
	//~ debugln (PIREG_SELECT, BIN);
	return PIREG_SELECT & (1 << PDREG_SELECT_BIT);
}
#endif

void setup_pad () {
	byte port = 0xFF;

	// Set port directions
	PDREG_SELECT_PORT |= (1 << PDREG_SELECT_BIT);	// Select line is an OUTPUT
	PDREG_PAD_PORT &= ~(PDREG_PAD_BITS);			// Other lines are INPUTs...
	POREG_PAD |= PDREG_PAD_BITS;					// ... with pull-ups

#ifdef PAD_TYPE_PIN
	// Init pad type out
	pinMode (PAD_TYPE_PIN, OUTPUT);
#endif

	// Guess pad type - Start with select line high for a while
	setSelect (HIGH);
	delay (10);

	// Bring select line low 1st time
	setSelect (LOW);
	delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);
	port = readPadPort ();
#ifdef DEBUG_PAD
	debug (F("Port Read #1 = "));
	debugln (port, BIN);
#endif
	if ((port & 0x0C) == 0) {
		/* Left and right are both pressed. As this usually does not happen,
		 * unless we have a very worn controller, we assume it is a Mega Drive
		 * pad.
		 *
		 * Now let's check whether it has 3 or 6 buttons
		 */

		// Assume 3 buttons for a start
		padType = PAD_MD;

		/* Now follow the protocol described at
		 * https://applause.elfmimi.jp/md6bpad-e.html
		 */
		setSelect (HIGH);			// High again (1st time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);
		setSelect (LOW);			// Low again (2nd time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

		/* We have A and Start now, see if we must enable padUseAB (that is when
		 * A is pressed at power-up).
		 */
		port = readPadPort ();
		if ((port & 0x10) == 0) {
			padUseAB = true;
		}

		setSelect (HIGH);			// High again (2nd time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);
		setSelect (LOW);			// Low (3rd time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

		// We should have all 0s now for the 6-button pad
		port = readPadPort ();
#ifdef DEBUG_PAD
		debug (F("Port Read #2 = "));
		debugln (port, BIN);
#endif
		if ((port & 0x0F) == 0x00) {
			setSelect (HIGH);		// High again (3rd time)
			delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);
			// Have Z Y X MD here
			setSelect (LOW);		// Low (4th time)
			delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

			// We should have all 1s now for the 6-button pad
			port = readPadPort ();
#ifdef DEBUG_PAD
			debug (F("Port Read #3 = "));
			debugln (port, BIN);
#endif
			if ((port & 0x0F) == 0x0F) {
				// This is a 6-button pad
				padType = PAD_MD_6BTN;
			}
		}

		// Bring select line high again
		setSelect (HIGH);

		if (padType == PAD_MD) {
			debugln (F("Detected Mega Drive pad"));
		} else {
			debugln (F("Detected Mega Drive 6-Button pad"));
		}

#ifdef PAD_TYPE_PIN
		digitalWrite (PAD_TYPE_PIN, HIGH);
#endif
	} else {
		// This is a SMS pad - Switch SELECT to INPUT with pull-up
		PDREG_SELECT_PORT &= ~(1 << PDREG_SELECT_BIT);
		setSelect (HIGH);

		padType = PAD_SMS;
		debugln (F("Detected Master System pad"));

#ifdef PAD_TYPE_PIN
		digitalWrite (PAD_TYPE_PIN, LOW);
#endif
	}
}

void setup_traces () {
	PDREG_TRACES_PORT |= PDREG_TRACES_BITS;	// Trace lines are all OUTPUTs

#ifdef PDREG_TRACE7_PORT
	// Trace 7 is on a different MCU port and has to be an OUTPUT as well
	PDREG_TRACE7_PORT |= 1 << PDREG_TRACE7_BIT;
#endif

	// Make sure no buttons are pressed at start
	write_sms_pad (0x00);
}

/******************************************************************************/

/*
 * The basic idea here is to make up a word where each bit represents the state
 * of a button, where 1 means pressed, for commodity's sake. The bit-button
 * mapping is defined in the MdButton enum above.
 *
 * To get consistent readings, we should really read all of the pad pins at
 * once, at least with the 6-button pad, since tour source states that only data
 * read in 1.6 milli seconds from the first up-edge of Select is reliable.
 * In order to do this we try to connect all pins to a single port of our MCU.
 */
inline word read_md_pad () {
	static word pad_status = 0x0000;
	byte port;

	// Start with select line high for a while
	setSelect (HIGH);
	delay (10);

	// We can read up, down, left, right, C & B
	port = readPadPort ();
	pad_status = (pad_status & 0xFFC0)
	           | (~port & 0x3F);
	           ;

	// Bring select line low 1st time
	setSelect (LOW);
	delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

	// We can read Start & A
	port = readPadPort ();
	pad_status = (pad_status & 0xFF3F)
	           | ((~port & 0x30) << 2)
	           ;

	if (padType == PAD_MD_6BTN) {
		setSelect (HIGH);			// High again (1st time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);
		setSelect (LOW);			// Low again (2nd time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

		setSelect (HIGH);			// High again (2nd time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);
		setSelect (LOW);			// Low (3rd time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

		// All 0s at this point

		setSelect (HIGH);			// High again (3rd time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

		// We can read Z, Y, X & Mode
		port = readPadPort ();
		pad_status = (pad_status & 0xF0FF)
		           | ((((word) ~port) & 0x000F) << 8)
		           ;

		setSelect (LOW);			// Low (4th time)
		delayMicroseconds (SIXMD_BTN_PULSE_INTERVAL);

		// All 1s at this point
	}

	// Finally bring select line high again
	setSelect (HIGH);

	// Mask away bits representing non-existing buttons
	pad_status &= 0x0FFF;

#ifdef DEBUG_PAD
	debug (F("Pressed: "));
	if (pad_status & MD_BTN_UP)
		debug (F("Up "));
	if (pad_status & MD_BTN_DOWN)
		debug (F("Down "));
	if (pad_status & MD_BTN_LEFT)
		debug (F("Left "));
	if (pad_status & MD_BTN_RIGHT)
		debug (F("Right "));
	if (pad_status & MD_BTN_A)
		debug (F("A "));
	if (pad_status & MD_BTN_B)
		debug (F("B "));
	if (pad_status & MD_BTN_C)
		debug (F("C "));
	if (pad_status & MD_BTN_X)
		debug (F("X "));
	if (pad_status & MD_BTN_Y)
		debug (F("Y "));
	if (pad_status & MD_BTN_Z)
		debug (F("Z "));
	if (pad_status & MD_BTN_MODE)
		debug (F("Mode "));
	if (pad_status & MD_BTN_START)
		debug (F("Start "));
	debugln ();
#endif

	return pad_status;
}

inline byte read_sms_pad () {
	byte pad_status = ~readPadPort ();

#ifdef PIREG_SELECT
	/* Pad port pin 7 is on a different MCU port, so read it separately and
	 * append it
	 */
	if (readPadPin7 ()) {		// Active-low: True -> No light detected
		pad_status &= ~(SMS_BTN_TH);
	} else {
		pad_status |= (SMS_BTN_TH);
	}
#endif

	// Mask away bits representing non-existing buttons
	pad_status &= 0x7F;

#ifdef DEBUG_PAD
	debug (F("Pressed: "));
	if (pad_status & SMS_BTN_UP)
		debug (F("Up "));
	if (pad_status & SMS_BTN_DOWN)
		debug (F("Down "));
	if (pad_status & SMS_BTN_LEFT)
		debug (F("Left "));
	if (pad_status & SMS_BTN_RIGHT)
		debug (F("Right "));
	if (pad_status & SMS_BTN_B1)
		debug (F("B1 "));
	if (pad_status & SMS_BTN_B2)
		debug (F("B2 "));
	if (pad_status & SMS_BTN_TH)
		debug (F("TH "));
	debugln ();
#endif

	return pad_status;
}

inline void write_sms_pad (byte pad_status) {
#ifdef DEBUG_PAD
	debug (F("Sending SMS pad status: "));
	debugln (pad_status, BIN);
#endif

	// NOTE: 0 means pressed!

#ifdef PDREG_TRACE7_PORT
	// Trace 7 is on a different MCU port
	byte t7 = !(pad_status & SMS_BTN_TH);
	POREG_TRACE7 = (POREG_TRACE7 & ~(1 << PDREG_TRACE7_BIT)) | (t7 << PDREG_TRACE7_BIT);
#endif

	POREG_TRACES = ~pad_status & PDREG_TRACES_BITS;
}

boolean checkAutoFire (AutoFireButton& btn, boolean nowPressed) {
	boolean ret = false;

	if (btn.rate != AF_OFF && btn.rate < AF_MODES_NO) {
		unsigned long intv = 1000 / autofireHitsPerSec[btn.rate];	// ms between presses

		if (btn.pressStart != 0) {
			// Button was pressed before
			if (!nowPressed) {
				// Just released
				btn.pressStart = 0;
			} else {
				// Kept pressed
				//~ smsPad |= ((millis () - btn.pressStart) / intv) % 2 == 0 ? SMS_BTN_B1 : 0x00;
				ret = ((millis () - btn.pressStart) / intv) % 2 == 0;
			}
		} else {
			// Button was NOT pressed before
			if (nowPressed) {
				// Just pressed
				btn.pressStart = millis ();
			} else {
				// Not pressed
			}
		}
	} else {
		ret = nowPressed;
	}

	return ret;
}

inline byte mdPadToSms (word mdPad) {
	byte smsPad = 0x00;

	smsPad |= (mdPad & MD_BTN_UP) ? SMS_BTN_UP : 0x00;
	smsPad |= (mdPad & MD_BTN_DOWN) ? SMS_BTN_DOWN : 0x00;
	smsPad |= (mdPad & MD_BTN_LEFT) ? SMS_BTN_LEFT : 0x00;
	smsPad |= (mdPad & MD_BTN_RIGHT) ? SMS_BTN_RIGHT : 0x00;

	if (padUseAB) {
		/* Normally SMS buttons 1 and 2 are mapped to B and C on the MD pad. But we
		 * can map them to A and B just as easily.
		 */
		smsPad |= (mdPad & MD_BTN_A) ? SMS_BTN_B1 : 0x00;
		smsPad |= (mdPad & MD_BTN_B) ? SMS_BTN_B2 : 0x00;

		// Handle autofire
		smsPad |= checkAutoFire (afStatusX, mdPad & MD_BTN_X) ? SMS_BTN_B1 : 0x00;
		smsPad |= checkAutoFire (afStatusY, mdPad & MD_BTN_Y) ? SMS_BTN_B2 : 0x00;

#ifdef PAD_USE_THIRD_BTN_AS_2BTNS
		if ((mdPad & MD_BTN_C) || checkAutoFire (afStatusZ, mdPad & MD_BTN_Z)) {
			smsPad |= SMS_BTN_B1 | SMS_BTN_B2;
		}
#endif
	} else {
		// B -> B1, C -> B2
		smsPad |= (mdPad & MD_BTN_B) ? SMS_BTN_B1 : 0x00;
		smsPad |= (mdPad & MD_BTN_C) ? SMS_BTN_B2 : 0x00;

		// Handle autofire
		smsPad |= checkAutoFire (afStatusY, mdPad & MD_BTN_Y) ? SMS_BTN_B1 : 0x00;
		smsPad |= checkAutoFire (afStatusZ, mdPad & MD_BTN_Z) ? SMS_BTN_B2 : 0x00;

#ifdef PAD_USE_THIRD_BTN_AS_2BTNS
		if ((mdPad & MD_BTN_A) || checkAutoFire (afStatusX, mdPad & MD_BTN_X)) {
			smsPad |= SMS_BTN_B1 | SMS_BTN_B2;
		}
#endif
	}

	return smsPad;
}

void cycleAutoFire (AutoFireButton& btn) {
	btn.rate = static_cast<AutoFireRate> ((btn.rate + 1) % AF_MODES_NO);
}

void handle_pad () {
	static long last_combo_time = 0;

	switch (padType) {
		case PAD_SMS: {
			// Just relay data without much thinking
			byte pad_status = read_sms_pad ();
			write_sms_pad (pad_status);
			break;
		}

		case PAD_MD:
		case PAD_MD_6BTN: {
			word pad_status = read_md_pad ();

#ifdef PAD_LED_PIN
			digitalWrite (PAD_LED_PIN, pad_status);
#endif

			if (millis () - last_combo_time > IGNORE_COMBO_MS) {
				// Look for special combos
				if ((pad_status & COMBO_TRIGGER) == COMBO_TRIGGER) {
					if ((pad_status & COMBO_RESET) == COMBO_RESET) {
						debugln (F("Reset combo detected"));
						reset_console ();
						last_combo_time = millis ();
					} else if ((pad_status & COMBO_50HZ) == COMBO_50HZ) {
						debugln (F("50 Hz combo detected"));
						set_mode (VID_50HZ);
						last_combo_time = millis ();
					} else if ((pad_status & COMBO_60HZ) == COMBO_60HZ) {
						debugln (F("60 Hz combo detected"));
						set_mode (VID_60HZ);
						last_combo_time = millis ();
					}
				} else if ((pad_status & COMBO_AUTOFIRE_X) == COMBO_AUTOFIRE_X) {
					cycleAutoFire (afStatusX);
					last_combo_time = millis ();
				} else if ((pad_status & COMBO_AUTOFIRE_Y) == COMBO_AUTOFIRE_Y) {
					cycleAutoFire (afStatusY);
					last_combo_time = millis ();
				} else if ((pad_status & COMBO_AUTOFIRE_Z) == COMBO_AUTOFIRE_Z) {
					cycleAutoFire (afStatusZ);
					last_combo_time = millis ();
				} else if (pad_status & MD_BTN_START) {
					// Pause console - Do not update last_combo_time here
					pause_console ();
				}
			}

			// Send pad status to SMS
			byte smsPad = mdPadToSms (pad_status);
			write_sms_pad (smsPad);

			break;
		}
	}
}

void setup () {
#ifdef ENABLE_SERIAL_DEBUG
	Serial.begin (9600);
#endif

	debugln (F("Starting up..."));

	// Enable reset
	enableReset ();

	// Setup leds
#ifdef MODE_LED_R_PIN
	pinMode (MODE_LED_R_PIN, OUTPUT);
#endif

#ifdef MODE_LED_G_PIN
	pinMode (MODE_LED_G_PIN, OUTPUT);
#endif

#ifdef MODE_LED_SINGLE_PIN
	pinMode (MODE_LED_SINGLE_PIN, OUTPUT);
#endif

#ifdef PAD_LED_PIN
	pinMode (PAD_LED_PIN, OUTPUT);
#endif

	// Init video mode
	pinMode (VIDEOMODE_PIN, OUTPUT);
	current_mode = VID_50HZ;

#ifdef MODE_ROM_OFFSET
	byte tmp = EEPROM.read (MODE_ROM_OFFSET);
	debug (F("Loaded video mode from EEPROM: "));
	debugln (tmp);
	if (tmp < VID_MODES_NO) {
		// Palette EEPROM value is good
		current_mode = static_cast<VideoMode> (tmp);
	}
#endif
	set_mode (current_mode);
	mode_last_changed_time = 0;   // No need to save what we just loaded

	// Prepare to read pad
	setup_pad ();

	// Prepare traces port
	setup_traces ();

	// Prepare pause button/line
#if defined (PAUSE_IN_PIN) && !defined (ARDUINO_NANO)
	pinMode (PAUSE_IN_PIN, INPUT_PULLUP);
#endif
	disablePause ();

	// Prepare reset button
#if defined (RESET_IN_PIN) && !defined (ARDUINO_NANO)
	pinMode (RESET_IN_PIN, INPUT_PULLUP);
#endif

	// Finally release the reset line
	disableReset ();
}

void loop () {
	handle_reset_button ();
	handle_pause_button ();
	handle_pad ();
	save_mode ();
}
