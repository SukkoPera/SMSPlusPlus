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

// Check if we should disable some features because of low flash space
//~ #if FLASHEND < 2048
		//~ /* We only have 2 kb flash, let's take special measures:
		 //~ * - Only use a single flashing led to signal current mode
		 //~ * - On ATtiny24 we also always save video mode when changed, without
		 //~ *   checking if it has actually changed: this will wear out EEPROM a bit
		 //~ *   more quickly but it will still take ages ;)
		 //~ */
		//~ #warning Low flash space mode enabled
		//~ #define LOW_FLASH
//~ #endif


#if defined __AVR_ATtinyX61__
/*
 * On ATtinyX61's all features are supported. We even read all buttons with a
 * single instruction.
 *
 * The connection layout puts the SELECT signal on the INT1 pin. This will
 * probably be needed if we ever want to read 6-button pads. LED is connected to
 * PWM-capable pins.
 *
 *                    ,-----_-----.
 *           Reset In |1   9  0 20| Pad Port Pin 1
 *            LED Red |2   8  1 19| Pad Port Pin 2
 *          Reset Out |3   7  2 18| Pad Port Pin 7
 *          LED Green |4   6 14 17| Pad Port Pin 3
 *                +5V |5        16| GND
 *                GND |6        15| +5V
 * JP3/4 (Video Mode) |7   5 10 14| Pad Port Pin 4
 *           LED Blue |8   4 11 13| Pad Port Pin 6
 *   JP1/2 (Language) |9   3 12 12| Pad Port Pin 9
 *                    |10(15)13 11|
 *                    `-----------'
 */
#define RESET_IN_PIN 9
#define RESET_OUT_PIN 7
#define VIDEOMODE_PIN 5
#define LANGUAGE_PIN 3

#ifdef LOW_FLASH
	#define MODE_LED_SINGLE_PIN 8
#else
	#define MODE_LED_R_PIN 8
	#define MODE_LED_G_PIN 6
	#define MODE_LED_B_PIN 4
#endif

#elif defined __AVR_ATtinyX313__
/*
 * On ATtinyX13's all features are supported. We even read all buttons with a
 * single instruction.
 *
 * Again, the connection layout puts the SELECT signal on the INT1 pin. LED is
 * connected to PWM-capable pins.
 *
 *                     ,-----_-----.
 *                     |1   9  0 20| +5V
 *      Pad Port Pin 3 |2   8  1 19| Video Mode
 *      Pad Port Pin 4 |3   7  2 18| Pause Out
 *          [Reset In] |4   6 14 17| Reset Out
 *            Pause In |5        16| Pad Port Trace 7
 *      Pad Port Pin 6 |6        15| Pad Port Trace 9
 *      Pad Port Pin 9 |7   5 10 14| Pad Port Trace 6
 *      Pad Port Pin 7 |8   4 11 13| Pad Port Trace 4
 *  Multiplexer Select |9   3 12 12| Pad Port Trace 3
 *                 GND |10(15)13 11| Led
 *                     `-----------'
 */
//~ #define RESET_IN_PIN 13
//~ #define RESET_OUT_PIN 14
//~ #define VIDEOMODE_PIN 16
//~ #define LANGUAGE_PIN 15

//~ #ifdef LOW_FLASH
		//~ #define MODE_LED_SINGLE_PIN 10
//~ #else
		//~ #define MODE_LED_R_PIN 10
		//~ #define MODE_LED_G_PIN 11
		//~ #define MODE_LED_B_PIN 12
//~ #endif

#elif defined __AVR_ATmega328__ || defined __AVR_ATmega328P__ || defined __AVR_ATmega168__
/*
 * Arduino Uno/Nano/Micro/Whatever, use a convenience #define till we come up
 * with something better
 */
#define ARDUINO328

/*
Multiplexer Select
Led Red
Led Green
Reset In
Pause In
                                    ,-----_-----.
                        [Reset] PC6 |1     A5 28| PC5
               Pad Port Trace 1 PD0 |2   0 A4 27| PC4
               Pad Port Trace 2 PD1 |3   1 A3 26| PC3
               Pad Port Trace 3 PD2 |4   2 A2 25| PC2 Video Mode Out
               Pad Port Trace 4 PD3 |5   3 A1 24| PC1 Pause Out
               Pad Port Trace 6 PD4 |6   4 A0 23| PC0 Reset Out
                                +5V |7        22| GND
                                GND |8        21|
 Pad Port Pin 9 (TR - Button 2) PB6 |9        20| +5V
                                PB7 |10    13 19| PB5 Pad Port Pin 7 (TH - Light Sensor - MD Select) [+ Built-in LED]
               Pad Port Trace 7 PD5 |11  5 12 18| PB4 Pad Port Pin 6 (TL - Button 1/Trigger)
               Pad Port Trace 9 PD6 |12  6 11 17| PB3 Pad Port Pin 4 (Right)
                                PD7 |13  7 10 16| PB2 Pad Port Pin 3 (Left)
            Pad Port Pin 1 (Up) PB0 |14  8  9 15| PB1 Pad Port Pin 2 (Down)
                                    `-----------'




					+----[PWR]-------------------| USB |--+
					|                            +-----+  |
					|         GND/RST2  [ ][ ]            |
					|       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |
					|          5V/MISO2 [ ][ ]  A4/SDA[ ] |
					|                             AREF[ ] |
					|                              GND[ ] |
					| [ ]N/C                    SCK/13[X] | PB5 Pad Port Pin 9 (SMS: TR/B2 - MD: C/Start)
					| [ ]IOREF                 MISO/12[X] | PB4 Pad Port Pin 6 (SMS: TL/B1/Trigger - MD: B/A)
					| [ ]RST                   MOSI/11[X]~| PB3 Pad Port Pin 4 (SMS: Right - MD: Right/Mode)
					| [ ]3V3    +---+               10[X]~| PB2 Pad Port Pin 3 (SMS: Left - MD: Left/X)
			    +5V | [X]5v    -| A |-               9[X]~| PB1 Pad Port Pin 2 (SMS: Down - MD: Down/Y)
			    GND | [X]GND   -| R |-               8[X] | PB0 Pad Port Pin 1 (SMS: Up - MD: Up/Z)
					| [ ]GND   -| D |-                    |
					| [ ]Vin   -| U |-               7[X] | PD7 Pad Port Pin 7 (SMS: TH/Light Sensor - MD: Select)
					|          -| I |-               6[X]~| Pad Port Trace 7
   Pad Port Trace 1 | [X]A0    -| N |-               5[X]~| Pause In
   Pad Port Trace 2 | [X]A1    -| O |-               4[X] | Pause Out
   Pad Port Trace 3 | [X]A2     +---+           INT1/3[X]~| Reset Out
   Pad Port Trace 4 | [X]A3                     INT0/2[X] | Video Mode/Led Red
   Pad Port Trace 6 | [X]A4/SDA  RST SCK MISO     TX>1[ ] |
   Pad Port Trace 9 | [X]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |
				    |            [ ] [ ] [ ]              |
				    |  UNO_R3    GND MOSI 5V  ____________/
				    \_______________________/


                                    THE FOLLOWING ARE WRONG!

 * ------------
 * Arduino Nano
 * ------------
 *                                   +-----+
 *                      +------------| USB |------------+
 *                      |            +-----+            |
 * 	     (Built-in LED) | [ ]D13/SCK        MISO/D12[ ] |
 *                      | [ ]3.3V           MOSI/D11[X]~| LED Blue
 *                      | [ ]V.ref     ___    SS/D10[X]~| LED Green
 *       Pad Port Pin 1 | [X]A0       / N \       D9[X]~| LED Red
 *       Pad Port Pin 2 | [X]A1      /  A  \      D8[ ] |
 *            Reset Out | [X]A2      \  N  /      D7[ ] |
 *             Reset In | [X]A3       \_0_/       D6[X]~| Pad Port Pin 9
 *   JP3/4 (Video Mode) | [X]A4/SDA               D5[X]~| Pad Port Pin 6
 *     JP1/2 (Language) | [X]A5/SCL               D4[X] | Pad Port Pin 4
 *                      | [ ]A6              INT1/D3[X]~| Pad Port Pin 3
 *                      | [ ]A7              INT0/D2[X] | Pad Port Pin 7
 *                  +5V | [X]5V                  GND[X] | GND
 *                      | [ ]RST                 RST[ ] |
 *                      | [ ]GND   5V MOSI GND   TX1[ ] |
 *                      | [ ]Vin   [ ] [ ] [ ]   RX0[ ] |
 * 	                    |          [ ] [ ] [ ]          |
 *                      |          MISO SCK RST         |
 *                      | NANO-V3                       |
 *                      +-------------------------------+
 */

//~ #define RESET_IN_PIN A3
#define PAUSE_IN_PIN 5
#define PAUSE_OUT_PIN 4
#define RESET_OUT_PIN 3
#define VIDEOMODE_PIN 2
//~ #define MODE_LED_R_PIN 9
//~ #define MODE_LED_G_PIN 10
//~ #define PAD_LED_PIN LED_BUILTIN

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

#define ENABLE_SERIAL_DEBUG
//~ #define DEBUG_PAD

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
	MD_BTN_UP   =  1 << 0
};

// Master System Buttons - For internal use only
enum SmsButton {
	SMS_BTN_TH =    1 << 6,
	SMS_BTN_TR =    1 << 5,
	SMS_BTN_TL =    1 << 4,
	SMS_BTN_RIGHT = 1 << 3,
	SMS_BTN_LEFT =  1 << 2,
	SMS_BTN_DOWN =  1 << 1,
	SMS_BTN_UP   =  1 << 0,

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

/* Combos for video modes
 */
#define COMBO_50HZ MD_BTN_LEFT
#define COMBO_60HZ MD_BTN_RIGHT

#define PAD_USE_AB
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

/* Use a single led to indicate the video mode. This is enabled automatically
 * in place of the RGB led when low flash space is detected, but since this
 * does NOT disable the RGB led, it can be used together with it, provided that
 * you have a free pin.
 *
 * Basically, the single led is blinked 1-3 times according to which mode is set
 * (1 is EUR, see enum VideoMode below).
 */
//#define MODE_LED_SINGLE_PIN 3

/* Presses of the reset button longer than this amount of milliseconds will
 * switch to the next mode, shorter presses will reset the console.
 */
#define LONGPRESS_LEN 700

// Debounce duration for the reset/pause button
#define DEBOUNCE_MS 20

// Duration of the reset/pause pulse (milliseconds)
#define PULSE_LEN 250

// Microseconds
#define SIXMD_BTN_PULSE_INTERVAL 30

/*******************************************************************************
 * END OF SETTINGS
 ******************************************************************************/


#ifdef MODE_ROM_OFFSET
  #include <EEPROM.h>
#endif

enum VideoMode {
  VID_50HZ,
  VID_60HZ,
  VID_MODES_NO // Leave at end
};

enum PadType {
	PAD_SMS,		// Master System
	PAD_MD,			// Mega Drive/Genesis
	PAD_MD_6BTN		// Mega Drive/Genesis 6-Button
};

PadType padType = PAD_SMS;

VideoMode current_mode = VID_50HZ;

unsigned long mode_last_changed_time;


#ifdef ENABLE_SERIAL_DEBUG
	#define debug(...) Serial.print (__VA_ARGS__)
	#define debugln(...) Serial.println (__VA_ARGS__)
#else
	#define debug(...)
	#define debugln(...)
#endif


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
#ifdef ENABLE_MODE_LED_RGB
    byte c = 0;

#ifdef RGB_LED_COMMON_ANODE
    c = 255 - c;
#endif

#ifdef MODE_LED_R_PIN
    analogWrite (MODE_LED_R_PIN, c);
#endif

#ifdef MODE_LED_G_PIN
    analogWrite (MODE_LED_G_PIN, c);
#endif

#ifdef MODE_LED_B_PIN
    analogWrite (MODE_LED_B_PIN, c);
#endif

    // Keep off for a bit
    delay (200);

    // Turn led back on
    update_mode_leds ();
#endif  // ENABLE_MODE_LED_RGB

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
  //~ update_mode_leds ();

  mode_last_changed_time = millis ();
}

void change_mode (int increment) {
  // This also loops in [0, MODES_NO) backwards
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

  byte pressed_now = digitalRead (RESET_IN_PIN);
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
        debugln (F("Reset button hold"));
        ++hold_cycles;
        next_mode ();
      }
    }

    pressed_before = (pressed_now == LOW);
  }
#endif
}

// Pause is active low on SMS
void handle_pause_button () {
#ifdef PAUSE_IN_PIN
  static byte debounce_level = LOW;
  static bool pressed_before = false;
  static long last_int = 0, last_pressed = 0;
  static unsigned int hold_cycles = 0;

  byte pressed_now = digitalRead (PAUSE_IN_PIN);
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
        pause_console ();
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
#endif
}


void reset_console () {
	debugln (F("Resetting console"));

	digitalWrite (RESET_OUT_PIN, LOW);
	delay (PULSE_LEN);
	digitalWrite (RESET_OUT_PIN, HIGH);
}

void pause_console () {
	debugln (F("Pausing console"));

	digitalWrite (PAUSE_OUT_PIN, LOW);
	delay (PULSE_LEN);
	digitalWrite (PAUSE_OUT_PIN, HIGH);
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
	} else {
		// This is a SMS pad - Switch SELECT to INPUT with pull-up
		PDREG_SELECT_PORT &= ~(1 << PDREG_SELECT_BIT);
		setSelect (HIGH);

		padType = PAD_SMS;
		debugln (F("Detected Master System pad"));
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
 * The basic idea here is to make up a byte where each bit represents the state
 * of a button, where 1 means pressed, for commodity's sake. The bit-button
 * mapping is defined in the MdButton enum above.
 *
 * To get consistent readings, we should really read all of the pad pins at
 * once, since they must be interpreted according to the value of the SELECT
 * signal. In order to do this we try to connect all pins to a single port
 * of our MCU.
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

inline byte mdPadToSms (word mdPad) {
	byte smsPad = 0x00;

	smsPad |= (mdPad & MD_BTN_UP) ? SMS_BTN_UP : 0x00;
	smsPad |= (mdPad & MD_BTN_DOWN) ? SMS_BTN_DOWN : 0x00;
	smsPad |= (mdPad & MD_BTN_LEFT) ? SMS_BTN_LEFT : 0x00;
	smsPad |= (mdPad & MD_BTN_RIGHT) ? SMS_BTN_RIGHT : 0x00;

#ifdef PAD_USE_AB
	/* Normally SMS buttons 1 and 2 are mapped to B and C on the MD pad. But we
	 * can map them to A and B just as easily.
	 */
	smsPad |= (mdPad & MD_BTN_A) ? SMS_BTN_B1 : 0x00;
	smsPad |= (mdPad & MD_BTN_B) ? SMS_BTN_B2 : 0x00;

#ifdef PAD_USE_THIRD_BTN_AS_2BTNS
	if (mdPad & MD_BTN_C)
		smsPad |= SMS_BTN_B1 | SMS_BTN_B2;
#endif

#else
	// B -> B1, C -> B2
	smsPad |= (mdPad & MD_BTN_B) ? SMS_BTN_B1 : 0x00;
	smsPad |= (mdPad & MD_BTN_C) ? SMS_BTN_B2 : 0x00;

#ifdef PAD_USE_THIRD_BTN_AS_2BTNS
	if (mdPad & MD_BTN_A)
		smsPad |= SMS_BTN_B1 | SMS_BTN_B2;
#endif

#endif


	return smsPad;
}

#define IGNORE_COMBO_MS LONGPRESS_LEN

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

			if ((pad_status & COMBO_TRIGGER) == COMBO_TRIGGER && millis () - last_combo_time > IGNORE_COMBO_MS) {
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
			} else if (pad_status & MD_BTN_START) {
				// Pause console
				pause_console ();
			} else {
				// Send pad status to SMS
				byte smsPad = mdPadToSms (pad_status);
				write_sms_pad (smsPad);
			}

			break;
		}
	}
}

void setup () {
#ifdef ENABLE_SERIAL_DEBUG
	Serial.begin (9600);
#endif

	debugln (F("Starting up..."));

	// Enable reset (active low)
	pinMode (RESET_OUT_PIN, OUTPUT);
	digitalWrite (RESET_OUT_PIN, HIGH);

	// Setup leds
#ifdef MODE_LED_R_PIN
	pinMode (MODE_LED_R_PIN, OUTPUT);
#endif

#ifdef MODE_LED_G_PIN
	pinMode (MODE_LED_G_PIN, OUTPUT);
#endif

#ifdef MODE_LED_B_PIN
	pinMode (MODE_LED_B_PIN, OUTPUT);
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
	pinMode (PAUSE_IN_PIN, INPUT_PULLUP);
	pinMode (PAUSE_OUT_PIN, OUTPUT);
	digitalWrite (PAUSE_OUT_PIN, HIGH);

	// Finally release the reset line
	digitalWrite (RESET_OUT_PIN, HIGH);
}

void loop () {
	handle_reset_button ();
	handle_pause_button ();
	handle_pad ();
	save_mode ();
}
