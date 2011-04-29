/** \file
	\brief Main file - this is where it all starts, and ends
*/

/** \mainpage Teacup Reprap Firmware
	\section intro_sec Introduction
		Teacup Reprap Firmware (originally named FiveD on Arduino) is a firmware package for numerous reprap electronics sets.

		Please see README for a full introduction and long-winded waffle about this project
	\section install_sec	Installation
		\subsection step1 Step 1: Download
			\code git clone git://github.com/triffid/Teacup_Firmware \endcode
		\subsection step2 Step 2: configure
			\code cp config.[yourboardhere].h config.h \endcode
			Edit config.h to suit your machone
			Edit Makefile to select the correct chip and programming settings
		\subsection step3 Step 3: Compile
			\code make \endcode
			\code make program \endcode
		\subsection step4 Step 4: Test!
			\code ./func.sh mendel_reset
			./func.sh mendel_talk
			M115
			ctrl+d \endcode
*/

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"config.h"
#include	"fuses.h"

#include	"serial.h"
#include	"dda_queue.h"
#include	"dda.h"
#include	"gcode_parse.h"
#include	"timer.h"
#include	"temp.h"
#include	"sermsg.h"
#include	"watchdog.h"
#include	"debug.h"
#include	"sersendf.h"
#include	"heater.h"
#include	"analog.h"
#include	"pinio.h"
#include	"arduino.h"
#include	"clock.h"
#include	"intercom.h"

#ifdef SD
	#include	"diskio.h"
	#include	"ff.h"
	#include	"sd.h"

	// see sd.h for flag values
	volatile uint8_t sdflags;

	uint8_t sdbuffer[32];
	FATFS fatfs;
	FIL file;
	FILINFO fileinfo;
	DIR dir;
	FRESULT fr;

	DWORD get_fattime() {
		/*	uint32_t packed;

		uint8_t year = (2011 - 1980);
		uint8_t month = 4;
		uint8_t day = 28;
		uint8_t hour = 21;
		uint8_t minute = 20;
		uint8_t second = 34;

		packed = (((uint32_t) year) << 25) | ((month & 15UL) << 21) | ((day & 31UL) << 16) | ((hour & 31UL) << 11) | ((minute & 63UL) << 5) | ((second & 63UL) << 0);
		return packed;*/
		return ((2011UL - 1980UL) << 25) | (4UL << 21) | (28UL << 16) | (21UL << 11) | (20UL << 5) | 34UL;
	}
#endif /* SD */

/// initialise all I/O - set pins as input or output, turn off unused subsystems, etc
void io_init(void) {
	// disable modules we don't use
	#ifdef PRR
		PRR = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
	#elif defined PRR0
		PRR0 = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
		#if defined(PRUSART3)
			// don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
			PRR1 |= MASK(PRUSART3) | MASK(PRUSART2);
		#endif
		#if defined(PRUSART2)
			// don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
			PRR1 |= MASK(PRUSART2);
		#endif
	#endif
	ACSR = MASK(ACD);

	// setup I/O pins

	// X Stepper
	WRITE(X_STEP_PIN, 0);	SET_OUTPUT(X_STEP_PIN);
	WRITE(X_DIR_PIN,  0);	SET_OUTPUT(X_DIR_PIN);
	#ifdef X_MIN_PIN
		SET_INPUT(X_MIN_PIN);
		WRITE(X_MIN_PIN, 0); // pullup resistors off
	#endif
	#ifdef X_MAX_PIN
		SET_INPUT(X_MAX_PIN);
		WRITE(X_MAX_PIN, 0); // pullup resistors off
	#endif

	// Y Stepper
	WRITE(Y_STEP_PIN, 0);	SET_OUTPUT(Y_STEP_PIN);
	WRITE(Y_DIR_PIN,  0);	SET_OUTPUT(Y_DIR_PIN);
	#ifdef Y_MIN_PIN
		SET_INPUT(Y_MIN_PIN);
		WRITE(Y_MIN_PIN, 0); // pullup resistors off
	#endif
	#ifdef Y_MAX_PIN
		SET_INPUT(Y_MAX_PIN);
		WRITE(Y_MAX_PIN, 0); // pullup resistors off
	#endif

	// Z Stepper
	#if defined Z_STEP_PIN && defined Z_DIR_PIN
		WRITE(Z_STEP_PIN, 0);	SET_OUTPUT(Z_STEP_PIN);
		WRITE(Z_DIR_PIN,  0);	SET_OUTPUT(Z_DIR_PIN);
	#endif
	#ifdef Z_MIN_PIN
		SET_INPUT(Z_MIN_PIN);
		WRITE(Z_MIN_PIN, 0); // pullup resistors off
	#endif
	#ifdef Z_MAX_PIN
		SET_INPUT(Z_MAX_PIN);
		WRITE(Z_MAX_PIN, 0); // pullup resistors off
	#endif

	#if defined E_STEP_PIN && defined E_DIR_PIN
		WRITE(E_STEP_PIN, 0);	SET_OUTPUT(E_STEP_PIN);
		WRITE(E_DIR_PIN,  0);	SET_OUTPUT(E_DIR_PIN);
	#endif

	// Common Stepper Enable
	#ifdef STEPPER_ENABLE_PIN
		#ifdef STEPPER_INVERT_ENABLE
			WRITE(STEPPER_ENABLE_PIN, 0);
		#else
			WRITE(STEPPER_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(STEPPER_ENABLE_PIN);
	#endif

	// X Stepper Enable
	#ifdef X_ENABLE_PIN
		#ifdef X_INVERT_ENABLE
			WRITE(X_ENABLE_PIN, 0);
		#else
			WRITE(X_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(X_ENABLE_PIN);
	#endif

	// Y Stepper Enable
	#ifdef Y_ENABLE_PIN
		#ifdef Y_INVERT_ENABLE
			WRITE(Y_ENABLE_PIN, 0);
		#else
			WRITE(Y_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(Y_ENABLE_PIN);
	#endif

	// Z Stepper Enable
	#ifdef Z_ENABLE_PIN
		#ifdef Z_INVERT_ENABLE
			WRITE(Z_ENABLE_PIN, 0);
		#else
			WRITE(Z_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(Z_ENABLE_PIN);
	#endif

	// E Stepper Enable
	#ifdef E_ENABLE_PIN
		#ifdef E_INVERT_ENABLE
			WRITE(E_ENABLE_PIN, 0);
		#else
			WRITE(E_ENABLE_PIN, 1);
		#endif
		SET_OUTPUT(E_ENABLE_PIN);
	#endif

	#ifdef	STEPPER_ENABLE_PIN
		power_off();
	#endif

	#ifdef	TEMP_MAX6675
		// setup SPI
		WRITE(SCK, 0);				SET_OUTPUT(SCK);
		WRITE(MOSI, 1);				SET_OUTPUT(MOSI);
		WRITE(MISO, 1);				SET_INPUT(MISO);
		WRITE(SS, 1);					SET_OUTPUT(SS);
	#endif

	#ifdef TEMP_INTERCOM
		// Enable the RS485 transceiver
		SET_OUTPUT(RX_ENABLE_PIN);
		SET_OUTPUT(TX_ENABLE_PIN);
		WRITE(RX_ENABLE_PIN,0);
		disable_transmit();
	#endif
}

/// Startup code, run when we come out of reset
void init(void) {
	// set up watchdog
	wd_init();

	// set up serial
	serial_init();

	// set up G-code parsing
	gcode_init();

	// set up inputs and outputs
	io_init();

	// set up timers
	timer_init();

	// read PID settings from EEPROM
	heater_init();

	// set up dda
	dda_init();

	// start up analog read interrupt loop,
	// if any of the temp sensors in your config.h use analog interface
	analog_init();

	// set up temperature inputs
	temp_init();

	#ifdef SD
		sdflags = 0;
		disk_initialize(0);
		fr = f_mount(0, &fatfs);
		if (fr == FR_OK)
			sdflags = SDFLAG_MOUNTED;
	#endif

	// enable interrupts
	sei();

	// reset watchdog
	wd_reset();

	// say hi to host
	serial_writestr_P(PSTR("start\nok\n"));

}

/// this is where it all starts, and ends
///
/// just run init(), then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char() and check the clocks
int main (void)
{
	uint8_t c = 13;
	init();

	// main loop
	for (;;)
	{
		// if queue is full, no point in reading chars- host will just have to wait
		if ((serial_rxchars() != 0) && (queue_full() == 0)) {
			c = serial_popchar();

			#ifdef SD
			if (sdflags & SDFLAG_WRITING) {
				UINT n;
				fr = f_write(&file, &c, 1, &n);
				if ((n != 1) || (fr != FR_OK)) {
					// todo: spit an error or something
					sdflags &= ~SDFLAG_WRITING;
				}
			}
			#endif

			// todo: prevent moves being executed when loading to SD, in a way that allows us to precalculate and save some data eg; a D word for distance or a C word for c0, etc
			gcode_parse_char(c);
		}

		#ifdef SD
		if (c == 13) {
			if (queue_full() == 0) {
				if (sdflags & SDFLAG_READING) {
					UINT n;
					do {
						fr = f_read(&file, &c, 1, &n);
						if ((n == 1) && (fr == FR_OK)) {
							gcode_parse_char(c);
						}
						else {
							sdflags &= ~SDFLAG_READING;
						}
					} while (c != 13);
				}
			}
		}
		#endif

		clock();
	}
}
