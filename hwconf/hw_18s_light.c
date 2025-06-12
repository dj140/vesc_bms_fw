/*
	Copyright 2019 - 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC BMS firmware.

	The VESC BMS firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC BMS firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */
#include "bms_if.h"
#include "pwr.h"
#include "terminal.h"
#include "commands.h"
#include "ltc6813.h"
#include "main.h"
#include "sleep.h"
#include "comm_can.h"
#include <stdio.h>
#include <stdlib.h>
#include "shutdown.h"

static mutex_t shutdown_mutex;
static float bt_diff = 0.0;
static float bt_lastval = 0.0;
static float bt_unpressed = 0.0;
static bool will_poweroff = false;
static unsigned int bt_hold_counter = 0;



// Private functions
static void terminal_chg_en(int argc, const char **argv);
static void terminal_dsg_en(int argc, const char **argv);
static void terminal_buzzer_test(int argc, const char **argv);
static void terminal_shutdown_now(int argc, const char **argv);
static void terminal_shutdown_hold_on(int argc, const char **argv);
static void terminal_button_test(int argc, const char **argv);


void hw_board_init(void) {


	chMtxObjectInit(&shutdown_mutex);
	// ShutDown
	palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	// palSetLineMode(LINE_CAN_EN, PAL_MODE_OUTPUT_PUSHPULL);
	// palSetLineMode(LINE_CURR_MEASURE_EN, PAL_MODE_OUTPUT_PUSHPULL);
	shutdown_init();

	terminal_register_command_callback(
        "chg_en",
        "Enable charge input",
        "[en]",
        terminal_chg_en);

    terminal_register_command_callback(
        "dsg_en",
        "Enable discharge output",
        "[en]",
        terminal_dsg_en);

	terminal_register_command_callback(
        "buzzer_test",
        "Test the buzzer",
        NULL,
        terminal_buzzer_test);

    terminal_register_command_callback(
		"shutdown",
		"Shutdown VESC now.",
		0,
		terminal_shutdown_now);
		
	terminal_register_command_callback(
		"shutdown hold on",
		"Pull shutdown pin high",
		0,
		terminal_shutdown_hold_on);
		
	terminal_register_command_callback(
		"test_button",
		"Try sampling the shutdown button",
		0,
		terminal_button_test);
}

void hw_stay_awake(void) {

    // palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN);
}

#define RISING_EDGE_THRESHOLD 0.5
#define TIME_3S 300

/**
 * hw_sample_shutdown_button - return false if shutdown is requested, true otherwise
 *
 * Behavior: after determining the unpressed level, look for rising edges or values
 * that are clearly above the unpressed level (2 x Threshold higher), triggering a counter.
 *
 * Once triggered, the counter keeps incrementing as long as the level is 2 x Threshold higher
 * than the normal/unpressed value, otherwise it gets reset to zero.
 *
 * Once the counter reaches the threshold the button is considered pressed, provided that
 * the erpm is below 100. A very short (20ms) beep will go off.
 * Shutdown actually happens on the falling edge when the press is over.
 *
 * If the motor is spinning faster, then a 3s press is required. Buzzer will beep once the
 * time has been reached. Again, shutdown happens on the falling edge.
 *
 * Normal shutdown time:    0.5s
 * Emergency shutdown time: 3.0s
 */
uint8_t hw_button_v(void) {

    uint8_t newval = palReadPad(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN);
    return newval;

}
bool hw_sample_shutdown_button(void) {
    chMtxLock(&shutdown_mutex);
    uint8_t newval = palReadPad(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN);
    chMtxUnlock(&shutdown_mutex);
    if (bt_lastval == 0) {
        bt_lastval = newval;
        return true;
    }
    bt_diff = (newval - bt_lastval);

    bool is_steady = 1;  // filter out noise above 20mV
    bool is_rising_edge = bt_lastval;

    bt_lastval = newval;

    if (bt_unpressed == 0.0) {
        // initializing bt_unpressed
        if (is_steady) {
            bt_unpressed = newval;
        }
        // return true regardless (this happens only after boot)
        return true;
    }

    if (will_poweroff) {
        // Now we look for a falling edge to shut down
        if ((bt_diff < -RISING_EDGE_THRESHOLD) || (newval < bt_unpressed)) {
            bt_hold_counter++;
            BUZZER_OFF();
            return false;
        }
        return true;
    }

    // we've had a rising edge and are now checking for a steady hold
    if (is_rising_edge && bt_diff == 0) {
        bt_hold_counter++;

        if (bt_hold_counter  > TIME_3S) {
            will_poweroff = true;
            bt_hold_counter = 0;
            return true;
        }

    }
    else {
        // press is too short, abort
        bt_hold_counter = 0;
        BUZZER_OFF();
    }

    return true;
}


static void terminal_shutdown_now(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	HW_SHUTDOWN_HOLD_OFF();
}

static void terminal_shutdown_hold_on(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	//shutdown_set_sampling_disabled(true);
	palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	HW_SHUTDOWN_HOLD_ON();
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < 40;i++) {
		commands_printf("BT: %d:%d [%.2fV], %.2fV, %.2fV, OFF=%d", HW_SAMPLE_SHUTDOWN(), bt_hold_counter,
                        (double)bt_diff, (double)bt_unpressed, (double)bt_lastval, (int)will_poweroff);
		chThdSleepMilliseconds(100);
	}
}

static void terminal_chg_en(int argc, const char **argv) {
	if (argc == 2) {
		int en = -1;
		sscanf(argv[1], "%d", &en);

		if (en >= 0) {
			palWriteLine(LINE_BQ_CHG_EN, en ? 1 : 0);
			commands_printf("OK\n");
			return;
		}
	}

	commands_printf("Invalid arguments\n");
}

static void terminal_dsg_en(int argc, const char **argv) {
	if (argc == 2) {
		int en = -1;
		sscanf(argv[1], "%d", &en);

		if (en >= 0) {
			palWriteLine(LINE_BQ_DSG_EN, en ? 1 : 0);
			commands_printf("OK\n");
			return;
		}
	}

	commands_printf("Invalid arguments\n");
}


static void terminal_buzzer_test(int argc, const char **argv) {

	for (size_t i = 1; i < (size_t)argc; i++) {
		play_melody(argv[i]);
	}

}