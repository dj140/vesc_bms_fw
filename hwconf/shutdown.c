/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "shutdown.h"

#ifdef HW_SHUTDOWN_PIN
#include "buzzer.c"

// Private variables
bool volatile m_button_pressed = false;
static volatile float m_inactivity_time = 0.0;
static THD_WORKING_AREA(shutdown_thread_wa, 128);
static mutex_t m_sample_mutex;
static volatile bool m_init_done = false;
static volatile bool m_sampling_disabled = false;
static volatile bool m_shutdown_hold = false;

// Melodies
const char* MEL_JET_CONNECTED = "C5 E5 G5/2";
const char* MEL_JET_DISCONNECTED = "G5 E5 C5/2";
const char* MEL_ERROR = "C/1 P/1 C/1";

// Private functions
static THD_FUNCTION(shutdown_thread, arg);

void shutdown_init(void) {
	chMtxObjectInit(&m_sample_mutex);
	chThdCreateStatic(shutdown_thread_wa, sizeof(shutdown_thread_wa), NORMALPRIO, shutdown_thread, NULL);
	m_init_done = true;
}

// void shutdown_reset_timer(void) {
// 	m_inactivity_time = 0.0;
// }

// bool shutdown_button_pressed(void) {
// 	return m_button_pressed;
// }

// float shutdown_get_inactivity_time(void) {
// 	return m_inactivity_time;
// }

// void shutdown_set_sampling_disabled(bool disabled) {
// 	if (!m_init_done) {
// 		return;
// 	}

// 	chMtxLock(&m_sample_mutex);
// 	m_sampling_disabled = disabled;
// 	chMtxUnlock(&m_sample_mutex);
// }

// void shutdown_hold(bool hold) {
// 	m_shutdown_hold = hold;
// }

bool do_shutdown(bool resample) {
	chThdSleepMilliseconds(100);
	if (resample) {
		chMtxLock(&m_sample_mutex);
		play_melody(MEL_JET_DISCONNECTED);
		HW_SHUTDOWN_HOLD_OFF();
		chMtxUnlock(&m_sample_mutex);
	}
	return true;
}

static THD_FUNCTION(shutdown_thread, arg) {
	(void)arg;

	chRegSetThreadName("Shutdown");

	bool gates_disabled_here = false;
	float gate_disable_time = 0.0;
	systime_t last_iteration_time = chVTGetSystemTimeX();

	for(;;) {
		float dt = (float)chVTTimeElapsedSinceX(last_iteration_time) / (float)CH_CFG_ST_FREQUENCY;
		last_iteration_time = chVTGetSystemTimeX();

		chMtxLock(&m_sample_mutex);

		if (m_sampling_disabled) {
			chMtxUnlock(&m_sample_mutex);
			chThdSleepMilliseconds(10);
			continue;
		}

		bool sample = HW_SAMPLE_SHUTDOWN();
		chMtxUnlock(&m_sample_mutex);
		// bool clicked = m_button_pressed && !sample;
		m_button_pressed = !sample;

		// Note: When the gates are enabled, the push to start function
		// will prevent the regulator from shutting down. Therefore, the
		// gate driver has to be disabled.

		if (m_button_pressed) {
			gates_disabled_here = do_shutdown(true);
		}


		chThdSleepMilliseconds(10);
	}
}


#endif
