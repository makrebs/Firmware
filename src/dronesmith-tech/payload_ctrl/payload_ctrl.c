/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Dronesmith Technologies. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>
#include <fcntl.h>
#include <termios.h>

#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_cmd.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

static unsigned int error_counter = 0;
static unsigned int write_counter = 0;

#define SERIAL_DEV			"/dev/ttyS1" // External serial
#define SERIAL_BAUD			57600u

/**
 * daemon management function.
 */
__EXPORT int payload_ctrl_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int payload_ctrl_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("payload ctrl",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
					     2000,
					     px4_daemon_thread_main,
					     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\tRunning");
			warnx("\tWrites: %u", write_counter);
			warnx("\tErrors: %u", error_counter);
		} else {
			warnx("\tNot started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int px4_daemon_thread_main(int argc, char *argv[])
{

	warnx("[payload] starting\n");

	int serial_fd;

	/* subscribe to payload_cmd topic */
	int payload_cmd_sub_fd = orb_subscribe(ORB_ID(payload_cmd));
	orb_set_interval(payload_cmd_sub_fd, 1000);

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[1];
	fds[0].fd = payload_cmd_sub_fd;
	fds[0].events = POLLIN;

	/* open uart */
	serial_fd = open(SERIAL_DEV, O_RDWR | O_NOCTTY);

	if (serial_fd < 0) {
		warnx("[payload] ERR Could not open serial device!");
		return serial_fd;
	}

	// reset state
	error_counter = 0;
	write_counter = 0;

	/* Try to set baud rate */
	struct termios uart_config;
	// struct termios* uart_config_original = NULL;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	// if ((termios_state = tcgetattr(serial_fd, uart_config_original)) < 0) {
	// 	warnx("[payload] ERR GET CONF %s: %d\n", SERIAL_DEV, termios_state);
	// 	close(serial_fd);
	// 	return -1;
	// }

	/* Fill the struct for the new configuration */
	tcgetattr(serial_fd, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, SERIAL_BAUD) < 0 || cfsetospeed(&uart_config, SERIAL_BAUD) < 0) {
		warnx("[payload] ERR SET BAUD %s\n", SERIAL_DEV);
		close(serial_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("[payload] ERR SET CONF %s\n", SERIAL_DEV);
		close(serial_fd);
		return -1;
	}

	thread_running = true;

	while (!thread_should_exit) {

		int poll_ret = poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			// printf("[payload] Got no data within a second\n");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[payload] ERROR return value from poll(): %d\n"
							 , poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct payload_cmd_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(payload_cmd), payload_cmd_sub_fd, &raw);
				// printf("[payload] Command:\n");
				//
				// for (int i = 0; i < 20; ++i) {
				// 	printf("%X ", raw.command[i]);
				// }
				// printf("\n");

				// send cmd to payload
				write_counter++;
				int ret = write(serial_fd, &raw.command, 64);

				if (ret != 64) {
					printf("[payload] Error on write: %d\n", ret);
					error_counter++;
				}
			}
		}

		// sleep for 10ms
		usleep(10000);
	}

	warnx("[payload] exiting.\n");

	thread_running = false;

	return 0;
}
