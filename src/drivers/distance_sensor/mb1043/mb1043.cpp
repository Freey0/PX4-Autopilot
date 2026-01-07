/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "mb1043.hpp"

#include <lib/drivers/device/Device.hpp>
#include <cctype>

MB1043::MB1043(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_RNG_DEVTYPE_MB12XX);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND);

	_px4_rangefinder.set_min_distance(MB1043_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(MB1043_MAX_DISTANCE);
}

MB1043::~MB1043()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int MB1043::init()
{
	start();

	return PX4_OK;
}

int MB1043::collect()
{
    const hrt_abstime timestamp_sample = hrt_absolute_time();

    // Read all available bytes from the serial port
    char c;
    while (::read(_file_descriptor, &c, 1) > 0) {

        switch (_parse_state) {
        case ParseState::WAITING_FOR_R:
            // Look for the start of a frame
            if (c == 'R') {
                _linebuf_len = 0;
                _parse_state = ParseState::READING_DIGITS;
                perf_begin(_sample_perf); // Start performance counter
            }
            break;

        case ParseState::READING_DIGITS:
            if (isdigit(c)) {
                // Append digit to buffer
                _linebuf[_linebuf_len++] = c;

                // Check if we have received all 4 digits
                if (_linebuf_len >= 4) {
                    _linebuf[4] = '\0'; // Null-terminate the string
                    int distance_mm = atoi(_linebuf);
                    float distance_m = static_cast<float>(distance_mm) * 1e-3f;

                    // Validate and publish
                    if (distance_m >= MB1043_MIN_DISTANCE && distance_m <= MB1043_MAX_DISTANCE) {
                        _px4_rangefinder.update(timestamp_sample, distance_m);
                        perf_end(_sample_perf);
                    } else {
                        perf_count(_comms_errors);
                        perf_cancel(_sample_perf);
                    }

                    // Reset state machine to wait for the next frame
                    _parse_state = ParseState::WAITING_FOR_R;
                    return PX4_OK; // A frame has been processed
                }

            } else {
                // Protocol error: expected a digit but got something else. Reset.
                _parse_state = ParseState::WAITING_FOR_R;
                perf_count(_comms_errors);
                perf_cancel(_sample_perf);
            }
            break;
        }
    }

    // No full frame was received in this cycle
    return -EAGAIN;
}

int MB1043::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_file_descriptor)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	termios uart_config{};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Input flags - Turn off input processing:
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |  INLCR | IGNCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// No parity, one stop bit, disable flow control.
	uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);

	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);
	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);
	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);
	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void MB1043::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void MB1043::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(MB1043_MEASURE_INTERVAL, 0);
}

void MB1043::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);

	// Clear the work queue schedule.
	ScheduleClear();
}

void MB1043::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
