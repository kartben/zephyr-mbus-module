/* Zephyr serial backend to libmbus
 *
 * Copyright (C) 2022  Addiva Elektronik AB
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <autoconf.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/ring_buffer.h>

#include <stdio.h>
#include <strings.h>
#include <string.h>

#include "mbus/mbus-serial.h"
#include "mbus/mbus-protocol-aux.h"
#include "mbus/mbus-protocol.h"

#ifdef CONFIG_UART_1_NRF_TX_BUFFER_SIZE
# define MBUS_UART_FIFO_SIZE CONFIG_UART_1_NRF_TX_BUFFER_SIZE
#else
# error UART_1_NRF_TX_BUFFER_SIZE undefined
#endif

#define DEFAULT_BAUDRATE 2400
#define DEFAULT_PARITY   UART_CFG_PARITY_EVEN
#define PACKET_BUFSZ     384
#define RING_BUFSZ       1024
#define UART_DEVICE      "mbus0"
#define MBUS_LOG_MODULE  mbus

LOG_MODULE_DECLARE(MBUS_LOG_MODULE);

static uint8_t ring_buffer[RING_BUFSZ];
static struct ring_buf ringbuf;

static const struct device *dev;
static int timeout    = (3410000 / DEFAULT_BAUDRATE);
static int c_baudrate = DEFAULT_BAUDRATE;
static int c_parity   = DEFAULT_PARITY;

static char   rx_buf[PACKET_BUFSZ];
static int    rx_pos;

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(mbus_mq, sizeof(mbus_frame), 10, 4);

/*
 * ISR
 */
static void uart_isr(const struct device *dev, void *data)
{
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
	if (uart_irq_rx_ready(dev)) {
	    uint8_t c;

	    if (uart_fifo_read(dev, &c, 1) != 1)
		return;

	    if (rx_pos < sizeof(rx_buf)) {
		mbus_frame frame;
		int rc;

		rx_buf[rx_pos++] = c;

		rc = mbus_parse(&frame, rx_buf, rx_pos);
		if (rc < 0)
		    rx_pos = 0;

		if (rc == 0) {
		    LOG_HEXDUMP_DBG(rx_buf, rx_pos, "RX IN");

		    /* if queue is full, message is silently dropped */
		    k_msgq_put(&mbus_mq, &frame, K_NO_WAIT);

		    /* reset the buffer (it was copied to the msgq) */
		    rx_pos = 0;
		}
	    } else {
		mbus_error_str_set("frame overflow, dropping.");
		rx_pos = 0;
	    }
	}

	if (uart_irq_tx_ready(dev)) {
	    char buf[MBUS_UART_FIFO_SIZE];
	    size_t rlen, slen;

	    rlen = ring_buf_get(&ringbuf, buf, sizeof(buf));
	    if (!rlen) {
		uart_irq_tx_disable(dev);
		continue;
	    }

	    slen = uart_fifo_fill(dev, buf, rlen);
	    if (slen < rlen)
		mbus_error_str_set("drop %zd bytes", rlen - slen);
	    LOG_HEXDUMP_DBG(buf, slen, "TX OUT");
	}
    }
}

static int serial_line_set(mbus_handle *handle)
{
    struct uart_config uc = {
	.baudrate  = c_baudrate,
	.data_bits = UART_CFG_DATA_BITS_8,
	.parity    = c_parity,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    int rc;

    if (handle == NULL)
        return -1;

    rc = uart_configure(dev, &uc);
    if (rc < 0) {
        mbus_error_str_set("failed adjusting line settings %d 8%c1, error %d.",
                           c_baudrate, c_parity == UART_CFG_PARITY_EVEN
                           ? 'E' : 'N', rc);
	return -1;
    }

    if (c_baudrate <= 38400)
	timeout = 3410000 / c_baudrate;
    else
	timeout = 3410000 / 38400;

    settings_save_one("mbus/line/speed", &c_baudrate, sizeof(c_baudrate));
    settings_save_one("mbus/line/parity", &c_parity, sizeof(c_parity));

    return 0;
}

int mbus_serial_diag(mbus_handle *handle, char *buf, size_t len)
{
    struct uart_config uc;

    if (uart_config_get(dev, &uc))
        return -1;

    snprintf(buf, len, "%u %d%c%d", uc.baudrate, uc.data_bits + 5,
             (uc.parity == UART_CFG_PARITY_NONE ? 'N' :
              (uc.parity == UART_CFG_PARITY_EVEN ? 'E' : 'O')),
              uc.stop_bits == UART_CFG_STOP_BITS_1 ? 1 : 2);
    return 0;
}

int mbus_serial_connect(mbus_handle *handle)
{
    if (handle == NULL)
        return -1;

    dev = device_get_binding(UART_DEVICE);
    if (!dev) {
	mbus_error_str_set("cannot find device tree node %s", UART_DEVICE);
	return -1;
    }

    ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
    uart_irq_callback_user_data_set(dev, uart_isr, NULL);
    uart_irq_rx_enable(dev);

    return serial_line_set(handle);
}

int mbus_serial_set_baudrate(mbus_handle *handle, long baudrate)
{
    c_baudrate = baudrate;

    return serial_line_set(handle);
}

int mbus_serial_set_parity(mbus_handle *handle, int parity)
{
    switch (parity) {
    case 0:
        c_parity = UART_CFG_PARITY_NONE;
        break;
    case 1:
        c_parity = UART_CFG_PARITY_ODD;
        break;
    case 2:
        c_parity = UART_CFG_PARITY_EVEN;
        break;
    default:
        return -1;
    }

    return serial_line_set(handle);
}

int mbus_serial_reset(mbus_handle *handle)
{
    c_baudrate = DEFAULT_BAUDRATE;
    c_parity   = DEFAULT_PARITY;

    return serial_line_set(handle);
}

int mbus_serial_disconnect(mbus_handle *handle)
{
    if (handle == NULL)
        return -1;

    return 0;
}

void mbus_serial_data_free(mbus_handle *handle)
{
    mbus_serial_data *serial_data;

    if (handle)
    {
        serial_data = (mbus_serial_data *)handle->auxdata;
        if (serial_data == NULL)
            return;

        free(serial_data);
        handle->auxdata = NULL;
    }
}

int mbus_serial_send_frame(mbus_handle *handle, mbus_frame *frame)
{
    unsigned char buf[PACKET_BUFSZ];
    int len, rlen;

    if (handle == NULL || frame == NULL)
        return -1;

    len = mbus_frame_pack(frame, buf, sizeof(buf));
    if (len == -1) {
	mbus_error_str_set("M-Bus serial failed mbus_frame_pack(), ret %d", len);
        return -1;
    }

    rlen = ring_buf_put(&ringbuf, buf, len);
    if (rlen < len) {
        mbus_error_str_set("M-Bus serial failed sending complete frame, dropped %zd bytes", len - rlen);
        rlen = 0;
    }

    uart_irq_tx_enable(dev);

    if (rlen == 0)
	return -1;

    return 0;
}

int mbus_serial_recv_frame(mbus_handle *handle, mbus_frame *frame)
{
    int rc;

    if (handle == NULL || frame == NULL)
        return MBUS_RECV_RESULT_ERROR;

    /* read data until a packet is received */
    rc = k_msgq_get(&mbus_mq, frame, K_MSEC(timeout));
    switch (rc) {
    case 0:
	break;

    case -ENOMSG:
        return MBUS_RECV_RESULT_ERROR;

    case -EAGAIN:
        return MBUS_RECV_RESULT_TIMEOUT;
    }

    return MBUS_RECV_RESULT_OK;
}

/*
 * Settings
 */
static int set(const char *name, const char *key, void *data, int len, settings_read_cb cb, void *arg)
{
    const char *next;

    if (settings_name_steq(name, key, &next) && !next) {
        (void)cb(arg, data, len);
        LOG_HEXDUMP_INF(data, len, name);
        return 0;
    }

    return -1;
}

#define SET_TO(setting, dst) \
    (set(name, setting, &dst, sizeof(dst), read_cb, cb_arg) == 0)

static int mbus_line_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (SET_TO("speed", c_baudrate))
        return 0;
    if (SET_TO("parity", c_parity))
        return 0;

    return -1;
}

SETTINGS_STATIC_HANDLER_DEFINE(mbus, "mbus/line", NULL, mbus_line_set, NULL, NULL);

/**
 * Local Variables:
 *  indent-tabs-mode: nil
 *  c-file-style: "stroustrup"
 * End:
 */
