//------------------------------------------------------------------------------
// Copyright (C) 2011, Robert Johansson, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

#include <autoconf.h>
#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <stdio.h>
#include <strings.h>
#include <string.h>

#include "mbus-serial.h"
#include "mbus-protocol-aux.h"
#include "mbus-protocol.h"

#ifdef CONFIG_UART_1_NRF_TX_BUFFER_SIZE
# define MBUS_UART_FIFO_SIZE CONFIG_UART_1_NRF_TX_BUFFER_SIZE
#else
# error UART_1_NRF_TX_BUFFER_SIZE undefined
#endif

#define DEFAULT_BAUDRATE 2400
#define PACKET_BUFSZ     384
#define RING_BUFSZ       1024
#define UART_DEVICE      "mbus0"
#define MBUS_LOG_MODULE  mbus

LOG_MODULE_DECLARE(MBUS_LOG_MODULE);

static uint8_t ring_buffer[RING_BUFSZ];
static struct ring_buf ringbuf;

static const struct device *dev;
static int timeout;

static char   rx_buf[PACKET_BUFSZ];
static int    rx_pos;

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(mbus_mq, sizeof(mbus_frame), 10, 4);

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
		if (rc < 0) {
		    LOG_ERR("Failed parsing M-Bus frame, rc %d: %s", rc, mbus_error_str());
		    rx_pos = 0;
		}
		if (rc == 0) {
		    LOG_HEXDUMP_DBG(rx_buf, rx_pos, "RX IN");

		    /* if queue is full, message is silently dropped */
		    k_msgq_put(&mbus_mq, &frame, K_NO_WAIT);

		    /* reset the buffer (it was copied to the msgq) */
		    rx_pos = 0;
		}
	    } else {
		LOG_ERR("Frame overflow, dropping.");
		rx_pos = 0;
	    }
	}

	if (uart_irq_tx_ready(dev)) {
	    char buf[MBUS_UART_FIFO_SIZE];
	    size_t rlen, slen;

	    rlen = ring_buf_get(&ringbuf, buf, sizeof(buf));
	    if (!rlen) {
		LOG_DBG("Ring buffer empty, disable TX IRQ");
		uart_irq_tx_disable(dev);
		continue;
	    }

	    slen = uart_fifo_fill(dev, buf, rlen);
	    if (slen < rlen)
		LOG_ERR("Drop %zd bytes", rlen - slen);
	    LOG_HEXDUMP_DBG(buf, slen, "TX OUT");
	}
    }
}

int mbus_serial_connect(mbus_handle *handle)
{
    if (handle == NULL)
        return -1;

    dev = device_get_binding(UART_DEVICE);
    if (!dev) {
	LOG_ERR("cannot find device tree node %s", UART_DEVICE);
	return -1;
    }

    ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
    uart_irq_callback_user_data_set(dev, uart_isr, NULL);
    uart_irq_rx_enable(dev);

    return mbus_serial_set_baudrate(handle, DEFAULT_BAUDRATE);
}

int mbus_serial_set_baudrate(mbus_handle *handle, long baudrate)
{
    struct uart_config uc = {
	.baudrate  = baudrate,
	.data_bits = UART_CFG_DATA_BITS_8,
	.parity    = UART_CFG_PARITY_NONE, //UART_CFG_PARITY_EVEN,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    int rc;

    if (handle == NULL)
        return -1;

    rc = uart_configure(dev, &uc);
    if (rc < 0) {
	LOG_ERR("failed setting up M-Bus UART %ld 8E1, error %d", baudrate, -rc);
	return -1;
    }

    if (baudrate <= 38400)
	timeout = 3410000 / baudrate;
    else
	timeout = 3410000 / 38400;

    return 0;
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
        LOG_ERR("mbus_frame_pack failed");
        return -1;
    }

    rlen = ring_buf_put(&ringbuf, buf, len);
    if (rlen < len)
	LOG_ERR("Drop %zd bytes", len - rlen);

    if (rlen == 0)
	return -1;

    uart_irq_tx_enable(dev);

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

/**
 * Local Variables:
 *  indent-tabs-mode: nil
 *  c-file-style: "stroustrup"
 * End:
 */
