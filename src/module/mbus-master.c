/* M-Bus master module for Zephyr
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
#define MODULE mbus

#include <zephyr/zephyr.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#if CONFIG_CAF
#include <caf/events/module_state_event.h>
#endif
#include <posix/unistd.h>
#include <stdio.h>

#include "mbus/mbus.h"

#define MBUS_DEVICE  "mbus0"
#define NELEMS(v)    (sizeof(v) / sizeof(v[0]))
#define ENABLED(v)   v ? "enabled" : "disabled"

#define shell_dbg(s,f,a...)  if (debug) shell_print(s, f, ##a);
#define err(fmt, args...)  { if (shell) shell_error(shell, fmt, ##args); else LOG_ERR(fmt, ##args); }
#define wrn(fmt, args...)  { if (shell) shell_warn(shell, fmt, ##args);  else LOG_WRN(fmt, ##args); }
#define dbg(fmt, args...)  { if (shell) {shell_dbg(shell, fmt, ##args);} else LOG_DBG(fmt, ##args); }
#define log(fmt, args...)  { if (shell) shell_info(shell, fmt, ##args);  else LOG_INF(fmt, ##args); }

LOG_MODULE_REGISTER(MODULE);

/*
 * Local forward declarations
 */
extern int mbus_serial_diag(mbus_handle *handle, char *buf, size_t len);

/*
 * Work queue for processing M-Bus commands when using nRF CAF
 */
#if CONFIG_CAF
#define K_WORK_MODULE_NAME &(struct k_work_queue_config){STRINGIFY(MODULE)}

static K_THREAD_STACK_DEFINE(wq_stack, 16384);
static struct k_work_q wq;

/* Unlikely we'll use need >5 args in this module */
struct shell_cmd {
    struct k_work       work;
    int               (*cb)(const struct shell *, int, char **);
    const struct shell *sh;
    int                 argc;
    char               *argv[5];
};
#endif /* CONFIG_CAF */

/*
 * Module runtime state variables
 */
static mbus_handle *handle;
static int          debug;
static int          verbose;
static int          interactive;
static int          xml;

static int init_slaves(const struct shell *shell)
{
    log("sending ping frames to wake up devices ...");
    if (mbus_send_ping_frame(handle, MBUS_ADDRESS_NETWORK_LAYER, 1) == -1)
	goto fail;

    if (mbus_send_ping_frame(handle, MBUS_ADDRESS_BROADCAST_NOREPLY, 1) == -1)
	goto fail;

    return 0;
fail:
    err("Failed initializing M-Bus slaves (SND_NKE)");
    return -1;
}

static int secondary_select(const struct shell *shell, char *mask)
{
    log("sending secondary select for mask %s", mask);

    switch (mbus_select_secondary_address(handle, mask)) {
    case MBUS_PROBE_COLLISION:
	wrn("address mask [%s] matches more than one device.", mask);
	return -1;
    case MBUS_PROBE_NOTHING:
	wrn("address mask [%s] does not match any device.", mask);
	return -1;
    case MBUS_PROBE_ERROR:
	err("failed selecting secondary address [%s].", mask);
	return -1;
    case MBUS_PROBE_SINGLE:
	dbg("address mask [%s] matches a single device.", mask);
	break;
    }

    return MBUS_ADDRESS_NETWORK_LAYER;
}

static int parse_addr(const struct shell *shell, char *args)
{
    int address;

    if (!args) {
	log("missing required argument, address, can be primary or secondary.");
	return 1;
    }

    if (init_slaves(shell))
	return 1;

    if (mbus_is_secondary_address(args)) {
	if (secondary_select(shell, args) == -1)
	    return 1;
	address = MBUS_ADDRESS_NETWORK_LAYER;
    } else {
	address = atoi(args);
	if (address < 1 || address > 255) {
	    log("invalid primary address %s.", args);
	    return 1;
	}
    }
    dbg("using primary address %d ...", address);

    return address;
}

static int query_device(const struct shell *shell, int argc, char *argv[])
{
    mbus_frame_data data;
    mbus_frame reply;
    char *addr_arg;
    int address;

    if (argc < 2) {
	log("usage: request ADDR [ID]");
	return -1;
    }

    addr_arg = argv[1];
    address = parse_addr(shell, addr_arg);
    log("sending request frame to primary address %d", address);
    if (mbus_send_request_frame(handle, address) == -1) {
	err("failed sending M-Bus request to %d.", address);
	return 1;
    }

    if (mbus_recv_frame(handle, &reply) != MBUS_RECV_RESULT_OK) {
	err("failed receiving M-Bus response from %d.", address);
	return 1;
    }

    if (argc < 3 && !verbose && !xml) {
	mbus_hex_dump("RAW:", (const char *)reply.data, reply.data_size);
	return 0;
    }

    if (mbus_frame_data_parse(&reply, &data) == -1) {
	err("M-bus data parse error: %s", mbus_error_str());
	return 1;
    }

    if (argc < 3) {
	/* Dump entire response as XML */
	if (xml) {
	    char *xml_data;

	    if (!(xml_data = mbus_frame_data_xml(&data))) {
		err("failed generating XML output of M-BUS response: %s", mbus_error_str());
		return 1;
	    }

	    printf("%s", xml_data);
	    free(xml_data);
	} else {
	    mbus_frame_data_print(&data);
	}

	if (data.data_var.record)
	    mbus_data_record_free(data.data_var.record);
    } else {
	int record_id;

	/* Query for a single record */
	record_id = atoi(argv[2]);

	if (data.type == MBUS_DATA_TYPE_FIXED) {
	    /* TODO: Implement this -- Not fixed in BCT --Jachim */
	}
	if (data.type == MBUS_DATA_TYPE_VARIABLE) {
	    mbus_data_record *entry;
	    mbus_record *record;
	    int i;

	    for (entry = data.data_var.record, i = 0; entry; entry = entry->next, i++) {
		dbg("record ID %d DIF %02x VID %02x", i,
                    entry->drh.dib.dif & MBUS_DATA_RECORD_DIF_MASK_DATA,
                    entry->drh.vib.vif & MBUS_DIB_VIF_WITHOUT_EXTENSION);
	    }

	    for (entry = data.data_var.record, i = 0; entry && i < record_id; entry = entry->next, i++)
		;

	    if (i != record_id) {
		if (data.data_var.record)
		    mbus_data_record_free(data.data_var.record);

		mbus_frame_free(reply.next);
		return 1;
	    }

	    record = mbus_parse_variable_record(entry);
	    if (record) {
		if (record->is_numeric)
		    printf("%lf", record->value.real_val);
		else
		    printf("%s", record->value.str_val.value);
		if (verbose)
		    printf(" %s\n", record->unit);
		else
		    printf("\n");
	    }

	    if (data.data_var.record)
		mbus_data_record_free(data.data_var.record);
	}
	mbus_frame_free(reply.next);
    }

    return 0;
}

static int ping_address(const struct shell *shell, mbus_frame *reply, int address)
{
    int i, rc = MBUS_RECV_RESULT_ERROR;

    memset(reply, 0, sizeof(mbus_frame));

    for (i = 0; i <= handle->max_search_retry; i++) {
	dbg("%d ", address);

	if (mbus_send_ping_frame(handle, address, 0) == -1) {
	    wrn("scan failed sending ping frame: %s", mbus_error_str());
	    return MBUS_RECV_RESULT_ERROR;
	}

	rc = mbus_recv_frame(handle, reply);
	if (rc != MBUS_RECV_RESULT_TIMEOUT)
	    return rc;
    }

    return rc;
}

static int mbus_scan_1st_address_range(const struct shell *shell)
{
    int address;
    int rc = 1;

    for (address = 0; address <= MBUS_MAX_PRIMARY_SLAVES; address++) {
	mbus_frame reply;
	int rc;

	rc = ping_address(shell, &reply, address);
	if (rc == MBUS_RECV_RESULT_TIMEOUT)
	    continue;

	if (rc == MBUS_RECV_RESULT_INVALID) {
	    mbus_purge_frames(handle);
	    wrn("collision at address %d.", address);
	    continue;
	}

	if (mbus_frame_type(&reply) == MBUS_FRAME_TYPE_ACK) {
	    if (mbus_purge_frames(handle)) {
		wrn("collision at address %d.", address);
		continue;
	    }

	    log("found an M-Bus device at address %d.", address);
	    rc = 0;
	}
    }

    return rc;
}

static int cmd_ping(const struct shell *shell, int argc, char *argv[])
{
    mbus_frame reply;
    int address;
    int rc;

    address = atoi(argv[1]);
    if (address < 0 || address >= 250) {
        shell_warn(shell, "invalid or reserved address [0,250]");
        return 1;
    }

    rc = ping_address(shell, &reply, address);
    switch (rc) {
    case MBUS_RECV_RESULT_OK:
        shell_print(shell, "Reply from address %d.", address);
        break;
    case MBUS_RECV_RESULT_TIMEOUT:
        shell_warn(shell, "No response address %d.", address);
        break;
    case MBUS_RECV_RESULT_INVALID:
        mbus_purge_frames(handle);
        shell_warn(shell, "Collision detected, multiple device response.");
        break;
    default:
        break;
    }

    return rc;
}

static int scan_devices(const struct shell *shell, int argc, char *argv[])
{
    if (init_slaves(shell))
	return -1;

    return mbus_scan_1st_address_range(shell);
}

static int probe_devices(const struct shell *shell, int argc, char *argv[])
{
    char addr_mask[20] = "FFFFFFFFFFFFFFFF";

    log("Probing secondary addresses ...");
    if (mbus_is_secondary_address(addr_mask) == 0) {
        err("malformed secondary address mask. Must be 16 character HEX number.");
        return -1;
    }

    if (init_slaves(shell))
        return -1;

    return mbus_scan_2nd_address_range(handle, 0, addr_mask);
}

static int set_address(const struct shell *shell, int argc, char *argv[])
{
    mbus_frame reply;
    int curr, next;
    char *mask;

    mask = argv[1];
    if (!mbus_is_secondary_address(mask)) {
	curr = atoi(mask);
	if (curr < 0 || curr > 250) {
	    wrn("invalid secondary address [%s], also not a primary address (0-250).", argv[1]);
	    return 1;
	}
    } else {
	curr = MBUS_ADDRESS_NETWORK_LAYER;
    }

    next = atoi(argv[2]);
    if (next < 1 || next > 250) {
	wrn("invalid new primary address [%s], allowed 1-250.", argv[2]);
	return 1;
    }

    if (init_slaves(shell))
	return 1;

    if (mbus_send_ping_frame(handle, next, 0) == -1) {
	wrn("failed sending verification ping: %s", mbus_error_str());
	return 1;
    }

    if (mbus_recv_frame(handle, &reply) != MBUS_RECV_RESULT_TIMEOUT) {
	wrn("verification failed, primary address [%d] already in use.", next);
	return 1;
    }

    if (curr == MBUS_ADDRESS_NETWORK_LAYER) {
	if (secondary_select(shell, mask) == -1)
	    return 1;
    }

    for (int retries = 3; retries > 0; retries--) {
	if (mbus_set_primary_address(handle, curr, next) == -1) {
	    wrn("failed setting device [%s] primary address: %s", mask, mbus_error_str());
	    return 1;
	}

	if (mbus_recv_frame(handle, &reply) == MBUS_RECV_RESULT_TIMEOUT) {
	    if (retries > 1)
		continue;

	    wrn("No reply from device [%s].", mask);
	    return 1;
	}
	break;
    }

    if (mbus_frame_type(&reply) != MBUS_FRAME_TYPE_ACK) {
	wrn("invalid response from device [%s], exected ACK, got:", mask);
	mbus_frame_print(&reply);
	return 1;
    }

    dbg("primary address of device %s set to %d", mask, next);

    return 0;
}

static int show_status(const struct shell *sh, int argc, char *argv[])
{
    if (!handle) {
        shell_error(sh, "M-Bus module not initialized.");
        return 1;
    }

    shell_print(sh, "M-Bus module initialized, %s ready for commands.", MBUS_DEVICE);
    return 0;
}

static int toggle_debug(const struct shell *shell, int argc, char *argv[])
{
    debug ^= 1;

    shell_print(shell, "M-Bus debugging %s.", ENABLED(debug));
    LOG_LEVEL_SET(debug ? LOG_LEVEL_DBG : LOG_LEVEL_INF);

    mbus_parse_set_debug(debug);
    if (debug) {
        mbus_register_send_event(handle, mbus_dump_send_event);
        mbus_register_recv_event(handle, mbus_dump_recv_event);
    } else {
        mbus_register_send_event(handle, NULL);
        mbus_register_recv_event(handle, NULL);
    }

    return 0;
}

static int cmd_interactive(const struct shell *shell, int argc, char *argv[])
{
    interactive ^= 1;
    log("interactive mode %s", ENABLED(interactive));
    return 0;
}

static int toggle_verbose(const struct shell *shell, int argc, char *argv[])
{
    verbose ^= 1;
    log("verbose output %s", ENABLED(verbose));
    return 0;
}

static int toggle_xml(const struct shell *shell, int argc, char *argv[])
{
    xml ^= 1;
    log("XML output %s", ENABLED(xml));
    return 0;
}

static int cmd_set_parity(const struct shell *shell, size_t argc, char **argv, void *data)
{
    return mbus_serial_set_parity(handle, (int)data);
}

static int cmd_set_speed(const struct shell *shell, size_t argc, char **argv, void *data)
{
    return mbus_serial_set_baudrate(handle, (int)data);
}

static int cmd_diagnose(const struct shell *shell, size_t argc, char **argv)
{
    char status[32];

    if (mbus_serial_diag(handle, status, sizeof(status))) {
        shell_warn(shell, "Not connected.");
        return 1;
    }

    shell_print(shell, "serial: %s", status);
    return 0;
}

#ifndef CONFIG_CAF
#define sh_set_address    set_address
#define sh_scan_devices   scan_devices
#define sh_probe_devices  probe_devices
#define sh_query_device   query_device
#define sh_toggle_debug   toggle_debug
#define sh_show_status    show_status
#define sh_toggle_verbose toggle_verbose
#define sh_toggle_xml     toggle_xml
#else

static void mbus_cmd(struct k_work *work)
{
    struct shell_cmd *cmd = CONTAINER_OF(work, struct shell_cmd, work);

    if (cmd->cb(cmd->sh, cmd->argc, cmd->argv))
        shell_warn(cmd->sh, "Failed M-Bus command");
}

struct shell_cmd cmd = {
    .work = Z_WORK_INITIALIZER(mbus_cmd)
};

static int work_submit(const struct shell *sh, int argc, char *argv[], int (*cb)(const struct shell *, int, char **))
{
    int timeout = 120;

    if (argc >= NELEMS(cmd.argv)) {
        shell_warn(sh, "Too many arguments to command!");
        return 1;
    }

    if (k_work_busy_get(&cmd.work)) {
        shell_warn(sh, "Already busy with another M-Bus command, please wait.");
        return 1;
    }

    cmd.cb = cb;
    cmd.sh = interactive ? sh : NULL;
    cmd.argc = argc;
    for (int i = 0; i < NELEMS(cmd.argv); i++)
        cmd.argv[i] = argv[i];

    k_work_submit_to_queue(&wq, &cmd.work);
    while (interactive && k_work_busy_get(&cmd.work) && timeout--)
        sleep(1);

    return 0;
}

#define fnwrap(fn) static int _CONCAT(sh_,fn)(const struct shell *s, int c, char *v[]) { return work_submit(s, c, v, fn); }
fnwrap(set_address)
fnwrap(scan_devices)
fnwrap(probe_devices)
fnwrap(query_device)
fnwrap(show_status)
fnwrap(toggle_debug)
fnwrap(toggle_verbose)
fnwrap(toggle_xml)
#endif /* CONFIG_CAF */

SHELL_SUBCMD_DICT_SET_CREATE(parity_cmds, cmd_set_parity,
	(none, 0),
	(odd,  1),
	(even, 2)
);

SHELL_SUBCMD_DICT_SET_CREATE(speed_cmds, cmd_set_speed,
	(300,     300),
	(600,     600),
	(1200,   1200),
	(2400,   2400),
	(4800,   4800),
	(9600,   9600),
	(19200, 19200),
	(38400, 38400)
);

SHELL_STATIC_SUBCMD_SET_CREATE(module_shell,
    SHELL_CMD_ARG(diagnose,    NULL, "Diagnostic information, line status, etc.", cmd_diagnose, 0, 0),
    SHELL_CMD(parity, &parity_cmds,  "Set line parity", NULL),
    SHELL_CMD(speed, &speed_cmds,    "Set line speed", NULL),
    SHELL_CMD_ARG(address,     NULL, "Set primary address from secondary (mask) or current primary address.\nUsage: address <MASK | ADDR> NEW_ADDR", sh_set_address, 3, 0),
    SHELL_CMD_ARG(ping,        NULL, "Ping primary address", cmd_ping, 1, 1),
    SHELL_CMD_ARG(scan,        NULL, "Primary addresses scan", sh_scan_devices, 0, 0),
    SHELL_CMD_ARG(probe,       NULL, "Secondary addresses scan", sh_probe_devices, 0, 0),
    SHELL_CMD_ARG(request,     NULL, "Request data, full XML or single record.\nUsage: request <MASK | ADDR> [RECORD_ID]", sh_query_device, 2, 1),
    SHELL_CMD_ARG(status,      NULL, "Show status of M-Bus module", sh_show_status, 0, 0),
    SHELL_CMD_ARG(debug,       NULL, "Toggle debug mode", sh_toggle_debug, 0, 0),
    SHELL_CMD_ARG(interactive, NULL, "Toggle interactive mode", cmd_interactive, 0, 0),
    SHELL_CMD_ARG(verbose,     NULL, "Toggle verbose output (where applicable)", sh_toggle_verbose, 0, 0),
    SHELL_CMD_ARG(xml,         NULL, "Toggle XML output", sh_toggle_xml, 0, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(MODULE, &module_shell, "M-Bus commands", NULL);

int mbus_init(void)
{
    const char *port = MBUS_DEVICE;

    if ((handle = mbus_context_serial(port)) == NULL) {
        LOG_ERR("failed initializing M-Bus context: %s",  mbus_error_str());
        return 1;
    }

    if (mbus_connect(handle) == -1) {
        LOG_ERR("failed connecting to serial port %s", port);
	return 1;
    }

#ifdef CONFIG_CAF
    /* https://docs.zephyrproject.org/1.14.0/reference/kconfig/CONFIG_SYSTEM_WORKQUEUE_PRIORITY.html */
    k_work_queue_start(&wq, wq_stack, K_THREAD_STACK_SIZEOF(wq_stack),
                       CONFIG_SYSTEM_WORKQUEUE_PRIORITY, K_WORK_MODULE_NAME);
    module_set_state(MODULE_STATE_READY);
#endif

    return 0;
}

int mbus_exit(void)
{
    if (!handle)
        return 1;

    mbus_disconnect(handle);
    mbus_context_free(handle);
    handle = NULL;

    return 0;
}

#ifdef CONFIG_CAF
static bool handle_module_event(const struct module_state_event *evt)
{
    if (check_state(evt, MODULE_ID(main), MODULE_STATE_READY)) {
        LOG_DBG("Initializing M-Bus module ...");
        mbus_init();
        return true;
    }

    return false;
}

static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
        return handle_module_event(cast_module_state_event(aeh));

    __ASSERT_NO_MSG(false);

    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
#endif /* CONFIG_CAF */

/**
 * Local Variables:
 *  indent-tabs-mode: nil
 *  c-file-style: "stroustrup"
 * End:
 */
