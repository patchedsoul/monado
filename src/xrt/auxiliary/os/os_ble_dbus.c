// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  BLE implementation based on Linux Bluez/dbus.
 * @author Pete Black <pete.black@collabora.com>
 * @ingroup aux_os
 */

#include "os_ble.h"

#ifdef XRT_OS_LINUX

#include "util/u_misc.h"
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <dbus/dbus.h>

#include <stdio.h>

struct ble_notify
{
	struct os_ble_device base;
	DBusConnection *conn;
	DBusError err;
	int fd;
};

static int
os_ble_notify_read(struct os_ble_device *bdev, uint8_t *data, size_t length)
{
	struct ble_notify *dev = (struct ble_notify *)bdev;
	return read(dev->fd, data, length);
}

static void
os_ble_notify_destroy(struct os_ble_device *bdev)
{
	struct ble_notify *dev = (struct ble_notify *)bdev;

	close(dev->fd);
	dbus_connection_unref(dev->conn);
	free(dev);
}

int
os_ble_notify_open(const char *mac,
                   const char *endpoint,
                   struct os_ble_device **out_ble)
{
	DBusMessage *msg;
	DBusMessageIter args;
	DBusPendingCall *pending;

	struct ble_notify *bledev = U_TYPED_CALLOC(struct ble_notify);
	bledev->base.read = os_ble_notify_read;
	bledev->base.destroy = os_ble_notify_destroy;

	*out_ble = &bledev->base;

	dbus_error_init(&bledev->err);
	bledev->conn = dbus_bus_get(DBUS_BUS_SYSTEM, &bledev->err);
	if (dbus_error_is_set(&bledev->err)) {
		fprintf(stderr, "DBUS Connection Error: %s\n",
		        bledev->err.message);
		dbus_error_free(&bledev->err);
	}

	if (bledev->conn == NULL) {
		return -1;
	}
	char dbus_address[256]; // should be long enough

	// This is our GATT characteristic - the service and char endpoint
	// numbers are device specific

	snprintf(dbus_address, 256, "/org/bluez/hci0/dev_%s/%s", mac, endpoint);
	msg = dbus_message_new_method_call(
	    "org.bluez",                     // target for the method call
	    dbus_address,                    // object to call on
	    "org.bluez.GattCharacteristic1", // interface to call on
	    "AcquireNotify");                // method name

	if (msg == NULL) {
		fprintf(stderr, "Message Null after construction\n");
		return -1;
	}



	// create an empty array to pass as our options argument to
	// AcquireNotify
	const char *container_signature = "{sv}"; // dbus type signature string
	DBusMessageIter iter, options;
	dbus_message_iter_init_append(msg,
	                              &iter); // attach it to our dbus message
	dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY,
	                                 container_signature, &options);
	dbus_message_iter_close_container(&iter, &options);

	// send message and get a handle for a reply
	if (!dbus_connection_send_with_reply(bledev->conn, msg, &pending,
	                                     -1)) { // -1 is default timeout
		fprintf(stderr, "Out Of Memory!\n");
		return -1;
	}

	if (pending == NULL) {
		fprintf(stderr, "Pending Call Null\n");
		return -1;
	}
	dbus_connection_flush(bledev->conn);
	dbus_message_unref(msg);

	// block until we recieve a reply
	dbus_pending_call_block(pending);

	// get the reply message
	msg = dbus_pending_call_steal_reply(pending);
	if (msg == NULL) {
		fprintf(stderr, "Reply Null\n");
		exit(1);
	}
	// free the pending message handle
	dbus_pending_call_unref(pending);

	int current_type = 0;
	char *response;
	dbus_message_iter_init(msg, &args);
	while ((current_type = dbus_message_iter_get_arg_type(&args)) !=
	       DBUS_TYPE_INVALID) {
		if (current_type == DBUS_TYPE_STRING) {
			dbus_message_iter_get_basic(&args, &response);
			printf("DBus call returned message: %s\n", response);
		}
		if (current_type == DBUS_TYPE_UNIX_FD) {
			dbus_message_iter_get_basic(&args, &bledev->fd);
		}
		dbus_message_iter_next(&args);
	}
	// free reply
	dbus_message_unref(msg);

	return 0;
}

#endif
