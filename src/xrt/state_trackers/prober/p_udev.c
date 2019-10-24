// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Prober code interfacing to libudev.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup st_prober
 */

#include "util/u_misc.h"
#include "p_prober.h"

#include <stdio.h>
#include <string.h>
#include <libudev.h>
#include <inttypes.h>
#include <linux/hidraw.h>


/*
 *
 * Defines
 *
 */

#define HIDRAW_BUS_USB 3
#define HIDRAW_BUS_BLUETOOTH 5


/*
 *
 * Pre-declare functions.
 *
 */

static void
p_udev_enumerate_usb(struct prober *p, struct udev *udev);

static void
p_udev_add_usb(struct prober_device *pdev,
               uint8_t dev_class,
               const char *product,
               const char *path);

static void
p_udev_enumerate_v4l2(struct prober *p, struct udev *udev);

static void
p_udev_add_v4l(struct prober_device *pdev,
               uint32_t v4l_index,
               uint32_t usb_iface,
               const char *path);

static void
p_udev_enumerate_hidraw(struct prober *p, struct udev *udev);

static void
p_udev_add_hidraw(struct prober_device *pdev,
                  uint32_t interface,
                  const char *path);

static int
p_udev_get_interface_number(struct udev_device *raw_dev,
                            uint16_t *interface_number);

static int
p_udev_get_and_parse_uevent(struct udev_device *raw_dev,
                            uint32_t *out_bus_type,
                            uint16_t *out_vendor_id,
                            uint16_t *out_product_id,
                            uint64_t *out_bluetooth_serial);

static int
p_udev_get_usb_hid_address(struct udev_device *raw_dev,
                           uint32_t bus_type,
                           uint8_t *out_dev_class,
                           uint16_t *out_usb_bus,
                           uint16_t *out_usb_addr);

static int
p_udev_try_usb_relation_get_address(struct udev_device *raw_dev,
                                    uint8_t *out_dev_class,
                                    uint16_t *out_vendor_id,
                                    uint16_t *out_product_id,
                                    uint16_t *out_usb_bus,
                                    uint16_t *out_usb_addr);

static int
p_udev_get_vendor_id_product(struct udev_device *usb_device_dev,
                             uint16_t *vendor_id,
                             uint16_t *product_id);

static int
p_udev_get_usb_device_info(struct udev_device *usb_device_dev,
                           uint8_t *out_dev_class,
                           uint16_t *vendor_id,
                           uint16_t *product_id,
                           uint16_t *usb_bus,
                           uint16_t *usb_addr);

static int
p_udev_get_usb_device_address_path(struct udev_device *usb_dev,
                                   uint16_t *out_usb_bus,
                                   uint16_t *out_usb_addr);

static int
p_udev_get_usb_device_address_sysfs(struct udev_device *usb_dev,
                                    uint16_t *out_usb_bus,
                                    uint16_t *out_usb_addr);

static int
p_udev_get_sysattr_u16_base16(struct udev_device *dev,
                              const char *name,
                              uint16_t *out_value);

static int
p_udev_get_sysattr_u32_base10(struct udev_device *dev,
                              const char *name,
                              uint32_t *out_value);

XRT_MAYBE_UNUSED static void
p_udev_dump_device(struct udev_device *udev_dev, const char *name);


/*
 *
 * "Exported" functions.
 *
 */

int
p_udev_probe(struct prober *p)
{
	struct udev *udev = udev_new();
	if (!udev) {
		P_ERROR(p, "Can't create udev\n");
		return -1;
	}

	p_udev_enumerate_usb(p, udev);

	p_udev_enumerate_v4l2(p, udev);

	p_udev_enumerate_hidraw(p, udev);

	udev_unref(udev);

	return 0;
}


/*
 *
 * Internal functions.
 *
 */

static void
p_udev_enumerate_usb(struct prober *p, struct udev *udev)
{
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;

	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "usb");
	udev_enumerate_add_match_property(enumerate, "DEVTYPE", "usb_device");
	udev_enumerate_scan_devices(enumerate);

	devices = udev_enumerate_get_list_entry(enumerate);
	udev_list_entry_foreach(dev_list_entry, devices)
	{
		struct prober_device *pdev = NULL;
		struct udev_device *raw_dev = NULL;
		const char *sysfs_path = NULL;
		const char *dev_path = NULL;
		const char *product = NULL;
		uint8_t dev_class = 0;
		uint16_t vendor_id = 0;
		uint16_t product_id = 0;
		uint16_t usb_bus = 0;
		uint16_t usb_addr = 0;
		int ret;

		// Where in the sysfs is.
		sysfs_path = udev_list_entry_get_name(dev_list_entry);
		// Raw sysfs node.
		raw_dev = udev_device_new_from_syspath(udev, sysfs_path);
		// The thing we will open.
		dev_path = udev_device_get_devnode(raw_dev);
		// Product name.
		product = udev_device_get_sysattr_value(raw_dev, "product");


		ret = p_udev_get_usb_device_info(raw_dev, &dev_class,
		                                 &vendor_id, &product_id,
		                                 &usb_bus, &usb_addr);
		if (ret != 0) {
			P_ERROR(p, "Failed to get usb device info");
			goto next;
		}

		ret = p_dev_get_usb_dev(p, usb_bus, usb_addr, vendor_id,
		                        product_id, &pdev);

		P_SPEW(p,
		       "usb\n"
		       "\t\tptr:          %p (%i)\n"
		       "\t\tsysfs_path:   '%s'\n"
		       "\t\tdev_path:     '%s'\n"
		       "\t\tdev_class:    %02x\n"
		       "\t\tvendor_id:    %04x\n"
		       "\t\tproduct_id:   %04x\n"
		       "\t\tusb_bus:      %i\n"
		       "\t\tusb_addr:     %i\n"
		       "\t\tproduct:      '%s'",
		       (void *)pdev, ret, sysfs_path, dev_path, dev_class,
		       vendor_id, product_id, usb_bus, usb_addr, product);

		if (ret != 0) {
			P_ERROR(p, "p_dev_get_usb_device failed!");
			goto next;
		}

		// Add info to usb device.
		p_udev_add_usb(pdev, dev_class, product, dev_path);

	next:
		udev_device_unref(raw_dev);
	}

	udev_enumerate_unref(enumerate);
}

static void
p_udev_add_usb(struct prober_device *pdev,
               uint8_t dev_class,
               const char *product,
               const char *path)
{
	pdev->base.usb_dev_class = dev_class;

	if (product != NULL) {
		pdev->usb.product = strdup(product);
	}
	if (path != NULL) {
		pdev->usb.path = strdup(path);
	}
}

static void
p_udev_enumerate_v4l2(struct prober *p, struct udev *udev)
{
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;

	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "video4linux");
	udev_enumerate_scan_devices(enumerate);

	devices = udev_enumerate_get_list_entry(enumerate);

	udev_list_entry_foreach(dev_list_entry, devices)
	{
		struct prober_device *pdev = NULL;
		struct udev_device *raw_dev = NULL;
		const char *sysfs_path = NULL;
		const char *dev_path = NULL;
		uint8_t dev_class = 0;
		uint16_t vendor_id = 0;
		uint16_t product_id = 0;
		uint16_t usb_bus = 0;
		uint16_t usb_addr = 0;
		uint16_t usb_iface = 0;
		uint32_t v4l_index = 0;
		int ret;

		// Where in the sysfs is.
		sysfs_path = udev_list_entry_get_name(dev_list_entry);
		// Raw sysfs node.
		raw_dev = udev_device_new_from_syspath(udev, sysfs_path);
		// The thing we will open.
		dev_path = udev_device_get_devnode(raw_dev);

		ret = p_udev_try_usb_relation_get_address(
		    raw_dev, &dev_class, &vendor_id, &product_id, &usb_bus,
		    &usb_addr);
		if (ret != 0) {
			P_ERROR(p, "skipping none usb v4l device '%s'",
			        dev_path);
			goto next;
		}

		// USB interface.
		ret = p_udev_get_interface_number(raw_dev, &usb_iface);
		if (ret != 0) {
			P_ERROR(p,
			        "In enumerating V4L2 devices: "
			        "Failed to get interface number for '%s'",
			        sysfs_path);
			goto next;
		}

		// USB interface.
		ret =
		    p_udev_get_sysattr_u32_base10(raw_dev, "index", &v4l_index);
		if (ret != 0) {
			P_ERROR(p, "Failed to get v4l index.");
			goto next;
		}

		ret = p_dev_get_usb_dev(p, usb_bus, usb_addr, vendor_id,
		                        product_id, &pdev);

		P_SPEW(p,
		       "v4l\n"
		       "\t\tptr:          %p (%i)\n"
		       "\t\tsysfs_path:   '%s'\n"
		       "\t\tdev_path:     '%s'\n"
		       "\t\tvendor_id:    %04x\n"
		       "\t\tproduct_id:   %04x\n"
		       "\t\tv4l_index:    %u\n"
		       "\t\tusb_iface:    %i\n"
		       "\t\tusb_bus:      %i\n"
		       "\t\tusb_addr:     %i\n",
		       (void *)pdev, ret, sysfs_path, dev_path, vendor_id,
		       product_id, v4l_index, usb_iface, usb_bus, usb_addr);

		if (ret != 0) {
			P_ERROR(p, "p_dev_get_usb_device failed!");
			goto next;
		}

		// Add this interface to the usb device.
		p_udev_add_v4l(pdev, v4l_index, usb_iface, dev_path);

	next:
		udev_device_unref(raw_dev);
	}

	udev_enumerate_unref(enumerate);
}

static void
p_udev_add_v4l(struct prober_device *pdev,
               uint32_t v4l_index,
               uint32_t usb_iface,
               const char *path)
{
	size_t new_size = (pdev->num_v4ls + 1) * sizeof(struct prober_v4l);
	pdev->v4ls = realloc(pdev->v4ls, new_size);

	struct prober_v4l *v4l = &pdev->v4ls[pdev->num_v4ls++];
	U_ZERO(v4l);

	v4l->usb_iface = usb_iface;
	v4l->v4l_index = v4l_index;
	v4l->path = strdup(path);
}

static void
p_udev_enumerate_hidraw(struct prober *p, struct udev *udev)
{
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;

	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "hidraw");
	udev_enumerate_scan_devices(enumerate);

	devices = udev_enumerate_get_list_entry(enumerate);

	udev_list_entry_foreach(dev_list_entry, devices)
	{
		struct prober_device *pdev = NULL;
		struct udev_device *raw_dev = NULL;
		uint16_t vendor_id, product_id, interface;
		uint8_t dev_class = 0;
		uint16_t usb_bus = 0;
		uint16_t usb_addr = 0;
		uint32_t bus_type = 0;
		uint64_t bluetooth_id = 0;
		const char *sysfs_path;
		const char *dev_path;
		int ret;

		// Where in the sysfs is.
		sysfs_path = udev_list_entry_get_name(dev_list_entry);
		// Raw sysfs node.
		raw_dev = udev_device_new_from_syspath(udev, sysfs_path);
		// The thing we will open.
		dev_path = udev_device_get_devnode(raw_dev);

		// Bus type, vendor_id and product_id.
		ret = p_udev_get_and_parse_uevent(
		    raw_dev, &bus_type, &vendor_id, &product_id, &bluetooth_id);
		if (ret != 0) {
			P_ERROR(p, "Failed to get uevent info from device");
			goto next;
		}

		// HID interface.
		ret = p_udev_get_interface_number(raw_dev, &interface);
		if (ret != 0) {
			P_ERROR(p,
			        "In enumerating hidraw devices: "
			        "Failed to get interface number for '%s'",
			        sysfs_path);
			goto next;
		}

		// Get USB bus and address to de-dublicate devices.
		ret = p_udev_get_usb_hid_address(raw_dev, bus_type, &dev_class,
		                                 &usb_bus, &usb_addr);
		if (ret != 0) {
			P_ERROR(p, "Failed to get USB bus and addr.");
			goto next;
		}

		if (bus_type == HIDRAW_BUS_BLUETOOTH) {
			ret = p_dev_get_bluetooth_dev(
			    p, bluetooth_id, vendor_id, product_id, &pdev);
		} else if (bus_type == HIDRAW_BUS_USB) {
			ret = p_dev_get_usb_dev(p, usb_bus, usb_addr, vendor_id,
			                        product_id, &pdev);
		} else {
			// Right now only support USB & Bluetooth devices.
			P_ERROR(p,
			        "Ignoring none USB or Bluetooth hidraw device "
			        "'%u'.",
			        bus_type);
			goto next;
		}

		P_SPEW(p,
		       "hidraw\n"
		       "\t\tptr:          %p (%i)\n"
		       "\t\tsysfs_path:   '%s'\n"
		       "\t\tdev_path:     '%s'\n"
		       "\t\tbus_type:     %i\n"
		       "\t\tvendor_id:    %04x\n"
		       "\t\tproduct_id:   %04x\n"
		       "\t\tinterface:    %i\n"
		       "\t\tusb_bus:      %i\n"
		       "\t\tusb_addr:     %i\n"
		       "\t\tbluetooth_id: %012" PRIx64,
		       (void *)pdev, ret, sysfs_path, dev_path, bus_type,
		       vendor_id, product_id, interface, usb_bus, usb_addr,
		       bluetooth_id);

		if (ret != 0) {
			P_ERROR(p, "p_dev_get_usb_device failed!");
			goto next;
		}

		// Add this interface to the usb device.
		p_udev_add_hidraw(pdev, interface, dev_path);

	next:
		udev_device_unref(raw_dev);
	}

	udev_enumerate_unref(enumerate);
}

static void
p_udev_add_hidraw(struct prober_device *pdev,
                  uint32_t interface,
                  const char *path)
{
	size_t new_size =
	    (pdev->num_hidraws + 1) * sizeof(struct prober_hidraw);
	pdev->hidraws =
	    (struct prober_hidraw *)realloc(pdev->hidraws, new_size);

	struct prober_hidraw *hidraw = &pdev->hidraws[pdev->num_hidraws++];
	U_ZERO(hidraw);

	hidraw->interface = interface;
	hidraw->path = strdup(path);
}

static int
p_udev_get_usb_hid_address(struct udev_device *raw_dev,
                           uint32_t bus_type,
                           uint8_t *out_dev_class,
                           uint16_t *out_usb_bus,
                           uint16_t *out_usb_addr)
{
	uint16_t dummy_vendor, dummy_product;
	struct udev_device *usb_dev;

	if (bus_type != HIDRAW_BUS_USB) {
		return 0;
	}

	// Get the first USB device parent.
	// No we should not unref intf_dev, valgrind agrees.
	usb_dev = udev_device_get_parent_with_subsystem_devtype(raw_dev, "usb",
	                                                        "usb_device");
	if (usb_dev == NULL) {
		return -1;
	}

	return p_udev_get_usb_device_info(usb_dev, out_dev_class, &dummy_vendor,
	                                  &dummy_product, out_usb_bus,
	                                  out_usb_addr);
}

static int
p_udev_get_interface_number(struct udev_device *raw_dev, uint16_t *interface)
{
	struct udev_device *intf_dev;
	const char *str;

	// Make udev find the handle to the interface node.
	// No we should not unref intf_dev, valgrind agrees.
	intf_dev = udev_device_get_parent_with_subsystem_devtype(
	    raw_dev, "usb", "usb_interface");
	if (intf_dev == NULL) {
		return -1;
	}

	str = udev_device_get_sysattr_value(intf_dev, "bInterfaceNumber");
	if (str == NULL) {
		return -1;
	}

	*interface = (uint16_t)strtol(str, NULL, 16);

	return 0;
}

static int
p_udev_get_and_parse_uevent(struct udev_device *raw_dev,
                            uint32_t *out_bus_type,
                            uint16_t *out_vendor_id,
                            uint16_t *out_product_id,
                            uint64_t *out_bluetooth_serial)
{
	struct udev_device *hid_dev;
	char *serial_utf8 = NULL;
	uint64_t bluetooth_serial = 0;
	uint16_t vendor_id = 0, product_id = 0;
	uint32_t bus_type;
	const char *uevent;
	char *saveptr;
	char *line;
	char *tmp;
	int ret;

	// Dig through and find the regular hid node.
	hid_dev =
	    udev_device_get_parent_with_subsystem_devtype(raw_dev, "hid", NULL);
	if (hid_dev == NULL) {
		return -1;
	}

	uevent = udev_device_get_sysattr_value(hid_dev, "uevent");
	if (uevent == NULL) {
		return -1;
	}

	tmp = strdup(uevent);
	if (tmp == NULL) {
		return -1;
	}

	bool ok = false;
	line = strtok_r(tmp, "\n", &saveptr);
	while (line != NULL) {
		if (strncmp(line, "HID_ID=", 7) == 0) {
			ret = sscanf(line + 7, "%x:%hx:%hx", &bus_type,
			             &vendor_id, &product_id);
			if (ret == 3) {
				ok = true;
			}
		} else if (strncmp(line, "HID_NAME=", 9) == 0) {
			// printf("\t\tprocuct_name: '%s'\n", line + 9);
		} else if (strncmp(line, "HID_UNIQ=", 9) == 0) {
			serial_utf8 = &line[9];
			// printf("\t\tserial: '%s'\n", line + 9);
		}

		line = strtok_r(NULL, "\n", &saveptr);
	}

	if (ok && bus_type == HIDRAW_BUS_BLUETOOTH && serial_utf8 != NULL) {
		union {
			uint8_t arr[8];
			uint64_t v;
		} extract = {{0}};

		ret = sscanf(serial_utf8, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		             &extract.arr[5], &extract.arr[4], &extract.arr[3],
		             &extract.arr[2], &extract.arr[1], &extract.arr[0]);
		if (ret == 6) {
			bluetooth_serial = extract.v;
		}
	}

	free(tmp);

	if (ok) {
		*out_bus_type = bus_type;
		*out_vendor_id = vendor_id;
		*out_product_id = product_id;
		*out_bluetooth_serial = bluetooth_serial;
		return 0;
	}
	return -1;
}

static int
p_udev_try_usb_relation_get_address(struct udev_device *raw_dev,
                                    uint8_t *out_dev_class,
                                    uint16_t *out_vendor_id,
                                    uint16_t *out_product_id,
                                    uint16_t *out_usb_bus,
                                    uint16_t *out_usb_addr)
{
	struct udev_device *parent_dev, *usb_interface, *usb_device;

	parent_dev = udev_device_get_parent(raw_dev);
	usb_interface = udev_device_get_parent_with_subsystem_devtype(
	    raw_dev, "usb", "usb_interface");
	usb_device = udev_device_get_parent_with_subsystem_devtype(
	    raw_dev, "usb", "usb_device");

	// Huh, device has no direct parent?
	if (parent_dev == NULL) {
		return -1;
	}

	// Not directly sitting on the interface
	if (usb_interface != parent_dev) {
		return -1;
	}

	// Get the parent of the usb_interface which should be a usb_device.
	parent_dev = udev_device_get_parent(usb_interface);

	// This shouldn't really happen!
	if (usb_device != parent_dev) {
		return -1;
	}

	return p_udev_get_usb_device_info(usb_device, out_dev_class,
	                                  out_vendor_id, out_product_id,
	                                  out_usb_bus, out_usb_addr);
}

static int
p_udev_get_vendor_id_product(struct udev_device *usb_dev,
                             uint16_t *out_vendor_id,
                             uint16_t *out_product_id)
{
	uint16_t vendor_id, product_id;
	int ret;

	ret = p_udev_get_sysattr_u16_base16(usb_dev, "idVendor", &vendor_id);
	if (ret != 0) {
		return ret;
	}

	ret = p_udev_get_sysattr_u16_base16(usb_dev, "idProduct", &product_id);
	if (ret != 0) {
		return ret;
	}

	*out_vendor_id = vendor_id;
	*out_product_id = product_id;

	return 0;
}

static int
p_udev_get_usb_device_info(struct udev_device *usb_dev,
                           uint8_t *out_dev_class,
                           uint16_t *out_vendor_id,
                           uint16_t *out_product_id,
                           uint16_t *out_usb_bus,
                           uint16_t *out_usb_addr)
{
	uint16_t vendor_id, product_id, dev_class;
	int ret;

	// First get the vendor and product ids.
	ret = p_udev_get_vendor_id_product(usb_dev, &vendor_id, &product_id);
	if (ret != 0) {
		return ret;
	}

	ret =
	    p_udev_get_sysattr_u16_base16(usb_dev, "bDeviceClass", &dev_class);
	if (ret != 0) {
		return ret;
	}

	// We emulate what libusb does with regards to device bus and address.
	if (p_udev_get_usb_device_address_path(usb_dev, out_usb_bus,
	                                       out_usb_addr) == 0) {
		*out_dev_class = (uint8_t)dev_class;
		*out_vendor_id = vendor_id;
		*out_product_id = product_id;
		return 0;
	}

	// If for some reason we can't read the dev path fallback to sysfs.
	if (p_udev_get_usb_device_address_sysfs(usb_dev, out_usb_bus,
	                                        out_usb_addr) == 0) {
		*out_dev_class = (uint8_t)dev_class;
		*out_vendor_id = vendor_id;
		*out_product_id = product_id;
		return 0;
	}

	return -1;
}

static int
p_udev_get_usb_device_address_path(struct udev_device *usb_dev,
                                   uint16_t *out_usb_bus,
                                   uint16_t *out_usb_addr)
{
	uint16_t bus = 0, addr = 0;

	const char *dev_path = udev_device_get_devnode(usb_dev);
	if (dev_path == NULL) {
		return -1;
	}

	if (sscanf(dev_path, "/dev/bus/usb/%hu/%hu", &bus, &addr) != 2 &&
	    sscanf(dev_path, "/proc/bus/usb/%hu/%hu", &bus, &addr) != 2) {
		return -1;
	}

	*out_usb_bus = bus;
	*out_usb_addr = addr;

	return 0;
}

static int
p_udev_get_usb_device_address_sysfs(struct udev_device *usb_dev,
                                    uint16_t *out_usb_bus,
                                    uint16_t *out_usb_addr)
{
	uint16_t usb_bus = 0, usb_addr = 0;
	int ret;

	ret = p_udev_get_sysattr_u16_base16(usb_dev, "busnum", &usb_bus);
	if (ret != 0) {
		return ret;
	}

	ret = p_udev_get_sysattr_u16_base16(usb_dev, "devnum", &usb_addr);
	if (ret != 0) {
		return ret;
	}

	*out_usb_bus = usb_bus;
	*out_usb_addr = usb_addr;

	return 0;
}

static int
p_udev_get_sysattr_u16_base16(struct udev_device *dev,
                              const char *name,
                              uint16_t *out_value)
{
	const char *str = udev_device_get_sysattr_value(dev, name);
	if (str == NULL) {
		return -1;
	}

	*out_value = (uint16_t)strtol(str, NULL, 16);

	return 0;
}

static int
p_udev_get_sysattr_u32_base10(struct udev_device *dev,
                              const char *name,
                              uint32_t *out_value)
{
	const char *str = udev_device_get_sysattr_value(dev, name);
	if (str == NULL) {
		return -1;
	}

	*out_value = (uint32_t)strtol(str, NULL, 10);

	return 0;
}

static void
p_udev_dump_device(struct udev_device *udev_dev, const char *name)
{
	// clang-format off
	fprintf(stderr, "\t%s\n", name);
	fprintf(stderr, "\t\tptr:       %p\n", (void*)udev_dev);

	if (udev_dev == NULL) {
		return;
	}

	fprintf(stderr, "\t\tparent:    %p\n", (void*)udev_device_get_parent(udev_dev));
	fprintf(stderr, "\t\tdevpath:   %s\n", udev_device_get_devpath(udev_dev));
	fprintf(stderr, "\t\tdevnode:   %s\n", udev_device_get_devnode(udev_dev));
	fprintf(stderr, "\t\tdevtype:   %s\n", udev_device_get_devtype(udev_dev));
	fprintf(stderr, "\t\tsysname:   %s\n", udev_device_get_sysname(udev_dev));
	fprintf(stderr, "\t\tsysnum:    %s\n", udev_device_get_sysnum(udev_dev));
	fprintf(stderr, "\t\tsyspath:   %s\n", udev_device_get_syspath(udev_dev));
	fprintf(stderr, "\t\tsubsystem: %s\n", udev_device_get_subsystem(udev_dev));
	fprintf(stderr, "\t\tsysfs.product: %s\n", udev_device_get_sysattr_value(udev_dev, "product"));
	// clang-format on
}
