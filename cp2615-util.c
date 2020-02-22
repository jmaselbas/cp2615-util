#include <unistd.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include <libusb.h>

#include <arpa/inet.h>
#include "arg.h"

#define min(a,b)	((a) < (b) ? (a) : (b))
#define max(a,b)	((a) < (b) ? (b) : (a))

#define CP2615_VID 0x10c4
#define CP2615_PID 0xeac1
#define CP2615_IOP_EP_OUT (2 | LIBUSB_ENDPOINT_OUT)
#define CP2615_IOP_EP_IN (2 | LIBUSB_ENDPOINT_IN)

struct cp2615_iop {
	uint16_t magic;
	uint16_t len;
	uint16_t cmd;
	uint8_t data[64 - 6];
};
#define CP2615_IOP_HEADER_SIZE offsetof(struct cp2615_iop, data)

#define CP2615_IOP_CMD_I2C_TRANSFER 0xd400
#define CP2615_IOP_CMD_I2C_TRANSFER_RESULT 0xa400

struct iop_i2c_transfer {
	uint8_t tag;
	uint8_t addr;
	uint8_t rlen;
	uint8_t wlen;
	uint8_t data[];
};
#define IOP_I2C_TRANSFER_HEADER_SIZE offsetof(struct iop_i2c_transfer, data)

struct iop_i2c_result {
	uint8_t tag;
	uint8_t addr;
	uint8_t status;
	uint8_t rlen;
	uint8_t data[];
};
#define IOP_I2C_RESULT_HEADER_SIZE offsetof(struct iop_i2c_result, data)

libusb_context *usb;
libusb_device_handle *cp2615;
unsigned int interface;
unsigned int kernel;
unsigned char tag;

void cp2615_init(void);
void cp2615_fini(void);

void
die(const char *errstr, ...)
{
	va_list ap;

	va_start(ap, errstr);
	vfprintf(stderr, errstr, ap);
	va_end(ap);

	cp2615_fini();
	exit(1);
}
/*
int
libusb_bulk_transfer(
	libusb_device_handle* dev_handle,
	unsigned char endpoint,
	unsigned char* data,
	int length,
	int* actual_length,
	    unsigned int timeout
	)
	*/

/* https://www.silabs.com/documents/public/application-notes/an1139-cp2615-io-protocol.pdf */
void
cp2615_iop_header(struct cp2615_iop *iop, int len, int cmd)
{
	iop->magic = 0x2a2a;
	iop->len = htons(len);
	iop->cmd = htons(cmd);
}

ssize_t
cp2615_iop_i2c(int addr, int wlen, unsigned char *wbuf, int rlen, unsigned char *rbuf)
{
	struct cp2615_iop pkt;
	struct iop_i2c_transfer *i2c_xfer;
	struct iop_i2c_result *i2c_xfer_result;
	int ret;
	int got;
	int len;
	unsigned int cmd;

	wlen = min(wlen, 54);
	rlen = min(rlen, 54);

	len = CP2615_IOP_HEADER_SIZE + IOP_I2C_TRANSFER_HEADER_SIZE + wlen;
	cp2615_iop_header(&pkt, len, CP2615_IOP_CMD_I2C_TRANSFER);

	i2c_xfer = (void *)pkt.data;
	i2c_xfer->tag = tag;
	i2c_xfer->addr = (addr << 1);
	i2c_xfer->rlen = rlen;
	i2c_xfer->wlen = wlen;
	if (wlen)
		memcpy(i2c_xfer->data, wbuf, wlen);

	/* bulk out */
	got = 0;
	ret = libusb_bulk_transfer(cp2615, CP2615_IOP_EP_OUT, (void *)&pkt, len, &got, 1000);
	if (ret) {
		fprintf(stderr, "xfer failed, sent %d/%d: %s\n", got, len, libusb_strerror(ret));
		return -1;
	}

	/* bulk in */
	len = CP2615_IOP_HEADER_SIZE + IOP_I2C_RESULT_HEADER_SIZE + rlen;
	got = 0;
	ret = libusb_bulk_transfer(cp2615, CP2615_IOP_EP_IN, (void *)&pkt, len, &got, 1000);
	if (ret) {
		fprintf(stderr, "xfer failed, got %d/%d: %s\n", got, len, libusb_strerror(ret));
		return -1;
	}

	if (got < CP2615_IOP_HEADER_SIZE || got < ntohs(pkt.len) || got < len)
		fprintf(stderr, "truncated response got %d  / %d\n", got, ntohs(pkt.len));

	if (pkt.magic != 0x2a2a || ntohs(pkt.len) > sizeof(pkt))
		fprintf(stderr, "received invalid pkt header\n");

	cmd = ntohs(pkt.cmd);
	if (cmd != CP2615_IOP_CMD_I2C_TRANSFER_RESULT) {
		fprintf(stderr, "Not i2c transfer result\n");
		return -1;
	}

	i2c_xfer_result = (void *)pkt.data;
	if (i2c_xfer_result->status != 0) {
		fprintf(stderr, "i2c error: %d\n", i2c_xfer_result->status);
		return -1;
	}

	if (!rlen)
		return 0;

	if (i2c_xfer_result->rlen < rlen) {
		fprintf(stderr, "received invalid i2c length %d != %d\n", i2c_xfer_result->rlen, rlen);
	}

	memcpy(rbuf, i2c_xfer_result->data, rlen);

	return rlen;
}

void
cp2615_init(void)
{
	int err;

	err = libusb_init(&usb);
	if (err)
		die("libusb_init: %s\n", libusb_strerror(err));

	cp2615 = libusb_open_device_with_vid_pid(usb, CP2615_VID, CP2615_PID);
	if (!cp2615)
		die("Fail to open device %4x:%4x: %s\n",
		    CP2615_VID, CP2615_PID, strerror(errno));

	kernel = libusb_kernel_driver_active(cp2615, 1);
	if (kernel)
		if (libusb_detach_kernel_driver(cp2615, 1))
			die("Couldn't detach kernel driver!\n");

	err = libusb_claim_interface(cp2615, 1);
	if (err)
		die("libusb_claim_interface: %s\n", libusb_strerror(err));

	err = libusb_set_interface_alt_setting(cp2615, 1, 2);
	if (err)
		die("libusb_set_interface_alt: %s\n", libusb_strerror(err));
}

void
cp2615_fini(void)
{
	int err = 0;

	if (cp2615)
		err = libusb_release_interface(cp2615, 1);
	if (err)
		die("libusb_release_interface: %s\n", libusb_strerror(err));

	if (kernel)
		libusb_attach_kernel_driver(cp2615, 1);

	if (usb)
		libusb_exit(usb);
}

char *argv0;

void
usage(void)
{
	printf("usage: %s i2c [i2c-options]\n", argv0);
	die("");
}

void
i2c_usage(void)
{
	printf("usage: %s i2c [-hb] [W] <i2c-addr> [data...]\n", argv0);
	die("");
}

int
i2c_main(int argc, char *argv[])
{
	int i, ret;
	unsigned char *rbuf = NULL, *wbuf = NULL;
	unsigned int rlen = 0, wlen = 0;
	unsigned int addr;
	unsigned long int val;
	char *arg, *end = NULL;

	ARGBEGIN {
	case 'r':
		arg = EARGF(i2c_usage());
		val = strtoul(arg, &end, 0);
		if (val == ULONG_MAX || (end && *end != '\0'))
			die("invalid argument: %s\n", arg);
		rlen = val;
		break;
	default:
	case 'h':
		i2c_usage();
		break;
	} ARGEND;

	if (argc == 0)
		die("missing i2c address\n");

	val = strtoul(argv[0], &end, 0);
	if (val >= 0x80 || (end && *end != '\0'))
		die("invalid address: %s\n", argv[0]);
	addr = val;
	argv++;

	wlen = max(0, argc - 1);
	wbuf = calloc(wlen, sizeof(char));

	for (i = 0; i < wlen; i++) {
		arg = argv[i];
		val = strtoul(arg, &end, 0);
		if (val == ULONG_MAX || (end && *end != '\0'))
			die("invalid argument: %s\n", arg);
		wbuf[i] = val;
	}

	if (rlen)
		rbuf = calloc(rlen, sizeof(char));

	cp2615_init();

	ret = cp2615_iop_i2c(addr, wlen, wbuf, rlen, rbuf);
	if (ret < 0)
		return ret;

	if (rlen) {
		if (isatty(1)) {
			for (i = 0; i < rlen; i++) {
				if (!(i % 16))
					printf("%c%.8x: ", i > 0 ? '\n' : 0, i);
				printf("%.2x%c", rbuf[i], i % 2 ? ' ' : 0);
			}
			puts("");
		} else {
			fwrite(rbuf, sizeof(char), rlen, stdout);
		}
	}

	free(rbuf);
	free(wbuf);

	return 0;
}

void
shift(int argc, char **argv)
{
	if (argc)
		memmove(argv + 1, argv + 2, argc * sizeof(char *));
}

int
main(int argc, char *argv[])
{
	char *cmd;

	argv0 = argv[0];
	if (argc < 2)
		usage();

	cmd = argv[1];
	shift(argc--, argv);

	if (strcmp(cmd, "i2c") == 0)
		i2c_main(argc, argv);
	else
		usage();

	cp2615_fini();

	return 0;
}
