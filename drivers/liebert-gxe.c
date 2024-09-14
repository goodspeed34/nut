/* liebert-gxe.c - support for Liebert GXE Series UPS models via serial.

   Copyright (C) 2024  Gong Zhile <goodspeed@mailo.cat>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "config.h"
#include "main.h"
#include "attribute.h"
#include "serial.h"

#include "liebert-gxe.h"

#define DRIVER_NAME	"Liebert GXE Series UPS driver"
#define DRIVER_VERSION	"0.01"

#define SER_WAIT_SEC	1
#define PROBE_RETRIES   3

#define DATAFLAG_WARN_MASK (1)
#define DATAFLAG_ONOFF_MASK (1 << 4)

#ifndef htole16
#ifdef WORDS_BIGENDIAN
#define htole16(x) ((((x) >> 8) | ((x) << 8)))
#else  /* WORDS_BIGENDIAN */
#define htole16(x) (x)
#endif	/* WORDS_BIGENDIAN */
#endif	/* htole16 */

static char *devaddr = NULL;

#define ARRAY_SIZE(x) ((sizeof x) / (sizeof *x))
static const char *gxe_warns[] = {
	NULL,			/* DATAFLAG */
	"Inverter Out-of-Sync",
	"Unhealthy Main Circuit",
	"Rectifier Failure",
	"Inverter Failure",
	"Unhealthy Bypass",
	"Unhealthy Battery Voltage",
	NULL,			/* USER_DEFINED */
	NULL,			/* USER_DEFINED */
	"Power Module Overheated",
	"Unhealthy Fan",
	"Netural Input Missing",
	"Master Line Abnormally Turned-off",
	"Charger Failure",
	"Battery Discharge Declined",
	"Backup Power Supply Failure",
	"Ouput Overloaded",
	"Ouput Shorted",
	"Overload Timed-out",
	"Unhealthy Parallel Machine Current",
	"Parallel Machine Connection Failure",
	"Parallel Machine Address Error",
	"Unhealthy Internal Communication",
	"System Overloaded",
	"Battery Installed Backwards",
	"Battery Not Found",
};

/* Instcmd & Driver Init: SYSPARAM -OK-> WARNING -OK-> ONOFF -OK-> ANALOG */
/* If the dataflag set for WARNING/ONOFF, set next state respectively. */
static enum gxe_poll_state {
	GXE_ONOFF,
	GXE_ANALOG,
	GXE_WARNING,
	/* Scheduled System Parameters, Trigged by Instcmd */
	GXE_SYSPARAM,
} poll_state;

upsdrv_info_t upsdrv_info = {
	DRIVER_NAME,
	DRIVER_VERSION,
	"Gong Zhile <goodspeed@mailo.cat>",
	DRV_EXPERIMENTAL,
	{ NULL }
};

static void proto_lchecksum(uint16_t dlen, char *out)
{
	uint8_t lenchk = 0;
	uint16_t lelen = htole16(dlen);
	/* GCC complains uint16_t fits 7-bytes buffer, store here. */
	char fbuf[7];

	/* Sum all four 4 bits */
	lenchk += lelen & 0x000f;
	lenchk += lelen & 0x00f0;
	lenchk += lelen & 0x0f00;
	lenchk += lelen & 0xf000;

	lenchk %= 16;
	lenchk = ~lenchk + 1;

	snprintf(fbuf, 7, "%04X", (uint16_t) (lelen | lenchk << 12));
	memcpy(out, fbuf, 5);
}

static void proto_checksum(const char *buf, size_t len, char* out)
{
	size_t i;
	uint32_t sum = 0;
	/* GCC complains uint16_t fits 7-bytes buffer, store here. */
	char fbuf[7];

	for (i = 0; i < len; i++)
		sum += buf[i];
	sum %= 65536;

	snprintf(fbuf, 7, "%04X", (uint16_t) ~sum + 1);
	memcpy(out, fbuf, 5);
}

static size_t
frame_make(char *buf,
	   enum gxe_cmds cmdi,
	   const char *ver,
	   const char *adr,
	   const char *dptr,
	   size_t dlen)
{
	/* SOI */
	buf[0] = 0x7e;
	/* VER */
	memcpy(buf+1, ver, 2);
	/* ADR */
	memcpy(buf+3, adr, 2);
	/* CID1 & CID2 */
	memcpy(buf+5, gxe_cmds[cmdi], 4);
	/* LENGTH */
	proto_lchecksum(dlen, buf+9);
	/* INFO */
	memcpy(buf+13, dptr, dlen);
	/* CHKSUM */
	proto_checksum(buf+1, dlen+12, buf+13+dlen);
	/* EOI */
	*(buf+17+dlen) = 0x0d;

	return dlen+18;
}

static int
frame_send_and_recv(char *buf,
		    size_t buflen,
		    enum gxe_cmds cmdi,
		    const char *ver,
		    const char *adr,
		    const char *dptr,
		    size_t dlen)
{
	int ret;
	size_t framelen;
	char framebuf[32];

	ser_flush_io(upsfd);

	framelen = frame_make(framebuf, cmdi, ver, adr, dptr, dlen);
	upsdebug_hex(5, "send", framebuf, framelen);

	ret = ser_send_buf(upsfd, framebuf, framelen);
	if (ret <= 0) {
		upsdebugx(LOG_WARNING, "send: %s", ret ? strerror(errno) : "timeout");
		return ret;
	}

	ret = ser_get_buf(upsfd, buf, buflen, 1, 0);
	upsdebug_hex(5, "read", buf, ret);
	if (ret <= 0) {
		upsdebugx(LOG_WARNING, "read: %s", ret ? strerror(errno) : "timeout");
		return ret;
	}

	return ret;
}

static int val_from_hex(char *buf, size_t dlen)
{
	char valbuf[16];

	if (dlen > 15)
		return 0;

	memcpy(valbuf, buf, dlen);
	valbuf[dlen] = '\0';
	return strtol(valbuf, NULL, 16);
}

static void substr_from_hex(char *substr, size_t len, char *dbuf, size_t dlen)
{
	int val;
	size_t i;
	for (i = 0; i < dlen; i += 2) {
		val = val_from_hex(dbuf+i, 2);
		if (val == 0x20) {
			substr[i/2] = '\0';
			return;
		}
		if (i/2 > len-1)
			break;
		substr[i/2] = val;
	}
	substr[i/2] = '\0';
	upsdebugx(LOG_DEBUG, "substr: %s", substr);
}

int validate_ret(char *buf, int ret, int minlen)
{
	if (ret < 0) {
		upslog_with_errno(LOG_ERR, "Read failed");
		return -1;
	}

	if (ret < minlen) {
		upslogx(LOG_ERR, "Short read from UPS");
		return -1;
	}

	if ((ret = val_from_hex(buf+7, 2))) {
		upslogx(LOG_ERR, "Command failed: %s", RTN_TO_STR(ret));
		return -1;
	}

	return ret;
}

static int instcmd(const char *cmdname, const char *extra)
{
	enum gxe_cmds cmd = GXE_REMOTE_COMMAND;
	char cmdbuf[64], *data = NULL;
	int retry, ret, len = 4;

	if (!strcasecmp(cmdname, "test.battery.start"))
		data = "1002";
	else if (!strcasecmp(cmdname, "test.battery.stop"))
		data = "1003";
	else if (!strcasecmp(cmdname, "load.on"))
		data = "2001";
	else if (!strcasecmp(cmdname, "load.off"))
		data = "2003";
	else {
		upslogx(LOG_NOTICE, "instcmd: unknown command [%s] [%s]", cmdname, extra);
		return STAT_INSTCMD_UNKNOWN;
	}

	for (retry = 0; retry < PROBE_RETRIES; retry++) {
		ret = frame_send_and_recv(cmdbuf, 64, cmd, "21",
					  devaddr, data, len);

		if (!(validate_ret(cmdbuf, ret, 13) < 0)) {
			poll_state = GXE_SYSPARAM;
			return STAT_INSTCMD_HANDLED;
		}
	}

	upslogx(LOG_WARNING, "instcmd: remote failed response, try again");
	return STAT_INSTCMD_FAILED;
}

static void upsdrv_updateinfo_onoff(void)
{
	char onoff_buf[64];
	int ret, dflag, pwrval, rectval;

	ret = frame_send_and_recv(onoff_buf, 64,
				  GXE_GET_ONOFF_DATA,
				  "21", devaddr, NULL, 0);

	if (validate_ret(onoff_buf, ret, 13+0x14) < 0) {
		poll_state = GXE_ONOFF;
		dstate_datastale();
		return;
	}
	poll_state = GXE_ANALOG;

	/* DATAFLAG */
	dflag = val_from_hex(onoff_buf+13, 2);
	if (dflag & DATAFLAG_ONOFF_MASK)
		poll_state = GXE_ONOFF;
	if (dflag & DATAFLAG_WARN_MASK)
		poll_state = GXE_WARNING;

	status_init();

	/* Field 1, Power Supply (01=UPS, 02=Bypass) */
	pwrval = val_from_hex(onoff_buf+15, 2);
	/* Field 3, Rectifier Power Supply (E0=None, E1=CITYPWR, E2=BAT) */
	rectval = val_from_hex(onoff_buf+19, 2);

	if (pwrval == 0x01 && rectval == 0xe2)
		status_set("OB");
	else if (pwrval == 0x01)
		status_set("OL");
	else if (pwrval == 0x02)
		status_set("OL BYPASS");
	else upsdebugx(LOG_WARNING, "unknown ups state: %x %x", pwrval, rectval);

	status_commit();

	/* Field 4, Battery Status */
	switch(val_from_hex(onoff_buf+15+6, 2)) {
	case 0xe0:
		dstate_setinfo("battery.charger.status", "resting");
		break;
	case 0xe1:
	case 0xe2:
		dstate_setinfo("battery.charger.status", "charging");
		break;
	case 0xe3:
		dstate_setinfo("battery.charger.status", "discharging");
		break;
	default:
		upsdebugx(LOG_WARNING, "unknown battery status, ignored");
		break;
	}

	/* Field 5, Battery Test State */
	switch(val_from_hex(onoff_buf+15+8, 2)) {
	case 0xe0:
		dstate_setinfo("ups.test.result", "In progress");
		break;
	case 0xe1:
		dstate_setinfo("ups.test.result", "Idle");
		break;
	default:
		upsdebugx(LOG_WARNING, "unknown battery test state, ignored");
		break;
	}

	dstate_dataok();
}

static void upsdrv_updateinfo_analog(void)
{
	char analog_buf[128];
	int ret, dflag, volt;

	ret = frame_send_and_recv(analog_buf, 128,
				  GXE_GET_ANALOG_DATA,
				  "21", devaddr, NULL, 0);
	if (validate_ret(analog_buf, ret, 13+0x56) < 0) {
		dstate_datastale();
		return;
	}

	/* DATAFLAG, NOT RELIABLE SOMEHOW */
	dflag = val_from_hex(analog_buf+13, 2);
	if (dflag & DATAFLAG_ONOFF_MASK)
		poll_state = GXE_ONOFF;
	if (dflag & DATAFLAG_WARN_MASK)
		poll_state = GXE_WARNING;

	/* Field 1, AC_IN VOLTAGE */
	volt = val_from_hex(analog_buf+15, 4)/100;

	if (volt == 0 && status_get("OL")) {
		/* Oh no, power failed still online? */
		status_init();
		status_set("OB");
		status_commit();
		poll_state = GXE_WARNING;
	}

	if (volt > 0 && status_get("OB")) {
		/* Hum, power recovered still on battery? */
		status_init();
		status_set("OL");
		status_commit();
		poll_state = GXE_WARNING;
	}

	dstate_setinfo("input.voltage", "%.02f",
		       val_from_hex(analog_buf+15, 4)/100.0f);
	/* Field 4, AC_OUT VOLTAGE */
	dstate_setinfo("output.voltage", "%.02f",
		       val_from_hex(analog_buf+15+12, 4)/100.0f);
	/* Field 7, AC_OUT CURRENT */
	dstate_setinfo("output.current", "%.02f",
		       val_from_hex(analog_buf+15+24, 4)/100.0f);
	/* Field 10, DC VOLTAGE */
	dstate_setinfo("battery.voltage", "%.02f",
		       val_from_hex(analog_buf+15+36, 4)/100.0f);
	/* Field 11, AC_OUT FREQUENCY */
	dstate_setinfo("output.frequency", "%.02f",
		       val_from_hex(analog_buf+15+40, 4)/100.0f);
	/* Field 15, AC_IN FREQUENCY */
	dstate_setinfo("input.frequency", "%.02f",
		       val_from_hex(analog_buf+15+52, 4)/100.0f);
	/* Field 18, AC_OUT REALPOWER, kW */
	dstate_setinfo("ups.realpower", "%d",
		       val_from_hex(analog_buf+15+64, 4)*10);
	/* Field 19, AC_OUT POWER, kVA */
	dstate_setinfo("ups.power", "%d",
		       val_from_hex(analog_buf+15+68, 4)*10);
	/* Field 22, BATTERY BACKUP TIME, Min */
	dstate_setinfo("battery.runtime.low", "%.2f",
		       val_from_hex(analog_buf+15+80, 4)/100.0f*60.0f);

	dstate_dataok();
}

static void upsdrv_updateinfo_sysparam(void)
{
	char sysparam_buf[128];
	int ret;

	ret = frame_send_and_recv(sysparam_buf, 128,
				  GXE_GET_SYS_PARAM,
				  "21", devaddr, NULL, 0);
	if (validate_ret(sysparam_buf, ret, 13+0x6a) < 0) {
		dstate_datastale();
		return;
	}
	poll_state = GXE_WARNING;

	/* Field 6, Nominal Voltage */
	dstate_setinfo("output.voltage.nominal", "%d",
		       val_from_hex(sysparam_buf+13+18, 4));
	/* Field 7, Nominal Frequency */
	dstate_setinfo("output.frequency.nominal", "%d",
		       val_from_hex(sysparam_buf+13+18+4, 4));
	/* Field 10, Bypass Working Voltage Max, ALWAYS 115% */
	if (val_from_hex(sysparam_buf+13+18+16, 4) == 1)
		dstate_setinfo("input.transfer.bypass.high", "%f",
			       val_from_hex(sysparam_buf+13+18, 4)*1.15f);
	/* Field 11, Bypass Working Voltage Min, Volt */
	if (val_from_hex(sysparam_buf+13+18+20, 4) == 1)
		dstate_setinfo("input.transfer.bypass.low", "%d", 120);
	/* Field 21, Battery Test Interval, per 3 mons */
	dstate_setinfo("ups.test.interval", "%lu",
		       (long) val_from_hex(sysparam_buf+13+18+60, 4)*3*108000);

	dstate_dataok();
}

static void upsdrv_updateinfo_warning(void)
{
	char warn_buf[128];
	int ret, val;
	size_t i;

	ret = frame_send_and_recv(warn_buf, 128,
				  GXE_GET_WARNING_DATA,
				  "21", devaddr, NULL, 0);
	if (validate_ret(warn_buf, ret, 13+0x36) < 0) {
		poll_state = GXE_WARNING;
		dstate_datastale();
		return;
	}
	poll_state = GXE_ONOFF;

	alarm_init();
	for (i = 0; i < ARRAY_SIZE(gxe_warns); i++) {
		if (!gxe_warns[i])
			continue;
		val = val_from_hex(warn_buf+15+i*2, 2);
		switch(val)  {
		case 0x00:
			break;
		case 0x01:
		case 0x02:
		case 0x03:
		case 0xf0:
			alarm_set(gxe_warns[i]);
			break;
		default:
			upsdebugx(LOG_WARNING, "unexpected warning val %x", val);
			break;
		}
	}
	alarm_commit();

	dstate_dataok();
}

void upsdrv_updateinfo(void)
{
	switch(poll_state) {
	case GXE_ANALOG:
		upsdebugx(LOG_DEBUG, "Polling ANALOG data");
		upsdrv_updateinfo_analog();
		break;
	case GXE_ONOFF:
		upsdebugx(LOG_DEBUG, "Polling ONOFF data");
		upsdrv_updateinfo_onoff();
		break;
	case GXE_WARNING:
		upsdebugx(LOG_DEBUG, "Polling WARNING data");
		upsdrv_updateinfo_warning();
		break;
	case GXE_SYSPARAM:
		upsdebugx(LOG_DEBUG, "Polling SYSPARAM data");
		upsdrv_updateinfo_sysparam();
		break;
	}
}

void upsdrv_initinfo(void)
{
	char databuf[64];
	char recvbuf[64];
	int retry, ret;

	for (retry = 0; retry < PROBE_RETRIES; retry++) {
		ret = frame_send_and_recv(recvbuf, 64,
					  GXE_GET_VENDOR_INFO,
					  "21", devaddr, NULL, 0);
		if (ret > 34)
			break;
	}

	if (ret <= 0)
		fatal_with_errno(EXIT_FAILURE, "gxe: failed reading response");
	if (ret < 35)		/* Minimum Length for Name */
		fatalx(EXIT_FAILURE, "gxe: not enough data");

	/* UPS Name, 10 bytes */
	substr_from_hex(databuf, 64, recvbuf+13, 20);
	dstate_setinfo("ups.mfr", "EmersonNetworkPower");
	dstate_setinfo("ups.model", "%s", databuf);

	dstate_setinfo("ups.id", "%s", devaddr);

	dstate_addcmd("test.battery.start");
	dstate_addcmd("test.battery.stop");
	dstate_addcmd("load.off");
	dstate_addcmd("load.on");

	upsh.instcmd = instcmd;

	poll_state = GXE_SYSPARAM;
}

void upsdrv_help(void)
{
}

void upsdrv_makevartable(void)
{
	addvar(VAR_VALUE, "addr", "Override default UPS address");
}

void upsdrv_initups(void)
{
	upsfd = ser_open(device_path);

	devaddr = "01";		/* Default Address is 0x01 */
	if (testvar("addr"))
		devaddr = getval("addr");

	/* UPS behaves wird on the serial line. If two frames arrived in
	 * a brust, the device will only response to the first frame. After
	 * testing, the minimum interval is 5 sec which's already beyond the
	 * stale tolerance.
	 */
	poll_interval = 5;

	usleep(100000);
}

void upsdrv_shutdown(void)
{
	upslogx(LOG_INFO, "GXE UPS can't fully shutdown, NOOP");
}

void upsdrv_cleanup(void)
{
	ser_close(upsfd, device_path);
}
