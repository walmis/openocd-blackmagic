/***************************************************************************
 *   Copyright (C) 2019 by Valmantas Paliksa                               *
 *   Valmantas Paliksa <walmis@gmail.com>                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include "libusb_common.h"
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <stddef.h>
#include <errno.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>

#define RESP_TIMEOUT 500

/* Protocol error messages */
#define REMOTE_ERROR_UNRECOGNISED 1
#define REMOTE_ERROR_WRONGLEN     2

/* Start and end of message identifiers */
#define REMOTE_SOM         '!'
#define REMOTE_EOM         '#'
#define REMOTE_RESP        '&'

/* Generic protocol elements */
#define REMOTE_START        'A'
#define REMOTE_TDITDO_TMS   'D'
#define REMOTE_TDITDO_NOTMS 'd'
#define REMOTE_IN_PAR       'I'
#define REMOTE_IN           'i'
#define REMOTE_NEXT         'N'
#define REMOTE_OUT_PAR      'O'
#define REMOTE_OUT          'o'
#define REMOTE_PWR_SET      'P'
#define REMOTE_PWR_GET      'p'
#define REMOTE_RESET        'R'
#define REMOTE_INIT         'S'
#define REMOTE_TMS          'T'
#define REMOTE_VOLTAGE      'V'
#define REMOTE_SRST_SET     'Z'
#define REMOTE_SRST_GET     'z'

/* Protocol response options */
#define REMOTE_RESP_OK     'K'
#define REMOTE_RESP_PARERR 'P'
#define REMOTE_RESP_ERR    'E'
#define REMOTE_RESP_NOTSUP 'N'

/* High level protocol elements */
#define REMOTE_HL_PACKET 'H'
#define REMOTE_INIT_SWDP    'S'
#define REMOTE_REG_READ     'r'
#define REMOTE_REG_WRITE    'y'

#define REMOTE_REGS_READ    'R'
#define REMOTE_MEM_READ     'm'
#define REMOTE_MEM_WRITE    'w'
#define REMOTE_STATE        'x'

#define HEX '%', '0', '2', 'x'
#define HEX_U32(x) '%', '0', '8', 'x'
#define CHR(x) '%', 'c'

#define REMOTE_INIT_SWDP_STR (char []){ REMOTE_SOM, REMOTE_HL_PACKET, REMOTE_INIT_SWDP, REMOTE_EOM, 0 }
#define REMOTE_REG_WRITE_STR (char []){ REMOTE_SOM, REMOTE_HL_PACKET, REMOTE_REG_WRITE, HEX, HEX_U32(v), REMOTE_EOM, 0 }
#define REMOTE_REG_READ_STR (char []){ REMOTE_SOM, REMOTE_HL_PACKET, REMOTE_REG_READ, HEX, REMOTE_EOM, 0 }
#define REMOTE_REGS_READ_STR (char []){ REMOTE_SOM, REMOTE_HL_PACKET, REMOTE_REGS_READ, REMOTE_EOM, 0 }
#define REMOTE_READ_MEM_STR (char []){ REMOTE_SOM, REMOTE_HL_PACKET, REMOTE_MEM_READ, \
										HEX_U32(address), HEX_U32(count), REMOTE_EOM, 0 }

#define REMOTE_WRITE_MEM_STR_BEGIN (char []){ REMOTE_SOM, REMOTE_HL_PACKET, REMOTE_MEM_WRITE, \
										HEX_U32(address), HEX_U32(count), 0 }
#define REMOTE_WRITE_MEM_STR_END (char []){ REMOTE_EOM, 0 }

#define REMOTE_RESET_STR (char []){ REMOTE_SOM, REMOTE_HL_PACKET, REMOTE_RESET, REMOTE_EOM, 0 }


struct bmp_handle {
	int fd;
	int is_socket;
};

#define NTOH(x) ((x<=9)?x+'0':'a'+x-10)
#define HTON(x) ((x<='9')?x-'0':((TOUPPER(x))-'A'+10))
#define TOUPPER(x) ((((x)>='a') && ((x)<='z'))?((x)-('a'-'A')):(x))
#define ISHEX(x) (						\
		(((x)>='0') && ((x)<='9')) ||					\
		(((x)>='A') && ((x)<='F')) ||					\
		(((x)>='a') && ((x)<='f'))						\
		)

static int bmp_write_mem (void *handle, uint32_t addr, uint32_t mem_width,
		uint32_t count, const uint8_t *buffer);
static int bmp_write_debug_reg (void *handle, uint32_t addr, uint32_t val);

static int bmp_send(struct bmp_handle *priv, void* buf, int len, int flags) {
	if(priv->is_socket) {
		return send(priv->fd, buf, len, flags);
	} else {
		return write(priv->fd, buf, len);
	}
}

static int set_interface_attribs(int fd, int speed, int parity)

/* A nice routine grabbed from
 * https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 */

{
	struct termios tty;

	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		fprintf(stderr, "error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "error %d from tcsetattr", errno);
		return -1;
	}

	tcflush(fd, TCIOFLUSH);

	return 0;
}

static int bmp_buffer_read(struct bmp_handle *priv, uint8_t *data, int maxsize)

{
	uint8_t *c;
	int s;
	int ret;
	fd_set rset;
	struct timeval tv;
	int f = priv->fd;

	c = data;
	tv.tv_sec = 0;
	tv.tv_usec = 1000 * RESP_TIMEOUT;

	/* Look for start of response */
	do {
		FD_ZERO(&rset);
		FD_SET(f, &rset);

		ret = select(f + 1, &rset, NULL, NULL, &tv);
		if (ret < 0) {
			return -4;
		}
		if (ret == 0) {
			return -3;
		}

		s = read(f, c, 1);
	} while ((s > 0) && (*c != REMOTE_RESP));

	/* Now collect the response */
	do {
		FD_ZERO(&rset);
		FD_SET(f, &rset);
		ret = select(f + 1, &rset, NULL, NULL, &tv);
		if (ret < 0) {
			return -4;
		}
		if (ret == 0) {
			return -3;
		}
		s = read(f, c, 1);
		if (*c == REMOTE_EOM) {
			*c = 0;
#ifdef DUMP_TRANSACTIONS
	  printf("       %s\n",data);
#endif
			return (c - data);
		} else
			c++;
	} while ((s >= 0) && (c - data < maxsize));

	return -3;
}

static int resolve_hostname(char *hostname , struct sockaddr_in * addr)
{
	struct addrinfo hints, *servinfo, *p;
	int rv;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC; // use AF_INET6 to force IPv6
	hints.ai_socktype = SOCK_STREAM;

	if ( (rv = getaddrinfo( hostname , 0 , &hints , &servinfo)) != 0)
	{
		LOG_ERROR("Failed to resolve %s (%s)", hostname, gai_strerror(rv));
		return -1;
	}

	// loop through all the results and connect to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next)
	{
		if(p->ai_addrlen == sizeof(*addr)) {
			memcpy(addr, p->ai_addr, sizeof(*addr));
			break;
		}
	}

	freeaddrinfo(servinfo); // all done with this structure
	return 0;
}

/** */
static int bmp_open (struct hl_interface_param_s *param, void **handle) {
	LOG_DEBUG("bmp_open");
	int ret;
	struct bmp_handle* priv = calloc(1, sizeof(struct bmp_handle));
	*handle = priv;

	char* bmp_hostport = getenv("BMP_HOST");

	if(bmp_hostport) {
		struct sockaddr_in addr;

		char* host = 0;

		int port;
		sscanf(bmp_hostport, "%m[^:]:%d", &host, &port);

		if(resolve_hostname(host, &addr) == 0) {
			addr.sin_port = htons(port);

		} else {
			free(host);
			goto err;
		}

		LOG_USER("BMP: Connecting to %s:%d", inet_ntoa(addr.sin_addr), ntohs(port));

		priv->fd = socket(AF_INET, SOCK_STREAM, 0);

		ret = connect(priv->fd, (struct sockaddr*)&addr, sizeof(addr));
		if(ret == 0) {

		} else {
			LOG_ERROR("connection to BMP host failed (%s)", strerror(errno));
			close(priv->fd);
			goto err;
		}

		int flags =1;
		setsockopt(priv->fd, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, sizeof(flags));
		setsockopt(priv->fd, IPPROTO_TCP, TCP_QUICKACK, (void *)&flags, sizeof(flags));

	} else {
		char* serial = getenv("BMP_SERIAL");
		if(!serial) {
			serial = "/dev/ttyACM0";
		}

		priv->fd = open(serial, O_RDWR|O_SYNC|O_NOCTTY);
		priv->is_socket = 0;
		if (priv->fd <0 )
		{
		  LOG_ERROR("Couldn't open serial port %s\n", serial);
		  goto err;
		}

		if (set_interface_attribs (priv->fd, 115000, 0) < 0)
		{
			goto err;
		}

	}

	for(int i = 0; i < 4; i++) {
		ret = write(priv->fd, REMOTE_INIT_SWDP_STR, sizeof(REMOTE_INIT_SWDP_STR));
		if(ret < 0) {
			LOG_ERROR("%s", strerror(errno));
			return ERROR_FAIL;
		}

		uint8_t buf[64];
		ret = bmp_buffer_read(priv, (uint8_t*)buf, sizeof(buf));
		if(ret > 0 && buf[0] == REMOTE_RESP_OK)
			return ERROR_OK;
	}
	if(ret < 0) {
		goto err;
	}

err:
	if(priv->fd) close(priv->fd);
	free(priv);
	return ERROR_FAIL;
}
/** */
static int bmp_close (void* handle) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;

	close(priv->fd);
	free(priv);

	return ERROR_OK;

}
/** */
static int bmp_reset (void *handle) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;

	LOG_DEBUG("bmp_reset");
	return ERROR_OK;
	int ret;

	ret = bmp_send(priv, REMOTE_RESET_STR, strlen(REMOTE_RESET_STR), 0);
	if(ret < 0) {
		LOG_ERROR("%s", strerror(errno));
		return ERROR_FAIL;
	}
	uint8_t command[32];
	ret = bmp_buffer_read(priv, (uint8_t*)command, sizeof(command));
	if(ret > 0 && command[0] == REMOTE_RESP_OK) {
		return ERROR_OK;
	}

	return ERROR_FAIL;
}
/** */
static int bmp_assert_srst (void *handle, int srst) {
	//struct bmp_handle* priv = (struct bmp_handle*)handle;
	LOG_DEBUG("bmp_assert_srst");

	return ERROR_COMMAND_NOTFOUND;

}
/** */
static int bmp_run (void* handle) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;
	return bmp_write_debug_reg(priv, DCB_DHCSR, DBGKEY|C_DEBUGEN);
}
/** */
static int bmp_halt (void* handle) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;
	return bmp_write_debug_reg(priv, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);
}
/** */
static int bmp_step (void* handle) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;
	bmp_write_debug_reg(priv, DCB_DHCSR, DBGKEY|C_HALT|C_MASKINTS|C_DEBUGEN);
	bmp_write_debug_reg(priv, DCB_DHCSR, DBGKEY|C_STEP|C_MASKINTS|C_DEBUGEN);
	return bmp_write_debug_reg(priv, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);

}
/** */
static int bmp_read_regs (void *handle) {
	LOG_ERROR("bmp_read_regs stub");
	return ERROR_COMMAND_NOTFOUND;

}
/** */
static int bmp_read_reg (void *handle, int num, uint32_t *val) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;
	int ret;
	LOG_DEBUG("bmp_read_reg %d", num);

	char command[512];
	ret = snprintf(command, sizeof(command), REMOTE_REG_READ_STR, num);

	ret = bmp_send(priv, command, ret, 0);
	if(ret < 0) {
		LOG_ERROR("%s", strerror(errno));
		return ERROR_FAIL;
	}

	ret = bmp_buffer_read(priv, (uint8_t*)command, sizeof(command));
	if(ret > 0 && command[0] == REMOTE_RESP_OK) {
		if(ret-1 == 8) {
			unhexify((uint8_t*)val, &command[1], (ret-1)/2);
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}
/** */
static int bmp_write_reg (void *handle, int num, uint32_t val) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;
	LOG_DEBUG("bmp_write_reg  %d", num);

	int ret;

	char command[64];
	ret = snprintf(command, sizeof(command), REMOTE_REG_WRITE_STR, num, val);

	ret = bmp_send(priv, command, ret, 0);
	if(ret < 0) {
		LOG_ERROR("%s", strerror(errno));
		return ERROR_FAIL;
	}

	ret = bmp_buffer_read(priv, (uint8_t*)command, sizeof(command));
	if(ret > 0 && command[0] == REMOTE_RESP_OK) {
		return ERROR_OK;
	}
	return ERROR_OK;

}
/** */
static int bmp_read_mem (void *handle, uint32_t addr, uint32_t data_width,
		uint32_t count, uint8_t *buffer) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;
	int ret;

	//LOG_DEBUG("bmp_read_mem 0x%x sz:%d cnt:%d", addr, size, count);

	const size_t bufsize = 512;
	char* tmpbuf = malloc(bufsize+32);
	size_t read_remaining = data_width*count;

	while(read_remaining) {
		count = MIN(read_remaining, bufsize/2);

		size_t num = snprintf(tmpbuf, bufsize, REMOTE_READ_MEM_STR, addr, count);
		ret = bmp_send(priv, tmpbuf, num, 0);
		if(ret < 0) {
			LOG_ERROR("%s", strerror(errno));
			return ERROR_FAIL;
		}

		ret = bmp_buffer_read(priv, (uint8_t*)tmpbuf, bufsize+32);
		if(ret > 0 && tmpbuf[0] == REMOTE_RESP_OK) {
			ret = unhexify(buffer, &tmpbuf[1], (ret-1)/2);
			addr += ret;
			buffer += ret;
			read_remaining -= ret;

			continue;
		} else {
			if(tmpbuf[0] == REMOTE_RESP_ERR) {
				LOG_ERROR("%s returned REMOTE_RESP_ERR at addr: 0x%08x", __func__, addr);
				break;
			} else {
				LOG_ERROR("%s error %d", __func__, ret);
				break;
			}
		}
	}

	free(tmpbuf);
	return read_remaining ? ERROR_FAIL : ERROR_OK;
}
/** */
static int bmp_write_mem (void *handle, uint32_t addr, uint32_t mem_width,
		uint32_t count, const uint8_t *buffer) {
	struct bmp_handle* priv = (struct bmp_handle*)handle;
	int ret;

	LOG_DEBUG("bmp_write_mem addr:%x cnt:%d val:0x%08x", addr,  mem_width*count, *(uint32_t*)buffer);

	size_t total_size = mem_width * count * 2;
	const size_t chunk_size = 512;
	size_t buffersize = MIN(chunk_size, total_size)+1;

	char* outbuf = malloc(buffersize);
	char  command[32];

	while(total_size > 0) {
		count = MIN(chunk_size, total_size)/2;
		LOG_DEBUG("\t wr chunk: addr: %08x bytes:%d", addr, count);
		ret = snprintf(command, sizeof(command), REMOTE_WRITE_MEM_STR_BEGIN, addr, count);
		ret = bmp_send(priv, command, ret, MSG_MORE);

		ret = hexify(outbuf, buffer, count, buffersize);

		total_size -= ret;
		buffer     += count;
		addr       += count;

		ret = bmp_send(priv, outbuf, ret, MSG_MORE);
		ret = bmp_send(priv, REMOTE_WRITE_MEM_STR_END, strlen(REMOTE_WRITE_MEM_STR_END), 0);

		ret = bmp_buffer_read(priv, (uint8_t*)command, sizeof(command));
		if(ret > 0 && command[0] == REMOTE_RESP_OK) {
			continue;
		} else {
			free(outbuf);
			LOG_ERROR("%s returned REMOTE_RESP_ERR at addr: 0x%08x count: %d", __func__, addr, count);

			return ERROR_FAIL;
		}
	}
	free(outbuf);
	return ERROR_OK;

}
/** */
static int bmp_write_debug_reg (void* handle, uint32_t addr, uint32_t val) {
	return bmp_write_mem(handle, addr, 4, 1, (void*)&val);
}
/**
 * Read the idcode of the target connected to the adapter
 *
 * If the adapter doesn't support idcode retrieval, this callback should
 * store 0 to indicate a wildcard match.
 *
 * @param handle A pointer to the device-specific handle
 * @param idcode Storage for the detected idcode
 * @returns ERROR_OK on success, or an error code on failure.
 */
static int bmp_idcode (void *handle, uint32_t *idcode) {
	//struct bmp_handle* priv = (struct bmp_handle*)handle;
	LOG_DEBUG("bmp_idcode");
	return ERROR_OK;

}
/** */
static int bmp_override_target (const char *targetname) {
	//struct bmp_handle* priv = (struct bmp_handle*)handle;
	LOG_DEBUG("bmp_override_target");
	return !strcmp(targetname, "cortex_m");

}
/** */
//static int bmp_custom_command (void *handle, const char *command) {
//	return ERROR_OK;
//
//}
/** */
static int bmp_speed(void *handle, int khz, bool query) {
	//struct bmp_handle* priv = (struct bmp_handle*)handle;
	LOG_DEBUG("bmp_speed");
	return ERROR_OK;

}
/**
 * Configure trace parameters for the adapter
 *
 * @param handle A handle to adapter
 * @param enabled Whether to enable trace
 * @param pin_protocol Configured pin protocol
 * @param port_size Trace port width for sync mode
 * @param trace_freq A pointer to the configured trace
 * frequency; if it points to 0, the adapter driver must write
 * its maximum supported rate there
 * @returns ERROR_OK on success, an error code on failure.
 */
static int bmp_config_trace (void *handle, bool enabled, enum tpiu_pin_protocol pin_protocol,
			uint32_t port_size, unsigned int *trace_freq) {
	//struct bmp_handle* priv = (struct bmp_handle*)handle;
	LOG_DEBUG("bmp_config_trace");
	return ERROR_OK;

}
/**
 * Poll for new trace data
 *
 * @param handle A handle to adapter
 * @param buf A pointer to buffer to store received data
 * @param size A pointer to buffer size; must be filled with
 * the actual amount of bytes written
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
static int bmp_poll_trace(void *handle, uint8_t *buf, size_t *size) {
	//struct bmp_handle* priv = (struct bmp_handle*)handle;
	LOG_DEBUG("bmp_poll_trace");
	return ERROR_OK;

}

static enum target_state bmp_get_status(void *handle)
{
	int result;
	uint32_t status;
	uint8_t  mem[4];

	result = bmp_read_mem(handle, DCB_DHCSR, 4, 1, mem);
	if  (result != ERROR_OK) {
		return TARGET_UNKNOWN;
	}
	status = le_to_h_u32(mem);
	enum target_state state;

	if (status & S_HALT)
		state = TARGET_HALTED;
	else if (status & S_RESET_ST)
		state = TARGET_RESET;
	else
		state = TARGET_RUNNING;

	return state;
}

static enum target_state bmp_state (void *handle) {

	return bmp_get_status(handle);

}

/** */
struct hl_layout_api_s blackmagic_layout_api = {
	/** */
	.open = bmp_open,
	/** */
	.close = bmp_close,
	/** */
	.idcode = bmp_idcode,
	/** */
	.state = bmp_state,
	/** */
	.reset = bmp_reset,
	/** */
	.assert_srst = bmp_assert_srst,
	/** */
	.run = bmp_run,
	/** */
	.halt = bmp_halt,
	/** */
	.step = bmp_step,
	/** */
	.read_regs = bmp_read_regs,
	/** */
	.read_reg = bmp_read_reg,
	/** */
	.write_reg = bmp_write_reg,
	/** */
	.read_mem = bmp_read_mem,
	/** */
	.write_mem = bmp_write_mem,
	/** */
	.write_debug_reg = bmp_write_debug_reg,
	/** */
	.override_target = bmp_override_target,
	/** */
	.speed = bmp_speed,
	/** */
	.config_trace = bmp_config_trace,
	/** */
	.poll_trace = bmp_poll_trace,
};
