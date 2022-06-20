/*
 * Copyright (c) 2001 - 2004 Bernd Walter Computer Technology
 * http://www.bwct.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $URL$
 * $Date$
 * $Author$
 * $Rev$
 */

#include <bwct/bwct.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/uio.h>

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#ifndef MAXSOCK
#define MAXSOCK 10
#endif

int main(int argc, char *argv[]);
void usage(void);

static Mutex device_mtx;
static int device;
static uint32_t speed;
static const char* parity;
static int trailing_zero = 0;

ssize_t
writen(int fd, const void *vptr, size_t n) {
	char *ptr = (char*)vptr;
	fd_set fds;
	size_t nleft = n;
	ssize_t nwritten;
	while (nleft > 0) {
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		select(fd + 1, NULL, &fds, NULL, NULL);
		if ((nwritten = write(fd, ptr, nleft)) < 0) {
			if (errno != EAGAIN && errno != EINTR)
				return(nwritten);
			nwritten = 0;
		}
		nleft -= nwritten;
		ptr += nwritten;
	}
	return (n);
}

class FConnect : public FTask {
private:
	union {
		uint8_t data[258];
		struct {
//			uint16_t refno;
			uint8_t address;
			uint8_t function;
			uint8_t cmd[1];
		};
	} packet;
	uint8_t packetlen;
	uint8_t header[6];

	void sendpacket();
	void getpacket();
	virtual void *threadstart();
	virtual void threadend();
	void work();

	class Network : public ::Network::Net {
	public:
		Network(int nfd)
		    : ::Network::Net(nfd) {}
	};
	uint16_t calccrc();
	int checkcrc();
	void setexception (uint8_t except);

public:
        class Listen : public ::Network::Listen {
	private:
		virtual FTask *newtask();
		virtual ::Network::Net * newcon(int clientfd);
	};
	FConnect() {
	}
	~FConnect() {
	}

};

FTask *
FConnect::Listen::newtask() {
	return new FConnect();
}

::Network::Net *
FConnect::Listen::newcon(int clientfd) {
	cassert(clientfd >= 0);
	socklen_t addrlen;
	a_ptr<char> addrdt;
	addrdt = new char[MAXSOCKADDR];
	struct sockaddr *addr = (struct sockaddr*)addrdt.get();
	addrlen = MAXSOCKADDR;
	if (::getsockname(clientfd, addr, &addrlen) < 0)
		throw Error("getsockname failed");
	FConnect::Network* nobj;
	nobj = new FConnect::Network(clientfd);
	return nobj;
}

void *
FConnect::threadstart() {
	log("new connect");
	// TODO do a type checking cast
	((::Network::Net*)file.get())->nodelay(1);
	work();
	return NULL;
}

void
FConnect::threadend() {
	delete this;
}

void
FConnect::setexception (uint8_t except) {
	packet.function |= 0x80;
	packet.cmd[0] = except;
	packetlen = 3;
}

uint16_t
FConnect::calccrc() {
	uint8_t pos;
	uint8_t i;
	int lsb;
	uint16_t crc = 0xffff;
	uint8_t len = packetlen - 2;

	for (pos = 0; pos < len; pos++) {
		crc ^= packet.data[pos];
		for (i = 8; i ; i--) {
			lsb = crc & 0x01;
			crc = crc >> 1;
			if (lsb)
				crc ^= 0xa001;
		}
	}
	log(String() + "CRC " + crc);
	return crc;
}

int
FConnect::checkcrc() {
	uint16_t crc = calccrc();
	return ((packet.data[packetlen - 2] == (crc & 0xff)) &&
	    (packet.data[packetlen - 1] == (crc >> 8)));
}

void
FConnect::sendpacket() {
	uint16_t crc;

	packetlen += 2;
	crc = calccrc();
	packet.data[packetlen - 2] = crc & 0xff;
	packet.data[packetlen - 1] = crc >> 8;
	tcflush(device, TCIFLUSH); // flush received data
	::writen(device, packet.data, packetlen);
	return;
}

void
FConnect::getpacket() {
	int res;
	ssize_t tmp;
	fd_set fds;
	struct timeval to;

	packetlen = 0;
	// wait for the first packet befor starting
	FD_ZERO(&fds);
	FD_SET(device, &fds);
	to.tv_sec = 0;
	to.tv_usec = 800000;
	res = select(device + 1, &fds, NULL, NULL, &to);
	if (res <= 0) {
		setexception(0x0b);
		return;
	}
	if (!FD_ISSET(device, &fds)) {
		setexception(0x0b);
		return;	// reaction timeout
	}
	// no range check needed because of overflow
	for (packetlen = 0; ; ) {
		FD_ZERO(&fds);
		FD_SET(device, &fds);
		to.tv_sec = 0;
		//to.tv_usec = (suseconds_t) (1000000 / speed * 11 * 1.5);
		// multitasking OS and USB serials have problems with low latency
		to.tv_usec = 800000;
		res = select(device + 1, &fds, NULL, NULL, &to);
		if (FD_ISSET(device, &fds)) {
			tmp = ::read(device, &packet.data[packetlen],
			    256 - packetlen);	// TODO check for parity error
			if (tmp >= 0) {
				packetlen += tmp;
			} else {
				// wait t3.5 to block other possible writes
				usleep((useconds_t) (1000000 / speed * 11 * 3.5));
				setexception(0x0b);
				return;
			}
		} else {
			packetlen -= trailing_zero;
			// wait t3.5 to block other possible writes
			usleep((useconds_t) (1000000 / speed * 11 * 3.5));
			if (!checkcrc()) {
				setexception(0x0b);
				return;
			}
			packetlen -= 2;
			return;
		}
	}
}

void
FConnect::work() {
	ssize_t res;
	uint8_t sbuf[256+6];

	for (;;) {
		// TODO: timeout handling
		res = file->read(&header, sizeof(header));
		if (res < (ssize_t)sizeof(header)) {
			return;
		}
		// TODO check header arguments;
		packetlen = header[4] << 8 | header[5];
		// TODO check packetlen;
		res = file->read(&packet.data[0], packetlen);
		if (res < packetlen) {
			return;
		}
		//packetlen -= 2;	// drop address and function bytes
		device_mtx.lock();
		sendpacket();
		getpacket();	// TODO: don't expect response from broadcast packets
				// TODO: we are a bridge, so we should implement 0xff response
		device_mtx.unlock();
		//packetlen += 2; // add address and function bytes
		header[4] = 0;
		header[5] = packetlen;
		memcpy(&sbuf[0], header, sizeof(header));
		memcpy(&sbuf[sizeof(header)], &packet.data[0], packetlen);
		file->write(sbuf, sizeof(header) + packetlen);
	}
}

void
opensio(const char* devname) {
	struct termios buf;
	int val;

	device = open(devname, O_RDWR);
	if (device < 0) {
		printf("open tty %s failed: %s\n",
		    devname, strerror(errno));
		exit(1);
	}
	val = fcntl(device, F_GETFL, 0);
	fcntl(device, F_SETFL, val | O_NONBLOCK);
	if (tcgetattr(device, &buf) < 0) {
		printf("tcgetattr failed: %s\n", strerror(errno));
		exit (1);
	}
	cfmakeraw(&buf);
	buf.c_iflag |= IGNBRK;
	buf.c_cflag &= ~(CSIZE | PARODD);
	buf.c_cflag |= CS8 | CLOCAL;
	switch (parity[0]) {
	case 'n':
		buf.c_cflag &= ~PARENB;
		break;
	case 'e':
		buf.c_cflag |= PARENB;
		buf.c_cflag &= ~PARODD;
		break;
	case 'o':
		buf.c_cflag |= PARENB;
		buf.c_cflag |= PARODD;
		break;
	}
	uint32_t speedvalue = speed;
#ifdef __linux__
	switch (speed) {
#ifdef B50
	case 50:
		speedvalue = B50;
		break;
#endif
#ifdef B75
	case 75:
		speedvalue = B75;
		break;
#endif
#ifdef B110
	case 110:
		speedvalue = B110;
		break;
#endif
#ifdef B134
	case 134:
		speedvalue = B134;
		break;
#endif
#ifdef B150
	case 150:
		speedvalue = B150;
		break;
#endif
#ifdef B200
	case 200:
		speedvalue = B200;
		break;
#endif
#ifdef B300
	case 300:
		speedvalue = B300;
		break;
#endif
#ifdef B600
	case 600:
		speedvalue = B600;
		break;
#endif
#ifdef B1200
	case 1200:
		speedvalue = B1200;
		break;
#endif
#ifdef B1800
	case 1800:
		speedvalue = B1800;
		break;
#endif
#ifdef B2400
	case 2400:
		speedvalue = B2400;
		break;
#endif
#ifdef B4800
	case 4800:
		speedvalue = B4800;
		break;
#endif
#ifdef B9600
	case 9600:
		speedvalue = B9600;
		break;
#endif
#ifdef B19200
	case 19200:
		speedvalue = B19200;
		break;
#endif
#ifdef B38400
	case 38400:
		speedvalue = B38400;
		break;
#endif
#ifdef B57600
	case 57600:
		speedvalue = B57600;
		break;
#endif
#ifdef B115200
	case 115200:
		speedvalue = B115200;
		break;
#endif
#ifdef B230400
	case 230400:
		speedvalue = B230400;
		break;
#endif
#ifdef B460800
	case 460800:
		speedvalue = B460800;
		break;
#endif
#ifdef B500000
	case 500000:
		speedvalue = B500000;
		break;
#endif
#ifdef B576000
	case 576000:
		speedvalue = B576000;
		break;
#endif
#ifdef B921600
	case 921600:
		speedvalue = B921600;
		break;
#endif
#ifdef B1000000
	case 1000000:
		speedvalue = B1000000;
		break;
#endif
#ifdef B1152000
	case 1152000:
		speedvalue = B50;
		break;
#endif
#ifdef B1500000
	case 1500000:
		speedvalue = B1500000;
		break;
#endif
#ifdef B2000000
	case 2000000:
		speedvalue = B2000000;
		break;
#endif
#ifdef B2500000
	case 2500000:
		speedvalue = B2500000;
		break;
#endif
#ifdef B3000000
	case 3000000:
		speedvalue = B3000000;
		break;
#endif
#ifdef B3500000
	case 3500000:
		speedvalue = B3500000;
		break;
#endif
#ifdef B4000000
	case 4000000:
		speedvalue = B4000000;
		break;
#endif
	}
#endif
	cfsetspeed(&buf, speedvalue);
	if (tcsetattr(device, TCSAFLUSH, &buf) < 0) {
		printf("tcsetattr failed: %s\n", strerror(errno));
		exit (1);
	}
}

int
main(int argc, char *argv[]) {
	FConnect::Listen listen;
	const char* ttypath;
	int ch;

	ttypath = NULL;
	device = -1;
	speed = 115200;
	parity = "even";

	while ((ch = getopt(argc, argv, "p:s:t:z")) != -1)
		switch (ch) {
		case 't':
			ttypath = optarg;
			break;
		case 's':
			speed = atoll(optarg);
			break;
		case 'p':
			parity = optarg;
			break;
		case 'z':
			trailing_zero = 1;
			break;
		case '?':
		default:
			usage();
			/* NOTREACHED */
	}
	argc -= optind;
	argv += optind;

	if (argc != 2 || ttypath == NULL)
		usage();

	opensio(ttypath);

	if (device == -1) {
		printf("failed to open device\n");
		exit(1);
	}

	listen.add_tcp(argv[0], argv[1], 100);
	daemon(0,0);

	listen.loop();
	return 0;
}

void
usage(void) {

	printf("usage: mb_tcpbridge_tty -t tty [-s speed] [-p parity] [-z] ip port\n");
	exit(1);
}

