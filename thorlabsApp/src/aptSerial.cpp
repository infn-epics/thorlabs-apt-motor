/*
 * aptSerial.cpp
 *
 * Transport layer for Thorlabs APT protocol on Linux.
 *
 * Supports two transport modes:
 *   1. Local USB serial via FTDI + POSIX termios
 *   2. Remote via TCP socket to a ser2net daemon
 *
 * The transport mode is auto-detected from the device address:
 *   - "/dev/ttyUSB0"       → local serial
 *   - "192.168.1.100:4001" → TCP / ser2net
 *
 * Serial communication parameters (from Thorlabs APT protocol spec):
 *   - 115200 baud, 8 data bits, 1 stop bit, no parity
 *   - RTS/CTS hardware flow control
 *
 * For ser2net, the serial parameters must be configured in the
 * ser2net configuration on the remote host.  The TCP link is raw bytes.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "aptSerial.h"
#include "aptProtocol.h"

extern int drvAptDebug;

enum AptTransport {
    APT_TRANSPORT_SERIAL,
    APT_TRANSPORT_TCP
};

struct AptSerial {
    int fd;
    enum AptTransport transport;
    char devicePath[256];
};

/*
 * Internal: write exactly 'count' bytes to the serial port.
 * Returns 0 on success, -1 on error.
 */
static int writeAll(int fd, const uint8_t* buf, size_t count)
{
    size_t written = 0;
    while (written < count) {
        ssize_t n = write(fd, buf + written, count - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("aptSerial: write error");
            return -1;
        }
        written += (size_t)n;
    }
    return 0;
}

/*
 * Internal: read exactly 'count' bytes from the serial port with timeout.
 * Returns 0 on success, -1 on error, -2 on timeout.
 */
static int readAll(int fd, uint8_t* buf, size_t count, int timeoutMs)
{
    size_t total = 0;
    while (total < count) {
        if (timeoutMs > 0) {
            fd_set readfds;
            struct timeval tv;
            FD_ZERO(&readfds);
            FD_SET(fd, &readfds);
            tv.tv_sec = timeoutMs / 1000;
            tv.tv_usec = (timeoutMs % 1000) * 1000;
            int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
            if (ret == 0) return -2;  /* timeout */
            if (ret < 0) {
                if (errno == EINTR) continue;
                perror("aptSerial: select error");
                return -1;
            }
        }

        ssize_t n = read(fd, buf + total, count - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("aptSerial: read error");
            return -1;
        }
        if (n == 0) {
            /* EOF / device disconnected */
            fprintf(stderr, "aptSerial: device disconnected\n");
            return -1;
        }
        total += (size_t)n;
    }
    return 0;
}

/*
 * Internal: detect whether the device address looks like a TCP endpoint.
 * Returns true if address is "host:port" (not starting with '/').
 */
static bool isTcpAddress(const char* addr)
{
    if (!addr || addr[0] == '/') return false;
    const char* colon = strrchr(addr, ':');
    if (!colon || colon == addr) return false;
    /* Check that everything after the last colon is a port number */
    for (const char* p = colon + 1; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
    }
    return (colon[1] != '\0');  /* at least one digit */
}

/*
 * Internal: open a TCP connection to a ser2net daemon.
 * address format: "host:port"  (e.g. "192.168.1.100:4001")
 */
static int openTcpConnection(const char* address)
{
    /* Parse host and port */
    char hostBuf[256];
    strncpy(hostBuf, address, sizeof(hostBuf) - 1);
    hostBuf[sizeof(hostBuf) - 1] = '\0';

    char* colon = strrchr(hostBuf, ':');
    if (!colon) return -1;
    *colon = '\0';
    const char* host = hostBuf;
    int port = atoi(colon + 1);

    if (port <= 0 || port > 65535) {
        fprintf(stderr, "aptSerial: invalid port number in %s\n", address);
        return -1;
    }

    /* Resolve the hostname */
    struct addrinfo hints, *res, *rp;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;      /* IPv4 or IPv6 */
    hints.ai_socktype = SOCK_STREAM;

    char portStr[8];
    snprintf(portStr, sizeof(portStr), "%d", port);

    int gaiErr = getaddrinfo(host, portStr, &hints, &res);
    if (gaiErr != 0) {
        fprintf(stderr, "aptSerial: cannot resolve %s: %s\n",
                host, gai_strerror(gaiErr));
        return -1;
    }

    /* Try each resolved address */
    int fd = -1;
    for (rp = res; rp != NULL; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;

        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
            break;  /* success */
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0) {
        fprintf(stderr, "aptSerial: cannot connect to %s: %s\n",
                address, strerror(errno));
        return -1;
    }

    /* Disable Nagle's algorithm — APT messages are small and latency-sensitive */
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    /* Enable TCP keepalive to detect broken links */
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));

    return fd;
}

/*
 * Internal: open a local serial device with termios configuration.
 */
static int openSerialDevice(const char* devicePath)
{
    int fd = open(devicePath, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        fprintf(stderr, "aptSerial: Cannot open %s: %s\n",
                devicePath, strerror(errno));
        return -1;
    }

    /* Set to blocking mode */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NDELAY);

    /* Configure serial port per APT protocol specification */
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "aptSerial: tcgetattr error: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Set baud rate to 115200 */
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    /* 8 data bits */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    /* 1 stop bit */
    tty.c_cflag &= ~CSTOPB;

    /* No parity */
    tty.c_cflag &= ~PARENB;

    /* Enable RTS/CTS hardware flow control */
    tty.c_cflag |= CRTSCTS;

    /* Enable receiver, ignore modem control lines */
    tty.c_cflag |= (CLOCAL | CREAD);

    /* Raw mode - no canonical processing, no echo, no signals */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* No software flow control */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* No output processing */
    tty.c_oflag &= ~OPOST;

    /* Minimum 1 byte, no timeout (blocking) */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    /* Disable special character handling */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "aptSerial: tcsetattr error: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Purge buffers (50ms dwell as per Thorlabs specification) */
    usleep(50000);
    tcflush(fd, TCIOFLUSH);
    usleep(50000);

    /* Set RTS (per Thorlabs protocol: FT_SetRts) */
    int modemBits;
    ioctl(fd, TIOCMGET, &modemBits);
    modemBits |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &modemBits);

    return fd;
}

AptSerial* aptSerialOpen(const char* devicePath)
{
    if (!devicePath) return NULL;

    int fd;
    enum AptTransport transport;

    if (isTcpAddress(devicePath)) {
        /* ---- TCP / ser2net mode ---- */
        fd = openTcpConnection(devicePath);
        if (fd < 0) return NULL;
        transport = APT_TRANSPORT_TCP;
        printf("aptSerial: Connected to ser2net at %s (TCP raw mode)\n",
               devicePath);
    } else {
        /* ---- Local serial device ---- */
        fd = openSerialDevice(devicePath);
        if (fd < 0) return NULL;
        transport = APT_TRANSPORT_SERIAL;
        printf("aptSerial: Opened %s at 115200 8N1 RTS/CTS\n", devicePath);
    }

    AptSerial* apt = (AptSerial*)calloc(1, sizeof(AptSerial));
    if (!apt) {
        close(fd);
        return NULL;
    }
    apt->fd = fd;
    apt->transport = transport;
    strncpy(apt->devicePath, devicePath, sizeof(apt->devicePath) - 1);

    return apt;
}

void aptSerialClose(AptSerial* apt)
{
    if (apt) {
        if (apt->fd >= 0) {
            close(apt->fd);
        }
        free(apt);
    }
}

int aptSendShortMessage(AptSerial* apt, uint16_t msgId,
                        uint8_t param1, uint8_t param2,
                        uint8_t dest, uint8_t source)
{
    if (!apt || apt->fd < 0) return -1;

    uint8_t buf[6];
    aptBuildShortMessage(buf, msgId, param1, param2, dest, source);

    if (drvAptDebug >= 3)
        printf("APT TX short: msgId=0x%04X p1=0x%02X p2=0x%02X dest=0x%02X\n",
               msgId, param1, param2, dest);
    if (drvAptDebug >= 4)
        printf("APT TX raw: %02X %02X %02X %02X %02X %02X\n",
               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    return writeAll(apt->fd, buf, 6);
}

int aptSendLongMessage(AptSerial* apt, uint16_t msgId,
                       const void* data, uint16_t dataLen,
                       uint8_t dest, uint8_t source)
{
    if (!apt || apt->fd < 0) return -1;

    uint8_t header[6];
    aptBuildLongHeader(header, msgId, dataLen, dest, source);

    if (drvAptDebug >= 3)
        printf("APT TX long: msgId=0x%04X dataLen=%u dest=0x%02X\n",
               msgId, dataLen, dest);
    if (drvAptDebug >= 4) {
        printf("APT TX hdr:  %02X %02X %02X %02X %02X %02X\n",
               header[0], header[1], header[2], header[3], header[4], header[5]);
        const uint8_t* d = (const uint8_t*)data;
        printf("APT TX data:");
        for (int i = 0; i < (int)dataLen && i < 32; i++)
            printf(" %02X", d[i]);
        if (dataLen > 32) printf(" ...");
        printf("\n");
    }

    int ret = writeAll(apt->fd, header, 6);
    if (ret != 0) return ret;

    if (dataLen > 0 && data != NULL) {
        ret = writeAll(apt->fd, (const uint8_t*)data, dataLen);
    }
    return ret;
}

int aptReceiveMessage(AptSerial* apt, uint8_t* header,
                      void* data, uint16_t dataMaxLen, uint16_t* dataLen,
                      int timeoutMs)
{
    if (!apt || apt->fd < 0) return -1;

    *dataLen = 0;

    /* Read the 6-byte header */
    int ret = readAll(apt->fd, header, 6, timeoutMs);
    if (ret != 0) {
        if (drvAptDebug >= 3 && ret == -2)
            printf("APT RX: timeout (%d ms)\n", timeoutMs);
        else if (drvAptDebug >= 3)
            printf("APT RX: read error (ret=%d)\n", ret);
        return ret;
    }

    if (drvAptDebug >= 4)
        printf("APT RX hdr:  %02X %02X %02X %02X %02X %02X\n",
               header[0], header[1], header[2], header[3], header[4], header[5]);

    /* Check if this is a long message */
    if (aptIsLongMessage(header)) {
        uint16_t payloadLen = aptGetDataLength(header);
        if (payloadLen > 0) {
            /*
             * Sanity check: the largest legitimate APT payload is 90 bytes
             * (MGMSG_HW_GET_INFO).  A stale/corrupted TCP frame (e.g. from
             * a previous ser2net session that was killed mid-message) can
             * claim an absurd payload length.  Attempting to read tens of
             * thousands of bytes would block indefinitely.  Refuse it.
             */
            if (payloadLen > 512) {
                fprintf(stderr, "aptSerial: CORRUPT frame ignored "
                        "(msgId=0x%04X claimed payload=%u bytes — "
                        "flushing connection)\n",
                        aptGetMsgId(header), payloadLen);
                aptSerialFlush(apt);
                return -1;
            }
            if (payloadLen > dataMaxLen) {
                fprintf(stderr, "aptSerial: data buffer too small "
                        "(need %u, have %u) — discarding\n",
                        payloadLen, dataMaxLen);
                /* Read and discard excess data */
                uint8_t discard[256];
                uint16_t remaining = payloadLen;
                while (remaining > 0) {
                    uint16_t chunk = remaining > 256 ? 256 : remaining;
                    ret = readAll(apt->fd, discard, chunk, timeoutMs);
                    if (ret != 0) return ret;
                    remaining -= chunk;
                }
                return -1;
            }
            ret = readAll(apt->fd, (uint8_t*)data, payloadLen, timeoutMs);
            if (ret != 0) return ret;
            *dataLen = payloadLen;

            if (drvAptDebug >= 3)
                printf("APT RX long: msgId=0x%04X dataLen=%u\n",
                       aptGetMsgId(header), payloadLen);
            if (drvAptDebug >= 4) {
                const uint8_t* d = (const uint8_t*)data;
                printf("APT RX data:");
                for (int i = 0; i < (int)payloadLen && i < 32; i++)
                    printf(" %02X", d[i]);
                if (payloadLen > 32) printf(" ...");
                printf("\n");
            }
        }
    } else {
        if (drvAptDebug >= 3)
            printf("APT RX short: msgId=0x%04X p1=0x%02X p2=0x%02X\n",
                   aptGetMsgId(header), header[2], header[3]);
    }

    return 0;
}

void aptSerialFlush(AptSerial* apt)
{
    if (!apt || apt->fd < 0) return;

    if (apt->transport == APT_TRANSPORT_SERIAL) {
        tcflush(apt->fd, TCIOFLUSH);
    } else {
        /* TCP: drain any pending data with a short non-blocking read loop */
        int flags = fcntl(apt->fd, F_GETFL, 0);
        fcntl(apt->fd, F_SETFL, flags | O_NONBLOCK);
        uint8_t discard[512];
        while (read(apt->fd, discard, sizeof(discard)) > 0) { /* drain */ }
        fcntl(apt->fd, F_SETFL, flags);  /* restore blocking mode */
    }
}

int aptSerialGetFd(AptSerial* apt)
{
    return apt ? apt->fd : -1;
}
